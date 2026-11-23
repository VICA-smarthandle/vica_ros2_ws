"""person_detector_node — YOLO(seg) 로 시각장애인을 찾아 `/vica/person_detection` 발행."""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from vica_interfaces.msg import PersonDetection, RobotState
from vica_interfaces.srv import RequestApproach
from vica_perception.approach_request_policy import ApproachRequestThrottle
from vica_perception.detection_gate import (
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_MIN_DISTANCE_M,
    DetectionGate,
    DetectionSample,
    GateThresholds,
    Point2D,
)
from vica_perception.inference_gate import (
    DEFAULT_STATE_TIMEOUT_S,
    InferenceGate,
    InferenceReason,
)
from vica_perception.patrol_profile import is_patrol_state, parse_watch_off
from vica_perception.patrol_wiring import PatrolWiring
from vica_perception.person_geometry import (
    bbox_center,
    body_depth_median_m,
    pixel_to_camera,
)

DEFAULT_ENGINE = "/workspaces/isaac_ros-dev/models/v6-blur-640/weights/best.engine"

DEFAULT_APPROACH_MIN_CONFIDENCE = 0.8

APPROACH_PARAM_DEFAULTS = {
    "approach_min_confidence": DEFAULT_APPROACH_MIN_CONFIDENCE,
    "approach_min_distance_m": DEFAULT_MIN_DISTANCE_M,
    "approach_max_distance_m": DEFAULT_MAX_DISTANCE_M,
}


def approach_gate_thresholds(get_param) -> GateThresholds:
    """노드 파라미터 → 접근 관문 문턱. get_param(이름) 이 값을 돌려준다."""
    return GateThresholds(
        min_confidence=float(get_param("approach_min_confidence")),
        min_distance_m=float(get_param("approach_min_distance_m")),
        max_distance_m=float(get_param("approach_max_distance_m")),
    )


def load_engine(path: str):
    """엔진을 올리고 한 번 돌려 본다. 기동(지팡이 엔진)과 첫 순찰(순찰 엔진)이 같이 쓴다."""
    from ultralytics import YOLO
    model = YOLO(path)
    model.predict(np.zeros((480, 640, 3), dtype=np.uint8), device=0, verbose=False)
    return model


class PersonDetectorNode(Node):
    """배선만 갖는다 — 판정은 detection_gate, 기하는 person_geometry."""

    def __init__(self, model, load_model=None) -> None:
        super().__init__("person_detector_node")
        self.declare_parameter("engine_path", DEFAULT_ENGINE)
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("gate_while_moving", True)
        self.declare_parameter("robot_state_timeout_s", DEFAULT_STATE_TIMEOUT_S)
        for name, default in APPROACH_PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self._model = model
        self._conf = float(self.get_parameter("conf_threshold").value)
        self._period_s = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self._target_frame = str(self.get_parameter("target_frame").value)

        thresholds = approach_gate_thresholds(
            lambda name: self.get_parameter(name).value)
        self._approach_min_m = thresholds.min_distance_m
        self._approach_max_m = thresholds.max_distance_m
        self._approach_min_conf = thresholds.min_confidence
        self._gate = DetectionGate(thresholds)
        self._infer_gate = InferenceGate(
            state_timeout_s=float(
                self.get_parameter("robot_state_timeout_s").value),
            enabled=bool(self.get_parameter("gate_while_moving").value),
        )
        self._last_infer_reason: InferenceReason | None = None
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._depth: Image | None = None
        self._depth_info: CameraInfo | None = None
        self._last_infer_mono = 0.0

        self._pub = self.create_publisher(PersonDetection, "/vica/person_detection", 10)
        self._approach_cli = self.create_client(
            RequestApproach, "/vica/mission/request_approach")
        self._throttle = ApproachRequestThrottle()
        self._pending_since_ns: int | None = None
        self._load_model = load_model
        self._patrol: PatrolWiring | None = None
        self._patrol_busy = False
        self._patrol_ok = (
            load_model is not None
            and "dialog_state" in RobotState.get_fields_and_field_types())
        if load_model is not None and not self._patrol_ok:
            self.get_logger().warning(
                "RobotState 에 dialog_state 가 없다(옛 vica_interfaces) — 순찰 프로필 꺼짐")
        self._patrol_watch_off: frozenset = frozenset()
        if self._patrol_ok:
            self.create_subscription(
                String, "/vica/patrol/watch", self._on_patrol_watch,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Image, "/camera/camera/color/image_raw",
                                 self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw",
                                 self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera/depth/camera_info",
                                 self._on_depth_info, qos_profile_sensor_data)
        self.create_subscription(RobotState, "/vica/robot_state",
                                 self._on_robot_state, 10)
        self.get_logger().info(
            "person_detector_node 시작 (conf %.2f, %.1f Hz, frame %s, "
            "주행 중 추론 %s, 접근 거리 %.1f~%.1f m, 접근 conf ≥ %.2f)"
            % (self._conf, 1.0 / self._period_s, self._target_frame,
               "차단" if self._infer_gate.enabled else "허용",
               self._approach_min_m, self._approach_max_m,
               self._approach_min_conf))

    def _on_depth(self, msg: Image) -> None:
        self._depth = msg

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._depth_info = msg

    def _on_robot_state(self, msg: RobotState) -> None:
        self._infer_gate.observe_state(
            time.monotonic_ns(), msg.is_moving, msg.is_paused)
        if self._patrol_busy or (
                self._patrol_ok and is_patrol_state(msg.dialog_state)):
            if self._patrol is None:
                self._patrol = PatrolWiring(
                    self, self._load_model,
                    rate_hz=1.0 / self._period_s,
                    state_timeout_s=float(
                        self.get_parameter("robot_state_timeout_s").value))
                self._patrol.set_watch_off(self._patrol_watch_off)
            self._patrol.on_dialog_state(msg.dialog_state)

    def _on_patrol_watch(self, msg: String) -> None:
        off = parse_watch_off(msg.data)
        if off is None:
            return
        if self._patrol is not None:
            self._patrol.set_watch_off(off)
        elif off != self._patrol_watch_off:
            self.get_logger().info(
                "순찰 감시 항목 알림 — 끔: %s" % (", ".join(sorted(off)) or "없음"))
        self._patrol_watch_off = off

    def patrol_shutdown(self) -> None:
        """종료 직전에 부른다. 순찰을 쓴 적이 없으면 아무것도 안 한다."""
        if self._patrol is not None:
            self._patrol.shutdown()

    def _on_color(self, msg: Image) -> None:
        if self._patrol_busy:
            self._patrol.on_color(msg)
            return
        now_ns = time.monotonic_ns()
        reason = self._infer_gate.reason(now_ns)
        if reason is not self._last_infer_reason:
            self.get_logger().info("추론 게이트: %s" % reason.value)
            self._last_infer_reason = reason
        if not self._infer_gate.should_infer(now_ns):
            return

        now_mono = time.monotonic()
        if now_mono - self._last_infer_mono < self._period_s:
            return
        self._last_infer_mono = now_mono

        img = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = img.reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]

        result = self._model.track(
            img, device=0, conf=self._conf, persist=True, verbose=False)[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return

        depth_m, depth_info = self._depth_frame()
        steady_ns = time.monotonic_ns()

        ids = boxes.id
        for i in range(len(boxes)):
            bbox = tuple(float(v) for v in boxes.xyxy[i].tolist())
            conf = float(boxes.conf[i])
            track_id = int(ids[i]) if ids is not None else PersonDetection.TRACK_ID_NONE

            dist_m, map_pt = self._locate(bbox, depth_m, depth_info, msg)

            out = PersonDetection()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self._target_frame
            out.confidence = conf
            out.track_id = track_id
            out.distance_m = dist_m
            out.pose.orientation.w = 1.0

            if map_pt is not None:
                out.pose.position.x = map_pt[0]
                out.pose.position.y = map_pt[1]
                out.pose.position.z = 0.0

            out.stable = False
            out.approachable = False
            if (track_id != PersonDetection.TRACK_ID_NONE
                    and map_pt is not None and math.isfinite(dist_m)):
                verdict = self._gate.observe(DetectionSample(
                    stamp_ns=steady_ns,
                    track_id=track_id,
                    confidence=conf,
                    position=Point2D(map_pt[0], map_pt[1]),
                    distance_m=dist_m,
                ))
                out.stable = verdict.stable
                out.approachable = verdict.approachable

            self._pub.publish(out)

            if out.approachable and self._throttle.should_send(track_id, steady_ns):
                self._request_approach(out)

    def _request_approach(self, detection: PersonDetection) -> None:
        """Mission Manager 에 접근을 요청한다. 응답은 로그로만 소비한다."""
        now_ns = time.monotonic_ns()
        if self._pending_since_ns is not None:
            if now_ns - self._pending_since_ns < 2_000_000_000:
                return
            self.get_logger().warning("접근 요청 응답 2초 무소식 — 버린 것으로 본다")
            self._pending_since_ns = None
        if not self._approach_cli.service_is_ready():
            self.get_logger().warning(
                "request_approach 서비스가 없다 — Mission Manager 미기동?",
                throttle_duration_sec=10.0)
            return
        import uuid
        req = RequestApproach.Request()
        req.request_id = str(uuid.uuid4())
        req.track_id = detection.track_id
        req.target = detection
        self._pending_since_ns = now_ns
        future = self._approach_cli.call_async(req)
        track = detection.track_id

        def _done(fut) -> None:
            self._pending_since_ns = None
            try:
                res = fut.result()
            except Exception as e:                     # noqa: BLE001 — 로그 후 계속
                self.get_logger().warning("접근 요청 실패: %s" % e)
                return
            if res.accepted:
                self.get_logger().info("접근 승인 (track %d): %s" % (track, res.message))
            else:
                self.get_logger().info(
                    "접근 거절 (track %d): %s" % (track, res.message),
                    throttle_duration_sec=5.0)

        future.add_done_callback(_done)

    def _depth_frame(self):
        """보관된 depth 를 미터 배열로. 없으면 (None, None)."""
        if self._depth is None or self._depth_info is None:
            return None, None
        d = self._depth
        if d.encoding not in ("16UC1", "mono16"):
            return None, None
        arr = np.frombuffer(bytes(d.data), dtype=np.uint16)
        arr = arr.reshape(d.height, d.width).astype(np.float32) / 1000.0
        return arr, self._depth_info

    def _locate(self, bbox, depth_m, depth_info, color_msg):
        """bbox → (distance_m, map 좌표 (x, y)) — 못 재면 (NaN, None)."""
        if depth_m is None:
            return math.nan, None
        sx = depth_m.shape[1] / color_msg.width
        sy = depth_m.shape[0] / color_msg.height
        dbox = (bbox[0] * sx, bbox[1] * sy, bbox[2] * sx, bbox[3] * sy)
        z = body_depth_median_m(depth_m, dbox)
        k = depth_info.k
        u, v = bbox_center(dbox)
        cam = pixel_to_camera(u, v, z, k[0], k[4], k[2], k[5])
        if cam is None:
            return math.nan, None

        pt = PointStamped()
        pt.header.frame_id = self._depth.header.frame_id
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x, pt.point.y, pt.point.z = cam
        try:
            to_map = self._tf.transform(pt, self._target_frame,
                                        timeout=rclpy.duration.Duration(seconds=0.1))
            to_base = self._tf.transform(pt, "base_link",
                                         timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warning("TF 변환 실패: %s" % e, throttle_duration_sec=5.0)
            return math.nan, None
        dist = math.hypot(to_base.point.x, to_base.point.y)
        return dist, (to_map.point.x, to_map.point.y)


def main(args=None) -> None:
    from ultralytics import YOLO  # noqa: F401
    rclpy.init(args=args)
    import os
    engine = os.environ.get("VICA_YOLO_ENGINE", DEFAULT_ENGINE)
    node = PersonDetectorNode(load_engine(engine), load_engine)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.patrol_shutdown()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
