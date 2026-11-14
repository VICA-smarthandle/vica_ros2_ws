"""person_detector_node — YOLO(seg) 로 시각장애인을 찾아 `/vica/person_detection` 발행.

Phase A5 골격. 계약은 `vica_interfaces/msg/PersonDetection.msg` 가 정본이다:

    - frame_id "map". **이 노드가 TF 변환을 끝내고** 발행한다. 소비자는 변환하지 않는다.
    - 5 Hz. CPU 병목이라 그 이상 올리지 않는다(설계 §6.2).
    - distance_m 은 몸통 depth 중앙값 기반. 못 재면 NaN (0.0 은 오독되므로 금지).
    - stable / approachable 판정은 detection_gate(A3) 가 한다. 시계열 이력을 가진
      쪽이 판정한다는 계약이다.
    - track_id 는 Ultralytics 내장 추적기의 것이다. id 가 없으면 TRACK_ID_NONE=0
      이고 이때 approachable 은 항상 false 다.

이 노드는 Isaac ROS 컨테이너 안에서 돈다(추론 위치 결정, 핸드오프 §1.1).
엔진(.engine)은 만든 기기에만 묶이므로 경로 기본값은 컨테이너 기준이다.

알려진 한계 [미검증]:
    - depth 가 color 에 정렬되지 않았다(run_d455.sh align_depth:=false). color bbox
      를 depth 이미지에 그대로 쓰므로 시차(수 cm)가 남는다. 문제가 되면
      align_depth 를 켜고 이 주석을 갱신할 것.
    - 검출률 실측(2026-08-24, 정지 2.1 m): 프레임 45~48 %, conf 는 잡히면 0.77.
      놓침의 절반은 conf 0 이라 문턱 조정으로 안 오른다. stable 창 통과 ~50 % —
      방아쇠는 몇 초 안에 당겨진다. 근본 개선은 로봇 시점 재학습이다.
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
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
from vica_perception.person_geometry import (
    bbox_center,
    body_depth_median_m,
    pixel_to_camera,
)

DEFAULT_ENGINE = "/workspaces/isaac_ros-dev/models/v6-blur-640/weights/best.engine"

# 접근 판정에 쓰는 conf 문턱. detection_gate 의 DEFAULT_MIN_CONFIDENCE(0.6)는
# 설계 때 가정값이고, 실측으로 올린 값을 노드 파라미터로 넣는다(그 파일의 규칙).
#
# 근거 — 2026-11-10 오탐 평가셋(~/vica_data/analysis/cane_fp_2026-11-10/eval_set,
# 31회차, E-stop 상태, 실제 관문 규칙 '3 s 이력·끊김 0.6 s' 그대로 오프라인 재생):
#     문턱 0.6  진짜(흰 지팡이 1.8~7.2 m, 9회차) 9/9, 오탐 장면 7
#     문턱 0.8  진짜 9/9, 오탐 장면 3 (흰 자를 지팡이처럼 짚기 2·나무 막대 1)
#     (8.45 m 진짜 1회차는 접근 상한 8.0 m 밖이라 셈에서 뺐다)
#     문턱 0.85 7.19 m 진짜를 놓침 / 0.9 는 7.19 m 와 옆모습 1.77 m 를 놓침
# 오탐은 사람 몸에 붙은 밝고 가는 물체(흰 줄·밀대 자루·나무 막대)에서 conf 가
# 0.6~0.9 를 오르내리고, 진짜는 0.86~0.94 로 꾸준하다.
# [대가] 7.19 m 진짜의 conf 중앙이 0.80 이라 접근 승인이 3.2 s → 9.0 s 로,
# 옆모습 1.77 m 는 3.1 s → 7.0 s 로 늦어진다(재생 기준).
# 같은 날 0.8 로 감지기를 돌린 실기 6회차: 진짜 1.76·3.09·6.31 m 모두 approachable,
# 목에 건 흰 줄 2회차 approachable 0, 밀대 자루를 지팡이처럼 세워 짚은 사람(2.58 m,
# conf 중앙 0.85)은 통과 — 지팡이처럼 짚은 밝은 막대는 문턱으로 못 거른다.
# [미검증] 7 m 이상 지연은 실기 재확인 못 함. 회차마다 1명·20 s.
DEFAULT_APPROACH_MIN_CONFIDENCE = 0.8

# 접근 관문에 들어가는 노드 파라미터와 기본값. 노드는 이 표대로 선언한다.
APPROACH_PARAM_DEFAULTS = {
    "approach_min_confidence": DEFAULT_APPROACH_MIN_CONFIDENCE,
    "approach_min_distance_m": DEFAULT_MIN_DISTANCE_M,
    "approach_max_distance_m": DEFAULT_MAX_DISTANCE_M,
}


def approach_gate_thresholds(get_param) -> GateThresholds:
    """노드 파라미터 → 접근 관문 문턱. get_param(이름) 이 값을 돌려준다.

    시험이 rclpy 노드를 만들지 않고 이름 배선을 확인하려고 뺐다 — 노드를 만들고
    지우는 시험은 rclpy destroy_node 에서 가끔 멈췄다(2026-11-10).
    """
    return GateThresholds(
        min_confidence=float(get_param("approach_min_confidence")),
        min_distance_m=float(get_param("approach_min_distance_m")),
        max_distance_m=float(get_param("approach_max_distance_m")),
    )


class PersonDetectorNode(Node):
    """배선만 갖는다 — 판정은 detection_gate, 기하는 person_geometry."""

    def __init__(self, model) -> None:
        super().__init__("person_detector_node")
        self.declare_parameter("engine_path", DEFAULT_ENGINE)
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("target_frame", "map")
        # 주행 중 추론 차단. 끄면 종전대로 항상 추론한다(inference_gate 참고).
        self.declare_parameter("gate_while_moving", True)
        self.declare_parameter("robot_state_timeout_s", DEFAULT_STATE_TIMEOUT_S)
        # 접근 대상으로 인정할 거리 범위. 기본값은 detection_gate 의 상수 그대로다
        # (1.5 ~ 4.0 m). 값을 바꾸려면 그 파일의 근거 주석을 먼저 읽을 것 —
        # 하한은 접근 goal(1.1 m)이 이미 지나간 자리인지, 상한은 도착까지 걸리는
        # 시간이 근거다. 잘못된 범위(하한 >= 상한)는 GateThresholds 가 기동 때
        # 예외로 막는다.
        # 접근 판정 conf 문턱 approach_min_confidence 도 같은 표에 있다(근거는
        # DEFAULT_APPROACH_MIN_CONFIDENCE 주석). 발행 문턱 conf_threshold(0.25)와
        # 다르다 — 그보다 낮은 상자는 아예 안 나가고, 이 문턱보다 낮은 상자는
        # 발행은 되지만 접근 증거에서 빠진다.
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
        # 마지막으로 로그에 남긴 사유. 프레임마다 찍으면 5 Hz 로 로그가 넘친다.
        self._last_infer_reason: InferenceReason | None = None
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._depth: Image | None = None
        self._depth_info: CameraInfo | None = None
        # 5 Hz 조절용. 벽시계가 아니라 단조 시계다 — 시간이 뒤로 가면 안 된다.
        self._last_infer_mono = 0.0

        self._pub = self.create_publisher(PersonDetection, "/vica/person_detection", 10)
        # ── 접근 요청 (Phase B3) ─────────────────────────────────────────
        # approachable 인 탐지만 Mission Manager 에 요청한다(srv 계약).
        # 접근 중의 재요청은 goal 갱신으로 쓰이므로 끊지 않되, 호출 자체는
        # track 당 초 1건으로 줄인다(approach_request_policy).
        self._approach_cli = self.create_client(
            RequestApproach, "/vica/mission/request_approach")
        self._throttle = ApproachRequestThrottle()
        # 미응답 재호출 방지 플래그 + 그 시각. 서비스가 죽으면 call_async 의
        # future 가 영영 안 끝나 플래그가 True 로 굳는다(2026-08-24 실기 -
        # Mission 재시작 사이에 보낸 호출이 그랬다). 2초 지나면 버린 것으로
        # 보고 다시 보낸다.
        self._pending_since_ns: int | None = None
        self.create_subscription(Image, "/camera/camera/color/image_raw",
                                 self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw",
                                 self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera/depth/camera_info",
                                 self._on_depth_info, qos_profile_sensor_data)
        # 주행 여부만 본다. QoS 는 기본 신뢰(depth 3 종과 달리 sensor_data 가
        # 아니다) — 1 Hz 상태 한 건을 놓치면 그만큼 판정이 늦는다.
        self.create_subscription(RobotState, "/vica/robot_state",
                                 self._on_robot_state, 10)
        self.get_logger().info(
            "person_detector_node 시작 (conf %.2f, %.1f Hz, frame %s, "
            "주행 중 추론 %s, 접근 거리 %.1f~%.1f m, 접근 conf ≥ %.2f)"
            % (self._conf, 1.0 / self._period_s, self._target_frame,
               "차단" if self._infer_gate.enabled else "허용",
               self._approach_min_m, self._approach_max_m,
               self._approach_min_conf))

    # ── 입력 보관 ──────────────────────────────────────────────────────────
    def _on_depth(self, msg: Image) -> None:
        self._depth = msg

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._depth_info = msg

    def _on_robot_state(self, msg: RobotState) -> None:
        # RobotState 에는 header 가 없다. 신선도는 수신 시각으로 잰다.
        self._infer_gate.observe_state(
            time.monotonic_ns(), msg.is_moving, msg.is_paused)

    # ── 본 처리 ────────────────────────────────────────────────────────────
    def _on_color(self, msg: Image) -> None:
        # 5 Hz 솎기보다 **먼저** 본다. 차단 중에는 `_last_infer_mono` 를 건드리지
        # 않으므로, 주행이 끝나면 다음 프레임에서 곧바로 추론이 돌아온다.
        now_ns = time.monotonic_ns()
        reason = self._infer_gate.reason(now_ns)
        if reason is not self._last_infer_reason:
            self.get_logger().info("추론 게이트: %s" % reason.value)
            self._last_infer_reason = reason
        if not self._infer_gate.should_infer(now_ns):
            return

        now_mono = time.monotonic()
        if now_mono - self._last_infer_mono < self._period_s:
            return                       # 5 Hz 로 솎는다
        self._last_infer_mono = now_mono

        img = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = img.reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]        # 모델은 BGR 를 기대한다

        result = self._model.track(
            img, device=0, conf=self._conf, persist=True, verbose=False)[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return                       # 발행 없음 = 사람 없음. 소비자는 수신
            #                              시각(STEADY)으로 stale 을 판정한다.

        depth_m, depth_info = self._depth_frame()
        steady_ns = time.monotonic_ns()

        ids = boxes.id
        for i in range(len(boxes)):
            bbox = tuple(float(v) for v in boxes.xyxy[i].tolist())
            conf = float(boxes.conf[i])
            track_id = int(ids[i]) if ids is not None else PersonDetection.TRACK_ID_NONE

            dist_m, map_pt = self._locate(bbox, depth_m, depth_info, msg)

            out = PersonDetection()
            out.header.stamp = msg.header.stamp      # 이미지 취득 시각(표시 전용)
            out.header.frame_id = self._target_frame
            out.confidence = conf
            out.track_id = track_id
            out.distance_m = dist_m
            out.pose.orientation.w = 1.0             # 방향은 판정하지 않는다(계약)

            if map_pt is not None:
                out.pose.position.x = map_pt[0]
                out.pose.position.y = map_pt[1]
                out.pose.position.z = 0.0            # 사람은 바닥에 서 있다

            # 시계열 판정. 위치·id 가 온전할 때만 이력에 넣는다 — 결손 표본으로
            # stable 을 세우면 가짜 방아쇠가 된다.
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
        """Mission Manager 에 접근을 요청한다. 응답은 로그로만 소비한다.

        승인·거절 판단은 전적으로 Mission Manager 몫이다(goal 권한의 경계).
        동기 대기하면 5 Hz 콜백이 서비스 왕복에 볼모로 잡히므로 비동기로 보내고,
        직전 응답이 안 왔으면 이번 건은 건너뛴다 — 요청은 다음 프레임에 또 온다.
        """
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

    # ── 보조 ──────────────────────────────────────────────────────────────
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
        # color bbox 를 depth 픽셀로. 해상도가 다르면 비율로 맞춘다.
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
        # stamp 0 = "가장 최신 TF". 이미지 시각을 넣으면 그 시각의 TF 가 아직
        # 안 와서 extrapolation into the future 로 매번 실패한다(B5 실기에서
        # 실측 5~90 ms 차). 단일 스레드 실행기 안에서는 timeout 대기 중에 TF
        # 콜백이 돌지 못하므로 기다려도 해결되지 않는다 — 최신값이 정답이다.
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x, pt.point.y, pt.point.z = cam
        try:
            # 최신 TF 사용(Time()) — 이미지 시각의 과거 TF 를 기다리다 5 Hz 를
            # 놓치는 것보다 수십 ms 의 자세 오차가 싸다.
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
    # ultralytics 는 무겁다 — rclpy 초기화 전에, 실패는 일찍 드러낸다.
    from ultralytics import YOLO
    rclpy.init(args=args)
    # 파라미터로 engine 경로를 받으려면 노드가 먼저 필요해 닭-달걀이 된다.
    # 골격에서는 기본 경로로 로드하고, 바꾸려면 환경변수로 준다.
    import os
    engine = os.environ.get("VICA_YOLO_ENGINE", DEFAULT_ENGINE)
    model = YOLO(engine)
    model.predict(np.zeros((480, 640, 3), dtype=np.uint8), device=0, verbose=False)
    node = PersonDetectorNode(model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
