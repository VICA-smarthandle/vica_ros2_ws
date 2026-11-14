"""순찰 프로필 ROS 배선 — 판정은 `patrol_profile`, 저장은 `patrol_media` 가 한다.

`person_detector_node` 가 순찰 상태(`dialog_state` 가 patrol…)를 **처음 볼 때** 만든다.
그 전에는 발행자·서비스 클라이언트·타이머·스레드가 없다. 순찰이 끝나면 타이머를
없애고 컬러 프레임을 곧바로 기존 경로로 돌려준다. 쓰던 영상 장면이 있으면 그 장면이
끝날 때까지만(최대 60 s) 더 붙잡는다.

여기서 하는 일:

    발행        `/vica/patrol/event` (std_msgs/String, JSON)
    노출        `/camera/camera/set_parameters` 비동기 호출
    순찰 엔진   첫 순찰 때 따로 띄운 스레드에서 한 번 올려 계속 둔다
    1 Hz 점검   status·노출 복구 재시도·상태 끊김

엔진을 내렸다 다시 올리지 않는다. 지팡이 엔진은 건드리지 않고, 순찰 엔진은 순찰이
끝나도 쥐고 있는다(추론은 안 한다). 근거 — 2026-11-14 컨테이너 실측: 순찰 시작·종료
3번에 RSS 가 1048 → 1120 → 1149 → 1204 MB 로 회마다 30~55 MB 늘고 돌아오지 않았다.
같이 올려 두는 비용은 한 번 30~40 MB 다.

노드와의 약속: `node._patrol_busy`(True 면 컬러 프레임과 `/vica/robot_state` 를
이 배선이 받는다)만 이 배선이 바꾼다.
"""
from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time
from typing import Callable, Optional

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from vica_perception.inference_gate import sec_to_ns
from vica_perception.patrol_media import (
    DEFAULT_MEDIA_DIR,
    JPEG_QUALITY,
    cleanup_media,
    image_to_bgr,
    make_jpeg_encoder,
    mean_brightness,
    patrol_zone,
    patrol_zone_name,
)
from vica_perception.patrol_profile import (
    LIGHT_EXPOSURE,
    PERSON_MIN_CONF,
    STATUS_PERIOD_S,
    ExposureCmd,
    PatrolPorts,
    PatrolProfile,
)

EVENT_TOPIC = "/vica/patrol/event"
CAMERA_PARAM_SERVICE = "/camera/camera/set_parameters"
DEFAULT_PATROL_ENGINE = "/workspaces/isaac_ros-dev/models/patrol-coco/yolo11n-seg.engine"
PERSON_CLASS = 0                      # COCO 사람


def exposure_request(cmd: ExposureCmd) -> SetParameters.Request:
    """노출 고정(`rgb_camera.exposure`=166) 또는 자동 노출 복구 요청."""
    if cmd is ExposureCmd.FIX:
        value = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                               integer_value=LIGHT_EXPOSURE)
        name = "rgb_camera.exposure"
    else:
        value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
        name = "rgb_camera.enable_auto_exposure"
    return SetParameters.Request(parameters=[Parameter(name=name, value=value)])


def _all_successful(response) -> bool:
    results = response.results
    return bool(results) and all(r.successful for r in results)


def restore_auto_exposure_blocking(timeout_s: float = 3.0) -> bool:
    """자동 노출을 켜고 응답까지 기다린다 — 종료 처리 전용.

    SIGINT·SIGTERM 을 받으면 rclpy 가 기본 컨텍스트를 먼저 닫아 노드의 클라이언트를
    못 쓴다. 그래서 새 컨텍스트로 한 번 부른다.
    """
    context = rclpy.Context()
    node = None
    try:
        rclpy.init(context=context)
        node = rclpy.create_node("person_detector_exposure_restore", context=context)
        client = node.create_client(SetParameters, CAMERA_PARAM_SERVICE)
        if not client.wait_for_service(timeout_sec=timeout_s / 2.0):
            return False
        future = client.call_async(exposure_request(ExposureCmd.AUTO))
        executor = SingleThreadedExecutor(context=context)
        rclpy.spin_until_future_complete(
            node, future, executor=executor, timeout_sec=timeout_s / 2.0)
        return future.done() and _all_successful(future.result())
    except Exception:                             # noqa: BLE001 — 종료 중이다. 못 하면 False
        return False
    finally:
        try:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown(context=context)
        except Exception:                         # noqa: BLE001
            pass


class PatrolWiring:
    """순찰 프로필 배선. 판정·순서는 `PatrolProfile` 이 한다."""

    def __init__(
        self,
        node,
        load_model: Callable[[str], object],
        rate_hz: float = 5.0,
        state_timeout_s: float = 3.0,
    ) -> None:
        self._node = node
        self._load_model = load_model
        self._engine_path = os.environ.get("VICA_PATROL_ENGINE", DEFAULT_PATROL_ENGINE)
        self._media_dir = os.environ.get("VICA_PATROL_MEDIA_DIR", DEFAULT_MEDIA_DIR)
        self._state_timeout_ns = sec_to_ns(state_timeout_s)
        self._state_ns: Optional[int] = None      # 마지막 /vica/robot_state
        self._stale_ticks = 0
        # 순찰 엔진. 한 번 올라오면 내리지 않는다. None 은 아직 없다는 뜻이다.
        self._patrol_model = None
        self._loading = False
        self._loaded: queue.SimpleQueue = queue.SimpleQueue()   # 엔진 스레드 → 실행기
        self._cleaning = threading.Lock()
        self._encode = None
        self._timer = None
        self._pub = node.create_publisher(String, EVENT_TOPIC, 10)
        self._cam = node.create_client(SetParameters, CAMERA_PARAM_SERVICE)
        if patrol_zone() is None:
            self._log("warn", "시간대 %s 를 읽지 못했다(VICA_PATROL_TZ) — "
                      "영상 이름·시각을 기기 시간대로 쓴다" % patrol_zone_name())
        self._profile = PatrolProfile(
            PatrolPorts(
                publish=self._publish,
                encode_jpeg=self._encode_jpeg,
                brightness=mean_brightness,
                infer=self._infer,
                request_exposure=self._request_exposure,
                exposure_ready=self._cam.service_is_ready,
                cleanup=self._cleanup,
                log=self._log,
            ),
            self._media_dir, rate_hz)

    # ---- 노드가 부르는 것 ----------------------------------------------------

    def on_dialog_state(self, state: str) -> None:
        """`/vica/robot_state` 한 건(순찰 상태이거나 프레임을 붙잡고 있는 동안만 온다)."""
        now_ns = time.monotonic_ns()
        self._state_ns = now_ns
        self._stale_ticks = 0
        self._apply_state(state, now_ns)

    def on_color(self, msg) -> None:
        """노드가 이미 받는 컬러 프레임. 기존 추론 경로 대신 여기로 온다."""
        now_ns = time.monotonic_ns()
        try:
            self._poll_engine()
            self._profile.on_frame(lambda: image_to_bgr(msg), msg.width, msg.height,
                                   now_ns, time.time())
            if not self._profile.wants_frames:
                self._sync()                      # 쓰던 장면이 끝났다 — 프레임을 돌려준다
        except Exception as e:                    # noqa: BLE001 — 순찰 처리로 노드가 죽으면 안 된다
            self._node.get_logger().error(
                "순찰 프레임 처리 실패: %s" % e, throttle_duration_sec=5.0)
            try:
                self._profile.on_error(now_ns)    # 노출이 고정된 채 남지 않게
            except Exception:                     # noqa: BLE001 — 1 Hz 점검이 다시 되돌린다
                pass

    def shutdown(self) -> None:
        """노드 종료 — 쓰던 장면을 바로 닫고, 노출이 고정돼 있으면 자동 노출로 되돌린다."""
        try:
            self._profile.shutdown(time.monotonic_ns(), time.time())
        except Exception:                         # noqa: BLE001 — 컨텍스트가 이미 닫혔을 수 있다
            pass
        if self._profile.exposure_dirty:
            ok = restore_auto_exposure_blocking()
            print("person_detector_node: 종료 중 자동 노출 복구 %s"
                  % ("완료" if ok else "실패 — rgb_camera.enable_auto_exposure 를 확인할 것"),
                  file=sys.stderr)

    # ---- 1 Hz 점검 -----------------------------------------------------------

    def _on_tick(self) -> None:
        now_ns = time.monotonic_ns()
        try:
            self._poll_engine()
            if (self._profile.active and self._state_ns is not None
                    and now_ns - self._state_ns > self._state_timeout_ns):
                # 엔진을 올리느라 실행기가 밀렸을 수 있다 — 점검 두 번 잇따라 볼 때만.
                self._stale_ticks += 1
                if self._stale_ticks >= 2:
                    self._log("warn", "/vica/robot_state 가 끊겼다 — 순찰 프로필에서 나간다")
                    self._apply_state("", now_ns)
            self._profile.on_tick(now_ns, time.time())
        except Exception as e:                    # noqa: BLE001
            self._node.get_logger().error(
                "순찰 주기 점검 실패: %s" % e, throttle_duration_sec=5.0)
        self._sync()

    def _apply_state(self, state: str, now_ns: int) -> None:
        was_active = self._profile.active
        self._profile.on_dialog_state(state, now_ns, time.time())
        if self._profile.active != was_active:
            self._log("info", "순찰 프로필 %s" % (
                "진입(%s)" % state if self._profile.active else "이탈"))
            if self._profile.active:
                self._poll_engine()           # 지난 순찰 때 시작한 올리기의 결과부터 받는다
                self._ensure_engine()
        self._sync()

    def _sync(self) -> None:
        """엔진·프로필 상태를 노드에 알리고, 할 일이 없으면 타이머를 없앤다."""
        self._profile.set_engine_loaded(self._patrol_model is not None)
        busy = self._profile.wants_frames     # 순찰이 끝나면(쓰던 장면까지 끝나면) 기존 경로로
        self._node._patrol_busy = busy
        need_timer = busy or self._profile.exposure_dirty
        if need_timer and self._timer is None:
            self._timer = self._node.create_timer(STATUS_PERIOD_S, self._on_tick)
        elif not need_timer and self._timer is not None:
            self._timer.cancel()
            self._node.destroy_timer(self._timer)
            self._timer = None

    # ---- 순찰 엔진 -----------------------------------------------------------

    def _ensure_engine(self) -> None:
        """순찰 엔진이 없으면 올린다 — 첫 순찰 때 한 번, 실패했으면 다음 순찰 때 다시."""
        if self._patrol_model is not None or self._loading:
            return
        self._loading = True
        path = self._engine_path
        self._log("info", "순찰 엔진 올리기 시작: %s" % path)

        def work() -> None:
            # 콜백·실행기를 막지 않게 따로 올린다. 결과만 큐로 넘긴다.
            began = time.monotonic()
            try:
                if not os.path.isfile(path):
                    raise FileNotFoundError("엔진 파일이 없다")
                model = self._load_model(path)
                self._loaded.put((model, "%.1f s" % (time.monotonic() - began)))
            except Exception as e:                # noqa: BLE001 — 실패해도 노드는 산다
                self._loaded.put((None, "%s: %s" % (type(e).__name__, e)))

        threading.Thread(target=work, name="vica_patrol_engine", daemon=True).start()

    def _poll_engine(self) -> None:
        """다 올라온 순찰 엔진을 받는다(실행기 쪽에서만 부른다)."""
        if self._loaded.empty():
            return
        try:
            model, note = self._loaded.get_nowait()
        except queue.Empty:
            return
        self._loading = False
        if model is None:
            self._log("error", "순찰 엔진 올리기 실패 — ready=false, 다음 순찰 때 다시 올린다: %s"
                      % note)
        else:
            self._patrol_model = model            # 순찰이 끝나도 놓지 않는다
            self._log("info", "순찰 엔진 준비 (올리는 데 %s)" % note)
        self._sync()

    # ---- PatrolPorts ---------------------------------------------------------

    def _publish(self, event: dict) -> None:
        self._pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

    def _encode_jpeg(self, img) -> bytes:
        if self._encode is None:
            self._encode = make_jpeg_encoder(JPEG_QUALITY)
        return self._encode(img)

    def _infer(self, img):
        """추적 없이 사람만. 문턱 아래 상자는 모델 뒤처리에서 버린다."""
        result = self._patrol_model.predict(
            img, device=0, conf=PERSON_MIN_CONF, classes=[PERSON_CLASS],
            verbose=False)[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return ()
        return boxes.conf.tolist()

    def _request_exposure(self, cmd: ExposureCmd) -> bool:
        if not self._cam.service_is_ready():
            return False
        future = self._cam.call_async(exposure_request(cmd))
        future.add_done_callback(lambda fut: self._on_exposure_done(cmd, fut))
        return True

    def _on_exposure_done(self, cmd: ExposureCmd, future) -> None:
        try:
            ok = _all_successful(future.result())
        except Exception:                         # noqa: BLE001
            ok = False
        self._profile.on_exposure_result(cmd, ok, time.monotonic_ns())
        self._sync()

    def _cleanup(self) -> None:
        """저장 폴더 정리(7일·1 GB). 파일이 많을 수 있어 따로 돌린다."""
        if not self._cleaning.acquire(blocking=False):
            return                                # 이미 도는 중

        def work() -> None:
            try:
                report = cleanup_media(self._media_dir, time.time())
                if report.removed:
                    self._log("info", "순찰 기록 정리: %d개 삭제, 남은 %.1f MB"
                              % (len(report.removed), report.kept_bytes / 1e6))
            except Exception as e:                # noqa: BLE001
                self._log("warn", "순찰 기록 정리 실패: %s" % e)
            finally:
                self._cleaning.release()

        threading.Thread(target=work, name="vica_patrol_cleanup", daemon=True).start()

    def _log(self, level: str, text: str) -> None:
        logger = self._node.get_logger()
        if level == "error":
            logger.error(text)
        elif level == "warn":
            logger.warning(text)
        else:
            logger.info(text)
