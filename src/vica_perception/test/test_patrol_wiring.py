"""순찰 프로필 배선 시험 — 가짜 노드로 돈다(GPU·카메라·ROS 통신 없음).

판정은 test_patrol_profile, 저장은 test_patrol_media 가 본다. 여기서는 배선만 본다:
순찰이 아닐 때 아무것도 만들지 않는지, 순찰 엔진을 한 번만 따로 올려 계속 두는지,
순찰이 끝나면 곧바로 기존 경로로 돌려주는지, 노출 요청이 명세대로 나가는지.

rclpy 노드를 만들고 지우는 시험은 destroy_node 에서 가끔 멈춰서(2026-11-10)
노드 없이 확인한다 — test_person_detector_params 와 같은 방식이다.
"""
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest
from rcl_interfaces.msg import ParameterType, SetParametersResult
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String

import vica_perception.patrol_wiring as wiring_mod
import vica_perception.person_detector_node as node_mod
from vica_perception.inference_gate import InferenceGate
from vica_perception.patrol_media import _pil_encoder
from vica_perception.patrol_profile import ExposureCmd
from vica_perception.patrol_wiring import (
    CAMERA_PARAM_SERVICE,
    DEFAULT_PATROL_ENGINE,
    EVENT_TOPIC,
    PatrolWiring,
    exposure_request,
)
from vica_perception.person_detector_node import PersonDetectorNode


# ---- 가짜 노드 ----------------------------------------------------------------


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, text, **_kw):
        self.lines.append(("info", text))

    def warning(self, text, **_kw):
        self.lines.append(("warn", text))

    def error(self, text, **_kw):
        self.lines.append(("error", text))


class FakeTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.canceled = False
        self.destroyed = False

    def cancel(self):
        self.canceled = True


class FakeFuture:
    def __init__(self, request):
        self.request = request
        self._callbacks = []
        self._response = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        if isinstance(self._response, Exception):
            raise self._response
        return self._response

    def complete(self, successful=True):
        self._response = SetParameters.Response(
            results=[SetParametersResult(successful=successful)])
        for callback in self._callbacks:
            callback(self)


class FakeClient:
    def __init__(self):
        self.ready = True
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = FakeFuture(request)
        self.futures.append(future)
        return future


class FakeNode:
    """배선이 쓰는 노드 기능만 흉내 낸다."""

    def __init__(self):
        object.__setattr__(self, "_model", "CANE-0")      # 기동 때 올린 지팡이 엔진
        self._patrol_busy = False
        self.logger = FakeLogger()
        self.published = []
        self.publisher_args = None
        self.client_args = None
        self.client = FakeClient()
        self.timers = []

    def __setattr__(self, name, value):
        assert name != "_model", "배선이 지팡이 엔진을 건드렸다"
        object.__setattr__(self, name, value)

    def create_publisher(self, msg_type, topic, qos):
        self.publisher_args = (msg_type, topic, qos)
        return SimpleNamespace(publish=lambda msg: self.published.append(msg))

    def create_client(self, srv_type, name):
        self.client_args = (srv_type, name)
        return self.client

    def create_timer(self, period, callback):
        timer = FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        timer.destroyed = True

    def get_logger(self):
        return self.logger

    # -- 시험이 쓰는 것 --

    def events(self, kind=None):
        out = [json.loads(m.data) for m in self.published]
        return [e for e in out if kind is None or e["kind"] == kind]

    def live_timers(self):
        return [t for t in self.timers if not t.destroyed]


class Loader:
    """가짜 엔진 로더 — 올린 경로를 적고 표식을 돌려준다."""

    def __init__(self):
        self.paths = []
        self.fail = {}                    # 경로 → 남은 실패 횟수

    def __call__(self, path):
        self.paths.append(path)
        if self.fail.get(path, 0) > 0:
            self.fail[path] -= 1
            raise RuntimeError("엔진 올리기 실패(시험)")
        return "MODEL:%s:%d" % (path, len(self.paths))


def color_msg(width=8, height=6):
    data = np.full((height, width, 3), 40, np.uint8).tobytes()
    return SimpleNamespace(data=data, width=width, height=height, encoding="bgr8")


@pytest.fixture
def rig(tmp_path, monkeypatch):
    engine = tmp_path / "patrol.engine"
    engine.write_bytes(b"engine")
    monkeypatch.setenv("VICA_PATROL_ENGINE", str(engine))
    monkeypatch.setenv("VICA_PATROL_MEDIA_DIR", str(tmp_path / "media"))
    # 호스트의 cv2 는 import 에서 죽으며 긴 경고를 찍는다 — 시험은 PIL 로 고정한다.
    monkeypatch.setattr(wiring_mod, "make_jpeg_encoder", _pil_encoder)
    node = FakeNode()
    loader = Loader()
    wiring = PatrolWiring(node, loader, rate_hz=5.0, state_timeout_s=3.0)
    return SimpleNamespace(node=node, loader=loader, wiring=wiring,
                           engine=str(engine), media=tmp_path / "media")


def wait_loaded(wiring, timeout_s=5.0):
    """엔진 스레드가 결과를 넘길 때까지 기다린다."""
    deadline = time.monotonic() + timeout_s
    while wiring._loaded.empty():
        assert time.monotonic() < deadline, "엔진 스레드가 끝나지 않았다"
        time.sleep(0.005)


def settle(wiring):
    """엔진 스레드가 끝나길 기다렸다가 실행기 쪽 처리(영상 콜백)를 한 번 돌린다."""
    wait_loaded(wiring)
    wiring.on_color(color_msg())


def load_starts(node):
    """순찰 엔진 올리기를 시작한 횟수(로그로 센다)."""
    return sum("순찰 엔진 올리기 시작" in text for _, text in node.logger.lines)


# ---- 계약 이름·요청 -----------------------------------------------------------


def test_토픽_서비스_기본_엔진_이름():
    assert EVENT_TOPIC == "/vica/patrol/event"
    assert CAMERA_PARAM_SERVICE == "/camera/camera/set_parameters"
    assert DEFAULT_PATROL_ENGINE == (
        "/workspaces/isaac_ros-dev/models/patrol-coco/yolo11n-seg.engine")


def test_노출_고정_요청은_exposure_정수_166():
    request = exposure_request(ExposureCmd.FIX)
    assert len(request.parameters) == 1
    param = request.parameters[0]
    assert param.name == "rgb_camera.exposure"
    assert param.value.type == ParameterType.PARAMETER_INTEGER
    assert param.value.integer_value == 166


def test_자동_노출_복구_요청은_enable_auto_exposure_true():
    request = exposure_request(ExposureCmd.AUTO)
    assert len(request.parameters) == 1
    param = request.parameters[0]
    assert param.name == "rgb_camera.enable_auto_exposure"
    assert param.value.type == ParameterType.PARAMETER_BOOL
    assert param.value.bool_value is True


def test_배선은_이벤트_발행자와_카메라_클라이언트만_만든다(rig):
    assert rig.node.publisher_args == (String, "/vica/patrol/event", 10)
    assert rig.node.client_args == (SetParameters, "/camera/camera/set_parameters")
    assert rig.node.timers == []              # 순찰 상태를 보기 전에는 타이머가 없다
    assert rig.node._patrol_busy is False
    assert rig.loader.paths == []


def test_환경변수가_없으면_명세의_기본_경로를_쓴다(monkeypatch):
    monkeypatch.delenv("VICA_PATROL_ENGINE", raising=False)
    monkeypatch.delenv("VICA_PATROL_MEDIA_DIR", raising=False)
    wiring = PatrolWiring(FakeNode(), Loader())
    assert wiring._engine_path == DEFAULT_PATROL_ENGINE
    assert wiring._media_dir == "/workspaces/isaac_ros-dev/vica_patrol"


def test_시간대를_못_읽으면_경고를_한_번_남긴다(rig, monkeypatch):
    monkeypatch.setenv("VICA_PATROL_TZ", "No/Such_Zone")
    wiring = PatrolWiring(rig.node, rig.loader)

    def warnings():
        return [text for level, text in rig.node.logger.lines
                if level == "warn" and "VICA_PATROL_TZ" in text]

    assert len(warnings()) == 1 and "No/Such_Zone" in warnings()[0]
    wiring.on_dialog_state("patrolling")
    settle(wiring)
    wiring.on_dialog_state("patrol_alarm")    # 장면을 써도(이름을 만들어도) 다시 경고하지 않는다
    wiring.shutdown()
    assert len(warnings()) == 1


def test_시간대를_읽으면_경고가_없다(rig):
    assert not any("VICA_PATROL_TZ" in text for _, text in rig.node.logger.lines)


# ---- 순찰 엔진: 첫 순찰 때 한 번 올려 계속 둔다 ---------------------------------
#
# 2026-11-14 컨테이너 실측: 엔진을 내렸다 올리면 회마다 RSS 가 30~55 MB 늘고 돌아오지
# 않았다. 그래서 지팡이 엔진은 건드리지 않고(FakeNode 가 지킨다) 순찰 엔진도 내리지 않는다.


def test_첫_순찰_때_순찰_엔진을_올리고_지팡이_엔진은_건드리지_않는다(rig):
    rig.wiring.on_dialog_state("patrol_dwell")
    assert rig.node._patrol_busy is True
    assert len(rig.node.live_timers()) == 1
    assert rig.node.live_timers()[0].period == 1.0

    settle(rig.wiring)
    assert rig.loader.paths == [rig.engine]
    assert rig.wiring._patrol_model == "MODEL:%s:1" % rig.engine
    assert rig.node._model == "CANE-0"

    rig.node.live_timers()[0].callback()
    status = rig.node.events("status")[-1]
    assert status["src"] == "detector"
    assert status["ready"] is True and status["camera_ok"] is True


def test_엔진이_올라오기_전에는_ready_false_이고_콜백은_막히지_않는다(rig):
    gate = {"open": False}

    def slow_loader(path):
        while not gate["open"]:
            time.sleep(0.005)
        return "MODEL"

    rig.wiring._load_model = slow_loader
    started = time.monotonic()
    rig.wiring.on_dialog_state("patrolling")
    for _ in range(10):
        rig.wiring.on_color(color_msg())      # 올리는 동안에도 프레임을 받는다
    rig.node.live_timers()[0].callback()
    assert time.monotonic() - started < 1.0
    assert rig.node.events("status")[-1]["ready"] is False
    gate["open"] = True
    settle(rig.wiring)
    rig.node.live_timers()[0].callback()
    assert rig.node.events("status")[-1]["ready"] is True


def test_순찰이_끝나면_곧바로_기존_경로로_돌려주고_순찰_엔진은_쥐고_있는다(rig):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    model = rig.wiring._patrol_model
    rig.wiring.on_dialog_state("idle")
    assert rig.node._patrol_busy is False     # 기다림 없이 기존 경로로
    assert rig.wiring._patrol_model is model  # 내리지 않는다
    assert rig.loader.paths == [rig.engine]   # 지팡이 엔진을 다시 올리지도 않는다
    assert rig.node.live_timers() == []       # 타이머도 없앤다
    assert all(t.canceled for t in rig.node.timers)
    assert rig.wiring._profile._clip.buffered == 0


def test_두_번째_순찰부터는_올리기_없이_바로_ready(rig):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    model = rig.wiring._patrol_model
    rig.wiring.on_dialog_state("idle")
    for _ in range(3):                        # 순찰 시작·종료를 되풀이해도
        rig.wiring.on_dialog_state("patrolling")
        assert rig.wiring._profile.engine_ready           # 들어오자마자 준비돼 있다
        assert len(rig.node.live_timers()) == 1           # 타이머는 다시 만든다
        rig.wiring.on_color(color_msg())
        rig.node.live_timers()[0].callback()
        assert rig.node.events("status")[-1]["ready"] is True
        rig.wiring.on_dialog_state("idle")
        assert rig.node._patrol_busy is False
    assert rig.loader.paths == [rig.engine]   # 올리기는 처음 한 번뿐
    assert load_starts(rig.node) == 1
    assert rig.wiring._patrol_model is model
    assert rig.wiring._loaded.empty()


def test_순찰_엔진_파일이_없으면_ready_false_이고_다음_순찰_때_다시_올린다(rig, monkeypatch):
    missing = rig.media.parent / "later.engine"
    monkeypatch.setenv("VICA_PATROL_ENGINE", str(missing))
    wiring = PatrolWiring(rig.node, rig.loader)
    wiring.on_dialog_state("patrol_dwell")
    settle(wiring)

    assert rig.loader.paths == []             # 로더를 부르기 전에 걸렀다
    assert any(level == "error" and "순찰 엔진 올리기 실패" in text
               for level, text in rig.node.logger.lines)
    timer = rig.node.live_timers()[0]
    timer.callback()
    assert rig.node.events("status")[-1]["ready"] is False
    timer.callback()
    assert load_starts(rig.node) == 1         # 같은 순찰에서는 다시 올리지 않는다

    wiring.on_dialog_state("idle")            # 미션이 detector_not_ready 로 끝냈다
    assert rig.node._patrol_busy is False     # 곧바로 기존 경로
    for future in list(rig.node.client.futures):
        future.complete(True)                 # 준비 대기 dwell 의 불 확인 뒤처리
    assert rig.node.live_timers() == []

    missing.write_bytes(b"engine")            # 그 뒤 엔진 파일이 생겼다
    wiring.on_dialog_state("patrolling")      # 다음 순찰
    settle(wiring)
    assert rig.loader.paths == [str(missing)]
    assert wiring._profile.engine_ready


def test_순찰_엔진_로더가_죽어도_노드는_살고_다음_순찰_때_다시_올린다(rig):
    rig.loader.fail[rig.engine] = 1
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    assert rig.wiring._patrol_model is None
    assert not rig.wiring._profile.engine_ready
    rig.wiring.on_dialog_state("idle")
    assert rig.node._patrol_busy is False

    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    assert rig.loader.paths == [rig.engine, rig.engine]
    assert rig.wiring._patrol_model == "MODEL:%s:2" % rig.engine
    assert rig.wiring._profile.engine_ready


def test_올리는_중에_순찰이_끝나도_곧바로_기존_경로이고_엔진은_다음_순찰이_쓴다(rig):
    gate = {"open": False}

    def slow_loader(path):
        while not gate["open"]:
            time.sleep(0.005)
        return "LATE"

    rig.wiring._load_model = slow_loader
    rig.wiring.on_dialog_state("patrolling")
    rig.wiring.on_dialog_state("idle")        # 엔진이 올라오기 전에 순찰이 끝났다
    assert rig.node._patrol_busy is False     # 올리기를 기다리지 않는다
    assert rig.node.live_timers() == []

    rig.wiring.on_dialog_state("patrolling")  # 아직 올리는 중에 다시 시작
    assert load_starts(rig.node) == 1         # 겹쳐 올리지 않는다
    assert not rig.wiring._profile.engine_ready
    rig.wiring.on_dialog_state("idle")

    gate["open"] = True                       # 순찰 밖에서 올리기가 끝난다
    wait_loaded(rig.wiring)
    rig.wiring.on_dialog_state("patrolling")  # 다음 순찰
    assert rig.wiring._patrol_model == "LATE"         # 들어오자마자 받아 쓴다
    assert rig.wiring._profile.engine_ready
    assert load_starts(rig.node) == 1


def test_순찰_밖에서_끝난_올리기가_실패였으면_다음_순찰_진입_때_다시_올린다(rig):
    rig.loader.fail[rig.engine] = 1
    rig.wiring.on_dialog_state("patrolling")
    rig.wiring.on_dialog_state("idle")        # 결과를 받기 전에 순찰이 끝났다
    wait_loaded(rig.wiring)
    rig.wiring.on_dialog_state("patrolling")  # 실패를 확인하고 곧바로 다시 올린다
    assert load_starts(rig.node) == 2
    settle(rig.wiring)
    assert rig.loader.paths == [rig.engine, rig.engine]
    assert rig.wiring._profile.engine_ready


# ---- 감시 항목 켜고 끄기(2026-11-14 사용자 요청) -------------------------------


def test_사람_감지를_꺼_둔_순찰에는_엔진을_올리지_않고_ready_다(rig):
    rig.wiring.set_watch_off(frozenset({"intruder"}))
    rig.wiring.on_dialog_state("patrol_dwell")
    rig.wiring.on_color(color_msg())
    rig.node.live_timers()[0].callback()
    assert load_starts(rig.node) == 0 and rig.loader.paths == []
    assert rig.wiring._patrol_model is None
    status = rig.node.events("status")[-1]
    assert status["ready"] is True and status["camera_ok"] is True


def test_다음_순찰에서_사람_감지를_켜면_그때_엔진을_올린다(rig):
    rig.wiring.set_watch_off(frozenset({"intruder"}))
    rig.wiring.on_dialog_state("patrolling")
    rig.wiring.on_dialog_state("idle")
    rig.wiring.set_watch_off(frozenset())     # 미션이 다음 경비 시작 전에 알린다
    assert load_starts(rig.node) == 0         # 순찰 밖에서는 올리지 않는다
    rig.wiring.on_dialog_state("patrol_dwell")
    assert load_starts(rig.node) == 1
    settle(rig.wiring)
    assert rig.wiring._profile.engine_ready


def test_순찰_중에_사람_감지가_켜졌다고_오면_그때_엔진을_올린다(rig):
    rig.wiring.set_watch_off(frozenset({"intruder"}))
    rig.wiring.on_dialog_state("patrolling")
    assert load_starts(rig.node) == 0
    rig.wiring.set_watch_off(frozenset())     # 알림이 상태보다 늦게 처리된 경우
    assert load_starts(rig.node) == 1
    rig.node.live_timers()[0].callback()
    assert rig.node.events("status")[-1]["ready"] is False    # 올라올 때까지는 준비 전
    settle(rig.wiring)
    rig.node.live_timers()[0].callback()
    assert rig.node.events("status")[-1]["ready"] is True


def test_감시_항목_알림만으로는_타이머도_프레임_가로채기도_생기지_않는다(rig):
    rig.wiring.set_watch_off(frozenset({"light", "sound"}))
    assert rig.node._patrol_busy is False
    assert rig.node.live_timers() == []
    assert rig.node.published == []


# ---- 추론·프레임 --------------------------------------------------------------


class FakeBoxes:
    def __init__(self, confs):
        self.conf = np.array(confs, dtype=np.float32)

    def __len__(self):
        return len(self.conf)


class FakeModel:
    def __init__(self, confs):
        self.confs = confs
        self.calls = []

    def predict(self, img, **kwargs):
        self.calls.append(kwargs)
        return [SimpleNamespace(boxes=FakeBoxes(self.confs))]

    def track(self, *_a, **_kw):              # 순찰 프로필은 추적을 쓰지 않는다
        raise AssertionError("track 을 불렀다")


def test_순찰_추론은_추적_없이_사람만_conf_0_5_로_부른다(rig):
    model = FakeModel([0.91, 0.55])
    rig.wiring._patrol_model = model
    confs = rig.wiring._infer(np.zeros((6, 8, 3), np.uint8))
    assert [round(c, 2) for c in confs] == [0.91, 0.55]
    assert model.calls == [dict(device=0, conf=0.5, classes=[0], verbose=False)]


def test_상자가_없으면_빈_목록이다(rig):
    rig.wiring._patrol_model = FakeModel([])
    assert list(rig.wiring._infer(np.zeros((6, 8, 3), np.uint8))) == []
    rig.wiring._patrol_model = SimpleNamespace(
        predict=lambda img, **kw: [SimpleNamespace(boxes=None)])
    assert list(rig.wiring._infer(np.zeros((6, 8, 3), np.uint8))) == []


def test_순찰_중_사람이_두_번_보이면_intruder_를_발행한다(rig):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring._patrol_model = FakeModel([0.8])
    for _ in range(3):
        time.sleep(0.2)                       # 5 Hz 솎기
        rig.wiring.on_color(color_msg())
    intruder = rig.node.events("intruder")
    assert len(intruder) == 1
    assert intruder[0]["hits"] == 2 and intruder[0]["conf"] == 0.8
    clip = rig.node.events("clip")[0]
    assert clip["state"] == "started" and clip["clip_id"] == intruder[0]["clip_id"]
    frames = rig.media / "clips" / clip["clip_id"] / "frames"
    assert (frames / "000001.jpg").read_bytes()[:2] == b"\xff\xd8"
    # /vica/person_detection 은 내지 않는다 — 발행자는 이벤트 하나뿐이다.
    assert all(isinstance(m, String) for m in rig.node.published)


def test_프레임_처리에서_예외가_나도_노드는_죽지_않는다(rig, monkeypatch):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)

    def boom(*_a, **_kw):
        raise ValueError("깨진 프레임")

    monkeypatch.setattr(rig.wiring._profile, "on_frame", boom)
    rig.wiring.on_color(color_msg())          # 예외가 밖으로 나오지 않는다
    assert any(level == "error" and "순찰 프레임 처리 실패" in text
               for level, text in rig.node.logger.lines)


def test_robot_state_가_끊기면_순찰_프로필에서_나간다(rig):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    timer = rig.node.live_timers()[0]
    timer.callback()
    assert rig.wiring._profile.active
    rig.wiring._state_ns -= 3_100_000_000     # 3.1 s 동안 상태가 없었다
    timer.callback()
    assert rig.wiring._profile.active         # 한 번은 봐준다(실행기가 밀렸을 수 있다)
    timer.callback()
    assert not rig.wiring._profile.active
    assert any("끊겼다" in text for _, text in rig.node.logger.lines)
    assert rig.node._patrol_busy is False     # 곧바로 기존 경로
    assert rig.node.live_timers() == []


def test_밀렸던_robot_state_가_들어오면_순찰_프로필을_유지한다(rig):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    timer = rig.node.live_timers()[0]
    rig.wiring._state_ns -= 3_100_000_000
    timer.callback()                          # 끊긴 것처럼 보인다
    rig.wiring.on_dialog_state("patrolling")  # 밀렸던 상태가 처리됐다
    rig.wiring._state_ns -= 3_100_000_000
    timer.callback()                          # 다시 처음부터 센다
    assert rig.wiring._profile.active


# ---- 순찰이 끝나도 쓰던 장면은 끝까지 -----------------------------------------


def test_순찰이_끝나도_쓰던_장면이_끝날_때까지는_프레임을_붙잡는다(rig):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring.on_dialog_state("patrol_alarm")            # 소리 경보 — 장면 시작
    clip_id = rig.node.events("clip")[0]["clip_id"]
    rig.wiring.on_dialog_state("idle")                    # 경비 종료
    assert rig.node._patrol_busy is True                  # 장면을 마저 쓴다
    assert len(rig.node.live_timers()) == 1
    status = len(rig.node.events("status"))
    rig.node.live_timers()[0].callback()
    assert len(rig.node.events("status")) == status       # 순찰이 아니다 — status 없음
    rig.wiring.on_dialog_state("idle")                    # 그동안에도 상태는 온다
    rig.wiring.on_color(color_msg())
    assert rig.node._patrol_busy is True

    rig.wiring._profile._clip._clip.hold_until_ns -= 10_000_000_000   # 10 s 가 지났다
    time.sleep(0.21)                                      # 다음 처리 프레임(5 Hz)
    rig.wiring.on_color(color_msg())                      # 끝 조건 — 닫고 곧바로 놓는다
    assert rig.node._patrol_busy is False
    assert rig.node.live_timers() == []
    assert [e["state"] for e in rig.node.events("clip")] == ["started", "done"]
    assert (rig.media / "clips" / clip_id / "meta.json").is_file()


def test_robot_state_가_끊겨도_쓰던_장면은_끝까지_쓴다(rig):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring.on_dialog_state("patrol_alarm")
    timer = rig.node.live_timers()[0]
    rig.wiring._state_ns -= 3_100_000_000
    timer.callback()
    timer.callback()                                      # 두 번 잇따라 — 순찰에서 나간다
    assert not rig.wiring._profile.active
    assert rig.wiring._profile.draining
    assert rig.node._patrol_busy is True
    assert [e["state"] for e in rig.node.events("clip")] == ["started"]

    rig.wiring._profile._clip._clip.hold_until_ns -= 10_000_000_000
    timer.callback()                                      # 프레임이 없어도 1 Hz 점검이 닫는다
    assert rig.node._patrol_busy is False
    assert rig.node.live_timers() == []
    assert [e["state"] for e in rig.node.events("clip")] == ["started", "done"]


def test_마저_쓰는_중의_종료는_기다리지_않고_바로_닫는다(rig, monkeypatch):
    monkeypatch.setattr(wiring_mod, "restore_auto_exposure_blocking", lambda: True)
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring.on_dialog_state("patrol_alarm")
    rig.wiring.on_dialog_state("idle")
    assert rig.wiring._profile.draining
    rig.wiring.shutdown()
    assert not rig.wiring._profile.wants_frames
    assert [e["state"] for e in rig.node.events("clip")] == ["started", "done"]


# ---- 노출 요청 ---------------------------------------------------------------


def run_frames(wiring, count):
    for _ in range(count):
        wiring.on_color(color_msg())


def test_dwell_에서_노출_고정_측정_자동_복구_순서로_서비스를_부른다(rig):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring.on_dialog_state("patrol_dwell")
    run_frames(rig.wiring, 1)
    futures = rig.node.client.futures
    assert [f.request.parameters[0].name for f in futures] == ["rgb_camera.exposure"]

    futures[0].complete(True)
    run_frames(rig.wiring, 5)                 # 5장 버림
    assert len(futures) == 1
    run_frames(rig.wiring, 1)                 # 밝기를 재고 곧바로 되돌린다
    assert [f.request.parameters[0].name for f in futures] == [
        "rgb_camera.exposure", "rgb_camera.enable_auto_exposure"]
    assert rig.wiring._profile.exposure_dirty
    futures[1].complete(True)
    assert not rig.wiring._profile.exposure_dirty


def test_노출_복구가_확인될_때까지_순찰이_끝나도_타이머가_남는다(rig):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring.on_dialog_state("patrol_dwell")
    run_frames(rig.wiring, 1)
    rig.node.client.futures[0].complete(True)             # 노출이 고정됐다
    rig.wiring.on_dialog_state("idle")                    # 그때 순찰이 끝났다
    names = [f.request.parameters[0].name for f in rig.node.client.futures]
    assert names[-1] == "rgb_camera.enable_auto_exposure"
    assert rig.node._patrol_busy is False                 # 프레임은 곧바로 돌려준다
    assert len(rig.node.live_timers()) == 1               # 복구 응답을 아직 못 받았다

    rig.node.client.futures[-1].complete(True)
    assert not rig.wiring._profile.exposure_dirty
    assert rig.node.live_timers() == []


def test_노출_요청이_거절되면_실패로_전한다(rig):
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring.on_dialog_state("patrol_dwell")
    run_frames(rig.wiring, 1)
    rig.node.client.futures[0].complete(False)            # 고정 거절
    names = [f.request.parameters[0].name for f in rig.node.client.futures]
    assert names == ["rgb_camera.exposure", "rgb_camera.enable_auto_exposure"]


# ---- 종료 처리 ---------------------------------------------------------------


def test_종료_때_노출이_고정돼_있으면_자동_노출로_되돌린다(rig, monkeypatch):
    calls = []
    monkeypatch.setattr(wiring_mod, "restore_auto_exposure_blocking",
                        lambda: calls.append(1) or True)
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring.on_dialog_state("patrol_dwell")
    run_frames(rig.wiring, 1)
    rig.node.client.futures[0].complete(True)             # 노출 고정 중에 종료 신호
    rig.wiring.shutdown()
    assert calls == [1]
    assert not rig.wiring._profile.active


def test_종료_때_클라이언트가_죽어_있어도_되돌리기를_시도한다(rig, monkeypatch):
    calls = []
    monkeypatch.setattr(wiring_mod, "restore_auto_exposure_blocking",
                        lambda: calls.append(1) or False)
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring.on_dialog_state("patrol_dwell")
    run_frames(rig.wiring, 1)

    def dead():
        raise RuntimeError("context is not valid")

    rig.node.client.service_is_ready = dead               # SIGINT 로 컨텍스트가 닫혔다
    rig.wiring.shutdown()                                 # 예외가 밖으로 나오지 않는다
    assert calls == [1]


def test_종료_때_노출을_건드린_적이_없으면_아무것도_부르지_않는다(rig, monkeypatch):
    calls = []
    monkeypatch.setattr(wiring_mod, "restore_auto_exposure_blocking",
                        lambda: calls.append(1) or True)
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring.shutdown()
    assert calls == []
    assert rig.node.client.futures == []


def test_종료_때_쓰던_장면을_닫는다(rig, monkeypatch):
    monkeypatch.setattr(wiring_mod, "restore_auto_exposure_blocking", lambda: True)
    rig.wiring.on_dialog_state("patrolling")
    settle(rig.wiring)
    rig.wiring.on_dialog_state("patrol_alarm")            # 소리 경보 장면
    clip_id = rig.node.events("clip")[0]["clip_id"]
    rig.wiring.shutdown()
    meta = json.loads((rig.media / "clips" / clip_id / "meta.json").read_text())
    assert meta["id"] == clip_id and meta["alarm"] == "sound"


# ---- 노드 쪽 배선: 순찰이 아닐 때는 기존 그대로 --------------------------------


class SpyWiring:
    made = []

    def __init__(self, node, load_model, rate_hz, state_timeout_s):
        self.args = (load_model, rate_hz, state_timeout_s)
        self.states = []
        self.frames = []
        self.watch = []
        self.calls = []                       # 불린 순서
        SpyWiring.made.append(self)

    def set_watch_off(self, off):
        self.watch.append(off)
        self.calls.append(("watch", off))

    def on_dialog_state(self, state):
        self.states.append(state)
        self.calls.append(("state", state))

    def on_color(self, msg):
        self.frames.append(msg)


def fake_self(**over):
    """PersonDetectorNode 의 메서드를 노드 없이 부르기 위한 대역."""
    base = dict(
        _infer_gate=InferenceGate(),
        _patrol=None, _patrol_busy=False, _patrol_ok=True,
        _patrol_watch_off=frozenset(),
        _load_model="LOADER", _period_s=0.2,
        get_parameter=lambda name: SimpleNamespace(value=3.0),
    )
    base.update(over)
    return SimpleNamespace(**base)


def state_msg(dialog_state, is_moving=False, is_paused=False):
    return SimpleNamespace(dialog_state=dialog_state, is_moving=is_moving,
                           is_paused=is_paused)


@pytest.fixture
def spy(monkeypatch):
    SpyWiring.made = []
    monkeypatch.setattr(node_mod, "PatrolWiring", SpyWiring)
    return SpyWiring


@pytest.mark.parametrize("state", [
    "", "idle", "navigating", "paused", "waiting", "seeking", "estopped"])
def test_순찰_상태가_아니면_배선을_만들지_않는다(spy, state):
    me = fake_self()
    for _ in range(5):
        PersonDetectorNode._on_robot_state(me, state_msg(state, is_moving=True))
    assert spy.made == []
    assert me._patrol is None and me._patrol_busy is False
    assert me._infer_gate.should_infer(time.monotonic_ns()) is False     # 기존 게이트는 그대로 돈다


def test_순찰_상태를_처음_보면_배선을_한_번_만들고_상태를_넘긴다(spy):
    me = fake_self()
    PersonDetectorNode._on_robot_state(me, state_msg("patrol_dwell"))
    PersonDetectorNode._on_robot_state(me, state_msg("patrolling", is_moving=True))
    assert len(spy.made) == 1
    assert spy.made[0].args == ("LOADER", 5.0, 3.0)
    assert spy.made[0].states == ["patrol_dwell", "patrolling"]


def test_먼저_받아_둔_감시_항목은_배선을_만들_때_상태보다_먼저_넘긴다(spy):
    logger = FakeLogger()
    me = fake_self(get_logger=lambda: logger)
    PersonDetectorNode._on_patrol_watch(me, SimpleNamespace(data='{"off": ["intruder"]}'))
    PersonDetectorNode._on_patrol_watch(me, SimpleNamespace(data='{"off": ["intruder"]}'))
    assert spy.made == [] and me._patrol_watch_off == {"intruder"}     # 알림만으로는 배선을 만들지 않는다
    assert logger.lines == [("info", "순찰 감시 항목 알림 — 끔: intruder")]   # 바뀔 때만 한 줄(실기 확인용)
    PersonDetectorNode._on_robot_state(me, state_msg("patrol_dwell"))
    assert spy.made[0].calls == [("watch", frozenset({"intruder"})), ("state", "patrol_dwell")]


def test_배선이_있으면_감시_항목_알림을_곧바로_넘기고_못_읽는_알림은_버린다(spy):
    me = fake_self()
    PersonDetectorNode._on_robot_state(me, state_msg("patrolling"))
    PersonDetectorNode._on_patrol_watch(me, SimpleNamespace(data='{"off": ["light", "sound"]}'))
    PersonDetectorNode._on_patrol_watch(me, SimpleNamespace(data="not json"))
    assert spy.made[0].watch == [frozenset(), frozenset({"light", "sound"})]
    assert me._patrol_watch_off == {"light", "sound"}


def test_순찰에서_나오는_첫_상태까지만_배선에_넘긴다(spy):
    me = fake_self()
    PersonDetectorNode._on_robot_state(me, state_msg("patrolling"))
    me._patrol_busy = True                    # 배선이 켠다
    PersonDetectorNode._on_robot_state(me, state_msg("idle"))
    assert spy.made[0].states == ["patrolling", "idle"]
    me._patrol_busy = False                   # 배선이 곧바로 끈다
    PersonDetectorNode._on_robot_state(me, state_msg("idle"))
    assert spy.made[0].states == ["patrolling", "idle"]


def test_옛_RobotState_에는_dialog_state_가_없어도_죽지_않는다(spy):
    me = fake_self(_patrol_ok=False)
    old_msg = SimpleNamespace(is_moving=False, is_paused=False)
    PersonDetectorNode._on_robot_state(me, old_msg)
    assert spy.made == []


def test_순찰_중_컬러_프레임은_배선으로만_간다(spy):
    patrol = SpyWiring(None, None, None, None)
    me = fake_self(_patrol=patrol, _patrol_busy=True, _infer_gate=None, _model=None)
    msg = color_msg()
    PersonDetectorNode._on_color(me, msg)     # 추론 게이트·지팡이 엔진을 건드리지 않는다
    assert patrol.frames == [msg]


def test_순찰이_아니면_컬러_프레임은_기존_경로를_탄다(spy):
    gate = InferenceGate()
    gate.observe_state(time.monotonic_ns(), is_moving=True, is_paused=False)
    logger = FakeLogger()
    me = fake_self(_infer_gate=gate, _last_infer_reason=None,
                   get_logger=lambda: logger)
    PersonDetectorNode._on_color(me, color_msg())
    assert logger.lines == [("info", "추론 게이트: moving")]      # 기존 첫 단계 그대로
    assert spy.made == []


def test_종료_처리는_순찰을_쓴_적이_없으면_아무것도_안_한다():
    PersonDetectorNode.patrol_shutdown(fake_self())       # 예외 없음
    calls = []
    PersonDetectorNode.patrol_shutdown(
        fake_self(_patrol=SimpleNamespace(shutdown=lambda: calls.append(1))))
    assert calls == [1]
