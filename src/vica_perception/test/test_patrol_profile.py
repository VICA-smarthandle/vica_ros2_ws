"""순찰 프로필 판정·처리 순서 시험 — ROS·GPU 없이 돈다.

정본은 `~/vica_data/patrol_plan/SPEC.md` 2·3.1·4.B. 값(0.5·3.0 s·2번·10 s·166·
밝기 10·5프레임)과 JSON 키는 미션·앱과의 계약이라 글자 그대로 고정한다.

시각은 전부 시험이 만들어 넣는다(단조 정수 나노초 + 벽시계 epoch 초).
"""
import json
import os
from datetime import datetime
from zoneinfo import ZoneInfo

import pytest

from vica_perception.inference_gate import sec_to_ns
from vica_perception.patrol_media import iso_local
from vica_perception.patrol_profile import (
    CAMERA_OK_S,
    INTRUDER_HITS,
    INTRUDER_WINDOW_S,
    LIGHT_DARK_BELOW,
    LIGHT_EXPOSURE,
    LIGHT_SKIP_FRAMES,
    PERSON_MIN_CONF,
    ExposureCmd,
    FrameUse,
    IntruderJudge,
    LightArming,
    LightCheck,
    PatrolPorts,
    PatrolProfile,
    event_clip,
    event_intruder,
    event_light,
    event_status,
    is_patrol_state,
    parse_watch_off,
)

WALL0 = datetime(2026, 11, 14, 20, 34, 55, tzinfo=ZoneInfo("Asia/Seoul")).timestamp()
FRAME_S = 1.0 / 15.0                  # 카메라 원본 15 Hz


def ns(seconds: float) -> int:
    return sec_to_ns(seconds)


# ---- 계약 값 ------------------------------------------------------------------


def test_값은_명세_그대로다():
    assert PERSON_MIN_CONF == 0.5
    assert (INTRUDER_WINDOW_S, INTRUDER_HITS) == (3.0, 2)
    assert LIGHT_EXPOSURE == 166
    assert LIGHT_DARK_BELOW == 10.0
    assert LIGHT_SKIP_FRAMES == 5
    assert CAMERA_OK_S == 3.0


@pytest.mark.parametrize("state", [
    "patrolling", "patrol_dwell", "patrol_alarm", "patrol_light", "patrol_paused"])
def test_순찰_상태_다섯_값은_모두_순찰_프로필이다(state):
    assert is_patrol_state(state) is True


@pytest.mark.parametrize("state", [
    "", "idle", "navigating", "paused", "waiting", "seeking", "estopped"])
def test_그_밖의_상태는_순찰_프로필이_아니다(state):
    assert is_patrol_state(state) is False


def test_이벤트_키는_계약_그대로다():
    assert event_status(1.5, True, False) == {
        "src": "detector", "kind": "status", "stamp": 1.5,
        "ready": True, "camera_ok": False}
    assert event_intruder(1.5, 0.87649, 2, "c1") == {
        "src": "detector", "kind": "intruder", "stamp": 1.5,
        "conf": 0.876, "hits": 2, "clip_id": "c1"}
    assert event_light(1.5, 35.04, "p.jpg") == {
        "src": "detector", "kind": "light", "stamp": 1.5,
        "brightness": 35.0, "photo": "p.jpg"}
    assert event_clip(1.5, "c1", "started", "sound") == {
        "src": "detector", "kind": "clip", "stamp": 1.5,
        "clip_id": "c1", "state": "started", "alarm": "sound"}
    assert event_clip(1.5, "c1", "done", "intruder", frames=75)["frames"] == 75


def test_이벤트는_JSON_으로_나간다():
    for ev in (event_status(1.5, True, True), event_intruder(1.5, 0.9, 2, "c"),
               event_light(1.5, 163.0, "p.jpg"), event_clip(1.5, "c", "done", "sound", 3)):
        assert json.loads(json.dumps(ev, ensure_ascii=False)) == ev


# ---- 사람 판정: 3초 안 2번 ----------------------------------------------------


def test_한_번_보인_것으로는_알리지_않는다():
    judge = IntruderJudge()
    assert judge.observe(ns(1.0), 0.9) is None


def test_3초_안에_두_번_보이면_한_번_알린다():
    judge = IntruderJudge()
    assert judge.observe(ns(1.0), 0.6) is None
    verdict = judge.observe(ns(3.4), 0.8)
    assert verdict is not None
    assert verdict.hits == 2
    assert verdict.conf == 0.8                # 창 안에서 가장 높은 값


def test_정확히_3초_간격은_인정하고_넘으면_버린다():
    judge = IntruderJudge()
    judge.observe(ns(1.0), 0.6)
    assert judge.observe(ns(4.0), 0.6) is not None

    judge = IntruderJudge()
    judge.observe(ns(1.0), 0.6)
    assert judge.observe(ns(4.001), 0.6) is None      # 앞의 것은 창 밖
    assert judge.observe(ns(5.0), 0.6) is not None    # 4.001 과 5.0


def test_conf_0_5_미만은_세지_않는다():
    judge = IntruderJudge()
    assert judge.observe(ns(1.0), 0.49) is None
    assert judge.observe(ns(1.2), 0.49) is None
    assert judge.observe(ns(1.4), 0.5) is None        # 첫 번째
    assert judge.observe(ns(1.6), 0.5) is not None    # 0.5 는 포함


def test_사이에_안_보인_프레임이_있어도_3초_안이면_된다():
    judge = IntruderJudge()
    judge.observe(ns(1.0), 0.7)
    for i in range(1, 10):
        assert judge.observe(ns(1.0 + 0.2 * i), 0.0) is None
    assert judge.observe(ns(3.0), 0.7) is not None


def test_알린_뒤에는_계속_보여도_다시_알리지_않는다():
    judge = IntruderJudge()
    judge.observe(ns(1.0), 0.9)
    assert judge.observe(ns(1.2), 0.9) is not None
    for i in range(100):
        assert judge.observe(ns(1.4 + 0.2 * i), 0.9) is None
    assert judge.fired


def test_NaN_은_사람이_아니다():
    judge = IntruderJudge()
    assert judge.observe(ns(1.0), float("nan")) is None
    assert judge.observe(ns(1.2), float("nan")) is None


# ---- 사람 판정: 다시 무장(재개·미션 무반응) -----------------------------------
#
# 2026-11-14 사용자 결정: 판정을 쉬는 시간은 없다(재개 뒤에도, 경비 시작 뒤에도).


def test_다시_무장하면_쉬는_시간_없이_바로_판정한다():
    judge = IntruderJudge()
    judge.observe(ns(1.0), 0.9)
    judge.observe(ns(1.2), 0.9)                       # 경보
    judge.rearm()                                     # 관리자가 재개
    assert not judge.fired
    assert judge.observe(ns(1.4), 0.9) is None        # 첫 번째
    assert judge.observe(ns(1.6), 0.9) is not None    # 두 번째 — 기다리지 않는다


def test_다시_무장하면_판정_창은_빈_채로_시작한다():
    judge = IntruderJudge()
    judge.observe(ns(1.0), 0.9)                       # 경보 전에 한 번 보였다
    judge.rearm()                                     # 일시정지 뒤 재개
    assert judge.observe(ns(1.5), 0.9) is None        # 재개 전 것과 합쳐 세지 않는다
    verdict = judge.observe(ns(1.7), 0.9)
    assert verdict is not None and verdict.hits == 2


def test_경보를_낸_두_번의_기록도_다시_무장하면_남지_않는다():
    judge = IntruderJudge()
    judge.observe(ns(1.0), 0.9)
    judge.observe(ns(1.2), 0.9)                       # 경보 — 창에 두 번이 들어 있다
    judge.rearm()
    assert judge.observe(ns(1.3), 0.9) is None        # 3 s 안이어도 새로 한 번째다


# ---- 불 켜짐: 무장·해제 -------------------------------------------------------


def test_어두움을_본_적이_없으면_밝아도_알리지_않는다():
    arming = LightArming()
    assert arming.observe(163.0) is False             # 불 켠 채 순찰 시작
    assert arming.observe(163.0) is False
    assert not arming.armed


def test_어두움을_본_뒤_밝으면_한_번_알리고_무장이_풀린다():
    arming = LightArming()
    assert arming.observe(1.4) is False               # 불 끔
    assert arming.armed
    assert arming.observe(35.0) is True               # 일부 켬
    assert not arming.armed
    assert arming.observe(163.0) is False             # 계속 밝다 — 다시 안 알린다


def test_다시_어두움을_보면_다시_무장된다():
    arming = LightArming()
    arming.observe(1.4)
    arming.observe(35.0)
    assert arming.observe(1.5) is False
    assert arming.observe(34.0) is True


def test_밝기_10_이_경계다():
    arming = LightArming()
    assert arming.observe(10.0) is False              # 10 은 밝음 — 무장 안 됨
    assert not arming.armed
    arming.observe(9.99)
    assert arming.armed
    assert arming.observe(10.0) is True


# ---- 불 켜짐: 노출 순서 -------------------------------------------------------


def test_노출_순서_고정_5장_버림_측정_복구_5장_버림():
    check = LightCheck()
    assert check.on_frame(ns(0.0)) is FrameUse.NORMAL
    assert check.start(ns(1.0)) is True               # 호출자가 FIX 를 보낸다
    assert check.busy and check.dirty
    assert check.on_frame(ns(1.01)) is FrameUse.SKIP  # 응답 대기
    assert check.on_result(ExposureCmd.FIX, True, ns(1.02)) is None

    uses = [check.on_frame(ns(1.1 + i * FRAME_S)) for i in range(6)]
    assert uses == [FrameUse.SKIP] * 5 + [FrameUse.MEASURE]
    assert check.on_measured(ns(1.5)) is ExposureCmd.AUTO
    assert check.on_frame(ns(1.51)) is FrameUse.SKIP  # 응답 대기
    assert check.on_result(ExposureCmd.AUTO, True, ns(1.52)) is None
    assert not check.dirty                            # 복구 확인

    uses = [check.on_frame(ns(1.6 + i * FRAME_S)) for i in range(6)]
    assert uses == [FrameUse.SKIP] * 5 + [FrameUse.NORMAL]
    assert not check.busy


def test_확인_중에는_다시_시작하지_않는다():
    check = LightCheck()
    assert check.start(ns(1.0)) is True
    assert check.start(ns(1.1)) is False


def test_고정_요청이_거절되면_재지_않고_복구한다():
    check = LightCheck()
    check.start(ns(1.0))
    assert check.on_result(ExposureCmd.FIX, False, ns(1.02)) is ExposureCmd.AUTO
    assert check.dirty
    check.on_result(ExposureCmd.AUTO, True, ns(1.05))
    assert not check.dirty


def test_고정_요청을_보내지도_못했으면_건드린_것이_없다():
    check = LightCheck()
    check.start(ns(1.0))
    check.on_not_sent(ExposureCmd.FIX)
    assert not check.busy and not check.dirty
    assert check.tick(ns(5.0)) is None


def test_고정_응답이_안_오면_1초_뒤_복구한다():
    check = LightCheck()
    check.start(ns(1.0))
    assert check.tick(ns(1.9)) is None
    assert check.tick(ns(2.1)) is ExposureCmd.AUTO


def test_고정한_채_프레임이_끊기면_3초_뒤_복구한다():
    check = LightCheck()
    check.start(ns(1.0))
    check.on_result(ExposureCmd.FIX, True, ns(1.02))
    assert check.tick(ns(3.9)) is None
    assert check.tick(ns(4.1)) is ExposureCmd.AUTO


def test_복구_응답이_없으면_추론을_재개하고_계속_다시_요청한다():
    check = LightCheck()
    check.start(ns(1.0))
    check.on_result(ExposureCmd.FIX, True, ns(1.02))
    for i in range(5):
        check.on_frame(ns(1.1 + i * FRAME_S))
    check.on_frame(ns(1.5))
    check.on_measured(ns(1.5))                        # AUTO 를 보냈다
    assert check.tick(ns(2.0)) is None                # 아직 1 s 안
    assert check.tick(ns(2.6)) is ExposureCmd.AUTO    # 다시 요청
    assert not check.busy                             # 추론은 재개
    assert check.dirty
    assert check.tick(ns(3.0)) is None                # 방금 보냈다
    assert check.tick(ns(3.7)) is ExposureCmd.AUTO    # 확인될 때까지
    check.on_result(ExposureCmd.AUTO, True, ns(3.71))
    assert not check.dirty
    assert check.tick(ns(5.0)) is None


def test_복구_요청이_실패하면_다시_요청한다():
    check = LightCheck()
    check.start(ns(1.0))
    check.on_result(ExposureCmd.FIX, False, ns(1.02))         # → AUTO
    check.on_result(ExposureCmd.AUTO, False, ns(1.05))        # 실패
    assert check.dirty
    assert check.tick(ns(2.1)) is ExposureCmd.AUTO


def test_복구_요청을_못_보냈으면_다음_점검에서_바로_다시_보낸다():
    check = LightCheck()
    check.start(ns(1.0))
    assert check.abort(ns(1.1)) is ExposureCmd.AUTO
    check.on_not_sent(ExposureCmd.AUTO)
    assert check.tick(ns(1.2)) is ExposureCmd.AUTO


@pytest.mark.parametrize("stage", ["fix_sent", "fix_settle", "measure"])
def test_고정한_동안_그만두면_자동_노출을_요청한다(stage):
    check = LightCheck()
    check.start(ns(1.0))
    if stage != "fix_sent":
        check.on_result(ExposureCmd.FIX, True, ns(1.02))
    if stage == "measure":
        for i in range(6):
            check.on_frame(ns(1.1 + i * FRAME_S))
    assert check.abort(ns(1.6)) is ExposureCmd.AUTO
    assert not check.busy
    assert check.dirty                                # 응답을 받아야 풀린다


def test_복구를_이미_요청했으면_그만둘_때_또_보내지_않는다():
    check = LightCheck()
    check.start(ns(1.0))
    check.on_result(ExposureCmd.FIX, True, ns(1.02))
    for i in range(6):
        check.on_frame(ns(1.1 + i * FRAME_S))
    check.on_measured(ns(1.5))
    assert check.abort(ns(1.51)) is None
    assert check.dirty                                # tick 이 응답을 지켜본다
    assert check.tick(ns(2.6)) is ExposureCmd.AUTO


def test_아무것도_안_했으면_그만둘_것도_없다():
    check = LightCheck()
    assert check.abort(ns(1.0)) is None
    assert check.tick(ns(2.0)) is None


def test_복구가_확인되기_전에는_새_확인을_시작하지_않는다():
    check = LightCheck()
    check.start(ns(1.0))
    check.abort(ns(1.1))
    assert check.start(ns(1.2)) is False
    check.on_result(ExposureCmd.AUTO, True, ns(1.3))
    assert check.start(ns(1.4)) is True


def test_늦게_온_고정_응답은_무시한다():
    check = LightCheck()
    check.start(ns(1.0))
    assert check.tick(ns(2.1)) is ExposureCmd.AUTO    # 응답이 없어 복구로 넘어갔다
    assert check.on_result(ExposureCmd.FIX, True, ns(2.2)) is None
    check.on_result(ExposureCmd.AUTO, True, ns(2.3))
    assert not check.dirty


# ---- 처리 순서(PatrolProfile) -------------------------------------------------


class Rig:
    """가짜 모델·카메라·발행으로 순찰 프로필을 돌린다."""

    def __init__(self, tmp_path, exposure_ready=True, auto_ack=True):
        self.media = str(tmp_path / "media")
        self.events = []
        self.exposure = []                    # 보낸 노출 요청
        self.logs = []
        self.cleanups = 0
        self.decodes = 0                      # 영상을 푼 횟수(프레임 복사)
        self.encodes = 0
        self.infers = 0
        self.confs = ()                       # 추론이 돌려줄 사람 conf
        self.brightness = 1.4
        self.exposure_ready = exposure_ready
        self.auto_ack = auto_ack              # 요청을 다음 프레임 전에 성공으로 응답
        self.infer_error = None
        self.brightness_error = None
        self._acks = []
        self.t = 0.0
        self.profile = PatrolProfile(PatrolPorts(
            publish=self.events.append,
            encode_jpeg=self._encode,
            brightness=self._brightness,
            infer=self._infer,
            request_exposure=self._request,
            exposure_ready=lambda: self.exposure_ready,
            cleanup=self._cleanup,
            log=lambda level, text: self.logs.append((level, text)),
        ), self.media, 5.0)
        self.profile.set_engine_loaded(True)

    def _encode(self, img):
        self.encodes += 1
        return b"\xff\xd8" + img.encode()

    def _brightness(self, img):
        if self.brightness_error is not None:
            raise self.brightness_error
        return self.brightness

    def _infer(self, img):
        self.infers += 1
        if self.infer_error is not None:
            raise self.infer_error
        return self.confs

    def _request(self, cmd):
        if not self.exposure_ready:
            return False
        self.exposure.append(cmd)
        if self.auto_ack:
            self._acks.append(cmd)
        return True

    def _cleanup(self):
        self.cleanups += 1

    # -- 시험이 쓰는 것 --

    def state(self, value):
        self.profile.on_dialog_state(value, ns(self.t), WALL0 + self.t)

    def frame(self):
        acks, self._acks = self._acks, []
        for cmd in acks:
            self.profile.on_exposure_result(cmd, True, ns(self.t))
        self.profile.on_frame(self._decode, 640, 480, ns(self.t), WALL0 + self.t)

    def _decode(self):
        self.decodes += 1
        return "img"

    def work(self):
        """프레임에 들인 일(풀기·압축·추론 횟수)."""
        return (self.decodes, self.encodes, self.infers)

    def run(self, seconds, tick=True):
        """카메라 15 Hz 로 seconds 만큼. 1 Hz 점검도 같이 돈다."""
        for _ in range(int(round(seconds * 15))):
            before = int(self.t + 1e-9)
            self.t = round(self.t + FRAME_S, 6)
            if tick and int(self.t + 1e-9) != before:
                self.profile.on_tick(ns(self.t), WALL0 + self.t)
            self.frame()

    def kinds(self, kind=None):
        if kind is None:
            return [e["kind"] for e in self.events if e["kind"] != "status"]
        return [e for e in self.events if e["kind"] == kind]


def test_순찰이_아니면_프레임이_와도_아무것도_하지_않는다(tmp_path):
    rig = Rig(tmp_path)
    rig.confs = (0.9,)
    for state in ("idle", "navigating", "waiting"):
        rig.state(state)
        rig.run(2.0)
    assert not rig.profile.active
    assert (rig.encodes, rig.infers, rig.cleanups) == (0, 0, 0)
    assert rig.events == [] and rig.exposure == []
    assert not os.path.exists(rig.media)


def test_순찰에_들어오면_정리를_시키고_status_를_1Hz_로_낸다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    assert rig.profile.active
    assert rig.cleanups == 1
    rig.run(5.0)
    status = rig.kinds("status")
    assert len(status) == 5
    assert all(set(e) == {"src", "kind", "stamp", "ready", "camera_ok"} for e in status)
    assert status[-1]["ready"] is True and status[-1]["camera_ok"] is True
    assert status[-1]["stamp"] == pytest.approx(WALL0 + 5.0)


def test_순찰_엔진이_아직_없으면_ready_false_이고_추론하지_않는다(tmp_path):
    rig = Rig(tmp_path)
    rig.profile.set_engine_loaded(False)
    rig.confs = (0.9,)
    rig.state("patrolling")
    rig.run(3.0)
    assert rig.infers == 0
    assert rig.kinds("status")[-1]["ready"] is False
    assert rig.kinds() == []
    assert rig.encodes > 0                    # 영상 버퍼는 돈다(소리 경보용)


def test_컬러_프레임이_3초_넘게_없으면_camera_ok_false(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.profile.on_tick(ns(0.5), WALL0 + 0.5)
    assert rig.kinds("status")[-1]["camera_ok"] is False      # 아직 한 장도 못 받았다
    rig.run(1.0)
    last = rig.t
    rig.profile.on_tick(ns(last + 3.0), WALL0)
    assert rig.kinds("status")[-1]["camera_ok"] is True
    rig.profile.on_tick(ns(last + 3.1), WALL0)
    assert rig.kinds("status")[-1]["camera_ok"] is False


def test_처리는_5Hz_로_솎는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(4.0)                              # 15 Hz 60장
    assert 20 <= rig.encodes <= 21
    assert rig.infers == rig.encodes


def test_사람이_3초_안에_두_번_보이면_intruder_한_번과_장면_시작(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(6.0)
    rig.confs = (0.3, 0.72)
    rig.run(1.0)
    rig.state("patrol_alarm")                 # 미션이 멈췄다
    rig.run(3.0)

    assert rig.kinds() == ["intruder", "clip"]            # 경보가 먼저다
    intruder, clip = rig.kinds("intruder")[0], rig.kinds("clip")[0]
    assert set(intruder) == {"src", "kind", "stamp", "conf", "hits", "clip_id"}
    assert intruder["conf"] == 0.72 and intruder["hits"] == 2
    assert intruder["clip_id"].endswith("_intruder")
    assert clip == {"src": "detector", "kind": "clip", "stamp": intruder["stamp"],
                    "clip_id": intruder["clip_id"], "state": "started",
                    "alarm": "intruder"}
    frames = os.listdir(os.path.join(rig.media, "clips", intruder["clip_id"], "frames"))
    assert len(frames) >= 25                  # 경보 앞 5 s 가 들어 있다


def test_경보_전에는_디스크에_쓰지_않는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(30.0)
    rig.state("patrol_dwell")
    rig.run(3.0)
    assert not os.path.exists(rig.media)


def test_conf_0_5_미만이면_계속_보여도_경보가_없다(tmp_path):
    rig = Rig(tmp_path)
    rig.confs = (0.49,)
    rig.state("patrolling")
    rig.run(10.0)
    assert rig.kinds() == []


def test_사람_장면은_안_보이고_10초_뒤에_끝나고_meta_와_done_을_남긴다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(6.0)
    rig.confs = (0.8,)
    rig.run(0.5)                              # 경보
    rig.state("patrol_alarm")
    rig.run(4.0)                              # 사람이 계속 보인다
    seen_until = rig.t
    rig.confs = ()
    cleanups = rig.cleanups
    rig.run(9.5)
    assert [e["state"] for e in rig.kinds("clip")] == ["started"]     # 아직 10 s 전
    rig.run(1.0)

    done = rig.kinds("clip")[-1]
    assert set(done) == {"src", "kind", "stamp", "clip_id", "state", "alarm", "frames"}
    assert done["state"] == "done" and done["alarm"] == "intruder"
    clip_dir = os.path.join(rig.media, "clips", done["clip_id"])
    with open(os.path.join(clip_dir, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)
    assert list(meta) == ["id", "alarm", "start", "end", "fps",
                          "frames", "width", "height", "start_epoch", "end_epoch"]
    assert meta["frames"] == done["frames"] == len(os.listdir(
        os.path.join(clip_dir, "frames")))
    assert meta["end"] in (iso_local(WALL0 + seen_until + 10.0),
                           iso_local(WALL0 + seen_until + 9.0))
    # 5 s 전 + 경보까지 0.5 s + 보인 4 s + 10 s ≈ 19.5 s
    assert 90 <= meta["frames"] <= 100
    assert rig.cleanups == cleanups + 1       # 장면을 끝낼 때 정리


def test_경보로_멈춘_동안에도_사람_장면이면_추론을_계속한다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.confs = (0.8,)
    rig.run(1.0)
    rig.state("patrol_alarm")
    before = rig.infers
    rig.run(2.0)
    assert rig.infers > before                # 마지막으로 보인 때를 알아야 한다


def process_one(rig):
    """처리(5 Hz)가 한 번 일어날 때까지 프레임을 넣는다(판정 상태에서만 쓴다)."""
    before = rig.infers
    while rig.infers == before:
        rig.run(FRAME_S, tick=False)


def test_경비_시작_직후에도_쉬지_않고_판정한다(tmp_path):
    rig = Rig(tmp_path)
    rig.confs = (0.8,)
    rig.state("patrol_dwell")                 # 경비 시작
    rig.exposure_ready = False                # 불 확인과 겹치지 않게
    process_one(rig)
    assert rig.kinds("intruder") == []
    process_one(rig)
    assert len(rig.kinds("intruder")) == 1    # 두 번째 처리 프레임에서 바로
    assert rig.t < 0.5


@pytest.mark.parametrize("hold", ["patrol_alarm", "patrol_paused", "patrol_light"])
def test_재개_직후에도_쉬지_않고_판정한다(tmp_path, hold):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(1.0)
    rig.state(hold)
    rig.run(3.0)
    rig.confs = (0.8,)
    rig.state("patrolling")                   # 관리자가 순찰 재개
    resumed = rig.t
    rig.run(0.5)
    intruder = rig.kinds("intruder")
    assert len(intruder) == 1
    assert intruder[0]["stamp"] < WALL0 + resumed + 0.5


def test_재개_직후_판정_창은_비어서_시작한다_경보_뒤(tmp_path):
    """경보를 낸 두 번의 기록이 남아 재개 첫 프레임에 곧바로 다시 내면 안 된다."""
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.confs = (0.8,)
    process_one(rig)
    process_one(rig)
    assert len(rig.kinds("intruder")) == 1
    rig.state("patrol_alarm")
    rig.run(0.5)
    rig.state("patrolling")                   # 재개 — 앞의 두 번은 아직 3 s 안이다
    process_one(rig)
    assert len(rig.kinds("intruder")) == 1    # 재개 뒤 한 번째
    process_one(rig)
    again = rig.kinds("intruder")
    assert len(again) == 2 and again[1]["hits"] == 2
    assert rig.t < 3.0


def test_재개_직후_판정_창은_비어서_시작한다_일시정지_뒤(tmp_path):
    """일시정지 전에 한 번 보인 것을 재개 뒤 한 번과 합쳐 경보를 내면 안 된다."""
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(1.0)
    rig.confs = (0.8,)
    process_one(rig)                          # 한 번 보였다
    assert rig.kinds() == []
    rig.state("patrol_paused")
    rig.run(1.0)                              # 일시정지 중에는 판정하지 않는다
    assert rig.kinds() == []
    rig.state("patrolling")                   # 재개 — 앞의 한 번은 아직 3 s 안이다
    process_one(rig)
    assert rig.kinds() == []                  # 재개 뒤 한 번째
    process_one(rig)
    intruder = rig.kinds("intruder")
    assert len(intruder) == 1 and intruder[0]["hits"] == 2
    assert rig.t < 4.0


# ---- 멈춘 상태: 쓰는 장면이 없으면 프레임에 일을 들이지 않는다 -------------------
#
# 관리자를 기다리는 시간이 길 수 있다(2026-11-14 감사). 버려질 압축·추론을 하지 않는다.


@pytest.mark.parametrize("hold", ["patrol_paused", "patrol_light"])
def test_멈춘_상태에서_쓰는_장면이_없으면_영상을_풀지도_압축하지도_추론하지도_않는다(
        tmp_path, hold):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(1.0)
    rig.state(hold)
    before = rig.work()
    rig.confs = (0.9,)
    rig.run(60.0)
    assert rig.work() == before
    assert rig.kinds() == []


def test_소리_장면이_끝난_뒤에는_멈춘_동안_프레임에_일을_들이지_않는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(6.0)
    rig.state("patrol_alarm")                 # 소리 경보 — 장면을 쓰는 10 s 는 일한다
    rig.run(11.0)
    assert [e["state"] for e in rig.kinds("clip")] == ["started", "done"]
    before = rig.work()
    rig.run(60.0)                             # 관리자를 기다린다
    assert rig.work() == before


def test_사람_장면이_끝난_뒤에는_멈춘_동안_추론도_하지_않는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.confs = (0.8,)
    rig.run(1.0)
    rig.state("patrol_alarm")
    rig.confs = ()                            # 사람이 떠났다
    rig.run(11.0)
    assert [e["state"] for e in rig.kinds("clip")] == ["started", "done"]
    before = rig.work()
    rig.confs = (0.9,)                        # 다시 나타나도 멈춘 동안은 보지 않는다
    rig.run(30.0)
    assert rig.work() == before
    assert len(rig.kinds("intruder")) == 1


def test_멈춘_동안에도_camera_ok_는_받은_프레임으로_잰다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(1.0)
    rig.state("patrol_paused")
    rig.run(10.0)
    status = rig.kinds("status")
    assert len(status) == 11
    assert all(e["camera_ok"] and e["ready"] for e in status[1:])   # 재개 조건이 유지된다


def test_재개_직후_경보의_장면은_재개_뒤_프레임으로만_시작한다(tmp_path):
    """멈춘 동안 버퍼를 채우지 않으므로 앞부분이 5 s 보다 짧다. 옛 프레임은 넣지 않는다."""
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(6.0)                              # 버퍼 25장
    rig.state("patrol_paused")
    rig.run(20.0)
    rig.confs = (0.8,)
    rig.state("patrolling")                   # 재개
    resumed = rig.t
    process_one(rig)
    process_one(rig)                          # 두 번째 처리 프레임에서 경보
    intruder = rig.kinds("intruder")
    assert len(intruder) == 1
    frames = os.listdir(os.path.join(rig.media, "clips", intruder[0]["clip_id"], "frames"))
    assert len(frames) == 2                   # 재개 뒤 두 장뿐 — 20 s 전 것은 없다
    rig.confs = ()
    rig.state("patrol_alarm")
    rig.run(11.0)
    with open(os.path.join(rig.media, "clips", intruder[0]["clip_id"], "meta.json"),
              encoding="utf-8") as f:
        meta = json.load(f)
    assert meta["start_epoch"] >= WALL0 + resumed


def test_미션이_반응하지_않으면_5초_뒤_다시_무장한다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.confs = (0.8,)
    rig.run(1.0)
    assert len(rig.kinds("intruder")) == 1
    rig.run(4.0)
    assert len(rig.kinds("intruder")) == 1    # 아직 기다린다
    rig.run(3.0)                              # 상태가 patrolling 그대로
    again = rig.kinds("intruder")
    assert len(again) == 2
    assert again[1]["clip_id"] == again[0]["clip_id"]     # 쓰던 장면에 잇는다
    assert [e["state"] for e in rig.kinds("clip")] == ["started"]


def test_추론이_잇따라_실패하면_ready_를_내리고_노드는_산다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(1.0)
    rig.infer_error = RuntimeError("TensorRT")
    rig.run(2.0)                              # 예외가 밖으로 나오지 않는다
    assert rig.kinds("status")[-1]["ready"] is False
    assert any(level == "error" for level, _ in rig.logs)
    calls = rig.infers
    rig.run(2.0)
    assert rig.infers == calls                # 더 부르지 않는다


# ---- 큰 소리 경보(내가 낸 경보가 아닌 patrol_alarm) ----------------------------


def test_큰_소리_경보_5초_전부터_10초_뒤까지_장면을_남긴다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(8.0)
    rig.state("patrol_alarm")                 # 음성 노드의 sound 로 미션이 멈췄다
    started = rig.kinds("clip")
    assert len(started) == 1
    assert started[0]["state"] == "started" and started[0]["alarm"] == "sound"
    assert started[0]["clip_id"].endswith("_sound")
    assert rig.kinds("intruder") == []

    rig.confs = (0.9,)                        # 사람이 보여도 소리 장면은 늘어나지 않는다
    infers = rig.infers
    rig.run(9.5)
    assert len(rig.kinds("clip")) == 1
    rig.run(1.0)
    done = rig.kinds("clip")[-1]
    assert done["state"] == "done" and done["alarm"] == "sound"
    assert 70 <= done["frames"] <= 77         # 5 s + 10 s, 5 Hz
    assert rig.infers == infers               # 멈춘 동안 추론하지 않는다


def test_카메라가_끊겨서_난_경보에는_장면을_만들지_않는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(5.0)
    rig.t += 3.5                              # 프레임이 3 s 넘게 없다
    rig.state("patrol_alarm")                 # 미션의 patrol_camera_lost
    assert rig.kinds() == []
    assert not os.path.exists(rig.media)


def test_사람_장면을_쓰는_중에_소리_경보가_오면_새_장면_없이_10초_늘린다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.confs = (0.8,)
    rig.run(1.0)
    rig.state("patrol_alarm")
    rig.confs = ()
    rig.run(2.0)
    rig.state("patrolling")                   # 재개
    rig.run(6.0)
    rig.state("patrol_alarm")                 # 이번엔 소리
    extended_at = rig.t
    rig.run(9.5)
    assert [e["state"] for e in rig.kinds("clip")] == ["started"]
    rig.run(1.0)
    clips = rig.kinds("clip")
    assert [e["state"] for e in clips] == ["started", "done"]
    assert clips[1]["stamp"] >= WALL0 + extended_at + 10.0


# ---- 감시 항목 켜고 끄기(2026-11-14 사용자 요청) -------------------------------


def watching(tmp_path, off, **kw):
    rig = Rig(tmp_path, **kw)
    rig.profile.set_watch(off, ns(0.0))
    return rig


def test_감시_항목_알림은_아는_이름만_읽고_못_읽으면_None():
    assert parse_watch_off('{"off": []}') == frozenset()
    assert parse_watch_off('{"off": ["light", "sound", "camera_lost", 3]}') == {"light", "sound"}
    for bad in ("", "nope", "[]", '{"off": "light"}', "{}", "null"):
        assert parse_watch_off(bad) is None


def test_사람_감지를_끄면_추론하지_않고_엔진_없이도_ready_다(tmp_path):
    rig = watching(tmp_path, ["intruder"])
    rig.profile.set_engine_loaded(False)      # 엔진을 올린 적이 없다
    assert not rig.profile.needs_engine
    rig.confs = (0.9,)
    rig.state("patrolling")
    rig.run(4.0)
    assert rig.infers == 0 and rig.kinds() == []
    assert rig.kinds("status")[-1]["ready"] is True
    assert rig.encodes > 0                    # 큰 소리 경보의 영상 앞 5 s 는 모은다


def test_사람_감지를_꺼도_엔진이_올라와_있으면_추론은_하지_않는다(tmp_path):
    rig = watching(tmp_path, ["intruder"])    # 엔진은 지난 순찰 때 올라왔다
    rig.confs = (0.9,)
    rig.state("patrolling")
    rig.run(4.0)
    assert rig.infers == 0 and rig.kinds() == []


def test_사람_감지를_꺼도_큰_소리_경보에는_장면을_찍는다(tmp_path):
    rig = watching(tmp_path, ["intruder"])
    rig.state("patrolling")
    rig.run(8.0)
    rig.state("patrol_alarm")                 # 음성 노드의 sound 로 미션이 멈췄다
    rig.run(10.5)
    assert [(e["state"], e["alarm"]) for e in rig.kinds("clip")] == [
        ("started", "sound"), ("done", "sound")]
    assert 70 <= rig.kinds("clip")[-1]["frames"] <= 77
    assert rig.infers == 0


def test_큰_소리를_끄면_미션이_낸_경보에_장면을_찍지_않는다(tmp_path):
    rig = watching(tmp_path, ["sound"])
    rig.state("patrolling")
    rig.run(8.0)
    rig.state("patrol_alarm")                 # 내가 내지 않은 경보 — 남은 것은 카메라 끊김뿐이다
    rig.run(3.0)
    assert rig.kinds() == []
    assert not os.path.exists(rig.media)


def test_큰_소리를_꺼도_사람_경보와_장면은_그대로다(tmp_path):
    rig = watching(tmp_path, ["sound"])
    rig.state("patrolling")
    rig.confs = (0.8,)
    rig.run(1.0)
    assert rig.kinds() == ["intruder", "clip"]


def test_사람과_큰_소리를_다_끄면_주행_중에는_영상을_풀지도_않는다(tmp_path):
    rig = watching(tmp_path, ["intruder", "sound"])
    rig.confs = (0.9,)
    rig.state("patrolling")
    rig.run(5.0)
    assert rig.work() == (0, 0, 0)
    assert rig.profile._clip.buffered == 0
    status = rig.kinds("status")
    assert len(status) == 5 and status[-1] == {
        "src": "detector", "kind": "status", "stamp": status[-1]["stamp"],
        "ready": True, "camera_ok": True}     # 카메라 끊김 감시는 그대로 돈다


def test_사람과_큰_소리를_다_꺼도_불_켜짐은_잡고_사진_한_장만_압축한다(tmp_path):
    rig = watching(tmp_path, ["intruder", "sound"])
    rig.state("patrolling")
    rig.run(1.0)
    dwell(rig, 1.4)                           # 불 끔
    assert rig.encodes == 0
    dwell(rig, 35.0)                          # 누가 불을 켰다
    lights = rig.kinds("light")
    assert len(lights) == 1
    assert os.path.isfile(os.path.join(rig.media, "photos", lights[0]["photo"]))
    assert (rig.encodes, rig.infers) == (1, 0)
    assert rig.decodes == 3                   # 밝기 2번 + 사진 1번
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO] * 2


def test_불_켜짐을_끄면_노출을_건드리지_않는다(tmp_path):
    rig = watching(tmp_path, ["light"])
    rig.state("patrolling")
    rig.run(1.0)
    dwell(rig, 1.4)
    dwell(rig, 35.0)
    assert rig.exposure == [] and rig.kinds() == []
    assert not rig.profile.exposure_dirty


def test_불_켜짐을_끄면_dwell_에서도_사람_판정을_쉬지_않는다(tmp_path):
    rig = watching(tmp_path, ["light"])
    rig.state("patrol_dwell")
    rig.confs = (0.8,)
    rig.run(0.5)                              # 노출 전환으로 버리는 프레임이 없다
    assert rig.kinds() == ["intruder", "clip"]


def test_노출을_고정한_동안_불_켜짐이_꺼지면_곧바로_자동_노출로_되돌린다(tmp_path):
    rig = Rig(tmp_path, auto_ack=False)
    enter_fixed(rig)
    rig.profile.set_watch(["light"], ns(rig.t))
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO]
    rig.brightness = 35.0
    rig.run(3.0)
    assert rig.kinds("light") == []


def test_dwell_도중에_불_켜짐이_켜졌다고_오면_그_dwell_부터_확인한다(tmp_path):
    rig = watching(tmp_path, ["light"])       # 지난 순찰의 값
    rig.state("patrol_dwell")                 # 상태가 알림보다 먼저 처리됐다
    rig.run(0.2)
    assert rig.exposure == []
    rig.profile.set_watch([], ns(rig.t))
    rig.run(1.5)
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO]


def test_주행_중에_불_켜짐이_켜지면_다음_dwell_에서_확인한다(tmp_path):
    rig = watching(tmp_path, ["light"])
    rig.state("patrolling")
    rig.run(0.5)
    rig.profile.set_watch([], ns(rig.t))
    rig.run(1.0)
    assert rig.exposure == []                 # 이동 중에는 건드리지 않는다
    dwell(rig, 1.4)
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO]


def test_감시_항목은_순찰이_끝나도_남고_다시_켜면_돌아온다(tmp_path):
    rig = watching(tmp_path, ["intruder", "light", "sound"])   # 셋 다 — 미션은 안 보내지만 받아도 된다
    rig.confs = (0.9,)
    rig.state("patrolling")
    rig.run(2.0)
    rig.state("idle")
    rig.state("patrolling")
    rig.run(2.0)
    assert rig.work() == (0, 0, 0) and rig.kinds() == []
    rig.profile.set_watch([], ns(rig.t))      # 모두 켬
    assert rig.profile.needs_engine
    rig.run(1.0)
    assert rig.kinds() == ["intruder", "clip"]


def test_같은_값을_다시_받으면_아무것도_하지_않는다(tmp_path):
    rig = watching(tmp_path, ["light"])
    logs = len(rig.logs)
    rig.state("patrolling")
    rig.confs = (0.8,)
    rig.run(0.25)                             # 한 번 봤다(판정 창에 1번)
    rig.profile.set_watch(["light"], ns(rig.t))
    rig.run(0.25)
    assert rig.kinds() == ["intruder", "clip"]            # 판정 창을 비우지 않았다
    assert len(rig.logs) == logs


# ---- 불 켜짐 확인 -------------------------------------------------------------


def dwell(rig, brightness, seconds=3.0):
    """지점 도착 → 3 s 정지 → 출발."""
    rig.brightness = brightness
    rig.state("patrol_dwell")
    rig.run(seconds)
    rig.state("patrolling")
    rig.run(1.0)


def test_dwell_에_들어올_때마다_노출을_고정했다가_자동으로_되돌린다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(2.0)
    assert rig.exposure == []                 # 이동 중에는 건드리지 않는다
    dwell(rig, 1.4)
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO]
    dwell(rig, 1.4)
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO] * 2
    assert not rig.profile.exposure_dirty
    assert rig.kinds() == []                  # 어두우면 경보 없음


def test_불_확인_동안_추론과_영상_처리를_쉬고_끝나면_재개한다(tmp_path):
    rig = Rig(tmp_path, auto_ack=False)
    rig.state("patrolling")
    rig.run(1.0)
    rig.state("patrol_dwell")
    rig.frame()                               # FIX 를 보낸다
    assert rig.exposure == [ExposureCmd.FIX]
    before = (rig.encodes, rig.infers)
    rig.profile.on_exposure_result(ExposureCmd.FIX, True, ns(rig.t))
    rig.run(5 * FRAME_S, tick=False)          # 5장 버림
    assert rig.exposure == [ExposureCmd.FIX]
    rig.run(FRAME_S, tick=False)              # 6번째 장으로 잰다
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO]
    rig.profile.on_exposure_result(ExposureCmd.AUTO, True, ns(rig.t))
    rig.run(5 * FRAME_S, tick=False)          # 5장 버림
    assert (rig.encodes, rig.infers) == before
    rig.run(FRAME_S, tick=False)
    assert rig.encodes == before[0] + 1       # 재개
    assert rig.infers == before[1] + 1


def test_어두움을_본_뒤_밝아지면_light_한_번과_사진(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(1.0)
    dwell(rig, 1.4)                           # 불 끔
    dwell(rig, 35.0)                          # 누가 불을 켰다
    lights = rig.kinds("light")
    assert len(lights) == 1
    assert set(lights[0]) == {"src", "kind", "stamp", "brightness", "photo"}
    assert lights[0]["brightness"] == 35.0
    photo = os.path.join(rig.media, "photos", lights[0]["photo"])
    assert lights[0]["photo"].endswith("_light.jpg")
    assert os.path.isfile(photo)
    assert rig.exposure[-1] is ExposureCmd.AUTO       # 사진은 자동 노출로 돌아온 뒤


def test_불_켜짐을_알린_뒤에는_다시_어두움을_볼_때까지_조용하다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(1.0)
    dwell(rig, 1.4)
    rig.brightness = 163.0
    rig.state("patrol_dwell")
    rig.run(2.0)
    rig.state("patrol_light")                 # 미션이 멈췄다
    rig.run(5.0)
    rig.state("patrolling")                   # 재개
    rig.run(1.0)
    dwell(rig, 163.0)                         # 불이 아직 켜져 있다
    dwell(rig, 163.0)
    assert len(rig.kinds("light")) == 1
    dwell(rig, 1.4)                           # 불이 꺼졌다
    dwell(rig, 34.0)                          # 다시 켜졌다
    assert len(rig.kinds("light")) == 2


def test_불을_켠_채_순찰을_시작하면_경보가_없다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrol_dwell")                 # 시작 직후 준비 대기
    rig.brightness = 163.0
    rig.run(3.0)
    rig.state("patrolling")
    rig.run(1.0)
    dwell(rig, 163.0)
    assert rig.kinds("light") == []
    assert rig.exposure.count(ExposureCmd.FIX) == rig.exposure.count(ExposureCmd.AUTO) == 2


def test_순찰을_다시_시작하면_무장도_처음부터다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(1.0)
    dwell(rig, 1.4)                           # 무장됨
    rig.state("idle")                         # 경비 종료
    rig.run(1.0)
    rig.state("patrolling")
    rig.run(1.0)
    dwell(rig, 163.0)                         # 새 순찰의 첫 확인이 밝다
    assert rig.kinds("light") == []


def test_불_켜짐_경보에_미션_반응이_없으면_다음_dwell_에서_다시_알린다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(1.0)
    dwell(rig, 1.4)
    dwell(rig, 35.0, seconds=3.0)             # light 를 냈지만 patrol_light 가 안 된다
    rig.run(6.0)
    dwell(rig, 35.0)
    assert len(rig.kinds("light")) == 2


def test_불_켜짐_경보가_다른_경보에_묻히면_재개_뒤_다시_알린다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(1.0)
    dwell(rig, 1.4)
    rig.brightness = 35.0
    rig.state("patrol_dwell")
    rig.run(2.0)
    assert len(rig.kinds("light")) == 1
    rig.state("patrol_alarm")                 # 같은 때 난 소리 경보가 먼저 닿았다
    rig.run(12.0)
    rig.state("patrolling")                   # 재개 — patrol_light 는 된 적이 없다
    rig.run(1.0)
    dwell(rig, 35.0)
    assert len(rig.kinds("light")) == 2


def test_밝기를_잰_직후_미션이_멈추면_알리지_않고_무장을_남긴다(tmp_path):
    rig = Rig(tmp_path, auto_ack=False)
    rig.state("patrolling")
    rig.run(1.0)
    rig.auto_ack = True
    dwell(rig, 1.4)                           # 무장
    rig.auto_ack = False
    rig.brightness = 35.0
    rig.state("patrol_dwell")
    rig.frame()
    rig.profile.on_exposure_result(ExposureCmd.FIX, True, ns(rig.t))
    rig.run(6 * FRAME_S, tick=False)          # 5장 버리고 잰다 — 밝다
    rig.state("patrol_alarm")                 # 사진을 찍기 전에 소리 경보로 멈췄다
    rig.profile.on_exposure_result(ExposureCmd.AUTO, True, ns(rig.t))
    rig.run(12.0)
    assert rig.kinds("light") == []           # 미션이 받지 않는 상태에서는 내지 않는다
    assert not os.path.exists(os.path.join(rig.media, "photos"))
    rig.auto_ack = True
    rig.state("patrolling")
    rig.run(1.0)
    dwell(rig, 35.0)
    assert len(rig.kinds("light")) == 1       # 다음 확인에서 알린다


def test_카메라_매개변수_서비스가_없으면_노출을_건드리지_않는다(tmp_path):
    rig = Rig(tmp_path, exposure_ready=False)
    rig.state("patrol_dwell")
    rig.run(3.0)
    assert rig.exposure == []
    assert not rig.profile.exposure_dirty
    assert rig.encodes > 0                    # 추론·영상은 그대로 돈다


def test_서비스가_dwell_도중에_생기면_그때_확인한다(tmp_path):
    rig = Rig(tmp_path, exposure_ready=False)
    rig.state("patrol_dwell")
    rig.run(1.0)
    rig.exposure_ready = True
    rig.run(2.0)
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO]


# ---- 노출 복구: 어떤 경로로 나가도 --------------------------------------------


def enter_fixed(rig):
    """노출이 고정된 채 프레임을 버리는 중까지 간다."""
    rig.state("patrolling")
    rig.run(1.0)
    rig.state("patrol_dwell")
    rig.run(3 * FRAME_S, tick=False)
    assert rig.exposure == [ExposureCmd.FIX]


@pytest.mark.parametrize("leave_state", ["idle", "estopped", "navigating", ""])
def test_노출_고정_중에_순찰이_끝나면_자동_노출을_요청한다(tmp_path, leave_state):
    rig = Rig(tmp_path)
    enter_fixed(rig)
    rig.state(leave_state)
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO]
    assert not rig.profile.active


def test_밝기를_재다_예외가_나도_자동_노출을_요청한다(tmp_path):
    rig = Rig(tmp_path)
    rig.brightness_error = ValueError("frame")
    rig.state("patrolling")
    rig.run(1.0)
    rig.state("patrol_dwell")
    with pytest.raises(ValueError):
        rig.run(1.0, tick=False)
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO]


def test_프레임_처리_예외_뒤처리도_자동_노출을_요청한다(tmp_path):
    rig = Rig(tmp_path)
    enter_fixed(rig)
    rig.profile.on_error(ns(rig.t))
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO]


def test_고정한_채_카메라가_끊기면_1Hz_점검이_되돌린다(tmp_path):
    rig = Rig(tmp_path)
    enter_fixed(rig)
    for i in range(1, 5):                     # 프레임 없이 점검만
        rig.profile.on_tick(ns(rig.t + i), WALL0 + rig.t + i)
    assert rig.exposure[-1] is ExposureCmd.AUTO


def test_복구_응답이_올_때까지_순찰이_끝난_뒤에도_다시_요청한다(tmp_path):
    rig = Rig(tmp_path, auto_ack=False)
    rig.state("patrolling")
    rig.run(1.0)
    rig.state("patrol_dwell")
    rig.frame()
    rig.profile.on_exposure_result(ExposureCmd.FIX, True, ns(rig.t))
    rig.state("idle")                         # 순찰 끝 — AUTO 요청, 응답은 아직
    assert rig.exposure == [ExposureCmd.FIX, ExposureCmd.AUTO]
    assert rig.profile.exposure_dirty         # 배선이 타이머를 살려 둔다
    events = len(rig.events)
    rig.profile.on_tick(ns(rig.t + 1.5), WALL0)
    assert rig.exposure[-2:] == [ExposureCmd.AUTO, ExposureCmd.AUTO]
    assert len(rig.events) == events          # 순찰이 아니면 status 를 내지 않는다
    rig.profile.on_exposure_result(ExposureCmd.AUTO, True, ns(rig.t + 1.6))
    assert not rig.profile.exposure_dirty
    rig.profile.on_tick(ns(rig.t + 3.0), WALL0)
    assert rig.exposure.count(ExposureCmd.AUTO) == 2


def test_복구_요청을_보낼_수_없어도_dirty_는_남는다(tmp_path):
    rig = Rig(tmp_path)
    enter_fixed(rig)
    rig.exposure_ready = False                # 카메라 노드가 사라졌다
    rig.state("idle")
    assert rig.profile.exposure_dirty         # 종료 처리가 이것을 보고 되돌린다
    rig.exposure_ready = True
    rig.profile.on_tick(ns(rig.t + 1.0), WALL0)
    assert rig.exposure[-1] is ExposureCmd.AUTO


# ---- 순찰 이탈: 쓰던 장면은 끝까지 저장한다 -----------------------------------
#
# 앱 확인창 문구(사용자 승인): "녹화 중인 영상은 끝까지 저장합니다."


def test_쓰는_장면이_없으면_순찰이_끝나는_즉시_프레임을_놓는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(6.0)
    rig.state("idle")                         # 경비 종료
    assert not rig.profile.wants_frames
    assert not rig.profile.draining
    assert rig.profile._clip.buffered == 0
    before = (rig.work(), len(rig.events))
    rig.run(3.0)
    assert (rig.work(), len(rig.events)) == before


def test_순찰이_끝나도_쓰던_사람_장면은_사람이_사라지고_10초_뒤까지_쓴다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.confs = (0.8,)
    rig.run(6.0)
    rig.state("patrol_alarm")
    rig.run(2.0)
    rig.state("idle")                         # 경비 종료 — 사람은 아직 보인다
    assert not rig.profile.active
    assert rig.profile.draining and rig.profile.wants_frames
    assert [e["state"] for e in rig.kinds("clip")] == ["started"]     # 닫지 않았다
    assert any("끝까지 저장" in text for _, text in rig.logs)

    infers = rig.infers
    rig.run(3.0)
    assert rig.infers > infers                # 사람 장면이라 추론도 계속한다
    seen_until = rig.t
    rig.confs = ()                            # 사람이 사라졌다
    rig.run(9.5)
    assert rig.profile.draining               # 아직 10 s 전
    rig.run(1.0)

    clips = rig.kinds("clip")
    assert [e["state"] for e in clips] == ["started", "done"]
    assert clips[1]["stamp"] >= WALL0 + seen_until + 10.0 - 0.3
    clip_dir = os.path.join(rig.media, "clips", clips[0]["clip_id"])
    with open(os.path.join(clip_dir, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)
    assert meta["frames"] == clips[1]["frames"] == len(os.listdir(
        os.path.join(clip_dir, "frames")))
    assert meta["end_epoch"] > WALL0 + seen_until + 9.0       # 종료 뒤 프레임이 들어 있다

    assert not rig.profile.draining and not rig.profile.wants_frames  # 프레임을 돌려준다
    assert rig.profile._clip.buffered == 0
    before = (rig.work(), len(rig.events))
    rig.run(3.0)
    assert (rig.work(), len(rig.events)) == before


def test_순찰이_끝나도_쓰던_소리_장면은_경보_10초_뒤까지_쓴다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(6.0)
    rig.state("patrol_alarm")                 # 소리 경보
    alarm_at = rig.t
    rig.run(2.0)
    rig.state("idle")                         # 경비 종료
    assert rig.profile.draining
    infers = rig.infers
    rig.confs = (0.9,)
    rig.run(7.5)
    assert rig.profile.draining
    rig.run(1.0)
    assert not rig.profile.wants_frames
    done = rig.kinds("clip")[-1]
    assert done["state"] == "done" and done["alarm"] == "sound"
    assert 70 <= done["frames"] <= 77         # 5 s 전 + 10 s 뒤 — 종료로 잘리지 않았다
    assert done["stamp"] >= WALL0 + alarm_at + 10.0 - 0.3
    assert rig.infers == infers               # 소리 장면은 추론하지 않는다


def test_순찰이_끝난_뒤_붙잡는_시간은_장면_상한_60초를_넘지_않는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.confs = (0.8,)                        # 사람이 끝까지 화면에 있다
    process_one(rig)
    first_frame = rig.t
    process_one(rig)                          # 경보 — 장면 시작
    rig.state("idle")                         # 곧바로 경비 종료
    left_at = rig.t
    rig.run(59.0)
    assert rig.profile.draining
    rig.run(1.5)
    assert not rig.profile.wants_frames
    done = rig.kinds("clip")[-1]
    assert done["state"] == "done"
    assert done["stamp"] <= WALL0 + first_frame + 60.0 + 0.3
    assert done["stamp"] - (WALL0 + left_at) <= 60.0


def test_마저_쓰는_동안에는_status_도_판정도_불_확인도_없다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(6.0)
    rig.state("patrol_alarm")                 # 소리 장면
    rig.run(1.0)
    rig.state("idle")
    events = len(rig.events)
    rig.confs = (0.9,)
    rig.run(5.0)
    assert rig.profile.draining
    assert len(rig.events) == events          # status·intruder·light 없음
    assert rig.exposure == []


def test_마저_쓰는_중_프레임이_끊겨도_1Hz_점검이_장면을_닫는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(6.0)
    rig.state("patrol_alarm")
    rig.state("idle")
    for i in range(1, 12):                    # 프레임 없이 점검만
        rig.profile.on_tick(ns(rig.t + i), WALL0 + rig.t + i)
    assert not rig.profile.wants_frames
    assert [e["state"] for e in rig.kinds("clip")] == ["started", "done"]


def test_마저_쓰는_중에_순찰이_다시_시작되면_같은_장면에_이어_쓴다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.confs = (0.8,)
    rig.run(1.0)
    rig.state("idle")                         # 경비 종료 — 장면을 마저 쓰는 중
    rig.run(1.0)
    rig.state("patrolling")                   # 다시 경비 시작
    assert rig.profile.active and not rig.profile.draining
    rig.run(1.0)
    intruder = rig.kinds("intruder")
    assert len(intruder) == 2                 # 새 순찰의 판정은 처음부터
    assert intruder[1]["clip_id"] == intruder[0]["clip_id"]
    assert [e["state"] for e in rig.kinds("clip")] == ["started"]     # 장면은 하나다
    rig.confs = ()
    rig.state("patrol_alarm")
    rig.run(11.0)
    assert [e["state"] for e in rig.kinds("clip")] == ["started", "done"]


def test_마저_쓰다_쓰기에_실패하면_곧바로_프레임을_놓는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(6.0)
    rig.state("patrol_alarm")
    clip_id = rig.kinds("clip")[0]["clip_id"]
    rig.state("idle")
    import shutil
    shutil.rmtree(os.path.join(rig.media, "clips", clip_id))          # 디스크가 사라졌다
    rig.run(1.0)
    assert not rig.profile.wants_frames
    assert any(level == "error" for level, _ in rig.logs)


def test_노드_종료는_쓰던_장면을_기다리지_않고_바로_닫는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.confs = (0.8,)
    rig.run(6.0)
    rig.state("patrol_alarm")
    rig.run(2.0)
    rig.profile.shutdown(ns(rig.t), WALL0 + rig.t)
    clips = rig.kinds("clip")
    assert [e["state"] for e in clips] == ["started", "done"]
    assert os.path.isfile(os.path.join(rig.media, "clips", clips[0]["clip_id"], "meta.json"))
    assert not rig.profile.wants_frames
    assert rig.profile._clip.buffered == 0


def test_마저_쓰는_중의_노드_종료도_바로_닫는다(tmp_path):
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.run(6.0)
    rig.state("patrol_alarm")
    rig.state("idle")
    assert rig.profile.draining
    rig.profile.shutdown(ns(rig.t), WALL0 + rig.t)
    assert [e["state"] for e in rig.kinds("clip")] == ["started", "done"]
    assert not rig.profile.wants_frames


def test_장면을_못_써도_경보는_낸다(tmp_path):
    blocker = tmp_path / "media"
    blocker.write_text("폴더 자리에 파일이 있다")
    rig = Rig(tmp_path)
    rig.state("patrolling")
    rig.confs = (0.8,)
    rig.run(1.0)
    intruder = rig.kinds("intruder")
    assert len(intruder) == 1 and intruder[0]["clip_id"] == ""
    assert rig.kinds("clip") == []
    assert any(level == "error" for level, _ in rig.logs)


# ---- 실측 회귀: 2026-11-11 밤 주행 중 프레임(T1~T6) -----------------------------
#
# 저장된 프레임별 사람 최고 conf 를 판정 규칙에 다시 넣는다(엔진·GPU 불필요).
# 기대값은 ~/vica_data/patrol_tests/2026-11-11_camera/README.txt 의 기록이다.
# 엔진을 돌리는 사진 회귀는 총괄이 컨테이너에서 따로 한다.

MOVE_FRAMES = os.path.expanduser(
    "~/vica_data/patrol_tests/2026-11-11_camera/analysis/move_frames.csv")


def _move_scenes():
    import csv
    if not os.path.isfile(MOVE_FRAMES):
        pytest.skip("실측 자료가 없다: %s" % MOVE_FRAMES)
    scenes = {}
    with open(MOVE_FRAMES, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            h, m, s = row["시각"].split(":")
            t = int(h) * 3600 + int(m) * 60 + float(s)
            scenes.setdefault(row["장면"], []).append((t, float(row["사람conf최고"])))
    return scenes


def _first_alarm_s(frames):
    judge = IntruderJudge()
    t0 = frames[0][0]
    for t, conf in frames:
        if judge.observe(ns(t - t0), conf) is not None:
            return t - t0
    return None


def test_실측_사람_없는_주행_301장에서는_경보가_없다():
    scenes = _move_scenes()
    assert len(scenes["T3"]) + len(scenes["T6"]) == 301
    assert _first_alarm_s(scenes["T3"]) is None       # 불 끔, 주행·회전
    assert _first_alarm_s(scenes["T6"]) is None       # 불 켬, 주행·회전


def test_실측_사람_있는_주행에서는_기록된_시각에_첫_경보가_난다():
    scenes = _move_scenes()
    assert _first_alarm_s(scenes["T1"]) == pytest.approx(0.6, abs=0.05)   # 불 끔 직진 0.49 m/s
    assert _first_alarm_s(scenes["T2"]) == pytest.approx(4.0, abs=0.05)   # 불 끔 회전
    assert _first_alarm_s(scenes["T4"]) < 1.0                              # 불 켬 직진
    assert _first_alarm_s(scenes["T5"]) < 1.0                              # 불 켬 회전
