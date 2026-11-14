"""순찰(경비) 프로필 판정과 처리 순서 (순수 로직, rclpy·ultralytics·cv2 비의존).

계약 정본은 `~/vica_data/patrol_plan/SPEC.md` 2·3.1·4.B 다. 이름·값·JSON 키는
미션·앱과 공유하므로 바꾸지 않는다.

순찰 프로필은 `/vica/robot_state` 의 `dialog_state` 가 `patrol` 로 시작하는 동안이다.
그동안 이 모듈이 정하는 것:

    사람 판정    conf ≥ 0.5 프레임이 3.0 s 안에 2번 → `intruder` 1번.
                 재개하면 판정 창을 비우고 곧바로 다시 센다(쉬는 시간 없음).
    불 켜짐      `patrol_dwell` 에 들어올 때마다 노출 고정 166 → 5프레임 버림 → 평균 밝기
                 → 자동 노출 복구 → 5프레임 버림. 어두움(< 10)을 본 뒤 밝으면 `light` 1번.
    영상 장면    처리 프레임(5 Hz)을 JPEG 로 최근 5 s 만 메모리에. 경보가 나면 디스크로.
                 순찰이 끝나도 쓰던 장면은 끝 조건까지 마저 쓴다(최대 60 s).
    상태         `status` 1 Hz.

멈춘 상태(`patrol_alarm`·`patrol_light`·`patrol_paused`)에서 쓰는 장면이 없으면 영상을
풀지도 압축하지도 추론하지도 않는다 — 관리자를 기다리는 시간이 길 수 있다.

관리자가 꺼 둔 감시 항목(`/vica/patrol/watch`, 2026-11-14)은 계산하지 않는다:

    사람 끔            추론 없음(순찰 엔진도 올리지 않는다). `status` 의 ready 는 엔진 없이 true
    불 켜짐 끔         노출을 건드리지 않는다
    사람·큰 소리 끔    5 Hz 장면 모으기 없음(불 켜짐 사진 한 장만 그때 압축한다)
    큰 소리 끔         미션이 낸 경보에 소리 장면을 찍지 않는다

카메라 끊김 감시(`status` 의 camera_ok)는 끌 수 없다.

모델·카메라·디스크·발행은 `PatrolPorts` 로 받는다. 그래서 GPU 없이 시험이 돈다.

시간 계약: 단조 시각은 호출자의 STEADY clock 정수 나노초, 벽시계는 epoch 초.
이 모듈은 시계를 직접 읽지 않는다(`inference_gate` 와 같다).
"""
from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Deque, Dict, FrozenSet, Iterable, Optional, Sequence, Tuple

from vica_perception.inference_gate import sec_to_ns
from vica_perception.patrol_media import (
    ALARM_INTRUDER,
    ALARM_SOUND,
    ClipRecorder,
    Frame,
    save_photo,
)

# ---- 상태 값 (SPEC 2절) -------------------------------------------------------

PATROL_PREFIX = "patrol"
STATE_PATROLLING = "patrolling"
STATE_DWELL = "patrol_dwell"
STATE_ALARM = "patrol_alarm"
STATE_LIGHT = "patrol_light"
STATE_PAUSED = "patrol_paused"
# 미션이 경보를 받는 상태. 판정도 이 상태에서만 한다.
JUDGE_STATES = (STATE_PATROLLING, STATE_DWELL)
# 멈춰서 관리자를 기다리는 상태. 여기서 판정 상태로 돌아오면 '재개'다.
HOLD_STATES = (STATE_ALARM, STATE_LIGHT, STATE_PAUSED)

# ---- 값 (SPEC 4.B) ------------------------------------------------------------

PERSON_MIN_CONF = 0.5
INTRUDER_WINDOW_S = 3.0
INTRUDER_HITS = 2

LIGHT_EXPOSURE = 166                  # rgb_camera.exposure
LIGHT_DARK_BELOW = 10.0               # 이 밝기 미만이 '어두움'
LIGHT_SKIP_FRAMES = 5                 # 노출 전환 뒤 버리는 프레임(카메라 원본 기준)

CAMERA_OK_S = 3.0                     # 이 안에 컬러 프레임을 받았으면 camera_ok
STATUS_PERIOD_S = 1.0

EVENT_SRC = "detector"

# 관리자가 켜고 끄는 감시 항목 — 미션의 PATROL_WATCH_KINDS 와 같은 이름이다.
WATCH_LIGHT = "light"
WATCH_KINDS = (ALARM_INTRUDER, WATCH_LIGHT, ALARM_SOUND)

# ---- 명세에 없는 내부 한도 ----------------------------------------------------

# 노출 요청의 응답을 기다리는 한도. 실측 응답 8~15 ms(2026-11-11).
EXPOSURE_ACK_TIMEOUT_S = 1.0
# 노출을 고정한 채 머무는 한도. 넘으면 재지 않고 자동 노출로 되돌린다.
LIGHT_CHECK_TIMEOUT_S = 3.0
# 자동 노출 복구가 확인될 때까지 다시 요청하는 간격.
EXPOSURE_RETRY_S = 1.0
# 경보를 냈는데 미션 상태가 이 시간 안에 안 바뀌면 다시 무장한다(이벤트 유실 대비).
UNANSWERED_REARM_S = 5.0
# 추론이 이만큼 잇따라 실패하면 ready 를 내린다.
INFER_FAIL_LIMIT = 3
# 5 Hz 솎기 여유. 15 Hz 영상의 3프레임(0.200 s)이 흔들려도 3프레임마다 처리한다.
PERIOD_SLACK_S = 0.01


def is_patrol_state(dialog_state: str) -> bool:
    """순찰 프로필로 돌아야 하는 상태인가."""
    return bool(dialog_state) and dialog_state.startswith(PATROL_PREFIX)


def parse_watch_off(text: str) -> Optional[FrozenSet[str]]:
    """`/vica/patrol/watch` 의 JSON(`{"off": ["light"]}`) → 꺼 둔 항목. 못 읽으면 None(지금 값을 그대로 둔다)."""
    try:
        off = json.loads(text).get("off")
    except (AttributeError, TypeError, ValueError):
        return None
    if not isinstance(off, list):
        return None
    return frozenset(kind for kind in WATCH_KINDS if kind in off)


# ---- 이벤트 (SPEC 3.1) --------------------------------------------------------


def event_status(stamp: float, ready: bool, camera_ok: bool) -> Dict[str, object]:
    return {"src": EVENT_SRC, "kind": "status", "stamp": stamp,
            "ready": bool(ready), "camera_ok": bool(camera_ok)}


def event_intruder(stamp: float, conf: float, hits: int, clip_id: str) -> Dict[str, object]:
    return {"src": EVENT_SRC, "kind": "intruder", "stamp": stamp,
            "conf": round(float(conf), 3), "hits": int(hits), "clip_id": clip_id}


def event_light(stamp: float, brightness: float, photo: str) -> Dict[str, object]:
    return {"src": EVENT_SRC, "kind": "light", "stamp": stamp,
            "brightness": round(float(brightness), 1), "photo": photo}


def event_clip(stamp: float, clip_id: str, state: str, alarm: str,
               frames: Optional[int] = None) -> Dict[str, object]:
    out: Dict[str, object] = {"src": EVENT_SRC, "kind": "clip", "stamp": stamp,
                              "clip_id": clip_id, "state": state, "alarm": alarm}
    if frames is not None:
        out["frames"] = int(frames)           # done 일 때만
    return out


# ---- 사람 판정 ----------------------------------------------------------------


@dataclass(frozen=True)
class IntruderVerdict:
    conf: float                       # 창 안 프레임 중 가장 높은 conf
    hits: int                         # 창 안 conf ≥ 문턱 프레임 수


class IntruderJudge:
    """conf ≥ 0.5 프레임이 3.0 s 안에 2번이면 한 번 알린다."""

    def __init__(
        self,
        min_conf: float = PERSON_MIN_CONF,
        window_s: float = INTRUDER_WINDOW_S,
        need_hits: int = INTRUDER_HITS,
    ) -> None:
        self._min_conf = min_conf
        self._window_ns = sec_to_ns(window_s)
        self._need = need_hits
        self._hits: Deque[Tuple[int, float]] = deque()
        self._fired = False

    @property
    def fired(self) -> bool:
        """알린 뒤 아직 다시 무장하지 않았다."""
        return self._fired

    def rearm(self) -> None:
        """다시 무장한다(재개·미션 무반응). 판정 창은 비워서 새로 센다."""
        self._hits.clear()
        self._fired = False

    def observe(self, now_ns: int, conf: float) -> Optional[IntruderVerdict]:
        """처리한 프레임 한 장의 사람 최고 conf(없으면 0). 판정이 서면 한 번만 돌려준다."""
        if self._fired:
            return None
        while self._hits and not (
                0 <= now_ns - self._hits[0][0] <= self._window_ns):
            self._hits.popleft()              # 창 밖(또는 시간 역전)
        if not conf >= self._min_conf:        # NaN 도 여기서 걸린다
            return None
        self._hits.append((now_ns, float(conf)))
        if len(self._hits) < self._need:
            return None
        self._fired = True
        return IntruderVerdict(conf=max(c for _, c in self._hits),
                               hits=len(self._hits))


# ---- 불 켜짐 ------------------------------------------------------------------


class LightArming:
    """어두움을 한 번 본 뒤에만 '밝음'을 경보로 본다. 경보 뒤에는 무장 해제."""

    def __init__(self, dark_below: float = LIGHT_DARK_BELOW) -> None:
        self._dark_below = dark_below
        self._armed = False

    @property
    def armed(self) -> bool:
        return self._armed

    def rearm(self) -> None:
        """경보가 미션에 닿지 않았을 때 — 다음 확인에서 다시 알린다."""
        self._armed = True

    def observe(self, brightness: float) -> bool:
        """노출 고정 평균 밝기 한 번. True 면 불 켜짐 경보를 낸다."""
        if brightness < self._dark_below:
            self._armed = True
            return False
        if not self._armed:
            return False
        self._armed = False
        return True


class ExposureCmd(Enum):
    FIX = "fix"                       # rgb_camera.exposure = 166 (자동 노출이 꺼진다)
    AUTO = "auto"                     # rgb_camera.enable_auto_exposure = true


class FrameUse(Enum):
    NORMAL = "normal"                 # 평소 처리(솎기·추론)
    SKIP = "skip"                     # 버린다 — 노출이 바뀌는 중
    MEASURE = "measure"               # 이 프레임으로 밝기를 잰다


class _Phase(Enum):
    IDLE = "idle"
    FIX_SENT = "fix_sent"             # 고정 요청을 보내고 응답 대기
    FIX_SETTLE = "fix_settle"         # 고정됨 — 프레임 버리는 중, 다 버리면 잰다
    AUTO_SENT = "auto_sent"           # 자동 복구 요청을 보내고 응답 대기
    AUTO_SETTLE = "auto_settle"       # 복구됨 — 프레임 버리는 중


class LightCheck:
    """노출 고정 → 5프레임 버림 → 밝기 → 자동 노출 복구 → 5프레임 버림.

    `dirty` 는 고정 요청을 보낸 뒤 복구 성공 응답을 아직 못 받았다는 뜻이다.
    dirty 인 동안 `tick` 이 복구 요청을 다시 낸다 — 어떤 경로로 빠져나가도
    자동 노출로 돌아가게 하려는 것이다.
    """

    def __init__(
        self,
        skip_frames: int = LIGHT_SKIP_FRAMES,
        ack_timeout_s: float = EXPOSURE_ACK_TIMEOUT_S,
        hold_timeout_s: float = LIGHT_CHECK_TIMEOUT_S,
        retry_s: float = EXPOSURE_RETRY_S,
    ) -> None:
        self._skip_frames = skip_frames
        self._ack_ns = sec_to_ns(ack_timeout_s)
        self._hold_ns = sec_to_ns(hold_timeout_s)
        self._retry_ns = sec_to_ns(retry_s)
        self._phase = _Phase.IDLE
        self._dirty = False
        self._skip_left = 0
        self._started_ns = 0
        self._auto_sent_ns: Optional[int] = None

    @property
    def busy(self) -> bool:
        """확인 중이다(이 동안 추론·장면 프레임을 쉰다)."""
        return self._phase is not _Phase.IDLE

    @property
    def dirty(self) -> bool:
        return self._dirty

    def start(self, now_ns: int) -> bool:
        """확인을 시작한다. True 면 호출자가 FIX 를 보낸다."""
        if self._phase is not _Phase.IDLE or self._dirty:
            return False
        self._phase = _Phase.FIX_SENT
        self._dirty = True
        self._started_ns = now_ns
        self._auto_sent_ns = None
        return True

    def on_frame(self, now_ns: int) -> FrameUse:
        """카메라 프레임 한 장을 어떻게 쓸지."""
        if self._phase is _Phase.IDLE:
            return FrameUse.NORMAL
        if self._phase is _Phase.FIX_SETTLE:
            if self._skip_left > 0:
                self._skip_left -= 1
                return FrameUse.SKIP
            return FrameUse.MEASURE           # 호출자가 재고 on_measured 를 부른다
        if self._phase is _Phase.AUTO_SETTLE:
            if self._skip_left > 0:
                self._skip_left -= 1
                return FrameUse.SKIP
            self._phase = _Phase.IDLE
            return FrameUse.NORMAL
        return FrameUse.SKIP                  # 응답 대기 중

    def on_measured(self, now_ns: int) -> Optional[ExposureCmd]:
        """밝기를 쟀다(또는 재다 실패했다) — 자동 노출로 되돌린다."""
        if self._phase is not _Phase.FIX_SETTLE:
            return None
        return self._restore(now_ns)

    def on_result(self, cmd: ExposureCmd, ok: bool, now_ns: int) -> Optional[ExposureCmd]:
        """카메라 매개변수 서비스의 응답. 이어서 보낼 요청이 있으면 돌려준다."""
        if cmd is ExposureCmd.FIX:
            if self._phase is not _Phase.FIX_SENT:
                return None                   # 늦은 응답 — 이미 복구 쪽으로 갔다
            if ok:
                self._phase = _Phase.FIX_SETTLE
                self._skip_left = self._skip_frames
                return None
            return self._restore(now_ns)      # 거절돼도 반쯤 적용됐을 수 있다
        if ok:
            self._dirty = False
            if self._phase is _Phase.AUTO_SENT:
                self._phase = _Phase.AUTO_SETTLE
                self._skip_left = self._skip_frames
        return None                           # 실패는 tick 이 다시 보낸다

    def on_not_sent(self, cmd: ExposureCmd) -> None:
        """요청을 보내지 못했다(서비스 없음)."""
        if cmd is ExposureCmd.FIX and self._phase is _Phase.FIX_SENT:
            self._phase = _Phase.IDLE         # 노출을 건드린 적이 없다
            self._dirty = False
        elif cmd is ExposureCmd.AUTO:
            self._auto_sent_ns = None         # tick 이 곧 다시 보낸다
            if self._phase is _Phase.AUTO_SENT:
                self._phase = _Phase.IDLE

    def tick(self, now_ns: int) -> Optional[ExposureCmd]:
        """주기 점검. 되돌릴 것이 남았으면 AUTO 를 돌려준다."""
        if self._phase in (_Phase.FIX_SENT, _Phase.FIX_SETTLE):
            limit = self._ack_ns if self._phase is _Phase.FIX_SENT else self._hold_ns
            if now_ns - self._started_ns > limit:
                return self._restore(now_ns)  # 응답·프레임이 안 온다 — 재지 않고 복구
            return None
        if self._phase is _Phase.AUTO_SENT:
            if now_ns - self._auto_sent_ns <= self._ack_ns:
                return None
            self._phase = _Phase.IDLE         # 응답을 더 기다리지 않고 추론을 재개한다
        if self._dirty and (self._auto_sent_ns is None
                            or now_ns - self._auto_sent_ns > self._retry_ns):
            self._auto_sent_ns = now_ns
            return ExposureCmd.AUTO
        return None

    def abort(self, now_ns: int) -> Optional[ExposureCmd]:
        """확인을 그만둔다(프로필 이탈·예외·종료). 되돌릴 것이 있으면 AUTO."""
        holding = self._phase in (_Phase.FIX_SENT, _Phase.FIX_SETTLE)
        self._phase = _Phase.IDLE
        self._skip_left = 0
        if self._dirty and (holding or self._auto_sent_ns is None):
            self._auto_sent_ns = now_ns
            return ExposureCmd.AUTO
        return None

    def _restore(self, now_ns: int) -> ExposureCmd:
        self._phase = _Phase.AUTO_SENT
        self._auto_sent_ns = now_ns
        return ExposureCmd.AUTO


# ---- 처리 순서 ----------------------------------------------------------------


@dataclass
class PatrolPorts:
    """순찰 프로필이 바깥에 시키는 일. 노드 배선이 채운다."""

    publish: Callable[[Dict[str, object]], None]        # /vica/patrol/event
    encode_jpeg: Callable[[object], bytes]              # BGR 영상 → JPEG
    brightness: Callable[[object], float]               # BGR 영상 → 평균 밝기
    infer: Callable[[object], Sequence[float]]          # BGR 영상 → 사람 conf 목록
    request_exposure: Callable[[ExposureCmd], bool]     # 보냈으면 True
    exposure_ready: Callable[[], bool]                  # 카메라 매개변수 서비스가 있다
    cleanup: Callable[[], None]                         # 저장 폴더 정리를 시킨다
    log: Callable[[str, str], None]                     # ("info"|"warn"|"error", 글)


class PatrolProfile:
    """순찰 프로필 한 벌 — 상태·프레임·주기 점검을 받아 판정하고 포트로 내보낸다."""

    def __init__(self, ports: PatrolPorts, media_dir: str, rate_hz: float = 5.0) -> None:
        self._p = ports
        self._media_dir = media_dir
        self._rate_hz = rate_hz
        self._period_ns = sec_to_ns(max(0.0, 1.0 / rate_hz - PERIOD_SLACK_S))
        self._camera_ok_ns = sec_to_ns(CAMERA_OK_S)
        self._rearm_ns = sec_to_ns(UNANSWERED_REARM_S)
        self._light = LightCheck()            # 노출 복구는 프로필 이탈 뒤에도 이어진다
        self._active = False
        self._engine_loaded = False           # 순찰 엔진이 올라왔다(배선이 알린다)
        # 감시 항목. 미션이 꺼 뒀다고 알린 것만 False — 알림을 못 받았으면 모두 켬이다.
        self._watch_intruder = True
        self._watch_light = True
        self._watch_sound = True
        self._reset()

    def _reset(self) -> None:
        self._state = ""
        self._draining = False                # 순찰은 끝났고 쓰던 장면을 마저 쓰는 중
        self._infer_broken = False            # 추론이 잇따라 실패했다(이번 순찰 동안)
        self._judge = IntruderJudge()
        self._arming = LightArming()
        self._clip = ClipRecorder(self._media_dir, self._rate_hz)
        self._frame_ns: Optional[int] = None          # 마지막 컬러 프레임
        self._proc_ns: Optional[int] = None           # 마지막 처리(5 Hz)
        self._light_due = False                       # 이번 dwell 의 불 확인을 아직 안 했다
        self._photo_brightness: Optional[float] = None    # 사진을 찍고 light 를 낼 차례
        self._intruder_sent_ns: Optional[int] = None  # intruder 를 내고 미션 반응 대기
        self._light_sent_ns: Optional[int] = None
        self._infer_fails = 0

    # ---- 조회 ---------------------------------------------------------------

    @property
    def active(self) -> bool:
        """순찰 프로필로 돌고 있다."""
        return self._active

    @property
    def draining(self) -> bool:
        """순찰은 끝났지만 쓰던 장면을 끝 조건까지 마저 쓰는 중이다."""
        return self._draining

    @property
    def wants_frames(self) -> bool:
        """컬러 프레임을 받아야 한다(순찰 중이거나 쓰던 장면이 남았다)."""
        return self._active or self._draining

    @property
    def state(self) -> str:
        return self._state

    @property
    def exposure_dirty(self) -> bool:
        """자동 노출 복구가 아직 확인되지 않았다."""
        return self._light.dirty

    @property
    def engine_ready(self) -> bool:
        """순찰 엔진이 올라와 있고 추론이 된다."""
        return self._engine_loaded and not self._infer_broken

    @property
    def needs_engine(self) -> bool:
        """사람 감지가 켜져 있다 — 꺼 두면 순찰 엔진을 올리지 않는다."""
        return self._watch_intruder

    @property
    def ready(self) -> bool:
        """`status` 의 ready — 사람 감지를 꺼 뒀으면 엔진 없이도 준비된 것이다."""
        return not self._watch_intruder or self.engine_ready

    def set_engine_loaded(self, loaded: bool) -> None:
        self._engine_loaded = bool(loaded)

    # ---- 입력: 감시 항목 -----------------------------------------------------

    def set_watch(self, off: Iterable[str], now_ns: int) -> None:
        """미션이 알린 꺼 둔 감시 항목. 값이 바뀔 때만 일한다(순찰 사이에도 값은 남는다)."""
        off = frozenset(off)
        watch = tuple(kind not in off for kind in WATCH_KINDS)
        if watch == (self._watch_intruder, self._watch_light, self._watch_sound):
            return
        light_was_off = not self._watch_light
        self._watch_intruder, self._watch_light, self._watch_sound = watch
        self._p.log("info", "순찰 감시 항목 — 끔: %s" % (
            ", ".join(kind for kind in WATCH_KINDS if kind in off) or "없음"))
        if not self._watch_intruder:
            self._judge.rearm()               # 세던 판정 창을 버린다
            self._intruder_sent_ns = None
        if not self._watch_light:
            self._light_due = False
            self._photo_brightness = None
            self._light_sent_ns = None
            self._send(self._light.abort(now_ns), now_ns)   # 확인 중이었으면 자동 노출로
        elif light_was_off and self._active and self._state == STATE_DWELL:
            self._light_due = True            # 알림이 상태보다 늦게 왔다 — 이번 dwell 부터 확인한다
        if not self._keeps_frames and not self._clip.recording:
            self._clip.clear()                # 모아 둔 5 s 를 버린다

    @property
    def _keeps_frames(self) -> bool:
        """5 Hz 장면을 모아 둘 이유가 있다 — 사람·큰 소리 경보의 영상 앞 5 s."""
        return self._watch_intruder or self._watch_sound

    # ---- 입력: 미션 상태 -----------------------------------------------------

    def on_dialog_state(self, state: str, now_ns: int, wall_s: float) -> None:
        if not is_patrol_state(state):
            self.leave(now_ns, wall_s)
            return
        if not self._active:
            open_clip = self._clip if self._draining else None
            self._reset()
            if open_clip is not None:
                self._clip = open_clip        # 마저 쓰던 장면은 이어서 쓴다
            self._active = True
            self._p.cleanup()                 # 순찰 시작 때 정리
        prev = self._state
        if state == prev:
            return
        self._state = state
        if state in JUDGE_STATES and prev in HOLD_STATES:
            # 재개 — 쉬는 시간 없이 곧바로 판정한다(2026-11-14 결정). 창만 비운다.
            self._judge.rearm()
            self._intruder_sent_ns = None
            if self._light_sent_ns is not None:
                self._light_sent_ns = None    # 불 켜짐 경보가 다른 경보에 묻혔다
                self._arming.rearm()
        self._light_due = state == STATE_DWELL and self._watch_light
        if state == STATE_ALARM:
            self._on_alarm_state(now_ns, wall_s)
        elif state == STATE_LIGHT:
            self._light_sent_ns = None        # 내 불 켜짐 경보가 닿았다

    def leave(self, now_ns: int, wall_s: float) -> None:
        """순찰 프로필에서 나간다(순찰 끝·상태 끊김). 쓰던 장면은 끝 조건까지 마저 쓴다."""
        if not self._active:
            return
        self._active = False
        self._state = ""
        self._light_due = False
        self._photo_brightness = None
        # 앱 확인창 문구가 "녹화 중인 영상은 끝까지 저장합니다." 다.
        self._draining = self._clip.recording
        if not self._draining:
            self._clip.clear()
        try:
            self._send(self._light.abort(now_ns), now_ns)
        finally:
            if self._draining:
                self._p.log("info", "순찰이 끝났다 — 쓰던 영상 장면은 끝까지 저장한다")

    def shutdown(self, now_ns: int, wall_s: float) -> None:
        """노드 종료 — 쓰던 장면을 기다리지 않고 바로 닫는다."""
        self._active = False
        self._draining = False
        self._state = ""
        try:
            self._finish_clip(wall_s)
        finally:
            self._clip.clear()
            self._send(self._light.abort(now_ns), now_ns)

    def _on_alarm_state(self, now_ns: int, wall_s: float) -> None:
        mine = self._intruder_sent_ns is not None
        self._intruder_sent_ns = None
        if mine:
            return                            # 내가 낸 사람 경보 — 장면은 이미 쓰는 중
        if not self._watch_sound:
            return                            # 큰 소리를 꺼 뒀다 — 남은 것은 카메라 끊김 경보, 기록 없음
        if self._clip.recording:
            self._clip.extend(now_ns)         # 앞 장면이 안 끝났다 — 5 s 더
            return
        if not self._camera_ok(now_ns):
            return                            # 카메라 끊김 경보 — 기록 없음
        self._start_clip(ALARM_SOUND, now_ns, wall_s)

    # ---- 입력: 카메라 프레임 -------------------------------------------------

    def on_frame(self, get_img: Callable[[], object], width: int, height: int,
                 now_ns: int, wall_s: float) -> None:
        """이미 받는 컬러 프레임 한 장(카메라 원본 주기). 영상은 필요할 때만 푼다."""
        if not self.wants_frames:
            return
        self._frame_ns = now_ns
        if self._active and not self._light_frame(get_img, now_ns):
            return
        judging = self._active and self._state in JUDGE_STATES
        if self._photo_brightness is not None and not judging:
            self._photo_brightness = None
            self._arming.rearm()              # 그새 미션이 멈췄다 — 다음 확인에서 다시
        photo_due = self._photo_brightness is not None
        # 장면을 모으는 때: 판정 중이고 사람·큰 소리 중 하나라도 켜져 있거나, 쓰는 장면이 있다.
        keeping = (judging and self._keeps_frames) or self._clip.recording
        if not keeping and not photo_due:
            return                            # 할 일이 없다 — 영상을 풀지 않는다

        if (not photo_due and self._proc_ns is not None
                and now_ns - self._proc_ns < self._period_ns):
            return                            # 5 Hz 로 솎는다
        self._proc_ns = now_ns
        img = get_img()
        jpeg = self._p.encode_jpeg(img)
        if photo_due:
            self._emit_light(jpeg, wall_s)
        if not keeping:
            return                            # 불 켜짐 사진 한 장만 찍었다

        best = 0.0
        watching = judging and self._watch_intruder
        if self.engine_ready and (watching or self._clip.follows_person):
            best = self._infer(img)
            if best >= PERSON_MIN_CONF:
                self._clip.mark_person(now_ns)
        self._push(Frame(now_ns, wall_s, jpeg, width, height))
        if watching:
            verdict = self._judge.observe(now_ns, best)
            if verdict is not None:
                self._on_intruder(verdict, now_ns, wall_s)
        self._close_if_due(now_ns, wall_s)

    def _light_frame(self, get_img: Callable[[], object], now_ns: int) -> bool:
        """불 켜짐 확인에 이 프레임을 쓴다. True 면 평소 처리로 넘어간다."""
        if (self._light_due and not self._light.busy
                and self._p.exposure_ready() and self._light.start(now_ns)):
            self._light_due = False
            self._send(ExposureCmd.FIX, now_ns)
        use = self._light.on_frame(now_ns)
        if use is FrameUse.NORMAL:
            return True
        if use is FrameUse.MEASURE:
            value: Optional[float] = None
            try:
                value = float(self._p.brightness(get_img()))
            finally:                          # 재든 못 재든 자동 노출로 되돌린다
                self._send(self._light.on_measured(now_ns), now_ns)
            self._on_brightness(value)
        return False

    def on_error(self, now_ns: int) -> None:
        """프레임 처리 중 예외 — 노출이 고정된 채 남지 않게 한다."""
        self._send(self._light.abort(now_ns), now_ns)

    # ---- 입력: 1 Hz 점검 -----------------------------------------------------

    def on_tick(self, now_ns: int, wall_s: float) -> None:
        self._send(self._light.tick(now_ns), now_ns)
        if self._active:
            self._p.publish(event_status(
                wall_s, self.ready, self._camera_ok(now_ns)))
        if self._active and self._state in JUDGE_STATES:
            if self._unanswered(self._intruder_sent_ns, now_ns):
                self._intruder_sent_ns = None
                self._judge.rearm()
                self._p.log("warn", "사람 경보에 미션 반응이 없다 — 다시 무장")
            if self._unanswered(self._light_sent_ns, now_ns):
                self._light_sent_ns = None
                self._arming.rearm()
                self._p.log("warn", "불 켜짐 경보에 미션 반응이 없다 — 다시 무장")
        self._close_if_due(now_ns, wall_s)    # 프레임이 끊겨도 장면은 닫는다

    # ---- 입력: 노출 요청 응답 ------------------------------------------------

    def on_exposure_result(self, cmd: ExposureCmd, ok: bool, now_ns: int) -> None:
        if not ok:
            self._p.log("warn", "카메라 노출 요청 실패: %s" % cmd.value)
        self._send(self._light.on_result(cmd, ok, now_ns), now_ns)

    # ---- 내부 ---------------------------------------------------------------

    def _camera_ok(self, now_ns: int) -> bool:
        return (self._frame_ns is not None
                and 0 <= now_ns - self._frame_ns <= self._camera_ok_ns)

    def _unanswered(self, sent_ns: Optional[int], now_ns: int) -> bool:
        return sent_ns is not None and now_ns - sent_ns > self._rearm_ns

    def _send(self, cmd: Optional[ExposureCmd], now_ns: int) -> None:
        if cmd is None:
            return
        sent = False
        try:
            sent = bool(self._p.request_exposure(cmd))
        finally:
            if not sent:
                self._light.on_not_sent(cmd)

    def _infer(self, img: object) -> float:
        try:
            confs = self._p.infer(img)
        except Exception as e:                # noqa: BLE001 — 추론 실패로 노드가 죽으면 안 된다
            self._infer_fails += 1
            if self._infer_fails >= INFER_FAIL_LIMIT:
                self._infer_broken = True
                self._p.log("error", "순찰 추론이 %d번 잇따라 실패 — ready=false: %s"
                            % (self._infer_fails, e))
            return 0.0
        self._infer_fails = 0
        return max((float(c) for c in confs), default=0.0)

    def _on_brightness(self, value: Optional[float]) -> None:
        if value is None or self._state not in JUDGE_STATES:
            return                            # 미션이 받지 않는 상태 — 무장도 그대로 둔다
        if self._arming.observe(value):
            self._photo_brightness = value    # 자동 노출이 돌아온 프레임을 사진으로

    def _emit_light(self, jpeg: bytes, wall_s: float) -> None:
        brightness = self._photo_brightness
        self._photo_brightness = None
        photo = ""
        try:
            photo = save_photo(self._media_dir, wall_s, jpeg)
        except OSError as e:
            self._p.log("error", "불 켜짐 사진 저장 실패: %s" % e)
        self._light_sent_ns = self._proc_ns
        self._p.publish(event_light(wall_s, brightness, photo))

    def _on_intruder(self, verdict: IntruderVerdict, now_ns: int, wall_s: float) -> None:
        started = False
        if self._clip.recording:
            self._clip.follow_person(now_ns)  # 쓰던 장면에 잇는다
            clip_id = self._clip.clip_id
        else:
            clip_id = self._start_clip(ALARM_INTRUDER, now_ns, wall_s, announce=False)
            started = bool(clip_id)
        self._intruder_sent_ns = now_ns
        self._p.publish(event_intruder(wall_s, verdict.conf, verdict.hits, clip_id))
        if started:                           # 미션이 경보를 먼저 받게 순서를 지킨다
            self._p.publish(event_clip(wall_s, clip_id, "started", ALARM_INTRUDER))

    def _start_clip(self, alarm: str, now_ns: int, wall_s: float,
                    announce: bool = True) -> str:
        try:
            clip_id = self._clip.start(alarm, now_ns, wall_s)
        except OSError as e:
            self._clip.abandon()
            self._p.log("error", "영상 장면 시작 실패: %s" % e)
            return ""
        if announce:
            self._p.publish(event_clip(wall_s, clip_id, "started", alarm))
        return clip_id

    def _push(self, frame: Frame) -> None:
        try:
            self._clip.push(frame)
        except OSError as e:
            self._clip.abandon()
            self._p.log("error", "영상 장면 쓰기 실패 — 장면을 버린다: %s" % e)

    def _finish_clip(self, wall_s: float) -> None:
        if not self._clip.recording:
            return
        try:
            done = self._clip.finish()
        except OSError as e:
            self._clip.abandon()
            self._p.log("error", "영상 장면 닫기 실패: %s" % e)
            return
        self._p.publish(event_clip(
            wall_s, done.clip_id, "done", done.alarm, frames=done.frames))
        self._p.cleanup()                     # 장면을 끝낼 때 정리

    def _close_if_due(self, now_ns: int, wall_s: float) -> None:
        if self._clip.due(now_ns):
            self._finish_clip(wall_s)
        if self._draining and not self._clip.recording:
            self._draining = False            # 마저 쓰기 끝 — 프레임을 기존 경로로 돌려준다
            self._clip.clear()
