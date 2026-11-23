"""순찰(경비) 프로필 판정과 처리 순서 (순수 로직, rclpy·ultralytics·cv2 비의존)."""
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


PATROL_PREFIX = "patrol"
STATE_PATROLLING = "patrolling"
STATE_DWELL = "patrol_dwell"
STATE_ALARM = "patrol_alarm"
STATE_LIGHT = "patrol_light"
STATE_PAUSED = "patrol_paused"
JUDGE_STATES = (STATE_PATROLLING, STATE_DWELL)
HOLD_STATES = (STATE_ALARM, STATE_LIGHT, STATE_PAUSED)


PERSON_MIN_CONF = 0.5
INTRUDER_WINDOW_S = 3.0
INTRUDER_HITS = 2

LIGHT_EXPOSURE = 166
LIGHT_DARK_BELOW = 10.0
LIGHT_SKIP_FRAMES = 5

CAMERA_OK_S = 3.0
STATUS_PERIOD_S = 1.0

EVENT_SRC = "detector"

WATCH_LIGHT = "light"
WATCH_KINDS = (ALARM_INTRUDER, WATCH_LIGHT, ALARM_SOUND)


EXPOSURE_ACK_TIMEOUT_S = 1.0
LIGHT_CHECK_TIMEOUT_S = 3.0
EXPOSURE_RETRY_S = 1.0
UNANSWERED_REARM_S = 5.0
INFER_FAIL_LIMIT = 3
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
        out["frames"] = int(frames)
    return out


@dataclass(frozen=True)
class IntruderVerdict:
    conf: float
    hits: int


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
            self._hits.popleft()
        if not conf >= self._min_conf:
            return None
        self._hits.append((now_ns, float(conf)))
        if len(self._hits) < self._need:
            return None
        self._fired = True
        return IntruderVerdict(conf=max(c for _, c in self._hits),
                               hits=len(self._hits))


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
    FIX = "fix"
    AUTO = "auto"


class FrameUse(Enum):
    NORMAL = "normal"
    SKIP = "skip"
    MEASURE = "measure"


class _Phase(Enum):
    IDLE = "idle"
    FIX_SENT = "fix_sent"
    FIX_SETTLE = "fix_settle"
    AUTO_SENT = "auto_sent"
    AUTO_SETTLE = "auto_settle"


class LightCheck:
    """노출 고정 → 5프레임 버림 → 밝기 → 자동 노출 복구 → 5프레임 버림."""

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
            return FrameUse.MEASURE
        if self._phase is _Phase.AUTO_SETTLE:
            if self._skip_left > 0:
                self._skip_left -= 1
                return FrameUse.SKIP
            self._phase = _Phase.IDLE
            return FrameUse.NORMAL
        return FrameUse.SKIP

    def on_measured(self, now_ns: int) -> Optional[ExposureCmd]:
        """밝기를 쟀다(또는 재다 실패했다) — 자동 노출로 되돌린다."""
        if self._phase is not _Phase.FIX_SETTLE:
            return None
        return self._restore(now_ns)

    def on_result(self, cmd: ExposureCmd, ok: bool, now_ns: int) -> Optional[ExposureCmd]:
        """카메라 매개변수 서비스의 응답. 이어서 보낼 요청이 있으면 돌려준다."""
        if cmd is ExposureCmd.FIX:
            if self._phase is not _Phase.FIX_SENT:
                return None
            if ok:
                self._phase = _Phase.FIX_SETTLE
                self._skip_left = self._skip_frames
                return None
            return self._restore(now_ns)
        if ok:
            self._dirty = False
            if self._phase is _Phase.AUTO_SENT:
                self._phase = _Phase.AUTO_SETTLE
                self._skip_left = self._skip_frames
        return None

    def on_not_sent(self, cmd: ExposureCmd) -> None:
        """요청을 보내지 못했다(서비스 없음)."""
        if cmd is ExposureCmd.FIX and self._phase is _Phase.FIX_SENT:
            self._phase = _Phase.IDLE
            self._dirty = False
        elif cmd is ExposureCmd.AUTO:
            self._auto_sent_ns = None
            if self._phase is _Phase.AUTO_SENT:
                self._phase = _Phase.IDLE

    def tick(self, now_ns: int) -> Optional[ExposureCmd]:
        """주기 점검. 되돌릴 것이 남았으면 AUTO 를 돌려준다."""
        if self._phase in (_Phase.FIX_SENT, _Phase.FIX_SETTLE):
            limit = self._ack_ns if self._phase is _Phase.FIX_SENT else self._hold_ns
            if now_ns - self._started_ns > limit:
                return self._restore(now_ns)
            return None
        if self._phase is _Phase.AUTO_SENT:
            if now_ns - self._auto_sent_ns <= self._ack_ns:
                return None
            self._phase = _Phase.IDLE
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


@dataclass
class PatrolPorts:
    """순찰 프로필이 바깥에 시키는 일. 노드 배선이 채운다."""

    publish: Callable[[Dict[str, object]], None]
    encode_jpeg: Callable[[object], bytes]
    brightness: Callable[[object], float]
    infer: Callable[[object], Sequence[float]]
    request_exposure: Callable[[ExposureCmd], bool]
    exposure_ready: Callable[[], bool]
    cleanup: Callable[[], None]
    log: Callable[[str, str], None]


class PatrolProfile:
    """순찰 프로필 한 벌 — 상태·프레임·주기 점검을 받아 판정하고 포트로 내보낸다."""

    def __init__(self, ports: PatrolPorts, media_dir: str, rate_hz: float = 5.0) -> None:
        self._p = ports
        self._media_dir = media_dir
        self._rate_hz = rate_hz
        self._period_ns = sec_to_ns(max(0.0, 1.0 / rate_hz - PERIOD_SLACK_S))
        self._camera_ok_ns = sec_to_ns(CAMERA_OK_S)
        self._rearm_ns = sec_to_ns(UNANSWERED_REARM_S)
        self._light = LightCheck()
        self._active = False
        self._engine_loaded = False
        self._watch_intruder = True
        self._watch_light = True
        self._watch_sound = True
        self._reset()

    def _reset(self) -> None:
        self._state = ""
        self._draining = False
        self._infer_broken = False
        self._judge = IntruderJudge()
        self._arming = LightArming()
        self._clip = ClipRecorder(self._media_dir, self._rate_hz)
        self._frame_ns: Optional[int] = None
        self._proc_ns: Optional[int] = None
        self._light_due = False
        self._photo_brightness: Optional[float] = None
        self._intruder_sent_ns: Optional[int] = None
        self._light_sent_ns: Optional[int] = None
        self._infer_fails = 0

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
            self._judge.rearm()
            self._intruder_sent_ns = None
        if not self._watch_light:
            self._light_due = False
            self._photo_brightness = None
            self._light_sent_ns = None
            self._send(self._light.abort(now_ns), now_ns)
        elif light_was_off and self._active and self._state == STATE_DWELL:
            self._light_due = True
        if not self._keeps_frames and not self._clip.recording:
            self._clip.clear()

    @property
    def _keeps_frames(self) -> bool:
        """5 Hz 장면을 모아 둘 이유가 있다 — 사람·큰 소리 경보의 영상 앞 5 s."""
        return self._watch_intruder or self._watch_sound

    def on_dialog_state(self, state: str, now_ns: int, wall_s: float) -> None:
        if not is_patrol_state(state):
            self.leave(now_ns, wall_s)
            return
        if not self._active:
            open_clip = self._clip if self._draining else None
            self._reset()
            if open_clip is not None:
                self._clip = open_clip
            self._active = True
            self._p.cleanup()
        prev = self._state
        if state == prev:
            return
        self._state = state
        if state in JUDGE_STATES and prev in HOLD_STATES:
            self._judge.rearm()
            self._intruder_sent_ns = None
            if self._light_sent_ns is not None:
                self._light_sent_ns = None
                self._arming.rearm()
        self._light_due = state == STATE_DWELL and self._watch_light
        if state == STATE_ALARM:
            self._on_alarm_state(now_ns, wall_s)
        elif state == STATE_LIGHT:
            self._light_sent_ns = None

    def leave(self, now_ns: int, wall_s: float) -> None:
        """순찰 프로필에서 나간다(순찰 끝·상태 끊김). 쓰던 장면은 끝 조건까지 마저 쓴다."""
        if not self._active:
            return
        self._active = False
        self._state = ""
        self._light_due = False
        self._photo_brightness = None
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
            return
        if not self._watch_sound:
            return
        if self._clip.recording:
            self._clip.extend(now_ns)
            return
        if not self._camera_ok(now_ns):
            return
        self._start_clip(ALARM_SOUND, now_ns, wall_s)

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
            self._arming.rearm()
        photo_due = self._photo_brightness is not None
        keeping = (judging and self._keeps_frames) or self._clip.recording
        if not keeping and not photo_due:
            return

        if (not photo_due and self._proc_ns is not None
                and now_ns - self._proc_ns < self._period_ns):
            return
        self._proc_ns = now_ns
        img = get_img()
        jpeg = self._p.encode_jpeg(img)
        if photo_due:
            self._emit_light(jpeg, wall_s)
        if not keeping:
            return

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
            finally:
                self._send(self._light.on_measured(now_ns), now_ns)
            self._on_brightness(value)
        return False

    def on_error(self, now_ns: int) -> None:
        """프레임 처리 중 예외 — 노출이 고정된 채 남지 않게 한다."""
        self._send(self._light.abort(now_ns), now_ns)

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
        self._close_if_due(now_ns, wall_s)

    def on_exposure_result(self, cmd: ExposureCmd, ok: bool, now_ns: int) -> None:
        if not ok:
            self._p.log("warn", "카메라 노출 요청 실패: %s" % cmd.value)
        self._send(self._light.on_result(cmd, ok, now_ns), now_ns)

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
            return
        if self._arming.observe(value):
            self._photo_brightness = value

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
            self._clip.follow_person(now_ns)
            clip_id = self._clip.clip_id
        else:
            clip_id = self._start_clip(ALARM_INTRUDER, now_ns, wall_s, announce=False)
            started = bool(clip_id)
        self._intruder_sent_ns = now_ns
        self._p.publish(event_intruder(wall_s, verdict.conf, verdict.hits, clip_id))
        if started:
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
        self._p.cleanup()

    def _close_if_due(self, now_ns: int, wall_s: float) -> None:
        if self._clip.due(now_ns):
            self._finish_clip(wall_s)
        if self._draining and not self._clip.recording:
            self._draining = False
            self._clip.clear()
