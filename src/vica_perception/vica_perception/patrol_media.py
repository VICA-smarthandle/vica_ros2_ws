"""순찰 영상 장면·사진·저장 폴더 정리 (rclpy·ultralytics·cv2 비의존).

계약 정본은 `~/vica_data/patrol_plan/SPEC.md` 4.B 다. 폴더 구조와 `meta.json`
키는 지도 HTTP 서버와 앱이 그대로 읽으므로 바꾸지 않는다.

    <media_dir>/clips/<clip_id>/frames/000001.jpg …   장면 프레임(JPEG)
    <media_dir>/clips/<clip_id>/meta.json             장면이 끝나면 쓴다
    <media_dir>/photos/<YYYYMMDD_HHMMSS>_light.jpg    불 켜짐 사진
    <media_dir>/events.jsonl                          미션이 쓴다 — 여기서 안 건드린다

디스크에는 경보가 났을 때만 쓴다. 평소에는 최근 5 s(25장)만 메모리에 둔다.

시간 계약: 단조 시각은 호출자의 STEADY clock 정수 나노초(`inference_gate` 와
같다), 벽시계는 epoch 초다. 이 모듈은 시계를 직접 읽지 않는다.

이름과 시각 글자는 `VICA_PATROL_TZ`(기본 Asia/Seoul) 시간대로 쓴다(`patrol_zone`).
"""
from __future__ import annotations

import io
import json
import math
import os
import shutil
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Deque, Dict, List, Optional, Tuple
from zoneinfo import ZoneInfo

import numpy as np

from vica_perception.inference_gate import sec_to_ns

DEFAULT_MEDIA_DIR = "/workspaces/isaac_ros-dev/vica_patrol"
CLIPS_DIR = "clips"
PHOTOS_DIR = "photos"
EVENTS_FILE = "events.jsonl"          # 미션 기록. 정리 대상이 아니다.

ALARM_INTRUDER = "intruder"
ALARM_SOUND = "sound"

CLIP_RATE_HZ = 5.0
CLIP_PRE_S = 5.0                      # 경보 앞 5 s
CLIP_POST_S = 10.0                    # 마지막으로 사람이 보인 뒤(소리는 경보 뒤) 10 s
CLIP_MAX_S = 60.0                     # 첫 프레임부터 장면 전체 길이 상한
JPEG_QUALITY = 75

MEDIA_KEEP_S = 7 * 24 * 3600.0        # 보관 7일
MEDIA_MAX_BYTES = 1_000_000_000       # 합계 1 GB
# 정리가 지우지 않는 최근 구간. 쓰고 있는 장면(최대 60 s)을 지키려는 값이다.
MEDIA_PROTECT_RECENT_S = 120.0


# ---- 이름·시각 ---------------------------------------------------------------

TZ_ENV = "VICA_PATROL_TZ"
DEFAULT_TZ = "Asia/Seoul"
_zones: Dict[str, Optional[ZoneInfo]] = {}        # 이름 → 시간대(못 읽었으면 None)


def patrol_zone_name() -> str:
    return os.environ.get(TZ_ENV) or DEFAULT_TZ


def patrol_zone() -> Optional[ZoneInfo]:
    """이름·시각 글자에 쓰는 시간대. 못 읽으면 None — 기기 시간대로 물러난다.

    이 노드의 컨테이너는 UTC, 미션·HTTP 서버가 도는 호스트는 KST 다(2026-11-14 확인).
    기기 시간대로 쓰면 clip_id·`start`/`end` 가 미션의 alarm_id 와 9시간 어긋난다.
    """
    name = patrol_zone_name()
    if name not in _zones:
        try:
            _zones[name] = ZoneInfo(name)
        except (KeyError, ValueError, OSError):   # 없는 이름·잘못된 꼴·tzdata 없음
            _zones[name] = None
    return _zones[name]


def _local(wall_s: float) -> datetime:
    return datetime.fromtimestamp(wall_s, patrol_zone())


def _stamp(wall_s: float) -> str:
    return _local(wall_s).strftime("%Y%m%d_%H%M%S")


def clip_id_for(wall_s: float, alarm: str) -> str:
    """`YYYYMMDD_HHMMSS_<alarm>` (`patrol_zone` 시각)."""
    return "%s_%s" % (_stamp(wall_s), alarm)


def photo_name_for(wall_s: float) -> str:
    """불 켜짐 사진 파일 이름."""
    return "%s_light.jpg" % _stamp(wall_s)


def iso_local(wall_s: float) -> str:
    """`2026-11-14T20:34:55` — `events.jsonl` 의 `time` 과 같은 꼴(`patrol_zone` 시각)."""
    return _local(wall_s).strftime("%Y-%m-%dT%H:%M:%S")


def _stamp_to_wall(name: str) -> Optional[float]:
    """이름 앞 `YYYYMMDD_HHMMSS` → epoch 초. 그 꼴이 아니면 None."""
    try:
        return datetime.strptime(name[:15], "%Y%m%d_%H%M%S").replace(
            tzinfo=patrol_zone()).timestamp()
    except (ValueError, OverflowError, OSError):
        return None


# ---- 영상 한 장 --------------------------------------------------------------


def image_to_bgr(msg) -> np.ndarray:
    """`sensor_msgs/Image`(rgb8·bgr8) → BGR 배열. 기존 추론 경로와 같은 변환이다."""
    img = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    img = img.reshape(msg.height, msg.width, -1)
    if msg.encoding == "rgb8":
        img = img[:, :, ::-1]
    return img


def mean_brightness(img: np.ndarray) -> float:
    """BGR(또는 흑백) 영상의 평균 밝기 0~255. 흑백 변환은 ITU-R 601 가중이다.

    11-10·11 실측 도구(PIL `convert('L')`)와 같은 식이라 기준 10 을 그대로 쓴다.
    """
    if img.ndim == 2 or img.shape[2] == 1:
        return float(img.mean())
    b, g, r = (float(v) for v in img.reshape(-1, img.shape[2])[:, :3].mean(axis=0))
    return 0.114 * b + 0.587 * g + 0.299 * r


def _cv2_encoder(quality: int) -> Callable[[np.ndarray], bytes]:
    import cv2                                    # 컨테이너에만 있다

    params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]

    def encode(img: np.ndarray) -> bytes:
        ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(img), params)
        if not ok:
            raise RuntimeError("JPEG 압축 실패")
        return buf.tobytes()

    return encode


def _pil_encoder(quality: int) -> Callable[[np.ndarray], bytes]:
    from PIL import Image

    def encode(img: np.ndarray) -> bytes:
        if img.ndim == 3:
            img = img[:, :, 2::-1]                # BGR → RGB
        out = io.BytesIO()
        Image.fromarray(np.ascontiguousarray(img)).save(
            out, format="JPEG", quality=int(quality))
        return out.getvalue()

    return encode


def make_jpeg_encoder(quality: int = JPEG_QUALITY) -> Callable[[np.ndarray], bytes]:
    """BGR 영상 → JPEG 바이트. cv2 가 없으면 PIL 로 한다."""
    try:
        return _cv2_encoder(quality)
    except Exception:                             # noqa: BLE001 — 호스트의 cv2 는 import 에서 죽는다
        return _pil_encoder(quality)


@dataclass(frozen=True)
class Frame:
    """메모리에 두는 JPEG 한 장."""

    mono_ns: int
    wall_s: float
    jpeg: bytes
    width: int
    height: int


# ---- 장면 길이 ---------------------------------------------------------------


def clip_end_ns(
    first_ns: int,
    hold_until_ns: int,
    last_person_ns: Optional[int],
    follow_person: bool,
    post_ns: int,
    max_ns: int,
) -> int:
    """장면이 끝나는 시각.

    소리 장면은 경보 뒤 10 s(`hold_until_ns`)에서 끝난다. 사람 장면은 마지막으로
    사람이 보인 뒤 10 s 까지 늘어난다. 어느 쪽이든 첫 프레임부터 60 s 를 넘지 않는다.
    """
    end = hold_until_ns
    if follow_person and last_person_ns is not None:
        end = max(end, last_person_ns + post_ns)
    return min(end, first_ns + max_ns)


@dataclass(frozen=True)
class ClipDone:
    """끝난 장면 — `clip done` 이벤트와 로그에 쓴다."""

    clip_id: str
    alarm: str
    frames: int
    meta: Dict[str, object]


class _OpenClip:
    def __init__(self, clip_id: str, alarm: str, path: str, now_ns: int,
                 wall_s: float, follow_person: bool, post_ns: int) -> None:
        self.clip_id = clip_id
        self.alarm = alarm
        self.path = path
        self.frames = 0
        self.first_ns = now_ns
        self.start_wall_s = wall_s
        self.end_wall_s = wall_s
        self.width = 0
        self.height = 0
        self.hold_until_ns = now_ns + post_ns
        self.follow_person = follow_person
        self.last_person_ns: Optional[int] = now_ns if follow_person else None


class ClipRecorder:
    """최근 5 s 를 메모리에 두고, 경보가 나면 그 5 s 와 이후 프레임을 디스크로 옮긴다."""

    def __init__(
        self,
        media_dir: str,
        rate_hz: float = CLIP_RATE_HZ,
        pre_s: float = CLIP_PRE_S,
        post_s: float = CLIP_POST_S,
        max_s: float = CLIP_MAX_S,
    ) -> None:
        if not rate_hz > 0.0:
            raise ValueError(f"처리 주기는 0보다 커야 한다: {rate_hz}")
        self._media_dir = media_dir
        self._fps = int(round(rate_hz))
        self._pre_ns = sec_to_ns(pre_s)
        self._post_ns = sec_to_ns(post_s)
        self._max_ns = sec_to_ns(max_s)
        self._ring: Deque[Frame] = deque(maxlen=max(1, math.ceil(rate_hz * pre_s)))
        self._clip: Optional[_OpenClip] = None

    # ---- 조회 ---------------------------------------------------------------

    @property
    def recording(self) -> bool:
        return self._clip is not None

    @property
    def clip_id(self) -> str:
        return self._clip.clip_id if self._clip is not None else ""

    @property
    def follows_person(self) -> bool:
        """사람이 보이는 동안 늘어나는 장면인가(추론을 계속 돌릴 이유)."""
        return self._clip is not None and self._clip.follow_person

    @property
    def buffered(self) -> int:
        return len(self._ring)

    def end_ns(self) -> Optional[int]:
        c = self._clip
        if c is None:
            return None
        return clip_end_ns(c.first_ns, c.hold_until_ns, c.last_person_ns,
                           c.follow_person, self._post_ns, self._max_ns)

    def due(self, now_ns: int) -> bool:
        """장면을 끝낼 때가 됐는가."""
        end = self.end_ns()
        return end is not None and now_ns >= end

    # ---- 입력 ---------------------------------------------------------------

    def push(self, frame: Frame) -> None:
        """처리한 프레임 한 장. 장면을 쓰는 중이면 디스크에도 쓴다."""
        self._ring.append(frame)
        end = self.end_ns()
        if end is not None and frame.mono_ns <= end:
            self._write(frame)

    def mark_person(self, now_ns: int) -> None:
        """이 프레임에 사람이 보였다(conf 문턱 이상)."""
        if self._clip is not None and self._clip.follow_person:
            self._clip.last_person_ns = now_ns

    def follow_person(self, now_ns: int) -> None:
        """쓰고 있는 장면을 사람 장면 규칙으로 바꾼다(소리 장면 중 사람 판정)."""
        if self._clip is not None:
            self._clip.follow_person = True
            self._clip.last_person_ns = now_ns

    def extend(self, now_ns: int) -> None:
        """쓰고 있는 장면을 지금부터 10 s 뒤까지 늘린다(사람 장면 중 소리 경보)."""
        if self._clip is not None:
            self._clip.hold_until_ns = max(
                self._clip.hold_until_ns, now_ns + self._post_ns)

    def start(self, alarm: str, now_ns: int, wall_s: float) -> str:
        """장면을 시작하고 메모리의 최근 5 s 를 디스크에 쓴다. clip_id 를 돌려준다."""
        if self._clip is not None:
            return self._clip.clip_id
        clips = os.path.join(self._media_dir, CLIPS_DIR)
        clip_id = clip_id_for(wall_s, alarm)
        while os.path.exists(os.path.join(clips, clip_id)):
            wall_s += 1.0                         # 같은 초에 같은 종류 — 이름을 1 s 민다
            clip_id = clip_id_for(wall_s, alarm)
        path = os.path.join(clips, clip_id)
        os.makedirs(os.path.join(path, "frames"))
        self._clip = _OpenClip(clip_id, alarm, path, now_ns, wall_s,
                               alarm == ALARM_INTRUDER, self._post_ns)
        for frame in self._ring:
            if 0 <= now_ns - frame.mono_ns <= self._pre_ns:
                self._write(frame)
        return clip_id

    def finish(self) -> Optional[ClipDone]:
        """장면을 닫고 `meta.json` 을 쓴다. 쓰는 장면이 없으면 None."""
        c = self._clip
        if c is None:
            return None
        self._clip = None
        # 재생 속도가 실제와 같게, 쓴 장수와 걸린 시간으로 잰 값을 적는다(프레임이 드물게 오면 5 보다 낮다).
        span_s = c.end_wall_s - c.start_wall_s
        fps = round((c.frames - 1) / span_s, 2) if c.frames >= 2 and span_s > 0.0 else float(self._fps)
        meta: Dict[str, object] = {
            "id": c.clip_id,
            "alarm": c.alarm,
            "start": iso_local(c.start_wall_s),
            "end": iso_local(c.end_wall_s),
            "fps": fps,
            "frames": c.frames,
            "width": c.width,
            "height": c.height,
            # 시간대와 무관한 값. 서버는 `start`/`end` 글자보다 이것을 먼저 쓴다.
            "start_epoch": round(c.start_wall_s, 3),
            "end_epoch": round(c.end_wall_s, 3),
        }
        # 서버가 반쯤 쓴 파일을 읽지 않게 이름 바꾸기로 한 번에 놓는다.
        tmp = os.path.join(c.path, "meta.json.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False)
        os.replace(tmp, os.path.join(c.path, "meta.json"))
        return ClipDone(c.clip_id, c.alarm, c.frames, meta)

    def abandon(self) -> None:
        """쓰기 실패 뒤 — 장면을 닫지 않고 손에서 놓는다(남은 파일은 정리가 지운다)."""
        self._clip = None

    def clear(self) -> None:
        """메모리의 프레임을 버린다(순찰 프로필 이탈)."""
        self._ring.clear()

    # ---- 내부 ---------------------------------------------------------------

    def _write(self, frame: Frame) -> None:
        c = self._clip
        name = "%06d.jpg" % (c.frames + 1)
        with open(os.path.join(c.path, "frames", name), "wb") as f:
            f.write(frame.jpeg)
        if c.frames == 0:
            c.first_ns = frame.mono_ns
            c.start_wall_s = frame.wall_s
            c.width, c.height = frame.width, frame.height
        c.frames += 1
        c.end_wall_s = frame.wall_s


def save_photo(media_dir: str, wall_s: float, jpeg: bytes) -> str:
    """불 켜짐 사진 1장을 `photos/` 에 쓰고 파일 이름을 돌려준다."""
    photos = os.path.join(media_dir, PHOTOS_DIR)
    os.makedirs(photos, exist_ok=True)
    name = photo_name_for(wall_s)
    tmp = os.path.join(photos, name + ".tmp")
    with open(tmp, "wb") as f:
        f.write(jpeg)
    os.replace(tmp, os.path.join(photos, name))
    return name


# ---- 저장 폴더 정리 ----------------------------------------------------------


@dataclass(frozen=True)
class CleanupReport:
    removed: Tuple[str, ...]          # 지운 묶음 이름(clip_id·사진 이름 줄기)
    kept_bytes: int                   # 남은 합계


@dataclass
class _Bundle:
    """이름 줄기가 같은 것들 — `<id>/`, `<id>.mp4`, `<id>.jpg` 는 한 장면이다."""

    stem: str
    paths: List[str]
    size: int
    wall_s: float


def _tree_size(path: str) -> int:
    try:
        st = os.lstat(path)
    except OSError:
        return 0
    if not os.path.isdir(path) or os.path.islink(path):
        return st.st_size
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.lstat(os.path.join(root, name)).st_size
            except OSError:
                pass
    return total


def _bundles(directory: str) -> List[_Bundle]:
    found: Dict[str, _Bundle] = {}
    try:
        entries = list(os.scandir(directory))
    except OSError:
        return []
    for entry in entries:
        if entry.name == EVENTS_FILE:
            continue
        stem = entry.name.split(".", 1)[0]
        try:
            mtime = entry.stat(follow_symlinks=False).st_mtime
        except OSError:
            continue
        b = found.setdefault(stem, _Bundle(stem, [], 0, 0.0))
        b.paths.append(entry.path)
        b.size += _tree_size(entry.path)
        b.wall_s = max(b.wall_s, mtime)
    for b in found.values():
        named = _stamp_to_wall(b.stem)        # 변환이 mtime 을 바꾸므로 이름의 시각이 먼저다
        if named is not None:
            b.wall_s = named
    return list(found.values())


def _remove(bundle: _Bundle) -> None:
    for path in bundle.paths:
        try:
            if os.path.isdir(path) and not os.path.islink(path):
                shutil.rmtree(path, ignore_errors=True)
            else:
                os.remove(path)
        except OSError:
            pass


def cleanup_media(
    media_dir: str,
    now_wall_s: float,
    max_age_s: float = MEDIA_KEEP_S,
    max_bytes: int = MEDIA_MAX_BYTES,
    protect_recent_s: float = MEDIA_PROTECT_RECENT_S,
) -> CleanupReport:
    """`clips/`·`photos/` 정리: 7일 넘은 것 삭제, 합계 1 GB 넘으면 오래된 것부터 삭제.

    `events.jsonl` 과 최근 2분 안의 것(쓰고 있는 장면)은 지우지 않는다.
    """
    bundles: List[_Bundle] = []
    for sub in (CLIPS_DIR, PHOTOS_DIR):
        bundles.extend(_bundles(os.path.join(media_dir, sub)))
    bundles.sort(key=lambda b: (b.wall_s, b.stem))        # 오래된 것부터

    removed: List[str] = []
    kept: List[_Bundle] = []
    for b in bundles:
        age = now_wall_s - b.wall_s
        if age > max_age_s and age >= protect_recent_s:
            _remove(b)
            removed.append(b.stem)
        else:
            kept.append(b)

    total = sum(b.size for b in kept)
    for b in kept:
        if total <= max_bytes:
            break
        if now_wall_s - b.wall_s < protect_recent_s:
            continue
        _remove(b)
        removed.append(b.stem)
        total -= b.size
    return CleanupReport(tuple(removed), total)
