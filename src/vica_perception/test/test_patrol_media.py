"""순찰 영상 장면·사진·저장 폴더 정리 시험 — ROS·GPU·cv2 없이 돈다.

정본은 `~/vica_data/patrol_plan/SPEC.md` 4.B. 폴더 구조·파일 이름·`meta.json`
키는 지도 HTTP 서버와 앱이 그대로 읽으므로 글자 그대로 고정한다.
"""
import io
import json
import os
import re
import time
from datetime import datetime
from types import SimpleNamespace
from zoneinfo import ZoneInfo

import numpy as np
import pytest

from vica_perception.inference_gate import sec_to_ns
from vica_perception.patrol_media import (
    ALARM_INTRUDER,
    ALARM_SOUND,
    CLIP_MAX_S,
    CLIP_POST_S,
    JPEG_QUALITY,
    MEDIA_KEEP_S,
    MEDIA_MAX_BYTES,
    ClipRecorder,
    Frame,
    _pil_encoder,
    _stamp_to_wall,
    cleanup_media,
    clip_end_ns,
    clip_id_for,
    image_to_bgr,
    iso_local,
    mean_brightness,
    patrol_zone,
    photo_name_for,
    save_photo,
)

# 2026-11-14 20:34:55 KST(= 11:34:55 UTC). 이름·시각 꼴을 눈으로 대조하기 쉬운 값이다.
WALL0 = datetime(2026, 11, 14, 20, 34, 55, tzinfo=ZoneInfo("Asia/Seoul")).timestamp()


def ns(seconds: float) -> int:
    return sec_to_ns(seconds)


def frame(t_s: float, jpeg: bytes = b"\xff\xd8jpeg") -> Frame:
    return Frame(ns(t_s), WALL0 + t_s, jpeg, 640, 480)


def times(t0: float, t1: float, step: float = 0.2):
    """t0 이상 t1 미만의 5 Hz 시각. 더해 가며 쌓이는 오차가 없게 매번 새로 만든다."""
    return [round(t0 + i * step, 3) for i in range(int(round((t1 - t0) / step)))]


def feed(rec: ClipRecorder, t0: float, t1: float) -> None:
    for t in times(t0, t1):
        rec.push(frame(t))


# ---- 이름·시각 꼴 -------------------------------------------------------------


def test_clip_id_는_날짜_시각_종류_꼴이다():
    assert clip_id_for(WALL0, ALARM_INTRUDER) == "20261114_203455_intruder"
    assert clip_id_for(WALL0, ALARM_SOUND) == "20261114_203455_sound"


def test_사진_이름과_시각_글자():
    assert photo_name_for(WALL0) == "20261114_203455_light.jpg"
    assert iso_local(WALL0) == "2026-11-14T20:34:55"


def test_저장_값은_명세_그대로다():
    assert JPEG_QUALITY == 75
    assert (CLIP_POST_S, CLIP_MAX_S) == (10.0, 60.0)
    assert MEDIA_KEEP_S == 7 * 24 * 3600
    assert MEDIA_MAX_BYTES == 1_000_000_000


# ---- 시간대: 컨테이너는 UTC, 미션·HTTP 서버는 KST -----------------------------


@pytest.fixture
def utc_machine(monkeypatch):
    """기기 시간대를 UTC 로 둔다(감지 노드가 도는 컨테이너와 같다)."""
    monkeypatch.setenv("TZ", "UTC")
    time.tzset()
    yield
    monkeypatch.undo()
    time.tzset()


def test_기본_시간대는_Asia_Seoul_이다(monkeypatch):
    monkeypatch.delenv("VICA_PATROL_TZ", raising=False)
    assert str(patrol_zone()) == "Asia/Seoul"
    monkeypatch.setenv("VICA_PATROL_TZ", "")           # 빈 값도 기본으로
    assert str(patrol_zone()) == "Asia/Seoul"


def test_기기가_UTC_여도_이름과_시각_글자는_KST_로_쓴다(utc_machine, monkeypatch):
    monkeypatch.delenv("VICA_PATROL_TZ", raising=False)
    assert time.strftime("%H", time.localtime(WALL0)) == "11"     # 기기 시각은 9시간 이르다
    assert clip_id_for(WALL0, ALARM_INTRUDER) == "20261114_203455_intruder"
    assert photo_name_for(WALL0) == "20261114_203455_light.jpg"
    assert iso_local(WALL0) == "2026-11-14T20:34:55"


def test_자정을_넘는_날짜도_KST_로_넘어간다(utc_machine, monkeypatch):
    monkeypatch.delenv("VICA_PATROL_TZ", raising=False)
    late = WALL0 + 4 * 3600                            # 15:34:55 UTC = 다음 날 00:34:55 KST
    assert clip_id_for(late, ALARM_SOUND) == "20261115_003455_sound"


def test_VICA_PATROL_TZ_로_시간대를_바꾼다(monkeypatch):
    monkeypatch.setenv("VICA_PATROL_TZ", "UTC")
    assert clip_id_for(WALL0, ALARM_INTRUDER) == "20261114_113455_intruder"
    assert iso_local(WALL0) == "2026-11-14T11:34:55"


@pytest.mark.parametrize("bad", ["No/Such_Zone", "KST", "/etc/localtime", "../x", "Asia"])
def test_시간대를_못_읽으면_기기_시간대로_물러난다(utc_machine, monkeypatch, bad):
    monkeypatch.setenv("VICA_PATROL_TZ", bad)
    assert patrol_zone() is None
    assert clip_id_for(WALL0, ALARM_INTRUDER) == "20261114_113455_intruder"   # 기기(UTC) 시각
    assert iso_local(WALL0) == "2026-11-14T11:34:55"
    assert _stamp_to_wall("20261114_113455_intruder") == WALL0


def test_이름의_시각을_같은_시간대로_되읽는다(utc_machine, monkeypatch):
    """정리(7일)가 이름으로 나이를 잰다 — 쓸 때와 읽을 때의 시간대가 같아야 한다."""
    monkeypatch.delenv("VICA_PATROL_TZ", raising=False)
    assert _stamp_to_wall(clip_id_for(WALL0, ALARM_SOUND)) == WALL0
    assert _stamp_to_wall(photo_name_for(WALL0)) == WALL0
    assert _stamp_to_wall("memo") is None


def test_meta_의_start_end_는_KST_글자이고_epoch_는_시간대와_무관하다(
        utc_machine, monkeypatch, tmp_path):
    monkeypatch.delenv("VICA_PATROL_TZ", raising=False)
    rec = ClipRecorder(str(tmp_path))
    feed(rec, 0.0, 2.0)                       # 0.0~1.8 s
    clip_id = rec.start(ALARM_INTRUDER, ns(1.8), WALL0 + 1.8)
    meta = rec.finish().meta
    assert clip_id == "20261114_203456_intruder"
    assert meta["start"] == "2026-11-14T20:34:55"
    assert meta["end"] == "2026-11-14T20:34:56"
    assert meta["start_epoch"] == pytest.approx(WALL0, abs=0.001)
    assert meta["end_epoch"] == pytest.approx(WALL0 + 1.8, abs=0.001)
    assert isinstance(meta["start_epoch"], float) and isinstance(meta["end_epoch"], float)


def test_기기가_UTC_여도_7일_정리는_KST_이름으로_나이를_잰다(utc_machine, monkeypatch, tmp_path):
    monkeypatch.delenv("VICA_PATROL_TZ", raising=False)
    now = WALL0
    old = make_clip(tmp_path, now - 7 * DAY - 60, 100)
    edge = make_clip(tmp_path, now - 7 * DAY + 60, 100)
    cleanup_media(str(tmp_path), now)
    assert os.listdir(tmp_path / "clips") == [edge]
    assert old != edge


# ---- 영상 한 장 ---------------------------------------------------------------


def test_평균_밝기_검정과_흰색():
    assert mean_brightness(np.zeros((4, 6, 3), np.uint8)) == 0.0
    assert mean_brightness(np.full((4, 6, 3), 255, np.uint8)) == pytest.approx(255.0)


def test_평균_밝기는_BGR_순서로_흑백_가중을_준다():
    blue = np.zeros((4, 6, 3), np.uint8)
    blue[:, :, 0] = 255                       # BGR 의 B
    red = np.zeros((4, 6, 3), np.uint8)
    red[:, :, 2] = 255
    assert mean_brightness(blue) == pytest.approx(0.114 * 255)
    assert mean_brightness(red) == pytest.approx(0.299 * 255)


def test_평균_밝기는_실측_도구_PIL_과_같은_값이다():
    """11-10·11 기준(끔 1.4 / 일부 35 / 모두 163)은 PIL 흑백 변환으로 잰 값이다."""
    from PIL import Image, ImageStat
    rng = np.random.default_rng(7)
    bgr = rng.integers(0, 256, (48, 64, 3), dtype=np.uint8)
    pil = ImageStat.Stat(Image.fromarray(bgr[:, :, ::-1].copy()).convert("L")).mean[0]
    assert mean_brightness(bgr) == pytest.approx(pil, abs=0.6)


def test_rgb8_영상은_BGR_로_뒤집는다():
    rgb = np.zeros((2, 3, 3), np.uint8)
    rgb[:, :, 0] = 200                        # R
    msg = SimpleNamespace(data=rgb.tobytes(), height=2, width=3, encoding="rgb8")
    bgr = image_to_bgr(msg)
    assert bgr.shape == (2, 3, 3)
    assert bgr[0, 0].tolist() == [0, 0, 200]
    msg.encoding = "bgr8"
    assert image_to_bgr(msg)[0, 0].tolist() == [200, 0, 0]


def test_JPEG_압축은_BGR_를_받아_제_색으로_담는다():
    from PIL import Image
    red_bgr = np.zeros((16, 24, 3), np.uint8)
    red_bgr[:, :, 2] = 255
    data = _pil_encoder(JPEG_QUALITY)(red_bgr)
    assert data[:2] == b"\xff\xd8"
    img = Image.open(io.BytesIO(data))
    assert img.size == (24, 16)
    r, g, b = img.convert("RGB").getpixel((5, 5))
    assert r > 200 and g < 60 and b < 60


# ---- 장면 끝 시각 -------------------------------------------------------------


def test_소리_장면은_경보_뒤_10초에_끝난다():
    end = clip_end_ns(first_ns=ns(95.0), hold_until_ns=ns(110.0),
                      last_person_ns=ns(109.0), follow_person=False,
                      post_ns=ns(10.0), max_ns=ns(60.0))
    assert end == ns(110.0)                   # 사람이 보여도 늘어나지 않는다


def test_사람_장면은_마지막으로_보인_뒤_10초에_끝난다():
    end = clip_end_ns(first_ns=ns(95.0), hold_until_ns=ns(110.0),
                      last_person_ns=ns(123.4), follow_person=True,
                      post_ns=ns(10.0), max_ns=ns(60.0))
    assert end == ns(133.4)


def test_장면은_첫_프레임부터_60초를_넘지_않는다():
    end = clip_end_ns(first_ns=ns(95.0), hold_until_ns=ns(110.0),
                      last_person_ns=ns(150.0), follow_person=True,
                      post_ns=ns(10.0), max_ns=ns(60.0))
    assert end == ns(155.0)


# ---- 메모리 버퍼 --------------------------------------------------------------


def test_평소에는_최근_25장만_메모리에_두고_디스크에_안_쓴다(tmp_path):
    rec = ClipRecorder(str(tmp_path / "media"))
    feed(rec, 0.0, 60.0)                      # 300장
    assert rec.buffered == 25
    assert not rec.recording
    assert not (tmp_path / "media").exists()  # 폴더조차 만들지 않는다


def test_버퍼를_비우면_메모리가_빈다(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    feed(rec, 0.0, 3.0)
    rec.clear()
    assert rec.buffered == 0


# ---- 장면 쓰기 ---------------------------------------------------------------


def test_사람_장면_5초_전부터_사람이_사라지고_10초_뒤까지(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    feed(rec, 0.0, 20.0)                      # 마지막 프레임 19.8 s
    clip_id = rec.start(ALARM_INTRUDER, ns(19.8), WALL0 + 19.8)
    assert clip_id == clip_id_for(WALL0 + 19.8, ALARM_INTRUDER)
    assert rec.recording and rec.follows_person

    ended_at = None
    for t in times(20.0, 60.0):
        if rec.due(ns(t)):
            ended_at = t
            break
        if t <= 23.0:
            rec.mark_person(ns(t))            # 23.0 s 까지 사람이 보인다
        rec.push(frame(t))
    assert ended_at == 33.0                   # 23.0 + 10 s

    done = rec.finish()
    frames_dir = tmp_path / "clips" / clip_id / "frames"
    names = sorted(os.listdir(frames_dir))
    # 경보 앞 15.0~19.8 s 의 25장 + 20.0~32.8 s 의 65장
    assert done.frames == len(names) == 25 + 65
    assert names[0] == "000001.jpg" and names[-1] == "%06d.jpg" % done.frames
    assert (frames_dir / "000001.jpg").read_bytes() == b"\xff\xd8jpeg"


def test_장면_시작은_5초보다_오래된_프레임을_빼고_쓴다(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    feed(rec, 0.0, 3.0)                       # 0~2.8 s (불 확인 등으로 그 뒤가 비었다)
    rec.push(frame(10.0))
    clip_id = rec.start(ALARM_SOUND, ns(10.0), WALL0 + 10.0)
    done = rec.finish()
    assert done.frames == 1                   # 5 s 안의 것은 10.0 s 한 장뿐
    assert done.meta["start"] == iso_local(WALL0 + 10.0)
    assert os.listdir(tmp_path / "clips" / clip_id / "frames") == ["000001.jpg"]


def test_소리_장면은_사람이_보여도_10초에_끝난다(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    feed(rec, 0.0, 8.0)
    rec.start(ALARM_SOUND, ns(8.0), WALL0 + 8.0)
    assert not rec.follows_person
    rec.mark_person(ns(15.0))
    assert rec.end_ns() == ns(18.0)
    assert not rec.due(ns(17.9)) and rec.due(ns(18.0))


def test_사람_장면은_60초에서_잘린다(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    feed(rec, 0.0, 10.0)
    rec.start(ALARM_INTRUDER, ns(9.8), WALL0 + 9.8)
    ended_at = None
    for t in times(10.0, 200.0):
        if rec.due(ns(t)):
            ended_at = t
            break
        rec.mark_person(ns(t))                # 계속 보인다
        rec.push(frame(t))
    done = rec.finish()
    assert ended_at == 65.0                   # 첫 프레임 5.0 s + 60 s
    assert done.frames == 300                 # 60 s × 5 Hz


def test_끝_시각을_넘긴_프레임은_쓰지_않는다(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    rec.push(frame(0.0))
    rec.start(ALARM_SOUND, ns(0.0), WALL0)
    rec.push(frame(10.0))
    rec.push(frame(10.2))                     # 끝(10.0 s) 뒤
    assert rec.finish().frames == 2


def test_쓰던_장면에_소리_경보가_겹치면_10초_더_늘린다(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    rec.push(frame(0.0))
    rec.start(ALARM_INTRUDER, ns(0.0), WALL0)
    assert rec.end_ns() == ns(10.0)
    rec.extend(ns(8.0))
    assert rec.end_ns() == ns(18.0)


def test_소리_장면_중_사람_판정이_나면_사람_규칙으로_바뀐다(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    rec.push(frame(0.0))
    rec.start(ALARM_SOUND, ns(0.0), WALL0)
    rec.follow_person(ns(9.0))
    assert rec.follows_person
    assert rec.end_ns() == ns(19.0)


def test_meta_json_키와_값(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    feed(rec, 0.0, 5.0)
    clip_id = rec.start(ALARM_INTRUDER, ns(4.8), WALL0 + 4.8)
    feed(rec, 5.0, 7.0)
    done = rec.finish()

    path = tmp_path / "clips" / clip_id / "meta.json"
    meta = json.loads(path.read_text(encoding="utf-8"))
    assert list(meta) == ["id", "alarm", "start", "end", "fps",
                          "frames", "width", "height",
                          "start_epoch", "end_epoch"]   # 뒤 둘은 2026-11-14 추가
    assert meta == done.meta
    assert meta["id"] == clip_id
    assert meta["alarm"] == "intruder"
    assert meta["start"] == iso_local(WALL0 + 0.0)
    assert meta["end"] == iso_local(WALL0 + 6.8)
    assert meta["fps"] == 5
    assert meta["frames"] == 25 + 10
    assert (meta["width"], meta["height"]) == (640, 480)
    assert meta["start_epoch"] == pytest.approx(WALL0, abs=0.001)
    assert meta["end_epoch"] == pytest.approx(WALL0 + 6.8, abs=0.001)
    assert not rec.recording
    assert not os.path.exists(str(path) + ".tmp")


def test_meta_fps_는_쓴_장수와_걸린_시간으로_잰_값이다(tmp_path):
    """고정 5 가 아니다 — 프레임이 드물게 오면 낮게 적어야 재생 속도가 실제와 같다."""
    rec = ClipRecorder(str(tmp_path))
    for t in times(0.0, 3.0, step=0.3):       # 0.0~2.7 s 에 10장(3.3 Hz)
        rec.push(frame(t))
    rec.start(ALARM_SOUND, ns(2.7), WALL0 + 2.7)
    meta = rec.finish().meta
    assert meta["frames"] == 10
    assert meta["fps"] == 3.33                # (10 − 1) / 2.7 s, 소수 둘째 자리

    rec = ClipRecorder(str(tmp_path))
    feed(rec, 10.0, 14.0)                     # 10.0~13.8 s 에 20장(5 Hz)
    rec.start(ALARM_SOUND, ns(13.8), WALL0 + 13.8)
    meta = rec.finish().meta
    assert meta["frames"] == 20
    assert meta["fps"] == 5.0                 # (20 − 1) / 3.8 s


def test_meta_fps_는_한_장뿐이면_처리_주기_값을_적는다(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    rec.push(frame(0.0))
    rec.start(ALARM_SOUND, ns(0.0), WALL0)
    meta = rec.finish().meta
    assert meta["frames"] == 1
    assert meta["fps"] == 5.0                 # 걸린 시간이 0 — 잴 수 없다


def test_같은_초에_같은_종류면_이름을_민다(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    first = rec.start(ALARM_SOUND, ns(0.0), WALL0)
    rec.finish()
    second = rec.start(ALARM_SOUND, ns(0.5), WALL0)
    assert first != second
    assert re.fullmatch(r"\d{8}_\d{6}_sound", second)


def test_쓰는_장면이_없으면_닫을_것도_없다(tmp_path):
    rec = ClipRecorder(str(tmp_path))
    assert rec.finish() is None
    assert rec.end_ns() is None and not rec.due(ns(1e6))


def test_불_켜짐_사진은_photos_에_쓴다(tmp_path):
    name = save_photo(str(tmp_path), WALL0, b"\xff\xd8photo")
    assert name == "20261114_203455_light.jpg"
    assert (tmp_path / "photos" / name).read_bytes() == b"\xff\xd8photo"
    assert os.listdir(tmp_path / "photos") == [name]


# ---- 저장 폴더 정리 -----------------------------------------------------------

DAY = 24 * 3600.0


def make_clip(root, wall_s: float, size: int, alarm: str = ALARM_INTRUDER,
              converted: bool = False) -> str:
    """장면 하나. converted 면 서버가 mp4·첫 장 jpg 로 바꾼 뒤의 모습."""
    clip_id = clip_id_for(wall_s, alarm)
    d = root / "clips" / clip_id
    d.mkdir(parents=True)
    (d / "meta.json").write_text("{}")
    if converted:
        (root / "clips" / (clip_id + ".mp4")).write_bytes(b"m" * size)
        (root / "clips" / (clip_id + ".jpg")).write_bytes(b"j" * 10)
    else:
        (d / "frames").mkdir()
        (d / "frames" / "000001.jpg").write_bytes(b"f" * size)
    return clip_id


def make_photo(root, wall_s: float, size: int = 100) -> str:
    (root / "photos").mkdir(parents=True, exist_ok=True)
    name = photo_name_for(wall_s)
    (root / "photos" / name).write_bytes(b"p" * size)
    return name


def test_7일_넘은_것만_지운다(tmp_path):
    now = WALL0
    old = make_clip(tmp_path, now - 7 * DAY - 60, 100)
    edge = make_clip(tmp_path, now - 7 * DAY + 60, 100)
    new = make_clip(tmp_path, now - 3600, 100)
    old_photo = make_photo(tmp_path, now - 8 * DAY)
    new_photo = make_photo(tmp_path, now - DAY)

    report = cleanup_media(str(tmp_path), now)

    assert sorted(os.listdir(tmp_path / "clips")) == sorted([edge, new])
    assert os.listdir(tmp_path / "photos") == [new_photo]
    assert set(report.removed) == {old, old_photo.split(".")[0]}


def test_변환된_mp4_와_첫_장도_같은_장면으로_함께_지운다(tmp_path):
    now = WALL0
    old = make_clip(tmp_path, now - 8 * DAY, 100, converted=True)
    keep = make_clip(tmp_path, now - DAY, 100, converted=True)

    cleanup_media(str(tmp_path), now)

    left = sorted(os.listdir(tmp_path / "clips"))
    assert left == sorted([keep, keep + ".jpg", keep + ".mp4"])
    assert not any(name.startswith(old) for name in left)


def test_나이는_파일_시각이_아니라_이름의_시각으로_본다(tmp_path):
    """서버가 mp4 로 바꾸면 파일 시각이 새로 찍힌다 — 그래도 8일 된 장면이다."""
    now = time.time()
    old = make_clip(tmp_path, now - 8 * DAY, 100, converted=True)   # mtime 은 지금
    cleanup_media(str(tmp_path), now)
    assert not any(n.startswith(old) for n in os.listdir(tmp_path / "clips"))


def test_합계가_한도를_넘으면_오래된_것부터_지운다(tmp_path):
    now = WALL0
    a = make_clip(tmp_path, now - 5 * DAY, 400)
    b = make_clip(tmp_path, now - 4 * DAY, 400)
    c = make_clip(tmp_path, now - 3 * DAY, 400)
    d = make_clip(tmp_path, now - 2 * DAY, 400, alarm=ALARM_SOUND)

    report = cleanup_media(str(tmp_path), now, max_bytes=1000)

    assert sorted(os.listdir(tmp_path / "clips")) == sorted([c, d])
    assert report.removed == (a, b)
    assert report.kept_bytes <= 1000


def test_한도_안이면_아무것도_안_지운다(tmp_path):
    now = WALL0
    make_clip(tmp_path, now - 2 * DAY, 400)
    make_photo(tmp_path, now - DAY, 100)
    report = cleanup_media(str(tmp_path), now, max_bytes=10_000)
    assert report.removed == ()
    assert report.kept_bytes >= 500


def test_사진도_합계에_들어가고_오래됐으면_먼저_지운다(tmp_path):
    now = WALL0
    photo = make_photo(tmp_path, now - 6 * DAY, 600)
    clip = make_clip(tmp_path, now - DAY, 600)
    cleanup_media(str(tmp_path), now, max_bytes=1000)
    assert os.listdir(tmp_path / "photos") == []
    assert os.listdir(tmp_path / "clips") == [clip]
    assert photo


def test_events_jsonl_은_지우지_않는다(tmp_path):
    now = WALL0
    (tmp_path / "events.jsonl").write_bytes(b"e" * 5000)
    (tmp_path / "clips").mkdir()
    (tmp_path / "clips" / "events.jsonl").write_bytes(b"e" * 5000)   # 잘못 놓여도
    old = time.time() - 30 * DAY
    for p in (tmp_path / "events.jsonl", tmp_path / "clips" / "events.jsonl"):
        os.utime(p, (old, old))
    make_clip(tmp_path, now - 9 * DAY, 100)

    cleanup_media(str(tmp_path), now, max_bytes=10)

    assert (tmp_path / "events.jsonl").stat().st_size == 5000
    assert os.listdir(tmp_path / "clips") == ["events.jsonl"]


def test_쓰고_있는_최근_장면은_한도를_넘어도_지우지_않는다(tmp_path):
    now = WALL0
    old = make_clip(tmp_path, now - DAY, 400)
    live = make_clip(tmp_path, now - 30, 5000)        # 30 s 전에 시작 — 쓰는 중
    report = cleanup_media(str(tmp_path), now, max_bytes=1000)
    assert os.listdir(tmp_path / "clips") == [live]
    assert report.removed == (old,)


def test_저장_폴더가_없으면_아무_일도_없다(tmp_path):
    report = cleanup_media(str(tmp_path / "none"), WALL0)
    assert report.removed == () and report.kept_bytes == 0
    assert not (tmp_path / "none").exists()


def test_이름이_시각_꼴이_아니면_파일_시각으로_본다(tmp_path):
    (tmp_path / "clips").mkdir()
    stray = tmp_path / "clips" / "memo.txt"
    stray.write_text("x")
    old = time.time() - 9 * DAY
    os.utime(stray, (old, old))
    fresh = tmp_path / "clips" / "today.txt"
    fresh.write_text("y")
    cleanup_media(str(tmp_path), time.time())
    assert os.listdir(tmp_path / "clips") == ["today.txt"]
