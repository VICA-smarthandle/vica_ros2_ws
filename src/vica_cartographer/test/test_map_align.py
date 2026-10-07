"""Tests for straightening a tilted map on save (2026-10-07).

로봇도 ROS 도 없이 도는 시험이다. 이 파일이 지키는 것:
  1. 기운 각도를 맞게 재고, 벽이 모자라면 "못 잼"이라고 말하는가
  2. 돌린 뒤에도 같은 벽이 **같은 실제 좌표**에 있는가 — origin 이 틀리면 지도는
     바르게 보여도 AMCL 이 레이저와 지도를 맞추지 못한다
  3. 2° 미만·정렬 안 함·못 잼이면 파일을 한 글자도 바꾸지 않는가
  4. 원본은 maps/.original/ 에만 남고 yaml·png 는 만들지 않는가(B안)
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from vica_cartographer import map_align
from vica_cartographer.map_align import (
    GRAY_FREE,
    GRAY_OCCUPIED,
    GRAY_UNKNOWN,
    ORIGINAL_DIRNAME,
    read_map_yaml,
    read_pgm,
    RESULT_PREFIX,
    rotate_map,
    straighten_saved_map,
    tilt_of_image,
    tilt_of_occupancy,
    write_pgm,
)
from vica_cartographer.mapping_session import (
    ALIGN_RESULT_PREFIX,
    ALIGN_RESULTS,
    parse_align_result,
    save_script_command,
)

RES = 0.05
YAML_TEXT = (
    'image: {name}.pgm\nmode: trinary\nresolution: 0.05\n'
    'origin: [-3.2, -4.1, 0]\nnegate: 0\noccupied_thresh: 0.65\n'
    'free_thresh: 0.25\n'
)


def room_image(tilt_deg: float, size: int = 240, half_w: float = 3.5,
               half_h: float = 2.2):
    """Draw a rectangular room (walls + free inside) tilted by tilt_deg, top-down."""
    image = np.full((size, size), GRAY_UNKNOWN, dtype=np.uint8)
    theta = math.radians(tilt_deg)
    centre = size * RES / 2
    rows, cols = np.mgrid[0:size, 0:size]
    x = (cols + 0.5) * RES - centre
    y = (size - 1 - rows + 0.5) * RES - centre
    u = x * math.cos(theta) + y * math.sin(theta)
    v = -x * math.sin(theta) + y * math.cos(theta)
    inside = (np.abs(u) < half_w) & (np.abs(v) < half_h)
    wall = inside & ((np.abs(u) > half_w - RES) | (np.abs(v) > half_h - RES))
    image[inside] = GRAY_FREE
    image[wall] = GRAY_OCCUPIED
    return image


def save_map(tmp_path, name: str, image):
    write_pgm(tmp_path / f'{name}.pgm', image, RES)
    (tmp_path / f'{name}.yaml').write_text(YAML_TEXT.format(name=name), encoding='utf-8')


# -- 1. 기울기 측정 ---------------------------------------------------------

@pytest.mark.parametrize('tilt', [0.0, 1.5, 5.5, -4.5, 12.0, -30.0])
def test_tilt_is_measured(tilt):
    measured = tilt_of_image(room_image(tilt))
    assert measured is not None
    assert measured == pytest.approx(tilt, abs=0.3)


def test_tilt_folds_into_plus_minus_45():
    """벽은 90° 마다 같은 모양이다. 50° 기운 방은 −40° 로 읽는다."""
    assert tilt_of_image(room_image(50.0)) == pytest.approx(-40.0, abs=0.3)


def test_few_walls_cannot_be_measured():
    """매핑을 막 시작해 벽이 몇 칸뿐이면 엉뚱한 각도 대신 None."""
    image = np.full((50, 50), GRAY_FREE, dtype=np.uint8)
    image[10, 10:30] = GRAY_OCCUPIED
    assert tilt_of_image(image) is None


def test_scattered_walls_cannot_be_measured():
    """방향 없는 점들(둥근 방·잡음)은 None — 팝업의 기울기 줄이 숨는다."""
    rng = np.random.default_rng(7)
    image = np.full((300, 300), GRAY_FREE, dtype=np.uint8)
    image[rng.integers(0, 300, 2000), rng.integers(0, 300, 2000)] = GRAY_OCCUPIED
    assert tilt_of_image(image) is None


def test_occupancy_grid_and_image_agree():
    """미리보기(OccupancyGrid, 아래줄 먼저)와 저장본(pgm, 위줄 먼저)이 같은 값."""
    image = room_image(5.5)
    bottom_up = image[::-1]
    data = np.where(bottom_up == GRAY_OCCUPIED, 100,
                    np.where(bottom_up == GRAY_FREE, 0, -1)).ravel().tolist()
    assert tilt_of_occupancy(data, 240, 240) == pytest.approx(tilt_of_image(image))


def test_occupancy_with_wrong_size_is_unknown():
    assert tilt_of_occupancy([0, 100], 3, 3) is None


# -- 2. 회전 ---------------------------------------------------------------

def test_rotation_straightens_the_room():
    image = room_image(5.5)
    rotated, _, _ = rotate_map(image, RES, -6.0, -6.0, tilt_of_image(image))
    assert abs(tilt_of_image(rotated)) < 0.3


def test_walls_keep_their_real_world_position():
    """옛 벽칸을 q = R(−θ)·p 로 옮긴 자리가 새 지도에서도 벽(또는 바로 옆)이다."""
    image = room_image(5.5)
    origin_x, origin_y = -6.0, -6.0
    tilt = tilt_of_image(image)
    rotated, new_x, new_y = rotate_map(image, RES, origin_x, origin_y, tilt)
    theta = math.radians(-tilt)
    rows, cols = np.nonzero(image == GRAY_OCCUPIED)
    px = origin_x + (cols + 0.5) * RES
    py = origin_y + (image.shape[0] - 1 - rows + 0.5) * RES
    qx = px * math.cos(theta) - py * math.sin(theta)
    qy = px * math.sin(theta) + py * math.cos(theta)
    new_c = np.floor((qx - new_x) / RES).astype(int)
    new_r = rotated.shape[0] - 1 - np.floor((qy - new_y) / RES).astype(int)
    # 찍기(splat) 덕에 옛 벽칸이 간 자리는 전부 벽이다.
    assert np.all(rotated[new_r, new_c] == GRAY_OCCUPIED)


def test_single_cell_obstacle_survives():
    """거꾸로 찾기만 하면 빠질 수 있는 한 칸 장애물도 남는다(0903_d 실측 1건)."""
    image = room_image(4.45)
    rng = np.random.default_rng(3)
    free = np.argwhere(image == GRAY_FREE)
    specks = free[rng.choice(len(free), 60, replace=False)]
    # 서로 붙지 않게 두 칸 간격 이상만 쓴다.
    image[specks[:, 0], specks[:, 1]] = GRAY_OCCUPIED
    rotated, _, _ = rotate_map(image, RES, 0.0, 0.0, 4.45)
    assert (rotated == GRAY_OCCUPIED).sum() >= (image == GRAY_OCCUPIED).sum()


def test_rotation_keeps_three_values_only():
    """색을 섞지 않는다 — 회색 중간값이 생기면 trinary 경계가 흔들린다."""
    rotated, _, _ = rotate_map(room_image(7.0), RES, 0.0, 0.0, 7.0)
    assert set(np.unique(rotated)) <= {GRAY_OCCUPIED, GRAY_FREE, GRAY_UNKNOWN}


def test_closing_fills_free_cells_only():
    """한 칸 틈은 빈칸일 때만 메우고, 모름(205) 칸은 그대로 둔다."""
    image = np.full((9, 9), GRAY_FREE, dtype=np.uint8)
    image[4, :] = GRAY_OCCUPIED
    image[4, 3] = GRAY_FREE      # 빈칸 틈 → 메운다
    image[4, 6] = GRAY_UNKNOWN   # 모름 틈 → 그대로
    filled = map_align._close_free_gaps(image)
    assert filled[4, 3] == GRAY_OCCUPIED
    assert filled[4, 6] == GRAY_UNKNOWN
    # 벽 옆 빈칸이 벽이 되지는 않는다(팽창 아님).
    assert filled[3, 3] == GRAY_FREE and filled[5, 3] == GRAY_FREE


# -- 3·4. 저장 단계 ---------------------------------------------------------

def test_align_rotates_and_hides_the_original(tmp_path):
    image = room_image(5.5)
    save_map(tmp_path, 'room', image)
    before = (tmp_path / 'room.pgm').read_bytes()

    result = straighten_saved_map(tmp_path, 'room', align=True)

    assert result['result'] == 'rotated'
    assert result['rotated_deg'] == pytest.approx(-5.5, abs=0.3)
    assert abs(tilt_of_image(read_pgm(tmp_path / 'room.pgm'))) < 0.3
    original = tmp_path / ORIGINAL_DIRNAME
    assert (original / 'room.pgm').read_bytes() == before
    memo = json.loads((original / 'room.json').read_text(encoding='utf-8'))
    assert memo['original_origin'][:2] == [-3.2, -4.1]
    assert memo['rotated_deg'] == pytest.approx(-5.5, abs=0.3)
    # B안: 숨김 폴더에 yaml·png 가 없어야 목록·Nav2 어디에도 안 뜬다.
    assert sorted(p.name for p in original.iterdir()) == ['room.json', 'room.pgm']
    # yaml 은 origin 줄만 바뀐다.
    lines, resolution, new_x, new_y = read_map_yaml(tmp_path / 'room.yaml')
    assert resolution == RES
    assert [ln for ln in lines if not ln.startswith('origin')] == [
        ln for ln in YAML_TEXT.format(name='room').splitlines()
        if not ln.startswith('origin')
    ]
    assert memo['new_origin'][:2] == [pytest.approx(new_x), pytest.approx(new_y)]
    # 임시 파일이 남지 않는다.
    assert not list(tmp_path.glob('*.align-tmp'))


@pytest.mark.parametrize('align,tilt,expected', [
    (True, 1.2, 'small'),
    (False, 5.5, 'not_requested'),
])
def test_untouched_cases_leave_files_alone(tmp_path, align, tilt, expected):
    save_map(tmp_path, 'room', room_image(tilt))
    pgm = (tmp_path / 'room.pgm').read_bytes()
    yaml = (tmp_path / 'room.yaml').read_text(encoding='utf-8')

    result = straighten_saved_map(tmp_path, 'room', align=align)

    assert result['result'] == expected
    assert result['tilt_deg'] == pytest.approx(tilt, abs=0.3)
    assert (tmp_path / 'room.pgm').read_bytes() == pgm
    assert (tmp_path / 'room.yaml').read_text(encoding='utf-8') == yaml
    assert not (tmp_path / ORIGINAL_DIRNAME).exists()


def test_messages_match_the_mockup(tmp_path):
    """목업 14번 문구. 숫자는 소수 한 자리."""
    save_map(tmp_path, 'a', room_image(1.2))
    assert straighten_saved_map(tmp_path, 'a', True)['message'] == (
        '기울기가 1.2°라 돌리지 않고 그대로 저장했습니다.'
    )
    save_map(tmp_path, 'b', room_image(5.5))
    assert straighten_saved_map(tmp_path, 'b', False)['message'] == (
        '정렬하지 않고 그대로 저장했습니다(기울기 5.5°).'
    )
    assert straighten_saved_map(tmp_path, 'b', True)['message'] == (
        '지도를 5.5° 돌려 바르게 세웠습니다.'
    )


def test_threshold_uses_the_shown_number(tmp_path):
    """1.96° 를 '2.0°라 돌리지 않았다'고 말하지 않는다 — 보이는 값으로 판정."""
    save_map(tmp_path, 'room', room_image(1.97))
    result = straighten_saved_map(tmp_path, 'room', True, min_deg=2.0)
    assert result['result'] == 'rotated'


def test_no_walls_is_reported(tmp_path):
    image = np.full((60, 60), GRAY_FREE, dtype=np.uint8)
    save_map(tmp_path, 'empty', image)
    result = straighten_saved_map(tmp_path, 'empty', True)
    assert result['result'] == 'no_walls'
    assert result['tilt_deg'] is None
    assert '벽 방향을 찾지 못해' in result['message']


def test_missing_files_fail_without_raising(tmp_path):
    result = straighten_saved_map(tmp_path, 'nothing', True)
    assert result['result'] == 'failed'


def test_orphan_original_is_set_aside_not_deleted(tmp_path):
    """손으로 지운 옛 지도의 원본이 남아 있으면 새 원본인 척하지 않게 옆으로 비킨다."""
    original = tmp_path / ORIGINAL_DIRNAME
    original.mkdir()
    (original / 'room.pgm').write_bytes(b'old')
    (original / 'room.json').write_text('{}', encoding='utf-8')
    save_map(tmp_path, 'room', room_image(5.5))

    straighten_saved_map(tmp_path, 'room', True)

    orphans = sorted(p.name for p in original.glob('room.*.orphan-*'))
    assert len(orphans) == 2
    assert (original / 'room.pgm').read_bytes() != b'old'


def test_pgm_roundtrip_reads_map_saver_header(tmp_path):
    path = tmp_path / 'x.pgm'
    path.write_bytes(b'P5\n# CREATOR: map_saver.cpp 0.050 m/pix\n3 2\n255\n'
                     + bytes([0, 254, 205, 205, 254, 0]))
    image = read_pgm(path)
    assert image.tolist() == [[0, 254, 205], [205, 254, 0]]
    write_pgm(path, image, RES)
    assert read_pgm(path).tolist() == image.tolist()


def test_nonzero_origin_yaw_is_refused(tmp_path):
    (tmp_path / 'r.yaml').write_text('resolution: 0.05\norigin: [0, 0, 0.3]\n')
    with pytest.raises(ValueError):
        read_map_yaml(tmp_path / 'r.yaml')


# -- 감독 노드와의 약속 -------------------------------------------------------

def test_cli_prints_a_result_line_the_supervisor_can_read(tmp_path, capsys):
    save_map(tmp_path, 'room', room_image(5.5))
    assert map_align.main(['save', str(tmp_path), 'room', '--align']) == 0
    parsed = parse_align_result(capsys.readouterr().out)
    assert parsed['result'] == 'rotated'
    assert parsed['tilt_deg'] == pytest.approx(5.5, abs=0.3)


def test_prefix_and_results_match_the_supervisor():
    assert RESULT_PREFIX == ALIGN_RESULT_PREFIX
    assert set(ALIGN_RESULTS) == {
        'rotated', 'small', 'no_walls', 'not_requested', 'failed'
    }


def test_parse_ignores_noise_and_takes_the_last_line():
    output = '\n'.join([
        '=== 지도 저장 ===',
        'VICA_ALIGN {broken',
        'VICA_ALIGN {"result": "small", "tilt_deg": 1.2}',
        'VICA_ALIGN {"result": "unknown_kind"}',
        'VICA_ALIGN {"result": "rotated", "tilt_deg": 5.5}',
        '앱 목록에 안 보이면 …',
    ])
    assert parse_align_result(output)['result'] == 'rotated'
    assert parse_align_result('') == {}
    assert parse_align_result('VICA_ALIGN {"result": "unknown_kind"}') == {}


def test_save_command_adds_align_only_when_asked():
    assert save_script_command('s.sh', 'm', False) == ['bash', 's.sh', 'm']
    assert save_script_command('s.sh', 'm', True) == ['bash', 's.sh', 'm', '--align']


# -- 10-07 독립 검토 반영 ------------------------------------------------------

def test_orphan_is_set_aside_even_without_rotating(tmp_path):
    """정렬 안 함·2° 미만으로 저장해도 옛 원본이 새 지도의 원본인 척하지 않는다."""
    original = tmp_path / ORIGINAL_DIRNAME
    original.mkdir()
    (original / 'room.pgm').write_bytes(b'old')
    save_map(tmp_path, 'room', room_image(5.5))

    straighten_saved_map(tmp_path, 'room', align=False)

    assert not (original / 'room.pgm').exists()
    assert len(list(original.glob('room.pgm.orphan-*'))) == 1


def test_failed_swap_restores_the_yaml_and_cleans_up(tmp_path, monkeypatch):
    """Yaml 은 바꿨는데 pgm 교체가 실패하면 yaml 을 되돌리고 원본 사본·메모도 치운다."""
    save_map(tmp_path, 'room', room_image(5.5))
    pgm = (tmp_path / 'room.pgm').read_bytes()
    yaml = (tmp_path / 'room.yaml').read_text(encoding='utf-8')
    real_replace = map_align.os.replace
    calls = []

    def flaky_replace(src, dst):
        calls.append(dst)
        if str(dst).endswith('room.pgm'):
            raise OSError('디스크 꽉 참(시험)')
        return real_replace(src, dst)

    monkeypatch.setattr(map_align.os, 'replace', flaky_replace)
    result = straighten_saved_map(tmp_path, 'room', align=True)

    assert result['result'] == 'failed'
    assert (tmp_path / 'room.pgm').read_bytes() == pgm
    assert (tmp_path / 'room.yaml').read_text(encoding='utf-8') == yaml
    assert not list((tmp_path / ORIGINAL_DIRNAME).iterdir())
    assert not list(tmp_path.glob('*.align-tmp'))


def test_chunked_rotation_matches_one_shot(monkeypatch):
    """줄 묶음으로 나눠 계산해도 한 번에 계산한 것과 칸 하나까지 같다."""
    image = room_image(5.5)
    whole, wx, wy = rotate_map(image, RES, -6.0, -6.0, 5.5)
    monkeypatch.setattr(map_align, 'ROTATE_CHUNK_CELLS', 300)
    chunked, cx, cy = rotate_map(image, RES, -6.0, -6.0, 5.5)
    assert (wx, wy) == (cx, cy)
    assert np.array_equal(whole, chunked)


def test_many_wall_points_are_thinned_but_angle_holds(monkeypatch):
    """벽칸이 많으면 건너뛰어 일부만 써도 같은 각도가 나온다(미리보기 CPU)."""
    image = room_image(-4.5, size=400, half_w=9.0, half_h=8.0)
    full = tilt_of_image(image)
    monkeypatch.setattr(map_align, 'MAX_TILT_POINTS', 300)
    thinned = tilt_of_image(image)
    assert thinned == pytest.approx(full, abs=0.2)
    assert thinned == pytest.approx(-4.5, abs=0.3)


def test_result_numbers_are_rounded_once(tmp_path):
    """앱이 받은 값을 다시 소수 한 자리로 찍어도 터미널 문구와 같은 숫자다."""
    save_map(tmp_path, 'room', room_image(5.5))
    result = straighten_saved_map(tmp_path, 'room', align=True)
    assert result['tilt_deg'] == round(result['tilt_deg'], 1)
    assert result['rotated_deg'] == -result['tilt_deg']
    assert f'{abs(result["rotated_deg"]):.1f}°' in result['message']


def test_unreadable_files_without_align_are_not_called_a_failed_rotation(tmp_path):
    """정렬을 안 골랐는데 '돌리지 못했다'고 말하지 않는다(2차 검토)."""
    (tmp_path / 'r.yaml').write_text('resolution: 0.05\norigin: [0, 0, 0.1]\n')
    write_pgm(tmp_path / 'r.pgm', room_image(5.5), RES)
    assert straighten_saved_map(tmp_path, 'r', align=False)['result'] == 'not_requested'
    assert straighten_saved_map(tmp_path, 'r', align=True)['result'] == 'failed'


def test_orphan_is_set_aside_even_when_files_cannot_be_read(tmp_path):
    original = tmp_path / ORIGINAL_DIRNAME
    original.mkdir()
    (original / 'gone.pgm').write_bytes(b'old')
    straighten_saved_map(tmp_path, 'gone', align=True)
    assert not (original / 'gone.pgm').exists()


def test_half_written_backup_is_removed(tmp_path, monkeypatch):
    """원본 사본을 쓰다가 실패해도 반쪽 파일이 원본인 척 남지 않는다(2차 검토)."""
    save_map(tmp_path, 'room', room_image(5.5))
    real_write = Path.write_bytes

    def broken_write(self, data):
        if self.parent.name == ORIGINAL_DIRNAME:
            real_write(self, data[:10])
            raise OSError('디스크 꽉 참(시험)')
        return real_write(self, data)

    monkeypatch.setattr(Path, 'write_bytes', broken_write)
    result = straighten_saved_map(tmp_path, 'room', align=True)
    assert result['result'] == 'failed'
    assert not list((tmp_path / ORIGINAL_DIRNAME).iterdir())


def test_failed_swap_restores_yaml_byte_for_byte(tmp_path, monkeypatch):
    save_map(tmp_path, 'room', room_image(5.5))
    crlf = (tmp_path / 'room.yaml').read_bytes().replace(b'\n', b'\r\n')
    (tmp_path / 'room.yaml').write_bytes(crlf)
    real_replace = map_align.os.replace

    def flaky(src, dst):
        if str(dst).endswith('room.pgm'):
            raise OSError('시험')
        return real_replace(src, dst)

    monkeypatch.setattr(map_align.os, 'replace', flaky)
    straighten_saved_map(tmp_path, 'room', align=True)
    assert (tmp_path / 'room.yaml').read_bytes() == crlf
