"""vica_2d.lua 가 앱과 맺은 약속을 고정한다."""

from pathlib import Path

CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'vica_2d.lua'


def test_tracked_pose_is_published_for_the_app_preview():
    text = CONFIG.read_text(encoding='utf-8')
    assert 'publish_tracked_pose = true' in text, (
        'vica_2d.lua 의 publish_tracked_pose 가 꺼져 있으면 매핑 화면에 로봇 '
        '화살표가 뜨지 않는다 (map_preview_node 가 /tracked_pose 를 구독한다).'
    )
