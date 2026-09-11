from mdrobot_can_control import mdrobot_can_keyboard_knob_node as motor_node

import pytest


def write_flags(tmp_path, interface, value):
    flags_dir = tmp_path / interface
    flags_dir.mkdir()
    (flags_dir / 'flags').write_text(value, encoding='utf-8')


def test_can_preflight_accepts_interface_with_iff_up(tmp_path):
    write_flags(tmp_path, 'can1', '0x1\n')

    motor_node.require_can_interface_up('can1', sys_class_net=tmp_path)


def test_can_preflight_blocks_down_interface(tmp_path):
    write_flags(tmp_path, 'can1', '0x0\n')

    with pytest.raises(
        RuntimeError,
        match=r'\[MOTOR START BLOCKED\].*can1.*DOWN',
    ):
        motor_node.require_can_interface_up('can1', sys_class_net=tmp_path)


def test_can_preflight_blocks_missing_interface(tmp_path):
    with pytest.raises(
        RuntimeError,
        match=r'\[MOTOR START BLOCKED\].*can1.*does not exist',
    ):
        motor_node.require_can_interface_up('can1', sys_class_net=tmp_path)
