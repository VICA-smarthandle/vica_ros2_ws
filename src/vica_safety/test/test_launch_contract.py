from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_safety_launch_contains_three_safety_nodes_and_no_motor():
    text = read("src/vica_safety/launch/safety_bringup.launch.py")

    assert 'executable="emergency_stop_node"' in text
    assert 'executable="safety_supervisor_node"' in text
    assert 'executable="app_emergency_node"' in text
    assert "keyboard_knob" not in text


def test_safety_launch_forces_can_f1_can1_and_colored_output():
    text = read("src/vica_safety/launch/safety_bringup.launch.py")

    for token in (
        '"input_mode": "can_f1"',
        '"can_iface": "can1"',
        '"driver_response_id": 0x701',
        '"RCUTILS_COLORIZED_OUTPUT", "1"',
    ):
        assert token in text


def test_motor_launch_contains_only_keyboard_knob():
    text = read("src/mdrobot_can_control/launch/motor_bringup.launch.py")

    assert 'executable="keyboard_knob"' in text
    assert "emergency_stop_node" not in text
    assert "safety_supervisor_node" not in text
    assert "app_emergency_node" not in text


def test_wheel_ekf_launch_does_not_start_motor():
    text = read("src/vica_localization/launch/wheel_ekf.launch.py")

    assert "mdrobot_can_control" not in text
    assert "keyboard_knob" not in text


def test_vica_safety_registers_nodes_and_action_dependency():
    setup_text = read("src/vica_safety/setup.py")
    package_text = read("src/vica_safety/package.xml")

    for executable in (
        "emergency_stop_node",
        "safety_supervisor_node",
        "app_emergency_node",
    ):
        assert f'"{executable} = vica_safety.' in setup_text
    assert "<depend>action_msgs</depend>" in package_text


def test_motor_package_no_longer_registers_safety_nodes():
    setup_text = read("src/mdrobot_can_control/setup.py")

    assert "emergency_stop_node =" not in setup_text
    assert "safety_supervisor_node =" not in setup_text
    assert "app_emergency_node =" not in setup_text


def test_mission_manager_uses_only_authoritative_central_latch():
    text = read(
        "src/vica_mission_manager/vica_mission_manager/mission_manager_node.py"
    )

    assert '"/emergency_stop"' in text
    assert '"/estop_state"' not in text
    assert "_estop_latched" not in text
