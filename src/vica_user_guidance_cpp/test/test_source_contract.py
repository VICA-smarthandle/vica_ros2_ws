"""C++ 판 소스 계약 — vica_user_guidance/test/test_launch_contract.py 의 안전 경계 검사를 C++ 에도 건다.

주석을 뺀 실행 코드(문자열 포함 — 토픽 이름이 문자열이다)에 주행 명령·goal·reset 심볼이 없어야 하고,
/estop_state 는 구독만 한다. 실행 파일 이름은 파이썬 판과 같아야 한다(터미네이터 guard).
"""
import pathlib
import re

PKG = pathlib.Path(__file__).resolve().parents[1]
SOURCES = sorted((PKG / 'src').glob('*.cpp')) + sorted((PKG / 'include').rglob('*.hpp'))

FORBIDDEN = ['cmd_vel_req', 'cmd_vel_safe', 'NavigateToPose', 'estop_reset', 'safety_reset']


def strip_comments(code):
    code = re.sub(r'/\*.*?\*/', ' ', code, flags=re.DOTALL)
    return re.sub(r'//[^\n]*', ' ', code)


def test_sources_never_touch_drive_command_path():
    assert SOURCES
    for path in SOURCES:
        code = strip_comments(path.read_text(encoding='utf-8'))
        for symbol in FORBIDDEN:
            assert symbol not in code, f'{path.name} 실행 코드에 {symbol!r}'


def test_estop_state_is_subscribed_never_published():
    node = (PKG / 'src' / 'user_guidance_driver_node.cpp').read_text(encoding='utf-8')
    assert re.search(r'create_subscription<std_msgs::msg::Bool>\(\s*"/estop_state"', node)
    for call in re.finditer(r'create_publisher<[^>]*>\(\s*("[^"]*")', node):
        assert call.group(1) == '"/vica/smart_handle_state"', call.group(0)


def test_executable_and_node_names_match_python():
    cmake = (PKG / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'add_executable(user_guidance_driver_node ' in cmake
    node = (PKG / 'src' / 'user_guidance_driver_node.cpp').read_text(encoding='utf-8')
    assert 'rclcpp::Node("user_guidance_driver_node")' in node


def test_parameter_names_and_order_match_python():
    """C++ 노드가 선언하는 파라미터 이름·순서가 파이썬 판 declare_parameter 와 같아야 한다."""
    py = (PKG.parent / 'vica_user_guidance' / 'vica_user_guidance' /
          'user_guidance_driver_node.py').read_text(encoding='utf-8')
    cpp = (PKG / 'src' / 'user_guidance_driver_node.cpp').read_text(encoding='utf-8')
    py_names = re.findall(r'declare_parameter\(\s*"([a-z_]+)"', py)
    # std::vector<std::string>> 처럼 꺾쇠가 겹치므로 '(' 앞의 마지막 '>' 까지를 형으로 본다.
    cpp_names = re.findall(r'declare_parameter<[^()]+?>\(\s*"([a-z_]+)"', cpp)
    assert py_names and cpp_names == py_names
