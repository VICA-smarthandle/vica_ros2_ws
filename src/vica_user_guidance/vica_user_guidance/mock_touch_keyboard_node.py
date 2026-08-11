"""터치센서를 키보드로 흉내내는 계측 도구.

`/vica/mock_user_contact`(std_msgs/Bool)을 주기 발행한다. 소비 쪽은
`user_guidance_driver_node` 이며 **enable_serial=false 일 때만** 구독을 연다.

## 왜 `ros2 topic pub` 이 아니라 이 도구인가

값 두 개를 바꾸는 것이 목적이 아니라 **잡았다 놓는 시점**을 만드는 것이 목적이다.
확인해야 하는 네 가지 중 셋이 시점에 달려 있다.

    1. 안 잡음                    -> 진입 안 됨
    2. 3초 잡음                   -> 진입
    3. 0.3초 놓침                 -> 판정 유예(0.5초) 안. 주행 유지
    4. 1초 놓침                   -> 정지 -> 재접촉 -> 재개

3·4 는 손으로 타이밍을 만들어야 재현된다. 명령을 두 번 치는 방식으로는
0.3초와 1초를 가를 수 없다.

`mdrobot_can_keyboard_knob_node` 와 같은 방식이다. 새 패턴이 아니다.

## 안전

이 노드는 진단 표시값 하나만 바꾼다. `/cmd_vel*` 도 E-stop 도 건드리지 않는다.
그래도 **실기 운용에서 띄우지 않는다** — user_contact 가 참이면 손 놓음 정지가
발동하지 않으므로, 사용자가 손을 놓아도 로봇이 계속 간다.

기본값은 False 다. 시작하자마자 "잡고 있다"가 되지 않는다(fail-safe 방향).

실행:
    ros2 run vica_user_guidance mock_touch_keyboard
"""
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# 발행 주기. 소비 쪽이 신선도를 보지 않더라도 끊긴 것과 놓은 것을 구분할 수 있게
# 계속 보낸다. 20 Hz 는 상향 프로토콜 목표치와 같다(vica_scenario.md 2-1.3).
PUBLISH_HZ = 20.0

_HELP = """
[터치센서 mock]  /vica/mock_user_contact

  스페이스 : 잡음/놓음 토글
  h        : 잡음 (hold)
  r        : 놓음 (release)
  q        : 종료 (놓음 상태로 끝낸다)

지금 상태: {state}
"""


class MockTouchKeyboardNode(Node):
    def __init__(self) -> None:
        super().__init__('mock_touch_keyboard_node')
        self.pub = self.create_publisher(Bool, '/vica/mock_user_contact', 10)
        self.contact = False
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_once)
        self.get_logger().info(
            'MOCK 터치센서 키보드. 계측 전용이며 실기 운용에서 띄우지 않는다.'
        )

    def publish_once(self) -> None:
        msg = Bool()
        msg.data = self.contact
        self.pub.publish(msg)

    def set_contact(self, value: bool) -> None:
        if value != self.contact:
            self.get_logger().info(f'user_contact -> {value}')
        self.contact = value


def _read_keys(node: MockTouchKeyboardNode) -> None:
    """터미널을 raw 모드로 두고 한 글자씩 읽는다.

    설정을 반드시 되돌린다 — 되돌리지 않으면 노드가 끝난 뒤에도 터미널이
    입력을 표시하지 않아 사용자가 셸이 죽은 줄 안다.
    """
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key == ' ':
                node.set_contact(not node.contact)
            elif key in ('h', 'H'):
                node.set_contact(True)
            elif key in ('r', 'R'):
                node.set_contact(False)
            elif key in ('q', 'Q', '\x03'):   # q 또는 Ctrl-C
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        # 놓은 상태로 끝낸다. 잡은 채로 종료하면 마지막 값이 참으로 남아
        # 소비 쪽이 "잡고 있다"로 읽은 채 멈춘다.
        node.set_contact(False)
        node.publish_once()
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = MockTouchKeyboardNode()
    print(_HELP.format(state='놓음'))
    reader = threading.Thread(target=_read_keys, args=(node,), daemon=True)
    reader.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
