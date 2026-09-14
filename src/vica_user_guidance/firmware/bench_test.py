#!/usr/bin/env python3
"""VICA Smart Handle 펌웨어 bench 시험 도구.

로봇과 분리한 상태에서 아두이노에 상태코드를 보내고 눈으로 확인한다.
주행 명령을 발행하지 않으며 ROS와 무관하게 단독 실행된다.

설계 계획서: 작업공간 루트의 devlog/2026-07-28-smart-handle-guidance-plan.md (Phase 4)
(문서는 별도 저장소에 있다. 이 파일은 vica_ros2_ws 저장소에 있다.)

펌웨어 빌드·업로드:
    export PATH="$HOME/bin:$PATH"
    arduino-cli compile --fqbn arduino:avr:nano firmware/smart_handle_firmware
    arduino-cli upload -p /dev/vica_smart_handle --fqbn arduino:avr:nano \
        firmware/smart_handle_firmware

사용법:
    python3 bench_test.py --list              # 시험 항목 보기
    python3 bench_test.py --case 1            # 1번 항목 실행
    python3 bench_test.py --all               # 전체 순차 실행
    python3 bench_test.py --send 5            # 코드 1회 수동 전송
    python3 bench_test.py --hold 2            # 코드를 10Hz로 계속 전송(Ctrl+C 중단)
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit(
        "pyserial이 필요합니다:\n"
        "  sudo apt install python3-serial\n"
        "  또는  pip install pyserial"
    )

# udev 규칙이 부여하는 고정 이름. 규칙 미설치 장비에서는 --port로 지정한다.
DEFAULT_PORT = "/dev/vica_smart_handle"
DEFAULT_BAUD = 115200

# 펌웨어 상태코드 (smart_handle_firmware.ino와 일치해야 한다)
STATE_NAMES = {
    0: "NORMAL",
    1: "LEFT",
    2: "RIGHT",
    3: "ESTOP",
    4: "LINK_LOST",
    5: "ARRIVED",
}

SEND_HZ = 10.0  # ROS 드라이버 노드와 동일한 주기

# 햅틱 수동 명령 (smart_handle_firmware.ino HAPTIC_CMD_* / protocol.py 와 일치).
# 상태코드가 아니라 applyState() 를 거치지 않는다 — LED·서보는 그대로, D10 의
# 진동모터만 패턴대로 한 번 떨린다. 드라이버 노드는 이 바이트를 보내지 않는다.
HAPTIC_CMDS = {
    "short": (0x10, "300ms on/150ms off x3 — 도착 패턴"),
    "long":  (0x11, "1200ms x1 — 비상 패턴"),
}

# 펌웨어 WATCHDOG_TIMEOUT_MS와 일치해야 한다 (8번 항목 안내 문구용)
WATCHDOG_TIMEOUT_MS = 1500


class Bench:
    """시리얼 포트로 1바이트 상태코드를 전송한다."""

    def __init__(self, port: str, baud: int, dry_run: bool = False) -> None:
        self.port_name = port
        self.dry_run = dry_run
        self.ser = None
        if dry_run:
            print(f"[dry-run] 포트를 열지 않습니다 ({port})")
            return
        try:
            # 나노는 DTR 토글로 리셋된다. 포트를 연 뒤 부트로더가 끝날 때까지 기다린다.
            self.ser = serial.Serial(port, baud, timeout=1, write_timeout=1)
        except serial.SerialException as exc:
            sys.exit(
                f"포트를 열 수 없습니다: {port}\n  {exc}\n\n"
                "확인 사항:\n"
                "  - 아두이노가 연결되어 있는가 (ls /dev/ttyUSB* /dev/ttyACM*)\n"
                "  - dialout 그룹에 속해 있는가 (groups | grep dialout)\n"
                "    미소속이면: sudo usermod -aG dialout $USER  후 재로그인\n"
                "  - 아두이노 IDE 시리얼 모니터가 포트를 점유하고 있지 않은가"
            )
        print(f"포트 열림: {port} @ {baud}")
        print("아두이노 리셋 대기 중... (2초)")
        time.sleep(2.0)

    def send(self, code: int) -> None:
        """상태코드 1바이트를 전송한다."""
        name = STATE_NAMES.get(code, "?")
        if self.dry_run or self.ser is None:
            print(f"  [dry-run] send {code} ({name})")
            return
        self.ser.write(bytes([code]))
        self.ser.flush()

    def hold(self, code: int, seconds: float) -> None:
        """상태코드를 SEND_HZ로 반복 전송한다 (ROS 드라이버와 동일한 동작)."""
        name = STATE_NAMES.get(code, "?")
        print(f"  코드 {code} ({name}) 를 {seconds:.1f}초간 {SEND_HZ:.0f}Hz로 전송")
        deadline = time.monotonic() + seconds
        interval = 1.0 / SEND_HZ
        while time.monotonic() < deadline:
            self.send(code)
            time.sleep(interval)

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()


def ask(prompt: str) -> bool:
    """기대 동작과 실제를 대조해 사용자에게 판정을 받는다."""
    try:
        answer = input(f"  >> {prompt} [y/n/s(kip)]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return answer.startswith("y")


# ── 시험 항목 ─────────────────────────────────────────────
# 각 항목: (번호, 제목, 실행 함수, 기대 동작, 검증 대상)

def case_1(b: Bench) -> None:
    print("  아무것도 전송하지 않고 30초 대기합니다.")
    print("  (아두이노를 방금 연결/리셋한 상태여야 의미가 있습니다)")
    for remaining in range(30, 0, -5):
        print(f"    {remaining}초 남음...", flush=True)
        time.sleep(5)


def case_2(b: Bench) -> None:
    b.hold(0, 3.0)


def case_3(b: Bench) -> None:
    b.hold(1, 5.0)


def case_4(b: Bench) -> None:
    b.hold(2, 5.0)


def case_5(b: Bench) -> None:
    b.hold(3, 4.0)


# config/user_guidance.yaml의 arrival_hold_sec와 같은 값이어야 한다.
# 펌웨어 재생 시간(3.5초)보다 짧으면 마지막 소등 프레임이 잘린다.
ARRIVAL_HOLD_SEC = 4.0


def case_6(b: Bench) -> None:
    b.hold(0, 1.0)                    # NORMAL에서 시작해야 상태 변화가 감지된다
    b.hold(5, ARRIVAL_HOLD_SEC)       # ROS driver가 유지하는 시간
    print("  코드 0으로 복귀 (ROS 재현) — 하늘색 상시 점등이어야 한다.")
    b.hold(0, 1.5)


def case_7(b: Bench) -> None:
    print("  NORMAL → 코드 5를 빠르게 5회 연속 전송합니다.")
    b.hold(0, 1.0)
    for i in range(5):
        b.send(5)
        print(f"    5 전송 ({i + 1}/5)")
        time.sleep(0.05)
    print(f"  이어서 {ARRIVAL_HOLD_SEC:.1f}초간 코드 5를 계속 전송합니다 (ROS 재현).")
    b.hold(5, ARRIVAL_HOLD_SEC)
    b.hold(0, 1.5)


def case_8(b: Bench) -> None:
    """단절을 '전송 중단'으로 재현한다.

    [설계 결정 2026-07-29] USB가 아두이노의 유일한 전원이라 케이블을 뽑으면
    아두이노도 함께 꺼져 LINK_LOST 표시를 볼 수 없다. 대신 포트는 열어둔 채
    전송만 멈춘다. 펌웨어 워치독은 lastRxMillis만 보므로 "1.5초 무신호"라는
    점에서 물리적 단절과 완전히 동일하다.

    전송 중단 방식은 오히려 낫다 — 케이블을 뽑으면 확인할 수 없는 **복구**
    (전송 재개 시 하늘색 복귀)까지 같은 항목에서 검증할 수 있다.
    """
    # 사람이 눈으로 색을 확인하는 시험이라 각 구간을 넉넉히 잡는다. 2~3초로는
    # 시선을 옮겨 색을 인지하기 전에 구간이 끝난다(2026-07-29 실측).
    print("  먼저 정상 수신 상태를 만듭니다 (하늘색).")
    b.hold(0, 5.0)
    print()
    print("  *** 지금부터 전송을 중단합니다. LED를 보세요 ***")
    print(f"  펌웨어는 {WATCHDOG_TIMEOUT_MS}ms 무신호 시 LINK_LOST로 전환합니다.")
    for elapsed in range(1, 9):
        time.sleep(1.0)
        mark = "  <- 여기서 빨간불" if elapsed == 2 else ""
        print(f"    무신호 {elapsed}초{mark}", flush=True)
    print()
    if not ask("양쪽 빨간색 상시 점등(점멸 아님)을 확인했습니까?"):
        print("  ! 단절 표시 실패 — 복구 확인은 의미가 없으므로 건너뜁니다.")
        return
    print()
    print("  전송을 재개합니다 — 하늘색으로 복귀해야 합니다.")
    b.hold(0, 6.0)


CASES = [
    (1, "부팅 직후 워치독 보류",
     case_1,
     "하늘색 유지. 빨간불이 뜨지 않아야 한다",
     "everConnected (핵심)"),
    (2, "기본 상태",
     case_2,
     "양쪽 하늘색 상시 점등, 서보 중앙(90도)",
     "NORMAL"),
    (3, "좌회전 안내",
     case_3,
     "왼쪽 줄에 주황색 물결, 서보 왼쪽 (2026-07-28 실측 확정)",
     "LED A/B 매핑 + 서보 방향"),
    (4, "우회전 안내",
     case_4,
     "오른쪽 줄에 주황색 물결, 서보 오른쪽 (2026-07-28 실측 확정)",
     "LED A/B 매핑 + 서보 방향"),
    (5, "비상정지 표시",
     case_5,
     "양쪽 주황색 빠른 점멸(0.3초). 서보는 중앙 복귀",
     "ESTOP"),
    (6, "도착 표시",
     case_6,
     "하늘색 점멸 정확히 3회(ON/OFF 각 0.5초) 후 하늘색 상시 복귀",
     "BLINK_ARRIVE 자동 종료"),
    (7, "도착 반복 전송",
     case_7,
     "코드 5를 여러 번 보내도 하늘색 점멸은 정확히 3회. 반복되면 실패",
     "무한 반복 버그 재발 방지 (핵심)"),
    (8, "수신 중 단절 (전송 중단 방식)",
     case_8,
     "전송 중단 1.5초 뒤 양쪽 빨간색 상시 점등(점멸 아님), 서보 중앙. "
     "전송 재개 시 하늘색 복귀",
     "워치독 + LINK_LOST (핵심)"),
]


def print_list() -> None:
    print("\nbench 시험 항목\n")
    for num, title, _, expect, target in CASES:
        mark = " *" if "핵심" in target else "  "
        print(f"{mark}{num}. {title}")
        print(f"     기대: {expect}")
        print(f"     검증: {target}")
    print("\n  * = 이번 확장의 핵심 검증 항목\n")


def run_case(b: Bench, entry) -> bool:
    num, title, func, expect, target = entry
    print()
    print("=" * 62)
    print(f"[{num}] {title}   —   검증: {target}")
    print("=" * 62)
    print(f"  기대 동작: {expect}")
    print()
    func(b)
    print()
    if num != 8:
        # 판정 대기 중에는 전송이 멈추므로 워치독이 발동한다. 8번은 그것 자체가
        # 시험 대상이라 제외한다.
        print("  (판정 대기 중 전송이 멈춰 1.5초 뒤 빨간불이 뜹니다 — 정상)")
    return ask("기대대로 동작했습니까?")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Smart Handle 펌웨어 bench 시험",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"기본: {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--list", action="store_true", help="시험 항목만 출력")
    parser.add_argument("--case", type=int, help="특정 항목 실행 (1~8)")
    parser.add_argument("--all", action="store_true", help="전체 순차 실행")
    parser.add_argument("--send", type=int, help="상태코드 1회 전송")
    parser.add_argument("--hold", type=int, help="상태코드를 10Hz로 계속 전송")
    parser.add_argument("--haptic", choices=sorted(HAPTIC_CMDS),
                        help="진동모터 수동 시험: short(도착 3회) / long(비상 1회)")
    parser.add_argument("--dry-run", action="store_true",
                        help="포트를 열지 않고 전송 내용만 출력")
    args = parser.parse_args()

    if args.list:
        print_list()
        return 0

    if not any([args.case, args.all, args.send is not None,
                args.hold is not None, args.haptic]):
        parser.print_help()
        return 1

    b = Bench(args.port, args.baud, args.dry_run)
    try:
        if args.haptic:
            code, desc = HAPTIC_CMDS[args.haptic]
            b.send(code)
            print(f"전송: 0x{code:02X} 햅틱 {args.haptic} ({desc})")
            print("  로봇과 분리한 상태에서 모터가 그 패턴대로 떨리면 정상입니다.")
            print("  핸들 노드가 떠 있으면 포트를 못 열어 실패합니다 — 노드를 먼저 내리세요.")
            return 0

        if args.send is not None:
            b.send(args.send)
            print(f"전송: {args.send} ({STATE_NAMES.get(args.send, '?')})")
            return 0

        if args.hold is not None:
            print("Ctrl+C로 중단")
            try:
                while True:
                    b.send(args.hold)
                    time.sleep(1.0 / SEND_HZ)
            except KeyboardInterrupt:
                print("\n중단됨")
            return 0

        if args.case:
            entry = next((c for c in CASES if c[0] == args.case), None)
            if entry is None:
                print(f"항목 {args.case} 없음. --list 로 확인하세요.")
                return 1
            ok = run_case(b, entry)
            print(f"\n결과: {'PASS' if ok else 'FAIL / 미확인'}")
            return 0 if ok else 1

        # --all
        results = []
        for entry in CASES:
            results.append((entry[0], entry[1], run_case(b, entry)))

        print()
        print("=" * 62)
        print("시험 요약")
        print("=" * 62)
        for num, title, ok in results:
            print(f"  {num}. {title:<24} {'PASS' if ok else 'FAIL / 미확인'}")
        failed = [n for n, _, ok in results if not ok]
        print()
        if failed:
            print(f"확인 필요 항목: {failed}")
            return 1
        print("전체 PASS")
        return 0
    finally:
        # 시험 종료 시 기본 상태로 되돌린다.
        try:
            b.send(0)
        except Exception:
            pass
        b.close()


if __name__ == "__main__":
    sys.exit(main())
