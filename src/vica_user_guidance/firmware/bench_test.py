#!/usr/bin/env python3
"""VICA Smart Handle 펌웨어 bench 시험 도구.

로봇과 분리한 상태에서 아두이노에 상태코드를 보내고 눈으로 확인한다.
주행 명령을 발행하지 않으며 ROS와 무관하게 단독 실행된다.

설계 계획서: 작업공간 루트의 devlog/2026-07-28-smart-handle-guidance-plan.md (Phase 4)
(문서는 별도 저장소에 있다. 이 파일은 vica_ros2_ws 저장소에 있다.)

펌웨어 빌드·업로드:
    export PATH="$HOME/bin:$PATH"
    arduino-cli compile --fqbn arduino:avr:nano firmware/smart_handle_firmware
    arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano \
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

DEFAULT_PORT = "/dev/ttyUSB0"
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


def case_6(b: Bench) -> None:
    b.hold(0, 1.0)          # NORMAL에서 시작해야 상태 변화가 감지된다
    b.hold(5, 3.0)          # arrival_hold_sec 상당


def case_7(b: Bench) -> None:
    print("  NORMAL → 코드 5를 빠르게 5회 연속 전송합니다.")
    b.hold(0, 1.0)
    for i in range(5):
        b.send(5)
        print(f"    5 전송 ({i + 1}/5)")
        time.sleep(0.05)
    print("  이어서 3초간 코드 5를 계속 전송합니다 (ROS 재현).")
    b.hold(5, 3.0)


def case_8(b: Bench) -> None:
    b.hold(0, 2.0)
    print()
    print("  *** 지금 USB 케이블을 뽑으세요 ***")
    print("  (아두이노 전원은 별도로 유지되어야 합니다.")
    print("   USB로만 전원을 공급 중이라면 이 항목은 건너뛰세요)")
    input("  뽑았으면 Enter...")
    print("  1.5초 이상 대기 후 LED를 확인하세요.")
    time.sleep(3.0)


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
     "한쪽 줄에 주황색 물결. 어느 쪽인지 기록할 것. 서보 회전 방향도 기록",
     "LED A/B 매핑 + 서보 방향 [미검증]"),
    (4, "우회전 안내",
     case_4,
     "3번과 반대쪽 줄에 주황색 물결. 서보도 반대 방향",
     "LED A/B 매핑 + 서보 방향 [미검증]"),
    (5, "비상정지 표시",
     case_5,
     "양쪽 주황색 빠른 점멸(0.3초). 서보는 직전 위치 유지",
     "ESTOP"),
    (6, "도착 표시",
     case_6,
     "녹색 점멸 정확히 3회 후 하늘색 복귀",
     "BLINK_ARRIVE 자동 종료"),
    (7, "도착 반복 전송",
     case_7,
     "코드 5를 여러 번 보내도 녹색 점멸은 정확히 3회. 반복되면 실패",
     "무한 반복 버그 재발 방지 (핵심)"),
    (8, "수신 중 단절",
     case_8,
     "USB 분리 1.5초 뒤 양쪽 빨간색 상시 점등(점멸 아님), 서보 중앙 복귀",
     "워치독 + LINK_LOST"),
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
    parser.add_argument("--dry-run", action="store_true",
                        help="포트를 열지 않고 전송 내용만 출력")
    args = parser.parse_args()

    if args.list:
        print_list()
        return 0

    if not any([args.case, args.all, args.send is not None, args.hold is not None]):
        parser.print_help()
        return 1

    b = Bench(args.port, args.baud, args.dry_run)
    try:
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
