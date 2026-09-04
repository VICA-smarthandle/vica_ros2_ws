"""Smart Handle 시리얼 전송 래퍼.

포트가 없어도 예외를 밖으로 던지지 않는다. 개발 PC와 하드웨어 미연결 운영 양쪽에서
노드가 계속 살아 있어야 한다. 전송 실패는 예외가 아니라 반환값과 fault_code로
보고한다.
"""

from typing import Callable, Optional

from . import protocol

# SmartHandleState.msg의 fault_code 상수와 일치해야 한다.
FAULT_NONE = 0
FAULT_PORT_OPEN = 1
FAULT_WRITE_FAIL = 2
FAULT_NOT_CONFIGURED = 3

DEFAULT_WRITE_TIMEOUT_SEC = 0.05
DEFAULT_RECONNECT_INTERVAL_NS = 2_000_000_000


class SerialLink:
    """1바이트 상태코드를 아두이노로 보낸다."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        enabled: bool = True,
        write_timeout_sec: float = DEFAULT_WRITE_TIMEOUT_SEC,
        reconnect_interval_ns: int = DEFAULT_RECONNECT_INTERVAL_NS,
        serial_factory: Optional[Callable[..., object]] = None,
    ) -> None:
        self.port_name = port
        self.baudrate = baudrate
        self.enabled = enabled
        self.write_timeout_sec = write_timeout_sec
        self.reconnect_interval_ns = reconnect_interval_ns

        self._factory = serial_factory
        self._port = None
        self._fault_code = FAULT_NOT_CONFIGURED
        self._last_state_code = protocol.STATE_NORMAL
        self._write_error_count = 0
        self._last_attempt_ns: Optional[int] = None

        if self.enabled:
            self._open()

    # ── 상태 조회 ──────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self._port is not None

    @property
    def fault_code(self) -> int:
        return self._fault_code

    @property
    def last_state_code(self) -> int:
        return self._last_state_code

    @property
    def write_error_count(self) -> int:
        return self._write_error_count

    # ── 전송 ───────────────────────────────────────────

    def send(self, state_code: int, now_ns: int) -> bool:
        """상태코드 1바이트를 전송한다. 성공하면 True.

        전송 불가 코드는 ValueError를 던진다 — 이는 런타임 장애가 아니라
        프로그래밍 오류이며, 조용히 넘기면 코드 4가 펌웨어 워치독을 무력화한다.
        """
        if not protocol.is_sendable(state_code):
            raise ValueError(
                f"전송할 수 없는 상태코드입니다: {state_code} "
                f"({protocol.STATE_NAMES.get(state_code, '?')}). "
                "코드 4는 펌웨어 워치독 전용이고 6·7은 펌웨어 미구현입니다."
            )

        if self._port is None:
            return False

        try:
            self._port.write(bytes([state_code]))
            self._port.flush()
        except Exception:
            # 어떤 예외든 밖으로 던지지 않는다. 안내 실패가 노드를 죽이면 안 된다.
            self._write_error_count += 1
            self._fault_code = FAULT_WRITE_FAIL
            self._close_port()
            self._last_attempt_ns = now_ns
            return False

        self._last_state_code = state_code
        self._fault_code = FAULT_NONE
        return True

    def send_raw(self, byte: int, now_ns: int) -> bool:
        """상태코드가 아닌 바이트(햅틱 명령 등)를 1개 전송한다. 성공하면 True.

        send() 와 갈라 둔 이유는 last_state_code 다. 햅틱 바이트를 send() 로 보내면
        진단이 "마지막 상태 16"이라는 헛소리를 하게 된다 — 그 필드는 아두이노가
        마지막으로 받은 **표시 상태**여야 한다. 오류 처리는 send() 와 같다.

        [주의] 상태코드 범위(0~7)는 여기로 보내지 않는다. 워치독 계약(코드 4 금지)
        을 우회하게 되기 때문이다. 그 검사는 send() 에만 있다.
        """
        if 0 <= byte <= protocol.STATE_CHARGED:
            raise ValueError(
                f"상태코드 범위({byte})는 send_raw 로 보내지 않는다. send() 를 쓸 것."
            )

        if self._port is None:
            return False

        try:
            self._port.write(bytes([byte]))
            self._port.flush()
        except Exception:
            self._write_error_count += 1
            self._fault_code = FAULT_WRITE_FAIL
            self._close_port()
            self._last_attempt_ns = now_ns
            return False

        self._fault_code = FAULT_NONE
        return True

    # ── 수신 ───────────────────────────────────────────

    def read_available(self, now_ns: int) -> bytes:
        """수신 버퍼에 쌓인 바이트를 전부 반환한다. 없거나 미연결이면 b"".

        블로킹하지 않는다 — in_waiting 만큼만 읽는다. 단일 스레드 executor 에서
        read 가 기다리면 상태코드 전송까지 같이 멈춘다.
        """
        if self._port is None:
            return b""
        try:
            waiting = self._port.in_waiting
            if not waiting:
                return b""
            data = self._port.read(waiting)
            return bytes(data) if data else b""
        except Exception:
            # 수신 중 단절도 전송 실패와 같은 경로로 처리한다. 포트를 닫아야
            # maybe_reconnect 가 살아난다.
            self._fault_code = FAULT_WRITE_FAIL
            self._close_port()
            self._last_attempt_ns = now_ns
            return b""

    # ── 재연결 ─────────────────────────────────────────

    def maybe_reconnect(self, now_ns: int) -> None:
        """backoff 간격이 지났으면 재연결을 1회 시도한다.

        지수 backoff를 쓰지 않는다. 안내 장치라 재시도 부담이 낮고, 케이블 재삽입
        후 복구가 빠른 편이 낫다.
        """
        if not self.enabled or self._port is not None:
            return

        if self._last_attempt_ns is None:
            # 생성자는 now_ns를 모르므로 여기서 기준점을 잡는다. 첫 호출은 시도하지
            # 않고 시각만 기록해야 backoff 간격이 실제로 지켜진다.
            self._last_attempt_ns = now_ns
            return

        elapsed = now_ns - self._last_attempt_ns
        if 0 <= elapsed < self.reconnect_interval_ns:
            return

        self._last_attempt_ns = now_ns
        self._open()

    def close(self) -> None:
        """포트를 닫는다. 열린 적 없어도 안전하다."""
        self._close_port()

    # ── 내부 ───────────────────────────────────────────

    def _open(self) -> None:
        factory = self._factory or self._default_factory
        try:
            self._port = factory(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=self.write_timeout_sec,
            )
        except Exception:
            self._port = None
            self._fault_code = FAULT_PORT_OPEN
            return
        self._fault_code = FAULT_NONE

    @staticmethod
    def _default_factory(**kwargs):
        """pyserial을 지연 import한다.

        enable_serial=False면 이 함수가 불리지 않으므로, pyserial이 설치되지 않은
        환경에서도 노드와 테스트가 동작한다.
        """
        import serial  # noqa: PLC0415 — 지연 import가 의도다

        return serial.Serial(**kwargs)

    def _close_port(self) -> None:
        if self._port is not None:
            try:
                self._port.close()
            except Exception:
                pass
            self._port = None
