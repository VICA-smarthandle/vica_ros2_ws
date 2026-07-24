"""Read-only SocketCAN interface checks for motor startup."""

from pathlib import Path


IFF_UP = 0x1


def require_can_interface_up(interface: str, sys_class_net='/sys/class/net') -> int:
    """Return interface flags or block motor startup when IFF_UP is absent."""
    interface = str(interface)
    if not interface or Path(interface).name != interface:
        raise RuntimeError(
            f'[MOTOR START BLOCKED] invalid CAN interface name: {interface!r}'
        )

    flags_path = Path(sys_class_net) / interface / 'flags'
    try:
        raw_flags = flags_path.read_text(encoding='utf-8').strip()
    except FileNotFoundError as exc:
        raise RuntimeError(
            f'[MOTOR START BLOCKED] CAN interface {interface} does not exist'
        ) from exc
    except OSError as exc:
        raise RuntimeError(
            f'[MOTOR START BLOCKED] cannot read CAN interface {interface}: {exc}'
        ) from exc

    try:
        flags = int(raw_flags, 0)
    except ValueError as exc:
        raise RuntimeError(
            f'[MOTOR START BLOCKED] invalid flags for CAN interface '
            f'{interface}: {raw_flags!r}'
        ) from exc

    if not flags & IFF_UP:
        raise RuntimeError(
            f'[MOTOR START BLOCKED] CAN interface {interface} is DOWN; '
            'configure and bring it UP before starting the motor'
        )
    return flags
