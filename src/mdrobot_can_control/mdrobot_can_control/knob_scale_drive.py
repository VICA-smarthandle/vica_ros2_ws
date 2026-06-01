#!/usr/bin/env python3
import time
import can

# ====== User settings ======
CAN_IFACE = "can1"
DRIVER_ID = 0x001

BASE_RPM = 300            # 필요하면 100, 300, 600 등 조절
SEND_HZ = 50

DEADZONE_PCT = 5          # 0~100 중 5 이하면 0
MIN_RPM_WHEN_MOVING = 40  # 0이면 비활성

KNOB_TIMEOUT_SEC = 0.8    # F1 끊기면 정지까지 허용 시간

# ====== Protocol ======
PID_PNT_IO_MONITOR = 0xF1  # 241
PID_PNT_VEL_CMD    = 0xCF  # 207
PID_COMMAND        = 0x0A  # 10
CMD_PNT_IO_MON_ON  = 0x55  # 85

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def le_i16_signed(value: int):
    value = int(value)
    value = clamp(value, -32768, 32767)
    u = value & 0xFFFF
    return u & 0xFF, (u >> 8) & 0xFF

def send_pnt_io_monitor_on(bus: can.Bus):
    bus.send(can.Message(
        arbitration_id=DRIVER_ID,
        is_extended_id=False,
        data=bytes([PID_COMMAND, CMD_PNT_IO_MON_ON, 0, 0, 0, 0, 0, 0])
    ))

def send_vel_cmd(bus: can.Bus, rpm1: int, rpm2: int, ret_type: int = 2):
    lo1, hi1 = le_i16_signed(rpm1)
    lo2, hi2 = le_i16_signed(rpm2)
    data = bytes([PID_PNT_VEL_CMD, 1, lo1, hi1, 1, lo2, hi2, ret_type & 0xFF])
    bus.send(can.Message(arbitration_id=DRIVER_ID, is_extended_id=False, data=data))

def main():
    # python-can 4.6+ 권장 인자: interface=
    bus = can.interface.Bus(channel=CAN_IFACE, interface="socketcan")

    # F1 broadcasting ON (2번 보내서 안정화)
    send_pnt_io_monitor_on(bus)
    time.sleep(0.05)
    send_pnt_io_monitor_on(bus)

    knob1 = 0
    knob2 = 0
    last_knob_time = 0.0

    period = 1.0 / float(SEND_HZ)
    next_send = time.time()
    last_print = 0.0

    print("Running. Ctrl+C to stop.")
    print(f"BASE_RPM={BASE_RPM}, DEADZONE={DEADZONE_PCT}%, MIN_RPM={MIN_RPM_WHEN_MOVING}, SEND_HZ={SEND_HZ}")

    try:
        while True:
            # 수신 버퍼 drain
            for _ in range(200):
                msg = bus.recv(timeout=0.0)
                if msg is None:
                    break
                d = msg.data
                if len(d) == 8 and d[0] == PID_PNT_IO_MONITOR:
                    # 중요: D1(=d[1])이 0일 때만 D6/D7가 노브값
                    if d[1] == 0:
                        knob1 = clamp(int(d[6]), 0, 100)
                        knob2 = clamp(int(d[7]), 0, 100)
                        last_knob_time = time.time()

            now = time.time()

            # 타임아웃 처리
            if last_knob_time == 0.0 or (now - last_knob_time) > KNOB_TIMEOUT_SEC:
                k1_eff, k2_eff = 0, 0
            else:
                k1_eff, k2_eff = knob1, knob2

            # 데드존
            if k1_eff <= DEADZONE_PCT: k1_eff = 0
            if k2_eff <= DEADZONE_PCT: k2_eff = 0

            # 스케일링
            rpm1 = int(round(BASE_RPM * (k1_eff / 100.0)))
            rpm2 = int(round(BASE_RPM * (k2_eff / 100.0)))

            # 최소 rpm 클램프(꿈틀 방지)
            if MIN_RPM_WHEN_MOVING > 0:
                if k1_eff > 0 and abs(rpm1) < MIN_RPM_WHEN_MOVING:
                    rpm1 = MIN_RPM_WHEN_MOVING if BASE_RPM >= 0 else -MIN_RPM_WHEN_MOVING
                if k2_eff > 0 and abs(rpm2) < MIN_RPM_WHEN_MOVING:
                    rpm2 = MIN_RPM_WHEN_MOVING if BASE_RPM >= 0 else -MIN_RPM_WHEN_MOVING

            # 주기 송신
            if now >= next_send:
                send_vel_cmd(bus, rpm1, rpm2, ret_type=2)
                next_send += period

            # 디버그(5Hz)
            if now - last_print > 0.2:
                print(f"knob=({k1_eff:3d},{k2_eff:3d}) -> rpm=({rpm1:4d},{rpm2:4d})")
                last_print = now

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        try:
            send_vel_cmd(bus, 0, 0, ret_type=0)
            time.sleep(0.02)
            send_vel_cmd(bus, 0, 0, ret_type=0)
        except Exception:
            pass
        try:
            bus.shutdown()
        except Exception:
            pass
        print("Stopped.")

if __name__ == "__main__":
    main()