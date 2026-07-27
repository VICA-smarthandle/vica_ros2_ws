# VICA Safety STEADY_TIME 실기 검증·머지 체크리스트

문서 기준일: 2026-07-26
실기 검증일: 2026-07-27 (**세 검증 모두 통과**, 5절 기록 참조)
대상 브랜치: `feat/safety-steady-clock` (repo `vica_ros2_ws`)
관련 계약: 워크스페이스 `guideline/vica_system_health_monitoring_draft.md`
(monotonic timeout 항목)

이 문서는 대상 변경과 같은 브랜치(`feat/safety-steady-clock`)에 함께 커밋되어
있으므로, Jetson 실기에서 이 브랜치를 checkout하면 코드와 함께 딸려온다.

## 0. 배경

Safety 입력 수신부터 motor 최종 정지까지 모든 경과시간 판단을 단일
`ClockType.STEADY_TIME` clock과 정수 나노초로 통일한 변경이다. 개발용 컴퓨터의
단위·계약 테스트(`colcon test vica_safety` 64/64, motor watchdog 단위테스트)는
통과했다. 아래 3가지 종단 검증은 개발용 컴퓨터로 불가능하며 Jetson 실기에서만
확인할 수 있다.

**세 검증이 모두 통과하기 전에는 바닥 주행에 적용하지 않는다.**

## 1. Jetson에서 브랜치 가져오기 (pull)

```bash
cd <vica_ros2_ws 경로>
git status                       # 로컬 미커밋 변경이 있으면 먼저 정리
git fetch origin
git checkout feat/safety-steady-clock
git pull                         # upstream 추적되어 있으므로 인자 없이 최신화
```

주의:

- 이 브랜치를 처음 가져오는 것이므로 `git pull`이 아니라 `checkout`으로 시작한다.
- Jetson 로컬에 커밋 안 한 변경이 있으면 checkout이 막힐 수 있다. `git stash` 또는
  커밋으로 먼저 정리한다.
- `python-can`이 실기에 설치되어 있어야 `colcon test`와 노드 실행이 가능하다.

## 2. 실기 검증 (통과 기준 = 아래 3가지 실동작)

`colcon test` 통과는 로직 확인일 뿐 종단 확인이 아니다. 아래가 진짜 판정 기준이다.

### 2.1 실제 Safety–CAN–motor 종단 (바퀴 띄운 상태)

- [x] 바퀴를 바닥에서 띄운다(무부하).
- [x] 정상 주행: `/cmd_vel_safe`와 knob이 모두 살아 있을 때 바퀴가 knob 비율대로 회전.
- [x] cmd 끊김: `/cmd_vel_safe` 발행을 멈추면 `cmd_timeout_sec` 내에 0 rpm 정지.
- [x] knob 끊김: F1 monitor(knob) 수신을 끊으면 `knob_timeout_sec` 내에 0 rpm 정지.
- [x] 물리 E-stop: 물리 버튼 → 즉시 정지·latch 유지, 관리자 앱 reset 전까지 재기동 불가.

### 2.2 `use_sim_time` + `/clock` 정지 내성

- [x] `use_sim_time:=true`로 Safety·motor 노드를 실행한다.
- [x] `/clock` 발행을 멈춘다.
- [x] STEADY_TIME watchdog이 계속 발화하여 stale 판정·정지를 유지하는지 확인
      (ROS 시간이 멈춰도 monotonic clock은 흐르므로 정지해야 정상).

### 2.3 실행 중 시스템 시간 역전 내성

- [x] 노드 실행 중 `sudo date -s '<과거 시각>'`으로 시스템 시간을 뒤로 점프시킨다.
- [x] age가 음수가 되어도 timeout 판정이 정상(정지 유지)인지 확인
      (wall clock이었다면 stale 판정이 뒤집혀 위험. steady clock은 영향 없어야 함).
- [x] 검증 후 시스템 시간을 복원한다(`sudo date -s` 또는 NTP 재동기화).

주의: `sudo timedatectl set-ntp false`로 NTP를 먼저 끄지 않으면 시간이 즉시
되돌려져 시험이 성립하지 않는다. 또한 표준 `ros2 topic pub`은 wall clock 타이머를
쓰므로 시간 역전과 동시에 발행이 멈춘다. 명령 공급원은 STEADY_TIME 타이머로
직접 구현해야 한다(5.3 참조).

### 2.4 참고: 실기에서 로직 재확인 (선택)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vica_safety mdrobot_can_control
colcon test --packages-select vica_safety mdrobot_can_control \
  --event-handlers console_direct+
colcon test-result --verbose
```

주의: `mdrobot_can_control`의 `test_flake8`/`test_pep257` 실패는 이 변경 이전부터
있던 기존 스타일 위반(knob node 등)이며 시간 판정과 무관하다. 기능 테스트
(`test_motor_watchdog` 등)와 `vica_safety` 통과 여부로 판단한다.

## 3. 결과에 따른 조치

### 3.1 세 검증 모두 통과 → dev로 머지

```bash
cd <vica_ros2_ws 경로>
git checkout dev
git pull origin dev                         # dev 최신화
git merge feat/safety-steady-clock
# 충돌 시 해결 후 커밋
git push origin dev
```

- 안전 크리티컬 변경이므로 팀 리뷰(PR)를 거치면 더 좋다.
  PR: `https://github.com/VICA-smarthandle/vica_ros2_ws/pull/new/feat/safety-steady-clock`

### 3.2 방향은 맞으나 버그 → feat 브랜치에서 수정 후 재검증

- 브랜치를 삭제하지 말 것(작업 보존). feat에서 고치고 2절부터 다시.

### 3.3 접근 자체를 버릴 때만 → feat 브랜치 삭제

```bash
git checkout dev
git branch -D feat/safety-steady-clock               # 로컬 삭제
git push origin --delete feat/safety-steady-clock    # 원격 삭제
```

- 로컬만 지우면 원격에 남으므로 둘 다 지운다.
- dev는 이 변경을 받지 않았으므로 되돌릴(롤백) 것이 없다.
- 이 체크리스트 문서도 브랜치와 함께 사라진다(브랜치 전용 문서이므로 의도된 동작).

## 4. 비변경·유지 사항 (검증 시 참고)

- timeout 파라미터 값, 토픽 이름, QoS, Safety 정책(fail-closed·중앙 래치·관리자 단일
  reset)은 이번 변경에서 바뀌지 않았다.
- 사용자·앱 표시용 timestamp는 UTC wall clock을 유지한다
  (`app_emergency_node.publish_state`).
- 판정용 wall clock(`time.time()`/`time.monotonic()`)은 5개 대상 파일에서 제거됨.
  남은 `time.sleep()`은 순수 지연이라 무관하다.

## 5. 실기 검증 결과 (2026-07-27)

환경: Jetson Orin NX(`Z-jet`), ROS 2 Humble, `can1` 50 kbps, 바퀴 무부하(띄움),
knob1 = 48%, 검증 시점 HEAD = `759fad2`(아래 5.4 수정 포함).

측정 분해능 한계: motor node의 상태 로그는 0.2초 주기로 throttle된다. 따라서
아래 "정지까지" 값은 **상한**이며 실제 발화는 최대 0.23초 더 이를 수 있다.

### 5.1 §2.1 종단 (4/4 통과)

| 항목 | 측정 | 기준 |
|---|---|---|
| 정상 주행 | `cmd=(+0.10)` → `rpm +15/+15`, 149샘플 연속 안정 | 끊김 없을 것 |
| cmd 끊김 | `limit` 0.48 → 0.00, **≤234 ms** | `cmd_timeout_sec` 0.5 s |
| knob 끊김 | `cmd=(+0.10)` 유지 + `limit=(0.00)`, 마지막 F1 후 **0.912 s** | `knob_timeout_sec` 0.8 s |
| 물리 E-stop | 감지 → 0 rpm **120 ms** | 즉시 정지 |

- cmd 끊김은 `safety_supervisor_node`를 종료해 `/cmd_vel_safe` 자체를 끊어 측정했다.
  `/cmd_vel_req`만 끊으면 supervisor가 0을 계속 발행하므로 motor node 자체
  watchdog이 아니라 supervisor gate를 보게 된다. 두 경로 모두 확인함(후자 ≤233 ms).
- knob 끊김은 `tc qdisc add dev can1 ingress` + `basic action drop`으로 **수신만**
  차단했다. 송신은 살아 있어 드라이버가 계속 명령을 받고 ACK했으므로(TX 15 Hz,
  오류 0), 정지 원인이 소프트웨어임이 분리 확인된다. 문서화된
  `CMD_PNT_IO_MONITOR_OFF(86)` 명령으로도 같은 상태를 만들 수 있다(sudo 불필요).
- 차단 해제 즉시 15 rpm 재개 — stale 판정이 영구 latch되지 않음도 확인.
- 물리 E-stop 해제만으로는 `ESTOP_RELEASED_WAIT_RESET`을 유지(13초 관측),
  관리자 reset 후에만 `READY_TO_GO` 전이. 주행 명령이 살아 있는 동안의 reset은
  `reason=/cmd_vel_req is not zero`로 거부됨(설계대로).

### 5.2 §2.2 `/clock` 정지 내성 (통과)

- `use_sim_time:=true` 적용 확인, `/clock`을 20 ms 주기로 발행하다 45초에 중단하여
  ROS 시간을 **45.96 s에 동결**.
- 동결 상태에서 control_loop 로그 6초간 27건(≈4.5 Hz) — 타이머가 계속 발화.
- 동결 상태에서 cmd 끊김 → **233 ms** 내 0 rpm. ROS clock 기반이었다면 age가
  얼어붙어 timeout이 발생하지 않았을 것이다.

### 5.3 §2.3 시스템 시간 역전 내성 (통과)

- `sudo date -s`로 **−3914.6 s(−1.09 시간)** 점프를 로그 타임스탬프로 확인.
- 점프 순간 `limit=(0.48)` 유지 — cmd·knob 판정 모두 fresh, 뒤집힘 없음. 주행 무중단.
- 점프 후에도 노드 로그 ≈4.6 Hz 지속, `/cmd_vel_safe` 30.005 Hz 유지.
- 역전 상태에서 주행 → 명령 끊김 → **233 ms** 내 0 rpm(판정 기능 정상).
- 대조군: 표준 `ros2 topic pub`은 점프와 동시에 발행이 정지(6255회에서 멈춤).
  wall clock 타이머를 쓰는 도구는 실제로 망가지며, 대상 노드는 영향받지 않았다.

### 5.4 검증 중 발견·수정한 결함

`control_loop`이 판정 기준 시각 `now`를 캡처한 뒤 `drain_can_rx`가 자체적으로 더
나중 시각을 찍어, 정상 수신한 knob이 **음수 age로 stale 오판정**되었다. 제어 루프의
**33%**에서 `speed_ratio`가 0이 되어 초당 약 10회 출력이 끊겼다(동일 호출 순서로
복제한 진단 노드에서 음수 age 31.8%, 최악 −2.378 ms 재현).

`drain_can_rx(now_ns)`로 stamp를 사이클 기준 시각으로 주입해 수정하고 회귀 테스트
3건을 추가했다(`test_knob_cycle_stamp.py`). 수정 후 실기 flapping **0%**.
커밋 `759fad2`.

### 5.5 부수 확인 사항

- **드라이버 자체 통신 watchdog**: `PID_COM_WATCH_DELAY(185)`를 드라이버에서 직접
  read한 값이 `5` = **0.5초**. 통신이 0.5초 끊기면 드라이버가 스스로 정지한다.
  motor node를 강제 종료했을 때(CAN은 UP) 바퀴가 멈춘 것이 이 기능으로 설명된다.
  단 이 계층은 *통신 자체가 끊겼을 때만* 반응하므로, 통신은 살아 있고 knob
  프레임만 안 오는 경우는 소프트웨어 watchdog만 잡는다.
- **[범위 밖 결함]** `sudo ip link set can1 down` 시 motor node가
  `can.exceptions.CanOperationError`(`drain_can_rx`의 `bus.recv`) 미처리로 종료된다.
  이 변경 이전부터 있던 동작이며 시간 판정과 무관하다. 드라이버 자체 타임아웃이
  있어 폭주로는 이어지지 않으나, 정지 상태 유지·상태 보고 주체가 사라지므로 별도
  이슈로 다룰 것.

---

## 6. motor CAN health 실기 검증 (2026-07-28)

브랜치 `feat/motor-can-health`. §5.5의 **[범위 밖 결함]**(`can1 down` 시 motor
node 종료)을 해소했는지 확인한다. 환경은 §5와 동일하며 바퀴는 띄운 상태다.

### 6.1 결과 요약

| 항목 | 결과 |
| --- | --- |
| `can1 down` 시 motor node 생존 | 통과 |
| `[CAN FAULT]` 로그 | 통과 |
| 출력 0 유지 | 통과 |
| 중앙 걸쇠 체결 | 통과 (52 ms) |
| CAN 자체 재연결 | 통과 (노드 재기동 없이) |
| 조건 해소 후 관리자 reset 수락 | 통과 |
| motor node 강제 종료 → `motor_can_stale` | 통과 (537 ms) |

### 6.2 `can1 down` → 걸쇠 (시험 ②)

`ip link set can1 down` 이후 motor node는 pid를 유지한 채 계속 동작했다. 이
브랜치 이전에는 `bus.recv`의 `CanOperationError` 미처리로 여기서 프로세스가
종료됐다. 관측 시각(ROS clock):

| 시각 | 사건 |
| --- | --- |
| 1785193511.008 | `[CAN FAULT] phase=recv ... Network is down` |
| 1785193511.060 | `[ESTOP ACTIVE] ... source=motor_can` (down 후 **52 ms**) |
| 1785193511.509 | `[FAULT] ... source=motor_can,physical_stale` |

출력은 `limit=(0.00m/s,0.00rad/s)`, `rpm MOT1/R=+0, MOT2/L=+0`으로 유지됐다.
knob 프레임도 함께 끊기므로 CAN 게이트와 knob watchdog이 이중으로 0을 만든다.

이 구간에서 관리자 reset은 거부됐다:
`reason=active sources: motor_can,physical_stale`.

### 6.3 복구 → reset 수락 (시험 ②-b)

CAN이 한 번 끊기면 드라이버 동력이 차단되므로 드라이버 전원을 재투입한 뒤
`can1`을 올렸다. **motor node는 재기동하지 않았다.**

| 시각 | 사건 |
| --- | --- |
| 1785193677.717 | F1 프레임 재개 → `physical_stale` 해소, `source=motor_can` |
| 1785193678.683 | `[CAN RECOVERED] iface=can1` (노드가 스스로 재연결) |
| 1785193678.717 | `[WAIT RESET] ... source=none` |

이후 `/safety_reset` 수락:
`Safety reset 완료: Nav2 Goal 없음, 중앙 래치 해제, Supervisor READY_TO_GO 확인`.

재연결에 성공해도 주행은 스스로 재개되지 않는다. 걸쇠는 `vica_safety`가
소유하며 해제는 관리자 reset 하나뿐이라는 계약이 실기에서 확인됐다.

### 6.4 motor node 강제 종료 → `motor_can_stale` (시험 ③)

| 시각 | 사건 |
| --- | --- |
| 1785192978.410 | motor node에 SIGKILL |
| 1785192978.947 | `[FAULT] ... source=motor_can_stale,physical_f1` (**537 ms**) |

`motor_can_timeout_sec` 0.5초 설계값과 일치한다. 물리 원인과 함께 표기되며,
`[ESTOP ACTIVE]`가 아니라 `[FAULT]`로 분류된다. 노드를 다시 띄우면 보고가
재개되면서 `motor_can_stale`은 자동으로 해소되고 걸쇠만 남는다.

### 6.5 부수 확인 사항

- **"CAN은 살아 있고 드라이버만 죽은" 상태는 이 로봇에서 만들어지지 않는다.**
  드라이버 전원이 없으면 `ip link set can1 up`이 실패하고 `can1`은
  `state STOPPED`로 남는다(`mcp251xfd`, SPI). 드라이버 전원 차단 구간 내내
  `/motor/can_ok`는 `False`였고 `[CAN RECOVERED]`는 한 번도 찍히지 않았다.
  따라서 "재연결 성공만으로 `/motor/can_ok`가 거짓으로 true가 된다"는 우려는
  이 하드웨어에서 성립하지 않는다.
- **`can1 down`으로는 재현되지 않는 경로가 있다.** 인터페이스가 *존재하면*
  down 상태여도 `can.interface.Bus()`의 bind가 성공하므로 재개방이 실패하지
  않는다. 재개방이 실패하는 것은 인터페이스가 *사라지는* 경우(모듈 언로드,
  USB-CAN 탈거)이며, 이때 닫힌 socketcan 소켓은 `CanError`도 `OSError`도 아닌
  `ValueError`를 던진다(python-can 4.6.1 실측). 이 경로는 실기 대신
  `test_can_reconnect.py` 회귀 테스트 8건으로 고정했다.
- **운영 주의**: motor node를 재기동할 때마다 0.5초 뒤 `motor_can_stale`로
  걸쇠가 걸리므로 매번 관리자 reset이 필요하다. 설계 의도(fail-closed)대로지만
  현장에서 고장으로 오인하기 쉽다. 기동 매뉴얼에 반영할 것.
