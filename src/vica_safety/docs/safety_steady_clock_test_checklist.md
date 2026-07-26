# VICA Safety STEADY_TIME 실기 검증·머지 체크리스트

문서 기준일: 2026-07-26
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

- [ ] 바퀴를 바닥에서 띄운다(무부하).
- [ ] 정상 주행: `/cmd_vel_safe`와 knob이 모두 살아 있을 때 바퀴가 knob 비율대로 회전.
- [ ] cmd 끊김: `/cmd_vel_safe` 발행을 멈추면 `cmd_timeout_sec` 내에 0 rpm 정지.
- [ ] knob 끊김: F1 monitor(knob) 수신을 끊으면 `knob_timeout_sec` 내에 0 rpm 정지.
- [ ] 물리 E-stop: 물리 버튼 → 즉시 정지·latch 유지, 관리자 앱 reset 전까지 재기동 불가.

### 2.2 `use_sim_time` + `/clock` 정지 내성

- [ ] `use_sim_time:=true`로 Safety·motor 노드를 실행한다.
- [ ] `/clock` 발행을 멈춘다.
- [ ] STEADY_TIME watchdog이 계속 발화하여 stale 판정·정지를 유지하는지 확인
      (ROS 시간이 멈춰도 monotonic clock은 흐르므로 정지해야 정상).

### 2.3 실행 중 시스템 시간 역전 내성

- [ ] 노드 실행 중 `sudo date -s '<과거 시각>'`으로 시스템 시간을 뒤로 점프시킨다.
- [ ] age가 음수가 되어도 timeout 판정이 정상(정지 유지)인지 확인
      (wall clock이었다면 stale 판정이 뒤집혀 위험. steady clock은 영향 없어야 함).
- [ ] 검증 후 시스템 시간을 복원한다(`sudo date -s` 또는 NTP 재동기화).

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
