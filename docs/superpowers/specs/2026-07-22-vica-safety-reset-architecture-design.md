# VICA Safety 분리 및 Reset 오케스트레이션 설계

작성일: 2026-07-22
상태: 사용자 검토 대기

## 1. 목적

주행 액추에이터와 안전 판단을 서로 다른 패키지와 launch로 분리하고, 물리 버튼·앱·음성
E-stop을 하나의 중앙 래치로 통합한다. Reset은 `app_emergency_node`가 전체 절차를
오케스트레이션하며, 관리자 앱뿐 아니라 유지보수용 터미널 `/safety_reset`에서도 같은 안전
검사를 거쳐 요청할 수 있게 한다.

이 설계는 다음 원칙을 따른다.

- 모터 노드는 CAN 주행 명령 처리만 담당한다.
- Safety 노드는 모터 패키지와 독립적으로 실행된다.
- 물리 버튼을 놓거나 앱·음성 입력이 `false`가 되어도 E-stop 중앙 래치는 자동 해제되지 않는다.
- Reset 성공 전 Nav2 Goal을 모두 취소하며, 이전 Goal은 자동 재개하지 않는다.
- Reset 과정 중 일부 단계가 실패해도 모터 출력 허용 상태로 진행하지 않는다.
- 강제 reset이나 안전 조건 우회 경로는 만들지 않는다.

## 2. 패키지와 launch 경계

### 2.1 `vica_safety`

새 ROS 2 Python 패키지 `vica_safety`를 만들고 다음 노드를 소유하게 한다.

- `emergency_stop_node`: 물리·앱·음성 E-stop 통합 및 중앙 래치 소유
- `safety_supervisor_node`: 모든 안전 조건을 검사하고 `/cmd_vel_safe` 승인
- `app_emergency_node`: 앱·터미널 reset 요청과 Nav2 활성 Goal 확인·조건부 전체 취소 절차 오케스트레이션

실행 명령은 다음과 같다.

```bash
ros2 launch vica_safety safety_bringup.launch.py
```

이 launch는 위 세 안전 노드만 실행하며 모터 노드를 포함하지 않는다. 물리 버튼 입력은 현재
검증된 설정을 유지하여 `input_mode:=can_f1`, `can_interface:=can1`, CAN ID `0x701`을
명시한다.

필수 의존성은 최소한 다음을 포함한다.

- `rclpy`
- `geometry_msgs`
- `std_msgs`
- `std_srvs`
- `action_msgs`
- `python3-can`

### 2.2 `mdrobot_can_control`

`mdrobot_can_control`은 액추에이터 어댑터 역할만 유지한다. E-stop 래치, reset 서비스,
Safety Supervisor 및 앱 reset 오케스트레이션을 소유하지 않는다.

```bash
ros2 launch mdrobot_can_control motor_bringup.launch.py
```

`motor_bringup.launch.py`에는 모터 노드만 포함한다. 기존
`motor_safety_bringup.launch.py`의 안전 노드 실행 책임은 `vica_safety`로 이동하며, 사용자가
적용한 `input_mode=can_f1` 설정은 새 Safety launch에 보존한다.

### 2.3 `vica_localization`

엔코더와 EKF의 기존 launch는 변경하지 않는다.

```bash
ros2 launch vica_localization wheel_ekf.launch.py
```

이 launch는 `encoder_feedback`과 EKF만 실행한다. Localization, SLAM 또는 Nav2를 시작하는
행위가 모터 구동을 자동으로 활성화하지 않도록 모터 노드를 여기에 포함하지 않는다.

## 3. 안전 권한 분리

Reset은 하나의 버튼 동작처럼 보이지만 내부적으로 서로 다른 세 권한으로 나눈다.

| 권한 | 소유 노드 | 역할 |
| --- | --- | --- |
| Reset 절차 시작 | `app_emergency_node` | Goal idle 확인부터 최종 안전 상태 확인까지 순서 제어 |
| E-stop 기억 해제 | `emergency_stop_node` | 물리·앱·음성 원인이 모두 해제된 경우에만 중앙 래치 해제 |
| 주행 출력 재승인 | `safety_supervisor_node` | 모든 안전 조건과 정지 명령을 확인한 뒤에만 READY 상태 허용 |

따라서 중앙 래치가 해제되어도 즉시 주행할 수 없다. Safety Supervisor의 별도 검사를 통과해야
하며, Reset 전의 Goal은 복구하지 않는다.

## 4. E-stop 입력과 중앙 래치

### 4.1 입력

현재 외부 계약을 유지한다.

- 물리 버튼: `can1`의 F1 상태 프레임
- 앱: `/app_emergency_stop` (`std_msgs/msg/Bool`)
- 음성: `/voice_emergency_stop` (`std_msgs/msg/Bool`)

`emergency_stop_node`는 각 입력 상태와 최신 수신 시각을 별도로 보관한다. 다음 중 하나라도
참이면 중앙 래치를 `true`로 만든다.

- 물리 E-stop 활성
- 앱 E-stop 활성
- 음성 E-stop 활성
- 필수 물리 입력이 stale이거나 CAN 읽기 오류로 신뢰할 수 없음

통합 상태는 기존 계약인 `/emergency_stop` (`std_msgs/msg/Bool`)으로 발행한다.

### 4.2 래치 동작 예시

1. 물리 버튼을 누르면 현재 입력과 중앙 래치가 모두 `true`가 된다.
2. 물리 버튼을 놓으면 현재 입력은 `false`가 되지만 중앙 래치는 계속 `true`다.
3. 사용자가 `/safety_reset`을 요청하면 모든 원인 해제와 freshness를 확인한다.
4. 검사가 통과한 경우에만 내부 E-stop reset으로 중앙 래치를 `false`로 만든다.
5. Safety Supervisor가 추가 조건을 통과할 때까지 모터 출력은 계속 0이다.

## 5. 서비스 계약

### 5.1 공개 서비스

- `/safety_reset` (`std_srvs/srv/Trigger`): 영구 유지보수용 reset 진입점
- `/app_estop_reset` (`std_srvs/srv/Trigger`): Flutter 앱 호환 진입점
- `/app_estop_activate` (`std_srvs/srv/Trigger`): 앱 E-stop 활성화

`/safety_reset`과 `/app_estop_reset`은 `app_emergency_node`의 동일한 오케스트레이션
콜백으로 연결한다. 어느 경로도 내부 안전 검사를 우회하지 않는다.

관리자 인증은 현재 범위에 포함하지 않는다. `Trigger` 서비스는 호출자 신원을 표현하지
못하므로 `/safety_reset`은 인증되지 않은 로컬 유지보수 인터페이스라는 `[GAP]`이 남는다.
향후 앱 인증을 추가하더라도 이 서비스는 유지하되 SROS2, 로컬 셸 권한 또는 네트워크 접근
제어로 호출 범위를 제한한다.

### 5.2 내부 서비스

- `/vica_safety/internal/estop_reset` (`std_srvs/srv/Trigger`)
  - 소유자: `emergency_stop_node`
  - 역할: 중앙 E-stop 래치만 해제
  - 거부 조건: 물리·앱·음성 원인이 하나라도 활성, F1 상태 stale, CAN 상태 불명
- `/vica_safety/internal/supervisor_reset` (`std_srvs/srv/Trigger`)
  - 소유자: `safety_supervisor_node`
  - 역할: 안전 상태를 주행 재승인 가능한 상태로 전환
  - 거부 조건: `/emergency_stop` 활성 또는 stale, `/cmd_vel_req` 비영점, reset 불가능 상태,
    향후 추가되는 안전 조건 미충족

이름의 `internal`은 사용자가 직접 호출하는 정상 인터페이스가 아니라는 의미다. ROS 2 서비스
이름만으로 보안을 제공하지 않으므로 실제 접근 통제는 향후 SROS2 정책에서 보완한다.

## 6. Reset 오케스트레이션

`app_emergency_node`는 동시에 들어오는 reset 요청을 직렬화하고 다음 순서로 처리한다.

1. 앱 E-stop 원인을 `false`로 내린다.
2. Nav2 action server가 실행 중이면 fresh Action status에서 accepted, executing 또는
   canceling Goal 유무를 확인한다. status 수신 이력과 action server가 모두 없으면 Nav2
   미실행으로 판정해 Goal 검사를 생략한다.
3. 활성 Goal이 있으면 `NavigateToPose`의 `CancelGoal`에 전체 취소 요청을 보내고 terminal
   상태를 기다린다. 활성 Goal이 없으면 cancel 호출을 생략한다.
4. `/vica_safety/internal/estop_reset`을 호출한다.
5. fresh한 `/emergency_stop=false`를 확인한다.
6. `/vica_safety/internal/supervisor_reset`을 호출한다.
7. `/safety_state=READY_TO_GO`를 확인한다.
8. 모든 조건을 만족한 경우에만 공개 reset 요청을 성공으로 반환한다.

조건부 Nav2 전체 취소는 Mission Manager 밖에서 생성된 시험 Goal까지 정리하기 위한 방어
계층으로 유지한다. Mission Manager가 자신의 Goal을 취소하는 현재 동작도 유지한다. Goal이
없더라도 Supervisor의 `/cmd_vel_req=0` 또는 stale 검사는 생략하지 않는다.

각 단계에는 유한 timeout을 적용한다. 응답 메시지는 실패 단계와 원인을 구분하여 반환하며,
실패했다고 자동 재시도하거나 안전 조건을 건너뛰지 않는다.

## 7. 부분 실패 시 안전 상태

- Nav2 action server가 있는데 status 미수신, 이전 status stale 또는 활성 Goal 전체 취소
  실패: 내부 E-stop reset을 호출하지 않는다.
- 중앙 래치 reset 실패: Safety Supervisor reset을 호출하지 않는다.
- 중앙 래치는 해제됐지만 Supervisor reset 실패: `ESTOP_RELEASED_WAIT_RESET`에 머물고
  `/cmd_vel_safe`는 0을 유지한다.
- Reset 도중 입력이 다시 활성화됨: 즉시 재래치하고 Supervisor reset을 거부한다.
- `app_emergency_node` 종료: 중앙 래치와 Supervisor의 출력 차단은 그대로 유지된다.
- 모터 노드 재시작: 중앙 래치나 reset 권한에 영향을 주지 않는다.

## 8. 앱 상태 계약

Flutter 호환성을 위해 `/app_estop_state`는 JSON 문자열을 유지하고 기존 `active` 키를 보존한다.
단, `active`는 앱 버튼의 로컬 상태가 아니라 중앙 통합 E-stop 상태를 나타낸다.

```json
{
  "active": true,
  "app_active": false,
  "emergency_active": true,
  "safety_state": "ESTOP_ACTIVE",
  "reset_allowed": false,
  "message": "physical estop is still active",
  "timestamp": 0.0
}
```

추가 키를 읽지 않는 기존 앱도 `active`만으로 현재 통합 상태를 표시할 수 있다. 앱 reset은
`/app_estop_reset`을 계속 사용하므로 Flutter 측 공개 서비스 이름은 바꾸지 않는다.

## 9. 상태와 로그 가시성

백업 구현에서 재사용하는 부분은 다음 두 가지로 제한한다.

- `RCUTILS_COLORIZED_OUTPUT=1`을 통한 ROS 2 컬러 로그 강제 활성화
- 안전 상태별 `INFO`/`WARN`/`ERROR` 심각도 선택 방식

백업의 E-stop/reset 처리 흐름이나 노드 책임은 재사용하지 않는다.

`safety_bringup.launch.py`는 `RCUTILS_COLORIZED_OUTPUT=1`을 설정한다. 여기서
`estop=True`는 ROS 파라미터가 아니라 로그 메시지에 포함되는 현재 상태값이다. ROS 2 기본
컬러 로그는 `True`라는 값 하나만 색칠하지 않고, 로그 심각도에 따라 타임스탬프와 메시지를
포함한 로그 한 줄 전체를 색으로 구분한다.

터미널에서 상태 변화를 즉시 감지할 수 있도록 상태 전이와 reset 결과를 다음 심각도로
기록한다.

- E-stop 활성 전이: `ERROR`, `[ESTOP ACTIVE] estop=True source=<원인>`
- `FAULT` 전이: `ERROR`, `[FAULT] <원인>`
- 물리 입력은 해제됐지만 중앙 래치가 남은 상태: `WARN`,
  `[WAIT RESET] estop=True physical=False`
- Reset 조건 미충족 또는 절차 실패: `WARN` 또는 안전 상태 불명 시 `ERROR`,
  `[RESET REJECTED] step=<단계> reason=<원인>`
- Reset 전체 성공: `INFO`, `[RESET ACCEPTED] estop=False state=READY_TO_GO`
- `READY_TO_GO`, `RUNNING` 정상 전이: `INFO`, `[SAFETY READY]` 또는 `[RUNNING]`

상태 전이 시 즉시 한 번 기록하고, 같은 상태의 주기 로그는 throttle하여 터미널이 반복 메시지로
도배되지 않게 한다. 로그에는 이전 상태와 새 상태를 함께 넣어 변화 방향을 알 수 있게 한다.
상태명, `estop=True/False`, 입력 원인과 reset 실패 단계는 색상 없이 저장된 로그에서도 식별
가능해야 한다.

예상 출력 형태는 다음과 같다. 실제 ANSI 색상은 터미널과 ROS 2 로깅 설정에 따라 로그 한 줄
전체에 적용된다.

```text
[ERROR] [emergency_stop_node]: [ESTOP ACTIVE] IDLE -> ESTOP_ACTIVE estop=True source=physical_f1
[WARN]  [emergency_stop_node]: [WAIT RESET] ESTOP_ACTIVE -> ESTOP_RELEASED_WAIT_RESET estop=True physical=False
[WARN]  [app_emergency_node]: [RESET REJECTED] step=nav_goal_check reason=active goal remains
[INFO]  [app_emergency_node]: [RESET ACCEPTED] ESTOP_RELEASED_WAIT_RESET -> READY_TO_GO estop=False
```

## 10. 구현 범위와 문서 정합성

이번 구현 범위는 다음과 같다.

- `vica_safety` 패키지 생성 및 세 안전 노드 이동·수정
- 중앙 E-stop 래치와 내부 reset 서비스 구현
- Safety Supervisor의 내부 reset 및 출력 승인 조건 구현
- `app_emergency_node`의 Nav2 활성 Goal 확인·조건부 전체 취소와 reset 오케스트레이션 구현
- `action_msgs`를 포함한 패키지 의존성 정리
- Safety 전용 launch와 모터 전용 launch 분리
- 상태 로그 구분과 색상 환경 설정
- 관련 단위·launch 계약 테스트 추가
- `GOVERNANCE.md`, `AGENTS.md`, `guideline/`의 패키지·권한·서비스 설명 갱신
- 의미 있는 Safety 결정을 `devlog/2026-07-22.md`에 기록

다음은 이번 범위에서 제외한다.

- 관리자 로그인·권한 인증 구현
- SROS2 보안 정책 배포
- 모터와 encoder/localization launch 결합
- E-stop 해제 후 과거 Goal 자동 재개
- 실제 바퀴 구동을 포함한 실기기 reset 시험

## 11. 검증 계획

### 11.1 단위 테스트

- 물리·앱·음성·CAN fault 각각이 중앙 래치를 활성화하는지 확인
- 모든 입력을 놓아도 중앙 래치가 유지되는지 확인
- 활성 입력 및 stale F1 상태에서 내부 E-stop reset이 거부되는지 확인
- 모든 입력이 fresh하고 해제된 경우에만 중앙 래치가 해제되는지 확인
- 비영점 또는 stale `/cmd_vel_req`, 활성/stale E-stop에서 Supervisor reset 거부 확인
- 앱과 터미널 reset이 동일한 오케스트레이션 경로를 사용하는지 확인
- 동시 reset 직렬화, Nav2 취소 실패, 서비스 timeout, 중간 재활성화, 정상 READY 전환 확인

### 11.2 정적·빌드 테스트

- `vica_safety`와 `mdrobot_can_control` 대상 `colcon build` 및 `colcon test`
- `action_msgs` import와 package dependency 확인
- Safety launch에 안전 노드 세 개만 있고 모터가 없는지 확인
- motor launch에 모터 노드만 있는지 확인
- `wheel_ekf.launch.py`가 변경되지 않았고 모터를 포함하지 않는지 확인
- Safety launch가 `can_f1`, `can1`, `0x701`, 컬러 로그 설정을 유지하는지 확인
- producer/consumer 전체 검색으로 폐기된 서비스나 옛 패키지 경로가 남지 않았는지 확인

### 11.3 실기기 검증

실기기 시험은 `[미검증]`으로 남긴다. 이후 사용자가 바퀴를 띄운 상태, 주변 통제, 물리
E-stop 및 즉시 전원 차단 수단을 확인한 뒤 별도 절차로 수행한다. 초기 구현 검증 중에는
`can1` 상태를 변경하거나 모터·Nav2 Goal·실제 reset을 실행하지 않는다.

## 12. 완료 기준

- Safety 노드가 `vica_safety`로 분리되고 모터·Localization launch와 독립 실행된다.
- 물리·앱·음성 입력이 중앙 래치를 공유하며 입력 해제가 곧 reset이 되지 않는다.
- `/safety_reset`과 `/app_estop_reset`이 동일한 전체 안전 절차를 거친다.
- Nav2 status 확인, 필요한 Goal 전체 취소와 두 내부 reset 단계 중 하나라도 실패하면 주행이
  재승인되지 않는다.
- 앱과 터미널 로그에서 E-stop 활성, reset 대기, reset 성공·거부가 구분된다.
- 문서, 코드, launch, package dependency 및 테스트가 같은 권한 구조를 설명한다.
