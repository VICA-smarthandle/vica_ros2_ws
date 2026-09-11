# vica_system_monitor

VICA 전체 상태를 관측해 앱에 세부 오류를 표시하는 계층이다. **정지 경로에 들어가지 않는다.**

## 무엇을 하는가

```
[우리 노드]  mdrobot(구현됨) · safety · guidance · encoder
    └ diagnostic_updater ─────────────────────┐
[외부 대상]  rplidar · nvblox · D455 · /proc   │
    └ external_diagnostics_node (어댑터) ──────┤
                                              ▼
                                        /diagnostics
                                              ▼
                              diagnostic_aggregator (표준 패키지)
                                              ▼
                                       /diagnostics_agg
                                              ▼
[안전 신호 직접]  /emergency_stop · /safety_state ──▶ robot_health_monitor_node
                     TF map→base_footprint          ▼
                                    /robot/health  +  /robot/events
                                              ▼
                                     rosbridge → 앱 진단 화면
```

안전 신호는 aggregator를 거치지 않는다. 집계 지연 때문이다
(`guideline/vica_system_health_monitoring_draft.md` 3.1절).

## 관측 범위와 사각지대

이 표를 반드시 함께 읽어야 한다. "health가 정상이라고 했는데 왜 못 잡았나"를 막는다.

### 관측하는 것

| 신호 | 방법 |
| --- | --- |
| 모터 CAN 링크·cmd/knob age | 기존 `/diagnostics` |
| `/scan` 주기 | 어댑터 topic_rate |
| nvblox slice age/Hz | 어댑터 topic_rate |
| depth·color `camera_info` 주기 | 어댑터 topic_rate |
| `/odom` 실효 Hz (EKF baseline) | 어댑터 topic_rate |
| `/wheel/odom` 미발행 | 어댑터 topic_rate |
| 노드별 프로세스 CPU % | 어댑터 process_cpu (`/proc`) |
| E-stop 래치·safety enum | 모니터 직접 구독 |
| TF `map→base_footprint` age | 모니터 직접 tf2 |
| Nav2 lifecycle 상태 | 모니터 `GetState` 폴링 |

### 관측하지 못하는 것

기존 노드를 수정하지 않으면 원리적으로 볼 수 없다. **1차 범위에 없다는 뜻이며 나중에 못
고친다는 뜻이 아니다.** 해당 노드를 수정할 때 진단 발행을 함께 넣으면 된다.

| 신호 | 왜 불가 | 실제 위험 |
| --- | --- | --- |
| STT/TTS CPU 폴백 여부 | `VicaSTT` 내부 변수, `print`로만 나감 | 폴백 시 지연 3.7배·10배 |
| 긴급 감시 실효 hop·창 건너뜀 | 카운터가 없다 | 긴급어 사각지대 확대 |
| 마이크 무입력 | 오디오 콜백 내부 | **감시가 조용히 멈추는 실패 모드** |
| 목적지 카탈로그 부재 | warn 로그 한 줄 | 모든 안내가 `unknown_destination` |
| `_nav_lock` 보유 시간 | 내부 상태 | 데드락 진단 불가 |
| Smart Handle 서보·진동 실제 동작 | 아두이노 → 젯슨 상향 통신이 없다 | `guidance_readiness`를 `UNKNOWN`으로 보고한다 |

## 왜 readiness가 3상태인가

`RobotHealth`의 readiness 필드는 `UNKNOWN` / `NOT_READY` / `READY`다.

**`UNKNOWN`은 "정상"이 아니다. 관측 수단이 없다는 뜻이다.** Smart Handle은 상향 통신이 없어
서보·LED·진동이 실제로 동작했는지 확인할 수 없다(`SmartHandleState.msg` 주석). 이때 `READY`로
보고하면 관리자 앱이 초록불을 띄워 잘못된 안심을 준다.

하드웨어나 진단이 추가되면 같은 필드가 `UNKNOWN`에서 `READY`/`NOT_READY`로 바뀐다. **메시지와
앱을 고치지 않고 값만 달라지는 것이 이 설계의 목적이다.**

## 임계값을 코드에 두지 않는다

곧 `/imu/base_link`가 400 Hz에서 60 Hz로, `/local_costmap/voxel_grid`와 `/backup` action은
소멸하고, `/vica/tts_state`는 edge에서 heartbeat로 의미가 바뀐다. 하드코딩하면 전부 오탐이
된다.

- 임계값과 기대 토픽 목록은 전부 YAML에 둔다
- 계약 테스트로 설정 파일 간 이름 집합 일치를 강제한다
  (`vica_nav2/test/test_nav2_params_contract.py`와 같은 패턴)
- **1차에서는 모든 임계값을 `[미검증]`으로 두고 Jetson 실측 후 확정한다**

## freshness.py 사본에 대하여

`vica_safety`·`mdrobot_can_control`과 로직이 동일한 사본이다. 감시 패키지가 기능 패키지에
의존하지 않도록 사본을 둔다(초안 5.1절). 사본이 세 개가 되었으므로 향후 공용 시간 패키지로
통합할 여지가 있다. 계약은 동일하다 — 단일 STEADY_TIME clock, 정수 나노초, 미수신은 `None`,
시간 역전은 stale.

## 검증

```bash
# 순수 로직만. ROS 없이 실행된다
cd src/vica_system_monitor && python3 -m pytest test/ -v

# 패키지 전체
cd ../.. && colcon build --packages-select vica_interfaces vica_system_monitor
colcon test --packages-select vica_system_monitor && colcon test-result --verbose
```

노트북에서는 CAN·센서·TF·Nav2·GPU가 없어 종단 동작을 검증할 수 없다. 성공으로 쓰지 않는다.

## 관련 문서

- `guideline/vica_system_health_monitoring_draft.md` — 설계 초안
- `guideline/vica_architecture.md` 4.2절 — topic 계약
- `devlog/2026-07-30-nvblox-ghost-obstacle.md` — nvblox slice 감지 배경
