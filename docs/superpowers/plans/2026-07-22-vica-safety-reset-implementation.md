# VICA Safety Reset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 모터 패키지에서 Safety 책임을 `vica_safety`로 분리하고, 중앙 E-stop 래치·Nav2 활성 Goal 조건부 전체 취소·2단계 안전 reset·상태별 컬러 로그를 구현한다.

**Architecture:** `emergency_stop_node`가 물리 CAN F1·앱·음성 입력을 중앙 래치하고, `safety_supervisor_node`가 `/cmd_vel_req`를 최종 승인한다. `app_emergency_node`는 `/safety_reset`과 `/app_estop_reset`을 같은 직렬화된 절차로 받아 fresh Nav2 status 확인, 필요한 Goal 전체 취소, 중앙 래치 해제, Supervisor 재승인을 순서대로 수행한다.

**Tech Stack:** ROS 2 Humble, Python 3, `rclpy`, `python-can`, `geometry_msgs`, `std_msgs`, `std_srvs`, `action_msgs`, `launch`, `pytest`, `colcon`

## Global Constraints

- `vica_ros2_ws/`의 `dev` 브랜치에서만 수정한다.
- 사용자가 이미 변경한 `src/mdrobot_can_control/launch/motor_safety_bringup.launch.py`의 `input_mode=can_f1` 의도를 새 Safety launch에 보존한다.
- 물리·앱·음성 입력 해제만으로 중앙 래치를 자동 해제하지 않는다.
- `/safety_reset`은 영구 유지보수 인터페이스로 유지하되 내부 안전 검사를 우회하지 않는다.
- 관리자 인증과 SROS2 접근 통제는 이번 범위에서 구현하지 않고 `[GAP]`으로 문서화한다.
- 모터를 `wheel_ekf.launch.py`에 포함하지 않는다.
- 실제 `can1` 상태 변경, Nav2 Goal 전송, 모터 구동 및 실기기 reset은 수행하지 않는다.
- 백업에서는 `RCUTILS_COLORIZED_OUTPUT=1`과 상태별 로그 심각도 처리만 재사용한다.
- 사용자 요청이 없으므로 commit과 push를 수행하지 않는다.

---

## File Structure

- `src/vica_safety/vica_safety/emergency_latch.py`: ROS와 독립적인 중앙 래치·source freshness 규칙
- `src/vica_safety/vica_safety/emergency_stop_node.py`: CAN F1/앱/음성 입력과 중앙 래치 ROS 배선
- `src/vica_safety/vica_safety/safety_gate.py`: Supervisor reset 허용 조건과 상태 전이 규칙
- `src/vica_safety/vica_safety/safety_supervisor_node.py`: `/cmd_vel_req` 승인 및 `/cmd_vel_safe` 발행
- `src/vica_safety/vica_safety/reset_sequence.py`: reset 단계와 실패 결과를 표현하는 순수 모델
- `src/vica_safety/vica_safety/app_emergency_node.py`: 공개 서비스·Nav2 cancel/status·내부 서비스 호출
- `src/vica_safety/launch/safety_bringup.launch.py`: Safety 노드 세 개와 컬러 로그 환경 설정
- `src/vica_safety/test/`: 중앙 래치, gate, reset 절차, launch 계약 단위 테스트
- `src/mdrobot_can_control/launch/motor_bringup.launch.py`: motor 노드 전용 launch
- `src/mdrobot_can_control/`: Safety 실행 파일과 의존성을 제거한 actuator 전용 패키지

### Task 1: `vica_safety` 패키지와 중앙 래치 순수 모델

**Files:**
- Create: `src/vica_safety/package.xml`
- Create: `src/vica_safety/setup.py`
- Create: `src/vica_safety/setup.cfg`
- Create: `src/vica_safety/resource/vica_safety`
- Create: `src/vica_safety/vica_safety/__init__.py`
- Create: `src/vica_safety/vica_safety/emergency_latch.py`
- Create: `src/vica_safety/test/test_emergency_latch.py`

**Interfaces:**
- Produces: `EmergencyLatch.update_source(name: str, active: bool, now: float) -> None`
- Produces: `EmergencyLatch.mark_physical_seen(active: bool, now: float) -> None`
- Produces: `EmergencyLatch.evaluate(now: float) -> LatchSnapshot`
- Produces: `EmergencyLatch.try_reset(now: float) -> tuple[bool, str]`

- [ ] **Step 1: Write failing central latch tests**

```python
def test_releasing_physical_input_keeps_latch_until_reset():
    latch = EmergencyLatch(f1_timeout_sec=0.5)
    latch.mark_physical_seen(True, 1.0)
    latch.mark_physical_seen(False, 1.1)
    assert latch.evaluate(1.2).latched is True
    assert latch.try_reset(1.2)[0] is True
    assert latch.evaluate(1.2).latched is False

def test_reset_rejects_active_or_stale_physical_input():
    latch = EmergencyLatch(f1_timeout_sec=0.5)
    latch.mark_physical_seen(True, 1.0)
    assert latch.try_reset(1.1)[0] is False
    latch.mark_physical_seen(False, 1.2)
    assert latch.try_reset(2.0)[0] is False
```

- [ ] **Step 2: Run the tests and confirm the package/model is missing**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test/test_emergency_latch.py`
Expected: FAIL because `vica_safety.emergency_latch` does not exist.

- [ ] **Step 3: Implement the minimal source-aware latch**

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class LatchSnapshot:
    latched: bool
    active_sources: tuple[str, ...]
    physical_fresh: bool
    reset_allowed: bool

class EmergencyLatch:
    def __init__(self, f1_timeout_sec: float, initially_latched: bool = True):
        self.f1_timeout_sec = f1_timeout_sec
        self.latched = initially_latched
        self.sources = {"physical_f1": False, "app": False, "voice": False}
        self.last_physical_time = 0.0

    def update_source(self, name: str, active: bool, now: float) -> None:
        if name not in ("app", "voice"):
            raise ValueError(f"unsupported source: {name}")
        self.sources[name] = active
        if active:
            self.latched = True

    def mark_physical_seen(self, active: bool, now: float) -> None:
        self.sources["physical_f1"] = active
        self.last_physical_time = now
        if active:
            self.latched = True

    def evaluate(self, now: float) -> LatchSnapshot:
        physical_fresh = (
            self.last_physical_time > 0.0
            and now - self.last_physical_time <= self.f1_timeout_sec
        )
        active = [name for name, enabled in self.sources.items() if enabled]
        if not physical_fresh:
            active.append("physical_stale")
        if active:
            self.latched = True
        return LatchSnapshot(
            latched=self.latched,
            active_sources=tuple(sorted(active)),
            physical_fresh=physical_fresh,
            reset_allowed=self.latched and not active,
        )

    def try_reset(self, now: float) -> tuple[bool, str]:
        snapshot = self.evaluate(now)
        if snapshot.active_sources:
            return False, "active sources: " + ",".join(snapshot.active_sources)
        self.latched = False
        return True, "central estop latch cleared"
```

`evaluate()`는 active source 또는 stale physical 입력을 발견하면 즉시 재래치한다.
`try_reset()`은 physical fresh/해제, app 해제, voice 해제를 모두 확인한 경우에만 래치를
해제한다.

- [ ] **Step 4: Run central latch tests**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test/test_emergency_latch.py`
Expected: PASS.

### Task 2: 중앙 E-stop ROS 노드와 상태별 로그

**Files:**
- Create: `src/vica_safety/vica_safety/emergency_stop_node.py`
- Create: `src/vica_safety/test/test_emergency_stop_contract.py`
- Reference only: `src/mdrobot_can_control/mdrobot_can_control/emergency_stop_node.py`

**Interfaces:**
- Consumes: `EmergencyLatch`
- Produces: `/emergency_stop` and `/estop_state` (`std_msgs/msg/Bool`)
- Provides: `/vica_safety/internal/estop_reset` (`std_srvs/srv/Trigger`)
- Consumes: CAN F1 `0x701`, `/app_emergency_stop`, `/voice_emergency_stop`, test-only `/emergency_stop_input`

- [ ] **Step 1: Write contract tests for F1 and log markers**

```python
def test_f1_decoder_uses_both_0x10_bits():
    assert f1_frame_means_estop_active(bytes.fromhex("f100484800000000"), 2, 3, 0x10, 0) is True
    assert f1_frame_means_estop_active(bytes.fromhex("f100585800000000"), 2, 3, 0x10, 0) is False

def test_log_descriptor_marks_wait_reset_as_warning():
    severity, marker = describe_latch_transition("ESTOP_ACTIVE", "ESTOP_RELEASED_WAIT_RESET")
    assert severity == "warn"
    assert marker == "[WAIT RESET]"
```

- [ ] **Step 2: Confirm tests fail before the node module exists**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test/test_emergency_stop_contract.py`
Expected: FAIL on missing exports.

- [ ] **Step 3: Move and adapt the node**

Implement pure helpers with exact signatures:

```python
def f1_frame_means_estop_active(data: bytes, byte_1: int, byte_2: int,
                                mask: int, active_value: int) -> bool:
    return (
        data[byte_1] & mask == active_value
        and data[byte_2] & mask == active_value
    )

def describe_latch_transition(old: str, new: str) -> tuple[str, str]:
    if new in ("ESTOP_ACTIVE", "FAULT"):
        return "error", "[ESTOP ACTIVE]" if new == "ESTOP_ACTIVE" else "[FAULT]"
    if new == "ESTOP_RELEASED_WAIT_RESET":
        return "warn", "[WAIT RESET]"
    return "info", "[ESTOP CLEARED]"
```

The node must publish the latch, not a raw OR, call `EmergencyLatch.evaluate()` every timer cycle,
and route internal reset only through `EmergencyLatch.try_reset()`. Transition logs include
`old -> new`, `estop=True/False`, and `source=<active_sources>`; active/fault uses ERROR, wait-reset uses WARN,
and cleared/ready uses INFO.

- [ ] **Step 4: Run model and node contract tests**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test/test_emergency_latch.py src/vica_safety/test/test_emergency_stop_contract.py`
Expected: PASS.

### Task 3: Safety Supervisor gate와 내부 reset

**Files:**
- Create: `src/vica_safety/vica_safety/safety_gate.py`
- Create: `src/vica_safety/vica_safety/safety_supervisor_node.py`
- Create: `src/vica_safety/test/test_safety_gate.py`
- Reference only: `src/mdrobot_can_control/mdrobot_can_control/safety_supervisor_node.py`

**Interfaces:**
- Produces: `SafetyGate.request_reset(estop_active: bool, estop_fresh: bool, cmd_zero: bool) -> ResetDecision`
- Provides: `/vica_safety/internal/supervisor_reset` (`std_srvs/srv/Trigger`)
- Consumes: `/emergency_stop`, `/cmd_vel_req`
- Produces: `/cmd_vel_safe`, `/safety_state`

- [ ] **Step 1: Write failing gate tests**

```python
import pytest

@pytest.mark.parametrize("active,fresh,zero", [(True, True, True), (False, False, True), (False, True, False)])
def test_reset_rejects_any_unsafe_condition(active, fresh, zero):
    gate = SafetyGate()
    assert gate.request_reset(active, fresh, zero).accepted is False

def test_reset_accepts_only_fresh_released_zero_command():
    gate = SafetyGate()
    decision = gate.request_reset(False, True, True)
    assert decision.accepted is True
    assert decision.state == SafetyState.READY_TO_GO
```

- [ ] **Step 2: Confirm tests fail**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test/test_safety_gate.py`
Expected: FAIL because `SafetyGate` does not exist.

- [ ] **Step 3: Implement gate and adapt ROS node**

```python
class SafetyState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    ESTOP_ACTIVE = "ESTOP_ACTIVE"
    ESTOP_RELEASED_WAIT_RESET = "ESTOP_RELEASED_WAIT_RESET"
    READY_TO_GO = "READY_TO_GO"
    FAULT = "FAULT"

@dataclass(frozen=True)
class ResetDecision:
    accepted: bool
    state: SafetyState
    reason: str
```

Remove the public `/safety_reset` server from this node. Keep output zero unless reset is armed,
E-stop is fresh/released, and a live nonzero command exists. Log state transitions by severity with
`[ESTOP ACTIVE]`, `[FAULT]`, `[WAIT RESET]`, `[SAFETY READY]`, and `[RUNNING]` markers.

- [ ] **Step 4: Run gate tests**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test/test_safety_gate.py`
Expected: PASS.

### Task 4: Reset 오케스트레이터와 앱 상태

**Files:**
- Create: `src/vica_safety/vica_safety/reset_sequence.py`
- Create: `src/vica_safety/vica_safety/app_emergency_node.py`
- Create: `src/vica_safety/test/test_reset_sequence.py`
- Reference only: `src/mdrobot_can_control/mdrobot_can_control/app_emergency_node.py`

**Interfaces:**
- Provides: `/safety_reset`, `/app_estop_reset`, `/app_estop_activate`
- Consumes: `/emergency_stop`, `/safety_state`, `/navigate_to_pose/_action/status`
- Calls: `/navigate_to_pose/_action/cancel_goal`, `/vica_safety/internal/estop_reset`, `/vica_safety/internal/supervisor_reset`
- Produces: `/app_emergency_stop`, `/app_estop_state`

- [ ] **Step 1: Write failing sequence tests**

```python
def test_failure_stops_later_reset_steps():
    sequence = ResetSequence()
    sequence.begin()
    assert sequence.record("nav_goal_check", False, "active goal remains").success is False
    assert sequence.next_step is None

def test_success_requires_ready_to_go():
    sequence = ResetSequence()
    sequence.begin()
    for step in ("nav_goal_check", "estop_reset", "emergency_clear", "supervisor_reset"):
        sequence.record(step, True, "ok")
    assert sequence.record("ready", True, "READY_TO_GO").success is True
```

- [ ] **Step 2: Confirm tests fail**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test/test_reset_sequence.py`
Expected: FAIL because `ResetSequence` does not exist.

- [ ] **Step 3: Implement the sequence and ROS orchestration**

```python
RESET_STEPS = ("nav_goal_check", "estop_reset", "emergency_clear", "supervisor_reset", "ready")

@dataclass(frozen=True)
class SequenceResult:
    success: bool
    step: str
    message: str
```

Both public reset services call the same `_handle_reset()` method. A nonblocking lock rejects a
second concurrent reset. The callback lowers the app source and requires a fresh Nav2 status. It
skips the cancel service when no ACCEPTED/EXECUTING/CANCELING goal exists; otherwise it sends an
empty `CancelGoal.Request` and waits for terminal status. It then invokes both
internal services and waits for fresh `/emergency_stop=false` and `READY_TO_GO`. Every wait has a
finite deadline. Failure logs `[RESET REJECTED] step=<failed_step> reason=<message>`; success logs
`[RESET ACCEPTED] ESTOP_RELEASED_WAIT_RESET -> READY_TO_GO estop=False`.

`/app_estop_state` retains JSON `active` and adds `app_active`, `emergency_active`, `safety_state`,
`reset_allowed`, `message`, and `timestamp`. `active` equals authoritative `emergency_active`.

- [ ] **Step 4: Run reset model tests**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test/test_reset_sequence.py`
Expected: PASS.

### Task 5: Launch와 패키지 책임 분리

**Files:**
- Create: `src/vica_safety/launch/safety_bringup.launch.py`
- Create: `src/vica_safety/test/test_launch_contract.py`
- Create: `src/mdrobot_can_control/launch/motor_bringup.launch.py`
- Modify: `src/mdrobot_can_control/setup.py`
- Modify: `src/mdrobot_can_control/package.xml`
- Delete after replacement: `src/mdrobot_can_control/mdrobot_can_control/emergency_stop_node.py`
- Delete after replacement: `src/mdrobot_can_control/mdrobot_can_control/safety_supervisor_node.py`
- Delete after replacement: `src/mdrobot_can_control/mdrobot_can_control/app_emergency_node.py`
- Delete after replacement: `src/mdrobot_can_control/launch/safety_bringup.launch.py`
- Delete after replacement: `src/mdrobot_can_control/launch/motor_safety_bringup.launch.py`

**Interfaces:**
- Produces: `ros2 launch vica_safety safety_bringup.launch.py`
- Produces: `ros2 launch mdrobot_can_control motor_bringup.launch.py`

- [ ] **Step 1: Write launch source-contract tests**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

def test_safety_launch_contains_three_safety_nodes_and_no_motor():
    text = (ROOT / "src/vica_safety/launch/safety_bringup.launch.py").read_text()
    assert 'executable="emergency_stop_node"' in text
    assert 'executable="safety_supervisor_node"' in text
    assert 'executable="app_emergency_node"' in text
    assert "keyboard_knob" not in text

def test_safety_launch_forces_can_f1_can1_and_colored_output():
    text = (ROOT / "src/vica_safety/launch/safety_bringup.launch.py").read_text()
    for token in ('"input_mode": "can_f1"', '"can_iface": "can1"',
                  '"driver_response_id": 0x701', '"RCUTILS_COLORIZED_OUTPUT", "1"'):
        assert token in text

def test_motor_launch_contains_only_keyboard_knob():
    text = (ROOT / "src/mdrobot_can_control/launch/motor_bringup.launch.py").read_text()
    assert 'executable="keyboard_knob"' in text
    assert "emergency_stop_node" not in text
    assert "safety_supervisor_node" not in text

def test_wheel_ekf_launch_does_not_contain_motor_package():
    text = (ROOT / "src/vica_localization/launch/wheel_ekf.launch.py").read_text()
    assert "mdrobot_can_control" not in text
```

- [ ] **Step 2: Confirm tests fail for missing split launches**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test/test_launch_contract.py`
Expected: FAIL because the new launch files do not exist.

- [ ] **Step 3: Implement launch split and entry points**

The Safety launch uses `SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")` and starts
`emergency_stop_node`, `safety_supervisor_node`, and `app_emergency_node`. The emergency node
parameters are exactly `input_mode="can_f1"`, `can_iface="can1"`, `driver_response_id=0x701`, and
`log_f1_frames=True`. The motor launch starts only `keyboard_knob` with `can_iface="can1"`.

The new package registers all three safety console scripts and declares `action_msgs`. The motor
package retains only its motor console script and motor dependencies.

- [ ] **Step 4: Run launch tests and package metadata checks**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test/test_launch_contract.py`
Expected: PASS.

### Task 6: 기존 consumer와 문서 계약 정합화

**Files:**
- Modify: `src/vica_mission_manager/vica_mission_manager/mission_manager_node.py`
- Modify: `src/vica_mission_manager/vica_mission_manager/emergency_estop_bridge.py`
- Modify: `src/vica_mission_manager/vica_mission_manager/estop_pulse.py`
- Modify: `src/vica_mission_manager/launch/mission_manager.launch.py`
- Modify: `src/vica_mission_manager/README.md`
- Move/replace: `src/mdrobot_can_control/docs/estop_integration_development_direction.md` → `src/vica_safety/docs/estop_integration_development_direction.md`
- Modify: `GOVERNANCE.md`
- Modify: `AGENTS.md`
- Modify: `guideline/vica_architecture.md`
- Modify: `guideline/vica_scenario.md`
- Modify: `guideline/bt와 visual hierarchy of your folders and files.md`
- Create or Modify: `devlog/2026-07-22.md`

**Interfaces:**
- Mission Manager consumes central `/emergency_stop` latch and no longer depends on motor-owned `/estop_state` semantics.
- Documentation identifies `app_emergency_node` as public reset orchestrator, `emergency_stop_node` as latch owner, and `safety_supervisor_node` as drive re-enable owner.

- [ ] **Step 1: Search for stale contracts before edits**

Run: `rg -n 'keyboard_knob.*(래치|estop)|/estop_reset|mdrobot_can_control.*safety|/safety_reset' src guideline GOVERNANCE.md AGENTS.md`
Expected: Matches that identify every stale producer, consumer, launch, and document statement.

- [ ] **Step 2: Update consumers and shared documents**

Remove Mission Manager's `/estop_state` subscription and treat `/emergency_stop` as the authoritative
central latch. Preserve its own Goal cancel behavior for defense in depth. Update all listed documents
to the three-owner reset model, independent launches, permanent maintenance service, authentication
`[GAP]`, no auto-resume, and hardware test `[미검증]` status.

- [ ] **Step 3: Verify no stale public reset owner remains**

Run: `rg -n '/estop_reset|keyboard_knob.*래치|mdrobot_can_control.*(emergency_stop_node|safety_supervisor_node|app_emergency_node)' src guideline GOVERNANCE.md AGENTS.md`
Expected: No active-code or current-architecture claim assigns Safety/reset ownership to the motor package; historical devlog text may remain explicitly marked historical.

### Task 7: 대상 패키지 빌드·테스트와 최종 diff 검토

**Files:**
- Modify only as required by failures in Tasks 1-6.

- [ ] **Step 1: Run pure tests**

Run: `PYTHONPATH=src/vica_safety python3 -m pytest -q src/vica_safety/test`
Expected: PASS.

- [ ] **Step 2: Build only affected ROS packages**

Run: `colcon build --packages-select vica_safety mdrobot_can_control vica_mission_manager`
Expected: all three packages finish successfully. If the shell lacks sourced ROS dependencies, report
the missing environment as `[미검증]` rather than claiming success.

- [ ] **Step 3: Run package tests**

Run: `colcon test --packages-select vica_safety mdrobot_can_control vica_mission_manager --event-handlers console_direct+`
Run: `colcon test-result --verbose`
Expected: no failed tests in the selected packages.

- [ ] **Step 4: Inspect contracts and changes**

Run: `git diff --check`
Run: `git status --short`
Run: `git diff -- src/vica_safety src/mdrobot_can_control src/vica_mission_manager GOVERNANCE.md AGENTS.md guideline devlog`
Expected: no whitespace errors, no secrets or personal absolute paths, no unrelated changes, and the
user's `can_f1` intent is represented in `vica_safety/launch/safety_bringup.launch.py`.

- [ ] **Step 5: Report hardware limitations**

Do not run launch files against `can1`. Report CAN F1, physical button, Nav2 live Goal cancellation,
motor zero-output, color rendering, and actual reset as `[미검증]` until the user confirms wheels lifted,
area controlled, physical E-stop available, and immediate power cut-off available.
