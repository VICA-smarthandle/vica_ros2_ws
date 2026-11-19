"""도킹 흐름(2026-11-18) — 미션 쪽 판정. ROS 를 모른다(노드가 명령을 실행한다).

누가 무엇을 하나
  vica_docking_node  V자를 보고 실제로 들어가고(start)·나온다(undock). 결과는 /vica/dock/state.
  여기(DockFlow)      앱 요청을 받아 언제 무엇을 시킬지 정한다. 미션의 다른 상태 기계(MissionLogic)는
                      그동안 IDLE 이고, dock_mode 로 '도크 관련 중'임만 안다(음성·접근·고개 돌리기를 막는다).

흐름(11-18 사용자 결정, 앱 목업 62~73)
  도킹 시작(앱)  도크 위치가 저장돼 있어야 한다. 도크 가까이면 바로 들어가고, 준비 위치 바로 옆이면
                제자리에서 도크를 보게 돈 뒤 들어가고(11-19), 아니면 Nav2 로
                준비 위치(도크 앞 1 m)까지 간 뒤 들어간다. 준비 위치에서 V자를 못 찾거나 범위를 벗어나면
                준비 위치로 한 번 더 가서 다시 찾는다(가까이서 놓친 것은 도킹 노드가 스스로 물러나 다시 한다).
  도크에서 나오기(앱)  초음파 켜고 후진 → 제자리 회전 → 주행 모드(그 자리에서 기다림).
                중간에 멈추면(중지·비상정지) 도킹 노드가 기억한다(undock_pending) — 다시 누르면 남은 동작만 한다(11-19).
                그 상태에서 음성은 받지 않고("아직 밥을 덜 먹었어요"), 앱 원격 주행은 그대로 보낸다(85 cm 안쪽이면 먼저 물러남).
  도크에 있을 때 원격 주행(장소·홈·경비 등)  미션이 보낸 주행을 잡아 두고 먼저 나온 뒤 보낸다.
  중지(앱)  가는 중이면 Nav2 취소, 들어가거나 나오는 중이면 도킹 노드 cancel. 자동으로 다시 하지 않는다.
  비상정지   도킹 노드가 스스로 멈춘다(/estop_state). 여기서는 흐름을 끝내고 다시 하지 않는다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Optional

# MissionLogic.dock_mode 값 = 앱·손잡이로 나가는 dialog_state
MODE_NONE = ""
MODE_DOCKING = "docking"       # 준비 위치로 가는 중 + 들어가는 중(앱 화면 같음, 목업 63·64)
MODE_DOCKED = "docked"         # 손잡이 LED 상태 6(연두 숨쉬기)
MODE_UNDOCKING = "undocking"

# 준비 위치 바로 옆에서 '도킹 시작'(11-19 실기): Nav2 는 0.16 m 떨어진 목표에 경로를 못 그렸다('Path is empty').
# 준비 위치 좌표(+along = 도크 쪽, +lateral = 왼쪽)로 아래 범위 안이면 Nav2 주행 대신 제자리에서 도크를 보게 돈다.
# 이 각 안이면 이미 보고 있는 것 — 돌지 않는다(도킹 노드는 ±30° 까지 받는다).
#   좌우 0.25  도킹 노드의 좌우 출발 한계 0.30 m 에서 위치 오차 여유 0.05 를 뺀 값(11-19 검토: 0.6 m 원이면
#              옆으로 0.3~0.6 m 인 자리에서 돈 뒤 out_of_range 로 끝났다)
#   뒤로 0.60  준비 위치보다 먼 쪽 — 도킹 노드가 스스로 준비 위치까지 온다
#   앞으로 0.30 구동축~꼭짓점 0.851 m. 도킹 노드의 제자리 회전 한계(0.82 m, approach rot_clear 0.72 + 팔 0.10)보다 멀다.
#              도크에서 나온 자리는 along +0.15(구동축~꼭짓점 1.001 m)라 위치 오차가 있어도 이 안에 든다(11-19 재검토:
#              경계를 0.15 로 두었더니 나온 자리가 경계에 걸려, 등진 채 바로 진입 → V자를 못 찾고 실패했다).
#              그보다 도크 쪽이면 '도크 구역'이다(아래)
NEAR_SPIN_LATERAL_M = 0.25
NEAR_SPIN_BACK_M = 0.60
NEAR_SPIN_FWD_M = 0.30
NEAR_SPIN_SKIP_RAD = 0.35      # 20°
# 도크 구역: 준비 위치보다 도크 쪽으로 0.30 m 넘게 들어온 자리(구동축~꼭짓점 0.85 m 안)·좌우 0.45 m 안.
# 여기서 Nav2 로 돌면 손잡이(반경 0.67 m)가 구조물에 닿는다 — 도킹 노드의 near_dock 이 내려가 있어도
# (일이 끝날 때만 갱신된다) Nav2 를 쓰지 않고 바로 들어간다. 줄이 안 맞으면 도킹 노드가 물러나 다시 한다.
DOCK_ZONE_LATERAL_M = 0.45
DOCK_ZONE_DEPTH_M = 1.30       # 준비 위치에서 꼭짓점 너머까지(1.0 + 여유)
# 로봇이 도크 꼭짓점에서 이만큼 넘게 떨어지면 '도크 근처'·'나오다 멈춤' 표시는 낡은 것이다(미션이 내린다).
# 준비 위치가 꼭짓점에서 1.15 m 다 — 그 두 배 가까이. 위치 분산(x+y, m²)이 DOCK_FAR_COV 이하일 때만 판정한다
# (초기 위치를 막 잡은 값 0.5 — 수렴 전이거나 잘못 잡았으면 내리지 않는다).
DOCK_FAR_M = 2.0
DOCK_FAR_COV = 0.5


def staging_relation(px: float, py: float, pyaw_deg: float,
                     sx: float, sy: float, syaw_deg: float) -> tuple:
    """로봇이 준비 위치에서 어디에 있나. (along m, lateral m, 도크를 보려면 돌 각 rad).
    along +  = 준비 위치보다 도크 쪽, lateral + = 도크를 볼 때 왼쪽, 각 + = 반시계."""
    yaw = math.radians(syaw_deg)
    dx, dy = px - sx, py - sy
    along = dx * math.cos(yaw) + dy * math.sin(yaw)
    lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
    d = math.radians(syaw_deg - pyaw_deg)
    return along, lateral, math.atan2(math.sin(d), math.cos(d))


def classify_start(along: float, lateral: float) -> str:
    """'도킹 시작' 때 로봇 자리. zone = 도크 구역(바로 들어간다) · spin = 준비 위치 바로 옆(제자리 회전) · far = Nav2."""
    if NEAR_SPIN_FWD_M < along < DOCK_ZONE_DEPTH_M and abs(lateral) <= DOCK_ZONE_LATERAL_M:
        return "zone"
    if -NEAR_SPIN_BACK_M <= along <= NEAR_SPIN_FWD_M and abs(lateral) <= NEAR_SPIN_LATERAL_M:
        return "spin"
    return "far"

# 노드가 실행하는 명령
CALL_START = "start"
CALL_UNDOCK = "undock"
CALL_CANCEL = "cancel"


@dataclass
class Cmd:
    kind: str                  # call · nav · spin · restage · cancel_nav · release_held · drop_held · event
    arg: Any = None
    message: str = ""


@dataclass
class DockFlow:
    mode: str = MODE_NONE
    # 도킹 노드 상태(/vica/dock/state JSON) 최근 값
    node_state: str = "idle"
    near_dock: bool = False
    undock_pending: bool = False   # 나오다 멈춤(도킹 노드가 기억) — '도크에서 나오기'로 이어서 한다(11-19)
    node_seq: int = -1
    # 흐름 안 단계
    phase: str = ""            # going(Nav2) · entering(도킹 노드) · leaving(도킹 노드 해제)
    nav_retried: bool = False
    held: Any = None           # 도크에서 나온 뒤 보낼 미션 주행(Navigate·SpinInPlace)
    staging: Any = None        # 준비 위치 Destination
    canceling: bool = False
    events: list = field(default_factory=list)

    # ── 질문 ───────────────────────────────────────────────────────────
    def busy(self) -> bool:
        return self.phase != ""

    def blocks_motion(self) -> bool:
        """미션 주행을 바로 보내면 안 되는가(도크 안·근처 — 제자리 회전하면 구조물에 닿는다)."""
        return self.mode == MODE_DOCKED or (self.near_dock and self.phase == "")

    def owns_nav(self) -> bool:
        return self.phase == "going"

    # ── 앱 요청 ────────────────────────────────────────────────────────
    def request_start(self, logic_idle: bool, estop: bool, nav_ready: bool, staging,
                      near_spin: Optional[float] = None, in_zone: bool = False) -> tuple:
        """(허용, 사유, 명령). near_spin: 준비 위치 바로 옆(classify_start 'spin')일 때 도크를 보려면 돌 각(rad),
        아니면 None. in_zone: 로봇 위치로 본 도크 구역 안(classify_start 'zone')."""
        if estop:
            return False, "estop", []
        if self.mode == MODE_DOCKED:
            return False, "already_docked", []
        if self.busy() or not logic_idle:
            return False, "busy", []
        if staging is None:
            return False, "no_dock", []
        self.staging = staging
        self.nav_retried = False
        self.canceling = False
        if self.near_dock or in_zone:
            # 도크 가까이(범퍼~꼭짓점 85 cm 안): Nav2 로 돌면 구조물에 닿는다 — 바로 들어간다.
            # 도킹 노드가 줄이 안 맞으면 준비 위치로 물러나 다시 한다.
            return True, "ok", self._enter()
        if near_spin is not None:
            # 준비 위치 바로 옆(도크에서 나온 직후 등): 제자리에서 도크를 보게 돈 뒤 들어간다.
            # 못 찾거나 범위 밖이면 한 번 다시 한다 — restage 가 자리를 다시 보고 Nav2·회전·바로 진입을 고른다
            # (도킹 노드가 도크 위치를 고쳐 뒀으면 그 값으로).
            if abs(near_spin) < NEAR_SPIN_SKIP_RAD:
                return True, "ok", self._enter()
            if not nav_ready:
                return False, "nav_not_ready", []
            self.phase = "going"
            self.mode = MODE_DOCKING
            return True, "ok", [Cmd("spin", near_spin)]
        if not nav_ready:
            return False, "nav_not_ready", []
        return True, "ok", self._go()

    def request_undock(self, logic_idle: bool, estop: bool) -> tuple:
        if estop:
            return False, "estop", []
        if self.busy() or not logic_idle:
            return False, "busy", []
        if not (self.mode == MODE_DOCKED or self.near_dock or self.undock_pending):
            return False, "not_docked", []
        self.canceling = False
        self.phase = "leaving"
        self.mode = MODE_UNDOCKING
        return True, "ok", [Cmd("call", CALL_UNDOCK)]

    def request_cancel(self) -> tuple:
        if self.phase == "going":
            self._reset_flow()
            self.mode = MODE_NONE
            return True, "ok", [Cmd("cancel_nav")]
        if self.phase in ("entering", "leaving"):
            self.canceling = True
            return True, "ok", [Cmd("call", CALL_CANCEL)]
        return False, "idle", []

    # ── 미션이 주행을 보내려 할 때 ─────────────────────────────────────
    def hold(self, action) -> list:
        """도크 안·근처에서 미션이 주행을 보내려 한다 — 잡아 두고 먼저 나온다."""
        self.held = action
        self.canceling = False
        self.phase = "leaving"
        self.mode = MODE_UNDOCKING
        return [Cmd("call", CALL_UNDOCK)]

    def intercept(self, action, is_spin: bool) -> tuple:
        """미션이 주행(Navigate·SpinInPlace)을 보내려 한다. (판정, 명령).
        pass = 그대로 보낸다 · held = 잡아 뒀다(도크에서 나온 뒤 보낸다) · drop = 보내지 않는다(미션에는 실패로 알린다).

        11-19 검토: 예전에는 도크 안·근처에서 쉬는 중일 때만 잡았다. 흐름이 도는 중(가는 중·들어가는 중·나오는 중)에
        온 주행은 그대로 Nav2 로 나가 도킹 노드와 /cmd_vel_req 를 다퉜다(도킹 노드는 conflict 로 멈추고 Nav2 가
        도크 입구에서 혼자 주행)."""
        if self.phase == "leaving":
            if self.canceling or is_spin:
                return "drop", []      # 멈추는 중이거나, 제자리 회전 하나 — 나온 뒤에 할 일이 아니다
            self.held = action         # 나오는 중 — 나온 뒤 보낸다(잡아 둔 것이 있으면 새것으로 바꾼다)
            return "held", []
        if self.phase in ("going", "entering"):
            return "drop", []          # 도킹 중 — '도킹 중지' 뒤에 보낸다
        if self.blocks_motion():
            if is_spin:
                return "drop", []      # 제자리 회전 하나 때문에 도크에서 나오지 않는다
            return "held", self.hold(action)
        return "pass", []

    def on_mission_cancel(self, keep_leaving: bool = False) -> list:
        """미션이 주행을 취소했다(앱 취소·비상정지 등). 잡아 둔 주행이 있으면 버리고 나오기를 멈춘다.
        keep_leaving: 바로 뒤에 새 주행이 이어진다(앱이 다른 곳으로 다시 보냄) — 나오기는 멈추지 않는다."""
        if self.held is None:
            return []
        self.held = None
        if self.phase == "leaving" and not keep_leaving:
            self.canceling = True
            return [Cmd("call", CALL_CANCEL)]
        return []

    def on_estop(self, active: bool) -> list:
        if not active:
            return []
        cmds: list = []
        if self.phase == "going":
            self._reset_flow()
            self.mode = MODE_NONE
            cmds.append(Cmd("cancel_nav"))
        # 들어가거나 나오는 중이면 도킹 노드가 /estop_state 로 스스로 멈추고 결과를 알린다.
        return cmds

    # ── Nav2 결과(준비 위치로 가는 중) ─────────────────────────────────
    def on_nav_status(self, status: str) -> list:
        """status: succeeded · failed · canceled · running · none"""
        if self.phase != "going":
            return []
        if status == "succeeded":
            return self._enter()
        if status in ("failed", "canceled"):
            self._reset_flow()
            self.mode = MODE_NONE
            reason = "nav_failed" if status == "failed" else "canceled"
            return [Cmd("event", "dock_failed", reason)]
        return []

    # ── 도킹 노드 상태 ────────────────────────────────────────────────
    def on_node_state(self, st: dict) -> list:
        """/vica/dock/state 한 건. 일이 끝났는지는 last.result 가 채워졌는지로 안다 —
        도킹 노드는 일을 시작할 때 last.result 를 비우고 끝날 때 채운다(중간 상태는 놓쳐도 된다)."""
        seq = int(st.get("seq", -1))
        if seq >= 0 and seq == self.node_seq:
            return []
        # 번호가 뒤로 갔다 = 도킹 노드가 다시 떴다(번호는 1 부터 다시 센다). 버리면 번호가 옛 값을 넘을 때까지
        # 상태를 못 받는다(11-19 검토).
        restarted = 0 <= seq < self.node_seq
        self.node_seq = seq
        self.node_state = str(st.get("state", "idle"))
        self.near_dock = bool(st.get("near_dock", False))
        self.undock_pending = bool(st.get("undock_pending", False))
        if restarted and self.phase in ("entering", "leaving"):
            # 시킨 일 도중에 다시 떴다 — 그 일은 사라졌다. 흐름을 접는다.
            return self._fold("dock_node_restarted")
        last = st.get("last", {}) or {}
        action = str(last.get("action", ""))
        result = str(last.get("result", ""))
        reason = str(last.get("reason", ""))
        cmds: list = []
        if self.phase == "":
            # 켜질 때 도크 안이었거나(도킹 노드가 V자로 확인), 도킹 노드가 다시 떠서 도크 밖이라고 한다
            if self.node_state == "docked":
                self.mode = MODE_DOCKED
            elif self.mode == MODE_DOCKED:
                self.mode = MODE_NONE
            return cmds
        if self.node_state in ("docking", "undocking") or result == "":
            return cmds   # 아직 하는 중
        if self.phase == "entering" and action == "dock":
            if result == "success":
                self._reset_flow()
                self.mode = MODE_DOCKED
                cmds.append(Cmd("event", "dock_succeeded"))
            elif (reason in ("not_found", "out_of_range") and not self.nav_retried
                  and not self.canceling and self.staging is not None and not self.near_dock):
                # 준비 위치에서 못 찾음 — 준비 위치로 한 번 더 가서 다시 찾는다(11-18 사용자 "다시 찾는 거 좋아")
                # 도킹 노드가 준비 위치에서 본 값으로 도크 위치를 고쳤을 수 있다(11-19) — 노드가 dock.yaml 을
                # 다시 읽어 restage() 로 넘긴다.
                self.nav_retried = True
                cmds.append(Cmd("restage"))
            else:
                canceled = self.canceling
                self._reset_flow()
                self.mode = MODE_DOCKED if self.node_state == "docked" else MODE_NONE
                if not (canceled and reason == "canceled"):
                    cmds.append(Cmd("event", "dock_failed", reason or "unknown"))
        elif self.phase == "leaving" and action == "undock":
            held = self.held
            canceled = self.canceling
            self._reset_flow()
            if result == "success":
                self.mode = MODE_NONE
                if held is not None and canceled:
                    # 중지를 눌렀는데 거의 다 나와 성공으로 끝났다(남은 회전 10° 안, 11-19) — 주행은 보내지 않는다
                    cmds.append(Cmd("drop_held", held))
                elif held is not None:
                    cmds.append(Cmd("release_held", held))
                else:
                    cmds.append(Cmd("event", "undock_succeeded"))
            else:
                self.mode = MODE_DOCKED if self.node_state == "docked" else MODE_NONE
                if held is not None:
                    cmds.append(Cmd("drop_held", held))
                if not (canceled and reason == "canceled"):
                    cmds.append(Cmd("event", "undock_failed", reason or "unknown"))
        return cmds

    def on_call_result(self, call: str, ok: bool, message: str) -> list:
        """도킹 노드 서비스가 요청을 거절했다(busy·already_docked 등) — 흐름을 접는다."""
        if ok:
            return []
        if call == CALL_CANCEL:
            if message == "dock_node_down" and self.phase in ("entering", "leaving"):
                # 멈추라고 할 도킹 노드가 없다 — 하던 일도 없다. 안 접으면 영영 '하는 중'이다(11-19 검토).
                return self._fold(message)
            return []   # 그 밖의 cancel 거절(이미 끝남 등)은 상태 토픽이 알려 준다
        if message == "already_docked":
            self._reset_flow()
            self.mode = MODE_DOCKED
            return [Cmd("event", "dock_succeeded")]
        return self._fold(message or "rejected", "dock_failed" if call == CALL_START else "undock_failed")

    def _fold(self, reason: str, kind: str = "") -> list:
        """하던 일(들어가기·나오기)을 실패로 접는다."""
        if not kind:
            kind = "undock_failed" if self.phase == "leaving" else "dock_failed"
        held = self.held
        self._reset_flow()
        self.mode = MODE_DOCKED if self.node_state == "docked" else MODE_NONE
        cmds: list = []
        if held is not None:
            cmds.append(Cmd("drop_held", held))
        cmds.append(Cmd("event", kind, reason))
        return cmds

    def restage(self, staging, near_spin: Optional[float], in_zone: bool = False) -> list:
        """재시도: 다시 읽은 준비 위치로 간다. 바로 옆이면 Nav2 대신 제자리 회전(이미 보고 있으면 바로 진입),
        도크 구역 안이면 바로 진입."""
        if staging is not None:
            self.staging = staging
        if in_zone:
            return self._enter()
        if near_spin is not None:
            if abs(near_spin) < NEAR_SPIN_SKIP_RAD:
                return self._enter()
            self.phase = "going"
            self.mode = MODE_DOCKING
            return [Cmd("spin", near_spin)]
        return self._go()

    # ── 안쪽 ──────────────────────────────────────────────────────────
    def _go(self) -> list:
        self.phase = "going"
        self.mode = MODE_DOCKING
        return [Cmd("nav", self.staging)]

    def _enter(self) -> list:
        self.phase = "entering"
        self.mode = MODE_DOCKING
        return [Cmd("call", CALL_START)]

    def _reset_flow(self) -> None:
        self.phase = ""
        self.held = None
        self.canceling = False


def parse_node_state(text: str) -> Optional[dict]:
    import json
    try:
        st = json.loads(text)
    except (ValueError, TypeError):
        return None
    return st if isinstance(st, dict) else None
