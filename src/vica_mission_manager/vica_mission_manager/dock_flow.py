"""도킹 흐름(2026-11-18) — 미션 쪽 판정. ROS 를 모른다(노드가 명령을 실행한다)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Optional

MODE_NONE = ""
MODE_DOCKING = "docking"
MODE_DOCKED = "docked"
MODE_UNDOCKING = "undocking"

NEAR_SPIN_LATERAL_M = 0.25
NEAR_SPIN_BACK_M = 0.60
NEAR_SPIN_FWD_M = 0.30
NEAR_SPIN_SKIP_RAD = 0.35
DOCK_ZONE_LATERAL_M = 0.45
DOCK_ZONE_DEPTH_M = 1.30
DOCK_FAR_M = 2.0
DOCK_FAR_COV = 0.5


def staging_relation(px: float, py: float, pyaw_deg: float,
                     sx: float, sy: float, syaw_deg: float) -> tuple:
    """로봇이 준비 위치에서 어디에 있나. (along m, lateral m, 도크를 보려면 돌 각 rad)."""
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

CALL_START = "start"
CALL_UNDOCK = "undock"
CALL_CANCEL = "cancel"


@dataclass
class Cmd:
    kind: str
    arg: Any = None
    message: str = ""


@dataclass
class DockFlow:
    mode: str = MODE_NONE
    node_state: str = "idle"
    near_dock: bool = False
    undock_pending: bool = False
    node_seq: int = -1
    phase: str = ""
    nav_retried: bool = False
    held: Any = None
    staging: Any = None
    canceling: bool = False
    events: list = field(default_factory=list)

    def busy(self) -> bool:
        return self.phase != ""

    def blocks_motion(self) -> bool:
        """미션 주행을 바로 보내면 안 되는가(도크 안·근처 — 제자리 회전하면 구조물에 닿는다)."""
        return self.mode == MODE_DOCKED or (self.near_dock and self.phase == "")

    def owns_nav(self) -> bool:
        return self.phase == "going"

    def request_start(self, logic_idle: bool, estop: bool, nav_ready: bool, staging,
                      near_spin: Optional[float] = None, in_zone: bool = False) -> tuple:
        """(허용, 사유, 명령). near_spin: 준비 위치 바로 옆(classify_start 'spin')일 때 도크를 보려면 돌 각(rad),"""
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
            return True, "ok", self._enter()
        if near_spin is not None:
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

    def hold(self, action) -> list:
        """도크 안·근처에서 미션이 주행을 보내려 한다 — 잡아 두고 먼저 나온다."""
        self.held = action
        self.canceling = False
        self.phase = "leaving"
        self.mode = MODE_UNDOCKING
        return [Cmd("call", CALL_UNDOCK)]

    def intercept(self, action, is_spin: bool) -> tuple:
        """미션이 주행(Navigate·SpinInPlace)을 보내려 한다. (판정, 명령)."""
        if self.phase == "leaving":
            if self.canceling or is_spin:
                return "drop", []
            self.held = action
            return "held", []
        if self.phase in ("going", "entering"):
            return "drop", []
        if self.blocks_motion():
            if is_spin:
                return "drop", []
            return "held", self.hold(action)
        return "pass", []

    def on_mission_cancel(self, keep_leaving: bool = False) -> list:
        """미션이 주행을 취소했다(앱 취소·비상정지 등). 잡아 둔 주행이 있으면 버리고 나오기를 멈춘다."""
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
        return cmds

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

    def on_node_state(self, st: dict) -> list:
        """/vica/dock/state 한 건. 일이 끝났는지는 last.result 가 채워졌는지로 안다 —"""
        seq = int(st.get("seq", -1))
        if seq >= 0 and seq == self.node_seq:
            return []
        restarted = 0 <= seq < self.node_seq
        self.node_seq = seq
        self.node_state = str(st.get("state", "idle"))
        self.near_dock = bool(st.get("near_dock", False))
        self.undock_pending = bool(st.get("undock_pending", False))
        if restarted and self.phase in ("entering", "leaving"):
            return self._fold("dock_node_restarted")
        last = st.get("last", {}) or {}
        action = str(last.get("action", ""))
        result = str(last.get("result", ""))
        reason = str(last.get("reason", ""))
        cmds: list = []
        if self.phase == "":
            if self.node_state == "docked":
                self.mode = MODE_DOCKED
            elif self.mode == MODE_DOCKED:
                self.mode = MODE_NONE
            return cmds
        if self.node_state in ("docking", "undocking") or result == "":
            return cmds
        if self.phase == "entering" and action == "dock":
            if result == "success":
                self._reset_flow()
                self.mode = MODE_DOCKED
                cmds.append(Cmd("event", "dock_succeeded"))
            elif (reason in ("not_found", "out_of_range") and not self.nav_retried
                  and not self.canceling and self.staging is not None and not self.near_dock):
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
                return self._fold(message)
            return []
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
        """재시도: 다시 읽은 준비 위치로 간다. 바로 옆이면 Nav2 대신 제자리 회전(이미 보고 있으면 바로 진입),"""
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
