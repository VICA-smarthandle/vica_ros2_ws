#!/usr/bin/env python3
"""Nav2 최고 속도를 노드를 끄지 않고 바꾼다(2026-10-16, 시연장 '빠르게 보여주기'용).

    vica_nav2_speed.sh 0.9             # 0.9 m/s 로(가감속·충돌 감시 상자 함께)
    vica_nav2_speed.sh 0.5             # nav2_params.yaml 원래 값으로 되돌림
    vica_nav2_speed.sh 0.9 --dry-run   # 넣을 값만 보여 준다(ROS 안 씀)
    vica_nav2_speed.sh --show          # 지금 노드에 든 값·상태만 읽는다(바꾸지 않음)

순서
 1) 기준 파일(설치된 nav2_params.yaml = 지금 Nav2 가 읽은 것)의 값을 목표 속도 비율로 계산한다.
 2) /odom 으로 로봇이 서 있는지 본다. 움직이면 멈춘다 — 재설정은 진행 중인 목표를 끊는다.
 3) controller_server·velocity_smoother·collision_monitor 에 값을 넣는다(set_parameters).
 4) lifecycle_manager_navigation·lifecycle_manager_collision_monitor 에 RESET → STARTUP.
    Nav2 노드는 configure 때 파라미터를 다시 읽으므로 새 값으로 다시 선다(자작 VCC·
    LatchedGoalChecker 도 declare_parameter_if_not_declared 라 두 번째 configure 에서 새 값을 읽는다).
    AMCL·map_server·EKF·엔코더·모터·미션은 건드리지 않는다 → 초기 위치를 다시 찍지 않는다.
 5) 노드가 전부 active 이고 값이 들어갔는지 확인한다. 관리자 응답이 DDS 에서 사라지는 경합
    (nav2_map_test.launch.py collision_monitor 구조 타이머 주석, 2026-09-02)이 있어, 관리자가
    끝내지 못한 노드는 직접 deactivate·cleanup / configure·activate 를 보낸다.

바꾸는 값 — 기준 속도 대비 비율 k(0.5 -> 0.9 면 1.8). 10-06 0.5 -> 0.7(1.4배) 때와 같은 묶음이다.
  - FollowPath.desired_linear_vel = 목표, linear_accel·planned_linear_decel·curve_decel × k
    (planned 는 비상 제동 이하, curve 는 planned 이하 — test_vcc_limits_match_the_smoother 조건)
  - velocity_smoother.max_velocity[0] = 목표
  - 충돌 감시 PolygonStop·PolygonSlow 앞면을 VCC 정지거리 식(vcc_core.cpp 146행)이 늘어난 만큼 앞으로.
    0.5 -> 0.9 에서 정지거리 0.30 -> 0.644 m, 두 상자 앞면 +0.344 m.
그대로 두는 값: 비상 제동 1.25(물리 제동력이라 비율로 못 올림), 출발 1단계 0.25/0.5 s, 회전 0.5,
  curve_min_radius 0.8, slow_clearance 0.35, approach 0.6 — 10-06 0.7 때와 같다.
"""
import argparse
import sys
import time
from pathlib import Path

import yaml

STOP_LATENCY = 0.3   # vcc_core.hpp stop_latency 기본값(CAN·드라이버 지연)
STOP_MARGIN = 0.05   # vcc_core.hpp stop_margin 기본값
SPEED_MIN, SPEED_MAX = 0.2, 1.0   # 위는 Safety·모터 max_linear_mps(둘 다 1.0)
MANAGERS = ('lifecycle_manager_navigation', 'lifecycle_manager_collision_monitor')


def base_params_path():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('vica_nav2')) / 'config' / 'nav2_params.yaml'


def plan(base, v):
    """목표 속도 v 에서 넣을 값 {노드: {파라미터: 값}} 과 비율 k, 상자 앞면 이동량."""
    fp = base['controller_server']['ros__parameters']['FollowPath']
    sm = base['velocity_smoother']['ros__parameters']
    cm = base['collision_monitor']['ros__parameters']
    v0 = float(fp['desired_linear_vel'])
    k = v / v0
    brake = float(fp['max_linear_decel'])
    planned = min(round(float(fp['planned_linear_decel']) * k, 3), brake)
    curve = min(round(float(fp['curve_decel']) * k, 3), planned)
    ctrl = {
        'FollowPath.desired_linear_vel': round(v, 3),
        'FollowPath.linear_accel': round(float(fp['linear_accel']) * k, 3),
        'FollowPath.planned_linear_decel': planned,
        'FollowPath.curve_decel': curve,
    }
    max_velocity = [float(x) for x in sm['max_velocity']]
    max_velocity[0] = round(v, 3)

    def stop_dist(s):
        return s * s / (2.0 * brake) + s * STOP_LATENCY + STOP_MARGIN

    shift = stop_dist(v) - stop_dist(v0)
    coll = {}
    for name in ('PolygonStop', 'PolygonSlow'):
        pts = [float(x) for x in cm[name]['points']]
        front = max(pts[0::2])
        coll[f'{name}.points'] = [
            round(p + shift, 3) if i % 2 == 0 and abs(p - front) < 1e-9 else p
            for i, p in enumerate(pts)]
    targets = {
        'controller_server': ctrl,
        'velocity_smoother': {'max_velocity': max_velocity},
        'collision_monitor': coll,
    }
    return targets, k, shift, stop_dist(v)


def fmt(value):
    if isinstance(value, list):
        return '[' + ', '.join(f'{x:g}' for x in value) + ']'
    return f'{value:g}' if isinstance(value, float) else str(value)


def same(a, b):
    if isinstance(a, list) or isinstance(b, list):
        return (isinstance(a, list) and isinstance(b, list) and len(a) == len(b)
                and all(abs(x - y) < 1e-6 for x, y in zip(a, b)))
    return a is not None and b is not None and abs(a - b) < 1e-6


# ---------------------------------------------------------------------------- ROS
def ros_main(args, targets):
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState, GetState
    from nav2_msgs.srv import ManageLifecycleNodes
    from nav_msgs.msg import Odometry

    def to_param(name, value):
        pv = ParameterValue()
        if isinstance(value, list):
            pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = [float(x) for x in value]
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        return Parameter(name=name, value=pv)

    def from_value(pv):
        if pv.type == ParameterType.PARAMETER_DOUBLE:
            return pv.double_value
        if pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(pv.double_array_value)
        if pv.type == ParameterType.PARAMETER_INTEGER:
            return float(pv.integer_value)
        if pv.type == ParameterType.PARAMETER_STRING_ARRAY:
            return list(pv.string_array_value)
        return None

    class Tool(Node):
        def call(self, srv_type, name, req, timeout):
            cli = self.create_client(srv_type, name)
            try:
                if not cli.wait_for_service(timeout_sec=min(5.0, timeout)):
                    return None
                fut = cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
                return fut.result() if fut.done() else None
            finally:
                self.destroy_client(cli)

        def state(self, node):
            r = self.call(GetState, f'/{node}/get_state', GetState.Request(), 5.0)
            return r.current_state.label if r else '?'

        def transition(self, node, tid, timeout=60.0):
            req = ChangeState.Request()
            req.transition.id = tid
            r = self.call(ChangeState, f'/{node}/change_state', req, timeout)
            return bool(r and r.success)

        def managed(self, manager):
            req = GetParameters.Request(names=['node_names'])
            r = self.call(GetParameters, f'/{manager}/get_parameters', req, 5.0)
            return list(r.values[0].string_array_value) if r and r.values else None

        def manage(self, manager, command, timeout):
            req = ManageLifecycleNodes.Request()
            req.command = command
            r = self.call(ManageLifecycleNodes, f'/{manager}/manage_nodes', req, timeout)
            return None if r is None else bool(r.success)

        def get(self, node, names):
            r = self.call(GetParameters, f'/{node}/get_parameters',
                          GetParameters.Request(names=list(names)), 5.0)
            if not r:
                return {n: None for n in names}
            return {n: from_value(v) for n, v in zip(names, r.values)}

        def put(self, node, values):
            req = SetParameters.Request(parameters=[to_param(n, v) for n, v in values.items()])
            r = self.call(SetParameters, f'/{node}/set_parameters', req, 10.0)
            if not r:
                return {n: '응답 없음' for n in values}
            return {n: ('' if res.successful else (res.reason or '거부'))
                    for n, res in zip(values, r.results)}

        def moving(self, wait=2.0):
            got = []
            sub = self.create_subscription(Odometry, '/odom', got.append, 10)
            end = time.monotonic() + wait
            while time.monotonic() < end and not got:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.destroy_subscription(sub)
            if not got:
                return None
            t = got[-1].twist.twist
            return abs(t.linear.x) > 0.03 or abs(t.angular.z) > 0.05

    rclpy.init()
    t = Tool('vica_nav2_speed')
    try:
        node_lists = {}
        for m in MANAGERS:
            names = t.managed(m)
            if not names:
                print(f'[멈춤] /{m} 를 찾지 못했다 — Nav2 가 떠 있는지, ROS_DOMAIN_ID 가 같은지 확인')
                return 2
            node_lists[m] = names

        def report_states():
            ok = True
            for m, names in node_lists.items():
                states = {n: t.state(n) for n in names}
                ok &= all(s == 'active' for s in states.values())
                print(f'  {m}: ' + ', '.join(f'{n}={s}' for n, s in states.items()))
            return ok

        def report_values():
            ok = True
            for node, values in targets.items():
                now = t.get(node, values)
                for name, want in values.items():
                    hit = same(now[name], want)
                    ok &= hit
                    print(f'  {"OK " if hit else "다름"} {node}.{name} = {fmt(now[name])}'
                          + ('' if hit else f'  (목표 {fmt(want)})'))
            return ok

        if args.show:
            print('[지금 상태]')
            report_states()
            print(f'[지금 값 — 목표 {args.speed:g} m/s 와 비교]')
            report_values()
            return 0

        moving = t.moving()
        if moving is None and not args.force:
            print('[멈춤] /odom 이 2초 동안 안 들어온다 — EKF 를 먼저 확인(무시하려면 --force)')
            return 2
        if moving and not args.force:
            print('[멈춤] 로봇이 움직이고 있다 — 멈춘 뒤 다시 실행(재설정은 진행 중인 목표를 끊는다)')
            return 2

        start = time.monotonic()
        print('[1/3] 값 넣기')
        for node, values in targets.items():
            for name, err in t.put(node, values).items():
                print(f'  {"OK " if not err else "실패"} {node}.{name} = {fmt(values[name])}'
                      + (f'  ({err})' if err else ''))

        print('[2/3] Nav2 재설정(RESET -> STARTUP) — AMCL·지도·EKF·모터는 그대로')
        for m, names in node_lists.items():
            r = t.manage(m, ManageLifecycleNodes.Request.RESET, 90.0)
            print(f'  {m} RESET: {"끝" if r else "관리자 응답 없음/실패 — 남은 노드를 직접 내린다"}')
            for n in reversed(names):
                s = t.state(n)
                if s == 'active':
                    t.transition(n, Transition.TRANSITION_DEACTIVATE)
                    s = t.state(n)
                if s == 'inactive':
                    t.transition(n, Transition.TRANSITION_CLEANUP)
        for m, names in node_lists.items():
            r = t.manage(m, ManageLifecycleNodes.Request.STARTUP, 120.0)
            print(f'  {m} STARTUP: {"끝" if r else "관리자 응답 없음/실패 — 남은 노드를 직접 올린다"}')
            for n in names:
                s = t.state(n)
                if s == 'unconfigured':
                    t.transition(n, Transition.TRANSITION_CONFIGURE)
                    s = t.state(n)
                if s == 'inactive':
                    t.transition(n, Transition.TRANSITION_ACTIVATE)

        print('[3/3] 확인')
        states_ok = report_states()
        values_ok = report_values()
        took = time.monotonic() - start
        if states_ok and values_ok:
            print(f'[끝] {args.speed:g} m/s 적용 ({took:.1f} s). '
                  '모터 가변저항(knob)이 끝까지 올라가 있어야 이 속도가 나온다.')
            return 0
        print(f'[주의] 다 적용되지 않았다({took:.1f} s) — 위 "다름"/active 아닌 노드를 확인. '
              '안 되면 Nav2 칸을 통째로 다시 켠다(초기 위치 다시 찍기).')
        return 1
    finally:
        t.destroy_node()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description='Nav2 최고 속도를 노드를 끄지 않고 바꾼다.')
    ap.add_argument('speed', nargs='?', type=float, help='목표 최고 속도 m/s (예: 0.9, 원래 값 0.5)')
    ap.add_argument('--dry-run', action='store_true', help='넣을 값만 보여 준다(ROS 안 씀)')
    ap.add_argument('--show', action='store_true', help='지금 노드의 값·상태만 읽는다')
    ap.add_argument('--force', action='store_true', help='/odom 정지 확인을 건너뛴다')
    ap.add_argument('--base', type=Path, help='기준 파라미터 파일(기본: 설치된 vica_nav2 nav2_params.yaml)')
    args = ap.parse_args()

    base_path = args.base or base_params_path()
    base = yaml.safe_load(base_path.read_text(encoding='utf-8'))
    v0 = float(base['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'])
    if args.speed is None:
        if not args.show:
            ap.error('목표 속도를 주거나 --show 를 쓴다')
        args.speed = v0
    if not SPEED_MIN <= args.speed <= SPEED_MAX:
        ap.error(f'속도는 {SPEED_MIN}~{SPEED_MAX} m/s (위는 Safety·모터 상한 1.0)')

    targets, k, shift, stop = plan(base, args.speed)
    print(f'기준 {base_path} (최고 {v0:g} m/s) -> 목표 {args.speed:g} m/s, 비율 {k:.2f}')
    print(f'정지거리(VCC 식) {stop:.3f} m, 충돌 감시 상자 앞면 {shift:+.3f} m')
    if args.dry_run or not args.show:
        for node, values in targets.items():
            for name, value in values.items():
                print(f'  {node}.{name} = {fmt(value)}')
    if args.dry_run:
        return 0
    return ros_main(args, targets)


if __name__ == '__main__':
    sys.exit(main())
