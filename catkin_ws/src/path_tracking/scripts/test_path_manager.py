#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_path_manager : 최근접 waypoint 탐색이 자기겹침 구간에서 안 튀는지 검증.
------------------------------------------------------------------
실제 path_smooth.csv 위를 차가 순서대로 지나간다고 가정하고,
매 지점에서 PathManager 가 고른 인덱스가 맞는지 본다.

옛 방식(전역 최근접)과 새 방식(직전 인덱스 주변 탐색)을 나란히 비교한다.
실행:  python3 test_path_manager.py   (ROS 불필요)
"""
import os
import csv
import sys
from math import hypot, atan2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lib.point import Point
from lib.vehicle_state import VehicleState
from lib.path_manager import PathManager

LOCAL_PATH_SIZE = 50
LATERAL_OFFSET = 1.0        # 차가 경로 중앙에서 이만큼 벗어난 채 주행한다고 가정 [m]


def load_path():
    path_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'path')
    pts = []
    with open(os.path.join(path_dir, 'path_smooth.csv')) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            pts.append(Point(float(row[0]), float(row[1])))
    return pts


def global_nearest(path, position):
    """옛 방식: 매번 전체를 훑는다."""
    best_i, best_d2 = 0, float('inf')
    for i, p in enumerate(path):
        d2 = (p.x - position.x) ** 2 + (p.y - position.y) ** 2
        if d2 < best_d2:
            best_d2, best_i = d2, i
    return best_i


def simulated_positions(path):
    """경로를 따라가되 법선방향으로 LATERAL_OFFSET 만큼 벗어난 위치들"""
    out = []
    for i in range(len(path) - 1):
        dx = path[i + 1].x - path[i].x
        dy = path[i + 1].y - path[i].y
        L = hypot(dx, dy) or 1.0
        nx, ny = -dy / L, dx / L            # 왼쪽 법선
        yaw = atan2(dy, dx)
        out.append((i,
                    path[i].x + nx * LATERAL_OFFSET,
                    path[i].y + ny * LATERAL_OFFSET,
                    yaw))
    return out


def main():
    path = load_path()
    print('경로 %d점 로드, 차량은 경로에서 %.1fm 벗어난 채 주행한다고 가정\n'
          % (len(path), LATERAL_OFFSET))

    samples = simulated_positions(path)
    pm = PathManager(path, False, LOCAL_PATH_SIZE)
    pm.velocity_profile = [5.5] * len(path)

    new_bad, old_bad, short_local = [], [], []
    prev_new = None

    for true_i, x, y, yaw in samples:
        vs = VehicleState(x, y, yaw, 5.5)

        local_path, _ = pm.get_local_path(vs)
        new_i = pm.current_waypoint
        old_i = global_nearest(path, vs.position)

        # 1) 새 방식이 실제 위치에서 크게 벗어난 인덱스를 골랐나
        if abs(new_i - true_i) > 5:
            new_bad.append((true_i, new_i))
        # 2) 옛 방식은?
        if abs(old_i - true_i) > 5:
            old_bad.append((true_i, old_i))
        # 3) local path 가 말라붙었나 (경로 끝 부근은 정상이라 제외)
        if len(local_path) < LOCAL_PATH_SIZE and true_i < len(path) - LOCAL_PATH_SIZE:
            short_local.append((true_i, len(local_path)))
        # 4) 인덱스가 뒤로 크게 밀렸나
        if prev_new is not None and new_i < prev_new - PathManager.SEARCH_BACK:
            new_bad.append((true_i, new_i))
        prev_new = new_i

    n = len(samples)
    print('--- 옛 방식 (전역 최근접) ---')
    print('  엉뚱한 인덱스를 고른 지점: %d / %d (%.1f%%)'
          % (len(old_bad), n, 100.0 * len(old_bad) / n))
    if old_bad:
        print('  예시 (실제위치 -> 고른 인덱스):')
        for t, g in old_bad[:5]:
            print('    idx %4d 에 있는데  ->  %4d 을 고름  (%d칸 오차)'
                  % (t, g, abs(g - t)))

    print('\n--- 새 방식 (직전 인덱스 주변 탐색) ---')
    print('  엉뚱한 인덱스를 고른 지점: %d / %d' % (len(new_bad), n))
    print('  local path 가 말라붙은 지점: %d' % len(short_local))
    print('  전역 재탐색 발생 횟수: %d' % pm.relocated)
    print('  마지막 CTE: %.3fm (기대값 %.1fm)' % (pm.cte, LATERAL_OFFSET))

    ok = (not new_bad) and (not short_local)
    print('\n%s' % ('PASS - 겹침 구간에서도 안 튐' if ok else 'FAIL'))

    ok2 = test_finish_latch()
    return 0 if (ok and ok2) else 1


def test_finish_latch():
    """완주 후 두 바퀴째로 넘어가지 않는지 검증.

    우리 경로는 완주 지점(끝)과 코스 진입 지점이 물리적으로 같은 자리다.
    완주 감지가 없으면 경로 끝을 지나 10m(RESET_DISTANCE) 넘게 벗어나는 순간
    전역 재탐색이 바로 옆의 진입부 인덱스를 잡아 다시 달리기 시작한다.
    실제 주행에서 관측됐다 - 완주 0.7초 뒤 idx 4633 에서 909 로 뛰었다.

    경로 끝을 지나 진입부 쪽으로 계속 나아가는 상황을 재현한다.
    """
    path = load_path()
    n = len(path)
    pm = PathManager(path, False, LOCAL_PATH_SIZE)
    pm.velocity_profile = [0.0] * n     # get_local_path 가 참조하므로 채워둔다

    print('\n--- 완주 latch 검증 ---')

    # 경로 끝 20점을 정상 주행해 완주 상태로 만든다
    for i in range(n - 20, n):
        pm.get_local_path(VehicleState(path[i].x, path[i].y, 0, 0))

    if not pm.finished:
        print('  FAIL - 경로 끝에 닿았는데 finished 가 서지 않았다')
        return False
    print('  경로 끝 도달 -> finished=True, idx=%d' % pm.current_waypoint)

    # 끝점에서 진입부(실측 점프 대상 idx 909) 방향으로 30m 계속 나아간다
    ex, ey = path[n - 1].x, path[n - 1].y
    tx, ty = path[909].x, path[909].y
    d = hypot(tx - ex, ty - ey)
    ux, uy = (tx - ex) / d, (ty - ey) / d

    worst = 0
    for step in range(1, 31):
        pm.get_local_path(VehicleState(ex + ux * step, ey + uy * step, 0, 0))
        worst = max(worst, abs(pm.current_waypoint - (n - 1)))

    print('  끝점에서 진입부 방향 30m 진행 (실측 점프 지점까지 %.0fm)' % d)
    print('  그동안 인덱스가 끝(%d)에서 최대 %d칸 벗어남' % (n - 1, worst))

    ok = (worst == 0) and pm.finished
    print('\n%s' % ('PASS - 완주 후 두 바퀴째로 안 넘어감'
                    if ok else 'FAIL - 인덱스가 진입부로 튐'))
    return ok


if __name__ == '__main__':
    sys.exit(main())
