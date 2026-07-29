#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lap_logger : 한 바퀴 주행을 CSV 로 기록하고 위험 구간을 요약하는 진단 노드 (읽기 전용)

diag_tracker.py 는 1초에 한 줄 콘솔 출력이라 눈으로 보기엔 좋지만 한 바퀴를
분석하기엔 부족하다. 이 노드는 10Hz 로 CSV 에 쌓고, 종료할 때 어느 구간에서
얼마나 벗어났는지 요약해준다.

CTE 는 반드시 경로 '선분'에 수직 투영해서 잰다. 최근접 waypoint 까지의 거리로
재면 waypoint 간격(최대 1.007m) 때문에 차가 경로 위에 정확히 있어도 최대 0.5m
로 나온다. PathManager._perpendicular_distance() 를 그대로 쓴다.

주의: CTE 는 "기록한 경로로부터의 이탈"이지 "차로 중앙으로부터의 이탈"이 아니다.
사람이 직접 몰아 기록한 경로라 경로 자체가 차로 중앙에서 치우쳐 있을 수 있다.
CTE 가 작은데도 실선을 밟는다면 경로 쪽을 의심해야 한다.

사용법
  rosrun path_tracking lap_logger.py
  rosrun path_tracking lap_logger.py _out:=/tmp/lap1.csv _start_idx:=889

  _out       : CSV 저장 경로 (기본 /tmp/lap.csv)
  _start_idx : 요약에 포함할 시작 인덱스 (기본 889 = 대회코스 시작.
               접근구간은 대회 코스가 아니라 차량을 라인에 올리는 구간이므로
               기본적으로 요약에서 제외한다)
  _limit     : 위험 판정 기준 [m] (기본 0.654 = 차로 3.2m - 차폭 1.892m 의 편도 여유)
"""
import os
import csv

import rospy
from morai_msgs.msg import EgoVehicleStatus

from lib.point import Point
from lib.vehicle_state import VehicleState
from lib.path_manager import PathManager

LOCAL_PATH_SIZE, IS_CLOSED_PATH = 140, False
LOG_HZ = 10.0


class LapLogger:
    def __init__(self):
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        csv_path = os.path.join(pkg, 'path', 'path_smooth.csv')

        self.path = []
        with open(csv_path) as f:
            r = csv.reader(f)
            next(r)
            for row in r:
                self.path.append(Point(float(row[0]), float(row[1])))

        self.pm = PathManager(self.path, IS_CLOSED_PATH, LOCAL_PATH_SIZE)
        self.pm.velocity_profile = [0.0] * len(self.path)

        self.out_path  = rospy.get_param('~out', '/tmp/lap.csv')
        self.start_idx = int(rospy.get_param('~start_idx', 889))
        self.limit     = float(rospy.get_param('~limit', 0.654))

        self.rows = []          # (t, idx, cte, speed_kmh)
        self.last_log = 0.0
        self._reported = False  # 요약을 두 번 찍지 않도록

        self.f = open(self.out_path, 'w')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'idx', 'cte', 'speed_kmh', 'x', 'y'])

        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.callback)
        rospy.on_shutdown(self.report)
        rospy.loginfo('[lap_logger] %s 에 기록 (요약 시작 idx=%d, 한계 %.3fm)',
                      self.out_path, self.start_idx, self.limit)

    def callback(self, msg):
        now = rospy.Time.now().to_sec()
        if now - self.last_log < 1.0 / LOG_HZ:
            return
        self.last_log = now

        # /ego_status 의 velocity 는 km/h 다 (브릿지가 UDP 원본을 변환 없이 넘긴다)
        speed_kmh = (msg.velocity.x ** 2 + msg.velocity.y ** 2) ** 0.5
        vs = VehicleState(msg.position.x, msg.position.y, 0.0, speed_kmh / 3.6)

        self.pm.get_local_path(vs)          # current_waypoint 와 cte 를 갱신한다
        idx = self.pm.current_waypoint
        cte = self.pm.cte

        # 완주 후에는 기록하지 않는다. 인덱스가 경로 끝에 고정된 채 차만 나아가면
        # CTE 가 실제 이탈이 아닌데도 계속 커진다(실측 0.96 -> 9.75m). 이걸 그대로
        # 두면 요약에서 최악 구간으로 잡혀 판단을 그르친다.
        if self.pm.finished:
            if not self._reported:
                rospy.loginfo('[lap_logger] 완주 감지 - 기록을 멈추고 요약한다')
                self.report()
            return

        self.rows.append((now, idx, cte, speed_kmh))
        self.w.writerow(['%.3f' % now, idx, '%.3f' % cte, '%.2f' % speed_kmh,
                         '%.3f' % msg.position.x, '%.3f' % msg.position.y])

    def report(self):
        if self._reported:
            return
        self._reported = True
        if not self.f.closed:
            self.f.close()
        rows = [r for r in self.rows if r[1] >= self.start_idx]
        if not rows:
            rospy.logwarn('[lap_logger] idx %d 이후 샘플이 없다', self.start_idx)
            return

        ctes = [r[2] for r in rows]
        over = [r for r in rows if r[2] > self.limit]
        idx_lo, idx_hi = min(r[1] for r in rows), max(r[1] for r in rows)

        print('')
        print('=== lap_logger 요약 (idx %d ~ %d, 샘플 %d개) ===' % (idx_lo, idx_hi, len(rows)))
        print('  CTE 평균 %.3fm | 중앙값 %.3fm | 최대 %.3fm'
              % (sum(ctes) / len(ctes), sorted(ctes)[len(ctes) // 2], max(ctes)))
        print('  한계 %.3fm 초과: %d개 (%.1f%%)'
              % (self.limit, len(over), 100.0 * len(over) / len(rows)))

        if not over:
            print('  한계를 넘은 지점 없음')
            return

        # 연속된 초과 구간을 하나로 묶어 보고한다 (인덱스가 10 이상 벌어지면 다른 구간)
        print('')
        print('  초과 구간   idx범위        최대CTE   그때속도   위치')
        segs = []
        cur = None
        for _, idx, cte, spd in over:
            if cur and idx - cur[1] <= 10:
                cur[1] = idx
                if cte > cur[2]:
                    cur[2], cur[3] = cte, spd
            else:
                if cur:
                    segs.append(cur)
                cur = [idx, idx, cte, spd]
        if cur:
            segs.append(cur)

        segs.sort(key=lambda s: -s[2])
        for k, (a, b, cte, spd) in enumerate(segs[:20], 1):
            px, py = self.path[a].x, self.path[a].y
            print('  %4d      %4d~%-4d   %6.3fm  %6.1fkm/h  (%.0f, %.0f)'
                  % (k, a, b, cte, spd, px, py))
        print('')
        print('  총 %d개 구간. CSV: %s' % (len(segs), self.out_path))


def main():
    rospy.init_node('lap_logger', anonymous=True)
    LapLogger()
    rospy.spin()


if __name__ == '__main__':
    main()
