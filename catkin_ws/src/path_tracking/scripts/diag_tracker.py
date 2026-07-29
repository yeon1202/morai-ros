#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diag_tracker : path_tracker 와 똑같이 계산하되 /ctrl_cmd 는 발행하지 않는 진단 노드.
어디서 어긋나는지(최근접 인덱스, 경로까지 거리, 조향 목표점)를 찍어본다.
"""
import os, csv
from math import hypot, radians, degrees, atan2, sin, cos

import rospy
from morai_msgs.msg import EgoVehicleStatus

from lib.point import Point
from lib.vehicle_state import VehicleState
from lib.path_manager import PathManager
from lib.pure_pursuit import PurePursuit

# path_tracker.py 와 같은 값을 써야 진단이 의미가 있다 (LFD_GAIN 0.7 -> 0.5 반영)
WHEELBASE, LFD_GAIN, MIN_LFD, MAX_LFD = 3.0, 0.5, 4.0, 20.0
LOCAL_PATH_SIZE, IS_CLOSED_PATH = 140, False


class Diag:
    def __init__(self):
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        csv_path = os.path.join(pkg, 'path', 'path_smooth.csv')
        self.path = []
        with open(csv_path) as f:
            r = csv.reader(f); next(r)
            for row in r:
                self.path.append(Point(float(row[0]), float(row[1])))
        self.pm = PathManager(self.path, IS_CLOSED_PATH, LOCAL_PATH_SIZE)
        self.pm.velocity_profile = [20/3.6] * len(self.path)
        self.pp = PurePursuit(LFD_GAIN, WHEELBASE, MIN_LFD, MAX_LFD)
        self.n = 0
        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.cb)

    def cb(self, msg):
        self.n += 1
        if self.n % 20:                      # 초당 몇 번만 출력
            return
        speed = hypot(msg.velocity.x, msg.velocity.y) / 3.6   # /ego_status 는 km/h
        vs = VehicleState(msg.position.x, msg.position.y, radians(msg.heading), speed)

        # 최근접 waypoint
        best_i, best_d = 0, float('inf')
        for i, p in enumerate(self.path):
            d = (p.x - vs.position.x)**2 + (p.y - vs.position.y)**2
            if d < best_d:
                best_d, best_i = d, i
        cte = best_d ** 0.5

        local_path, _ = self.pm.get_local_path(vs)
        self.pp.path = local_path
        self.pp.vehicle_state = vs
        steer = self.pp.calculate_steering_angle()

        # pure_pursuit 이 실제로 고른 목표점 찾기 (같은 로직 재현)
        lfd = min(max(LFD_GAIN * speed, MIN_LFD), MAX_LFD)
        target, fwd_cnt = None, 0
        for p in local_path:
            diff = p - vs.position
            rot = diff.rotate(-vs.yaw)
            if rot.x > 0:
                fwd_cnt += 1
                if rot.distance() >= lfd:
                    target = (p, rot)
                    break

        # 경로 진행방향 vs 차량 heading 차이
        j = min(best_i + 5, len(self.path) - 1)
        path_yaw = atan2(self.path[j].y - self.path[best_i].y,
                         self.path[j].x - self.path[best_i].x)
        dyaw = degrees(atan2(sin(path_yaw - vs.yaw), cos(path_yaw - vs.yaw)))

        # 경로 시작점까지 거리 + 차 기준 방향 (수동주행으로 찾아갈 때 나침반 역할)
        s = self.path[0]
        s_dist = hypot(s.x - vs.position.x, s.y - vs.position.y)
        s_rel = degrees(atan2(sin(atan2(s.y - vs.position.y, s.x - vs.position.x) - vs.yaw),
                              cos(atan2(s.y - vs.position.y, s.x - vs.position.x) - vs.yaw)))
        way = "왼쪽" if s_rel > 0 else "오른쪽"

        rospy.loginfo(
            "pos(%.1f,%.1f) hd=%.1fdeg v=%.1f | 최근접 idx=%d 거리(CTE)=%.2fm | "
            "local[%d개] | 전방점=%d개 lfd=%.1f 목표=%s | steer=%.3frad | 경로방향차=%+.1fdeg "
            "|| 시작점까지 %.0fm, %.0f도 %s",
            vs.position.x, vs.position.y, degrees(vs.yaw), speed,
            best_i, cte, len(local_path), fwd_cnt, lfd,
            ("(%.1f,%.1f)" % (target[0].x, target[0].y)) if target else "없음!",
            steer, dyaw, s_dist, abs(s_rel), way)


if __name__ == '__main__':
    rospy.init_node('diag_tracker')
    Diag()
    rospy.spin()
