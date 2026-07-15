#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_tracker : 기록한 path.csv 를 따라가는지 확인하는 (검증용) 노드
------------------------------------------------------------------
흐름:  path.csv 로드
       -> PathManager 로 현재 위치 앞 구간(local path) 추출
       -> PurePursuit 로 조향각 계산
       -> 아주 단순한 목표속도 제어로 accel/brake
       -> /ctrl_cmd 발행
RViz 확인용:  /global_path (전체 경로),  /local_path (지금 따라가는 구간)

※ 조향 정밀 튜닝(lfd, 게인 등)은 control팀 몫. 여기선 "경로 잘 따라가나" 확인용.
"""
import os
import csv
from math import hypot, radians

import rospy
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from lib.point import Point
from lib.vehicle_state import VehicleState
from lib.path_manager import PathManager
from lib.pure_pursuit import PurePursuit

# ---- 파라미터 (control팀이 나중에 조정할 값들) ----
TARGET_SPEED_KMH = 20.0     # 검증용 (더 올리려면 이 값만 키우기)
WHEELBASE        = 3.0      # Ioniq5 축거 [m]
LFD_GAIN         = 0.7      # 전방주시거리 = LFD_GAIN * 속도 (속도 빠를수록 멀리 봄)
MIN_LFD          = 4.0
MAX_LFD          = 20.0
LOCAL_PATH_SIZE  = 50       # 앞으로 볼 waypoint 개수 (0.5m 간격 → 약 25m)
IS_CLOSED_PATH   = False    # 경로가 안 닫혀있음(시작≠끝). 폐곡선 코스로 다시 따면 True
MAX_STEER        = 0.5      # 안전 클램프 [rad] (약 28도) - 큰 조향각 튐 방지


class PathTracker:
    def __init__(self):
        # 1) path.csv 로드
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        # 스무딩된 경로(path_smooth.csv)가 있으면 그걸 우선 사용
        smooth_path = os.path.join(pkg_dir, 'path', 'path_smooth.csv')
        raw_path = os.path.join(pkg_dir, 'path', 'path.csv')
        csv_path = smooth_path if os.path.exists(smooth_path) else raw_path
        rospy.loginfo('[path_tracker] 경로 파일: %s', os.path.basename(csv_path))
        self.path = self._load_path(csv_path)
        if len(self.path) < LOCAL_PATH_SIZE:
            rospy.logwarn('waypoint가 %d개뿐. LOCAL_PATH_SIZE보다 적음', len(self.path))
        rospy.loginfo('[path_tracker] waypoint %d개 로드', len(self.path))

        # 2) 플래너 준비
        self.path_manager = PathManager(self.path, IS_CLOSED_PATH, LOCAL_PATH_SIZE)
        # 검증용: 일정 목표속도(곡률 기반 속도프로파일은 control팀이 나중에)
        self.target_mps = TARGET_SPEED_KMH / 3.6
        self.path_manager.velocity_profile = [self.target_mps] * len(self.path)

        self.pure_pursuit = PurePursuit(LFD_GAIN, WHEELBASE, MIN_LFD, MAX_LFD)

        # 3) ROS 입출력
        self.lattice_points = None       # lattice가 준 회피경로 (list of Point)
        self.lattice_stamp = None
        self.cmd_pub   = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.gpath_pub = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        self.lpath_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        rospy.Subscriber('/lattice_path', Path, self.lattice_callback)
        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.callback)

        self.gpath_pub.publish(self._to_path_msg(self.path))   # 전체 경로 1회 발행
        rospy.loginfo('[path_tracker] 시작 - 목표속도 %.0f km/h', TARGET_SPEED_KMH)

    def _load_path(self, csv_path):
        points = []
        with open(csv_path) as f:
            reader = csv.reader(f)
            next(reader)                       # 헤더(x,y,z) 건너뛰기
            for row in reader:
                points.append(Point(float(row[0]), float(row[1])))
        return points

    def callback(self, msg):
        # 현재 차량 상태
        speed = hypot(msg.velocity.x, msg.velocity.y)          # m/s
        vs = VehicleState(msg.position.x, msg.position.y,
                          radians(msg.heading), speed)

        # 앞 구간 + 목표속도
        local_path, target_velocity = self.path_manager.get_local_path(vs)
        self.lpath_pub.publish(self._to_path_msg(local_path))  # lattice가 이걸 받아 회피경로 생성

        # lattice 회피경로가 최근(0.3초 내)에 왔으면 그걸 따라감, 아니면 기준 local_path
        follow_path = local_path
        if (self.lattice_points and self.lattice_stamp and
                (rospy.Time.now() - self.lattice_stamp).to_sec() < 0.3 and
                len(self.lattice_points) > 1):
            follow_path = self.lattice_points

        # 조향 (pure pursuit)
        self.pure_pursuit.path = follow_path
        self.pure_pursuit.vehicle_state = vs
        steering = self.pure_pursuit.calculate_steering_angle()   # rad
        steering = max(-MAX_STEER, min(MAX_STEER, steering))      # 안전 클램프

        # 종방향: 아주 단순한 목표속도 유지
        if speed < target_velocity:
            accel, brake = 0.3, 0.0
        else:
            accel, brake = 0.0, 0.1

        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.accel = accel
        cmd.brake = brake
        cmd.front_steer = steering        # 브릿지가 0.70 보정 적용
        self.cmd_pub.publish(cmd)

    def lattice_callback(self, msg):
        # lattice가 준 회피경로를 Point 리스트로 저장
        self.lattice_points = [Point(p.pose.position.x, p.pose.position.y)
                               for p in msg.poses]
        self.lattice_stamp = rospy.Time.now()

    def _to_path_msg(self, points):
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = rospy.Time.now()
        for pt in points:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(pt.x)
            ps.pose.position.y = float(pt.y)
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        return p


if __name__ == '__main__':
    rospy.init_node('path_tracker')
    PathTracker()
    rospy.spin()
