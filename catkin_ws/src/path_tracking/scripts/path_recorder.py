#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint 기록기
------------------------------------------------------
/ego_status (차량 현재 상태)를 받아서, 0.5m 이동할 때마다
현재 위치(x, y, z)를 path.csv 에 한 줄씩 저장한다.

사용법: MORAI 를 주행(자동주행 or teleop)시키면서 이 노드를 켜두면
        지나온 경로가 waypoint 로 CSV 에 쌓인다. Ctrl+C 로 종료 시 저장 마무리.
"""
import os
import csv
from math import sqrt

import rospy
from morai_msgs.msg import EgoVehicleStatus


class PathRecorder:
    def __init__(self):
        # 저장 위치: path_tracking/path/path.csv
        #   __file__ = .../path_tracking/scripts/path_recorder.py
        #   dirname 두 번 올라가면 .../path_tracking
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.file_path = os.path.join(pkg_dir, 'path', 'path.csv')

        self.f = open(self.file_path, 'w')
        self.writer = csv.writer(self.f)
        self.writer.writerow(['x', 'y', 'z'])   # 헤더 한 줄

        self.prev_x = None      # 마지막으로 기록한 점
        self.prev_y = None
        self.count = 0

        # /ego_status 가 올 때마다 self.callback 실행
        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.callback)
        rospy.on_shutdown(self.cleanup)     # Ctrl+C 시 파일 잘 닫기
        rospy.loginfo('[path_recorder] 기록 시작 -> %s', self.file_path)

    def callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        # 첫 점이거나, 직전 기록점에서 0.5m 넘게 움직였을 때만 저장
        if self.prev_x is None or sqrt((x - self.prev_x) ** 2 + (y - self.prev_y) ** 2) > 0.5:
            self.writer.writerow([x, y, z])
            self.prev_x, self.prev_y = x, y
            self.count += 1
            rospy.loginfo('waypoint %d : (%.2f, %.2f)', self.count, x, y)

    def cleanup(self):
        self.f.close()
        rospy.loginfo('[path_recorder] 종료 - 총 %d개 저장: %s', self.count, self.file_path)


if __name__ == '__main__':
    rospy.init_node('path_recorder')
    PathRecorder()
    rospy.spin()
