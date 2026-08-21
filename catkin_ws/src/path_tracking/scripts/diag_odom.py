#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diag_odom : localization 의 /odom 과 MORAI 의 /ego_status 를 나란히 기록한다.

왜 필요한가
  우리 global path 는 /ego_status 의 position(MORAI 맵 로컬좌표)으로 기록했다.
  planning 이 /odom 으로 갈아타려면 그 둘이 같은 프레임이어야 한다. 어긋나면
  MAX_CTE(6.0m) 를 넘겨 path_tracker 가 정지하거나, 몇 미터 어긋난 채 주행한다.

  2026-08-03 에 /gps vs /ego_status 로 같은 측정을 했고 최대 11.34m 가 나왔다
  (docs/23-localization_node_review.md 7절). 원인은 팀 코드의 구면근사였고,
  받은 수정본은 자체 투영을 걷어내고 robot_localization 의 UTM 투영을 쓴다.
  그 수정이 실제로 맞았는지를 이 로거로 확인한다.

diag_frame.py 와 같은 원칙
  아무 판단도 하지 않는다. 두 토픽을 원시 그대로 각각 CSV 에 쌓기만 한다.
  한 줄로 짝짓지 않는다 - 주기가 달라(odom 40Hz / ego 20~26Hz) 짝을 지으면
  시각차만큼 오차가 섞인다. 15m/s 에서 25ms 면 0.38m 로, 재려는 값과 같은
  크기다. 분석 때 빠른 쪽을 느린 쪽 시각으로 보간한다.

사용법
  rosrun path_tracking diag_odom.py
  rosrun path_tracking diag_odom.py _tag:=lap2

  _out_dir : CSV 저장 폴더 (기본 /home/dev/catkin_ws/logs)
  _tag     : 파일명 꼬리표. odom_<tag>.csv / odomego_<tag>.csv

Ctrl+C 로 끝내면 몇 줄씩 쌓였는지 요약한다.
"""
import csv
import math
import os

import rospy
from nav_msgs.msg import Odometry
from morai_msgs.msg import EgoVehicleStatus


def yaw_deg(q):
    """쿼터니언 -> yaw [deg]. /odom 은 heading 을 각도로 주지 않는다."""
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(s, c))


class DiagOdom:
    def __init__(self):
        out_dir = rospy.get_param('~out_dir', '/home/dev/catkin_ws/logs')
        tag     = rospy.get_param('~tag', 'frame')
        self.f_odom = open(os.path.join(out_dir, 'odom_%s.csv' % tag), 'w')
        self.f_ego  = open(os.path.join(out_dir, 'odomego_%s.csv' % tag), 'w')
        self.w_odom = csv.writer(self.f_odom)
        self.w_ego  = csv.writer(self.f_ego)
        self.w_odom.writerow(['t', 'x', 'y', 'z', 'yaw_deg', 'vx', 'vy'])
        self.w_ego.writerow(['t', 'x', 'y', 'z', 'heading_deg', 'vel_x', 'vel_y'])
        self.n_odom = self.n_ego = 0
        self.frame_id = None

        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=200)
        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.ego_cb, queue_size=200)
        rospy.on_shutdown(self.finish)
        rospy.loginfo('[diag_odom] recording to %s (tag=%s)', out_dir, tag)

    @staticmethod
    def _stamp(msg):
        t = msg.header.stamp.to_sec()
        return t if t > 0.0 else rospy.Time.now().to_sec()

    def odom_cb(self, m):
        if self.frame_id is None:
            self.frame_id = m.header.frame_id
        p = m.pose.pose.position
        v = m.twist.twist.linear
        self.w_odom.writerow(['%.6f' % self._stamp(m),
                              '%.4f' % p.x, '%.4f' % p.y, '%.4f' % p.z,
                              '%.3f' % yaw_deg(m.pose.pose.orientation),
                              '%.4f' % v.x, '%.4f' % v.y])
        self.n_odom += 1

    def ego_cb(self, m):
        self.w_ego.writerow(['%.6f' % self._stamp(m),
                             '%.4f' % m.position.x, '%.4f' % m.position.y,
                             '%.4f' % m.position.z,
                             '%.3f' % m.heading,
                             '%.4f' % m.velocity.x, '%.4f' % m.velocity.y])
        self.n_ego += 1

    def finish(self):
        for f in (self.f_odom, self.f_ego):
            if not f.closed:
                f.close()
        print('')
        print('[diag_odom] /odom %d줄 (frame_id=%s) / /ego_status %d줄'
              % (self.n_odom, self.frame_id, self.n_ego))
        if self.n_odom == 0:
            print('  ⚠️ /odom 이 하나도 안 왔다. localization.launch 가 떠 있나?')


if __name__ == '__main__':
    rospy.init_node('diag_odom', anonymous=True)
    DiagOdom()
    rospy.spin()
