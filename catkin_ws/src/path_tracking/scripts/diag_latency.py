#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diag_latency : localization 파이프라인의 단계별 지연을 재기 위한 수동 로거.

왜 필요한가
  2026-08-21 측정에서 /odom 은 프레임·헤딩·속도가 다 정상인데 위치만 진행방향
  으로 0.22초(-2.58m) 늦는 게 확인됐다. 원인 후보가 네 단계에 걸쳐 있어서
  (UDP 브릿지 / localization_node / navsat_transform / EKF), 어느 단계가
  얼마나 먹는지 갈라내지 않으면 고칠 데를 못 고른다.

  파이프라인:
    MORAI ─UDP─> udp_bridge ─┬─ /ego_status ──────────────────────> (정답 기준)
                             │       └─┬─> loc_node ─> /odom/wheel_speed ─┐
                             ├─ /imu ──┘                                   │
                             │    └──> loc_node ─> /imu/ekf ───────┬───────┤
                             └─ /gps ─> loc_node ─> /gps/fix ──────┴─> navsat │
                                                        ─> /odometry/gps ─────┴─> EKF ─> /odom

두 가지를 따로 잰다
  (A) 통과 시간(벽시계). /gps -> /gps/fix -> /odometry/gps 는 stamp 가 그대로
      승계된다(localization_node.cpp:183 `out.header = msg->header`). 그래서
      각 토픽의 "도착 시각"을 stamp 로 짝지으면 단계 사이 소요 시간이 나온다.

  (B) 정보의 나이. EKF 는 출력 stamp 를 "지금" 으로 다시 찍어서 (A) 로는 안
      잡힌다. 대신 위치를 GT 와 시간축으로 상관시켜, 잔차를 최소로 만드는
      이동량 τ 를 구한다. /odometry/gps 와 /odom 각각 구하면 그 차이가
      EKF 가 더한 나이다.

  ⚠️ 둘은 단위가 다르다. 시뮬이 실시간의 0.37~0.64 배로 돌기 때문에
     (벽시계 지연) x (배속) = (차량이 실제로 겪는 지연) 이다.
     배속은 분석기가 (GT 이동거리/벽시계) ÷ (GT 속도) 로 역산한다.

원칙 (diag_odom.py 와 동일)
  아무 판단도 하지 않는다. 짝짓지도 않는다 - 토픽마다 주기가 달라서 기록
  시점에 짝을 지으면 시각차가 재려는 값에 섞인다. 원시 CSV 로 쌓기만 하고
  해석은 analyze_latency.py 가 한다.
  발행은 하나도 안 한다(구독 전용). 그래서 주행 중 라이브 마스터에 붙여도
  안전하다 - 런북의 "진단 노드를 라이브에 붙이지 말 것" 은 가짜 토픽을
  발행하는 노드 이야기다.

사용법
  rosrun path_tracking diag_latency.py
  rosrun path_tracking diag_latency.py _tag:=lap1

  _out_dir : CSV 저장 폴더 (기본은 lib/logdir.py 가 환경 보고 고른다)
  _tag     : 파일명 꼬리표.  lat_<토픽별이름>_<tag>.csv

Ctrl+C 로 끝내면 토픽별 줄 수와 실측 Hz 를 요약한다. 여기서 0 줄인 토픽이
있으면 그 단계가 아예 안 돌고 있다는 뜻이라 바로 알 수 있다.
"""
import csv
import math
import os

from lib.logdir import default_log_dir

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from morai_msgs.msg import EgoVehicleStatus, GPSMessage


def yaw_deg(q):
    """쿼터니언 -> yaw [deg]."""
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(s, c))


class Sink:
    """토픽 하나를 CSV 하나에 그대로 쌓는다.

    t_arr   : 콜백 진입 시각(벽시계). 이게 (A) 통과 시간의 재료다.
    t_stamp : 메시지가 달고 온 header.stamp. 단계 사이 짝짓기 키.
              이 둘의 차이가 곧 "도착했을 때 이 데이터가 몇 초짜리였나" 다.
    """

    def __init__(self, out_dir, tag, name, cols):
        self.name = name
        self.path = os.path.join(out_dir, 'lat_%s_%s.csv' % (name, tag))
        self.f = open(self.path, 'w')
        self.w = csv.writer(self.f)
        self.w.writerow(['t_arr', 't_stamp'] + cols)
        self.n = 0
        self.t_first = None
        self.t_last = None

    def row(self, t_arr, stamp, vals):
        # rospy 는 on_shutdown 을 부른 뒤에도 큐에 남은 콜백을 몇 개 더 실행한다.
        # 그때 이미 닫힌 파일에 쓰면 ValueError 가 나서 Ctrl+C 직후에 빨간
        # 트레이스백이 뜬다(기록된 데이터에는 영향 없다). 조용히 버린다.
        if self.f.closed:
            return
        if self.t_first is None:
            self.t_first = t_arr
        self.t_last = t_arr
        self.w.writerow(['%.6f' % t_arr, '%.6f' % stamp] + vals)
        self.n += 1

    def hz(self):
        if self.n < 2 or self.t_last <= self.t_first:
            return 0.0
        return (self.n - 1) / (self.t_last - self.t_first)

    def close(self):
        if not self.f.closed:
            self.f.close()


class DiagLatency:
    def __init__(self):
        out_dir = rospy.get_param('~out_dir', default_log_dir())
        tag = rospy.get_param('~tag', 'lat')

        # 파이프라인 순서대로. 뒤 단계 CSV 일수록 앞 단계 CSV 와 stamp 로 이어진다.
        self.s_ego   = Sink(out_dir, tag, 'ego',    ['x', 'y', 'heading_deg', 'vel_kmh'])
        self.s_gps   = Sink(out_dir, tag, 'gps',    ['lat', 'lon'])
        self.s_fix   = Sink(out_dir, tag, 'fix',    ['lat', 'lon'])
        self.s_imu   = Sink(out_dir, tag, 'imu',    ['yaw_deg', 'wz'])
        self.s_imue  = Sink(out_dir, tag, 'imuekf', ['yaw_deg', 'wz'])
        self.s_wheel = Sink(out_dir, tag, 'wheel',  ['vx', 'vy'])
        self.s_navs  = Sink(out_dir, tag, 'navsat', ['x', 'y'])
        self.s_odom  = Sink(out_dir, tag, 'odom',   ['x', 'y', 'yaw_deg', 'vx', 'vy'])
        self.sinks = [self.s_ego, self.s_gps, self.s_fix, self.s_imu,
                      self.s_imue, self.s_wheel, self.s_navs, self.s_odom]

        # tcp_nodelay 는 필수다. 없으면 Nagle 알고리즘이 작은 메시지를 최대
        # 40ms 씩 묶어 보내서, 우리가 재려는 파이프라인 지연 대신 TCP 버퍼링
        # 지연을 재게 된다. queue_size 도 크게 - 큐가 넘치면 조용히 버려져서
        # "그 단계가 느리다" 가 "그 단계가 안 온다" 로 위장된다.
        def sub(topic, typ, cb):
            rospy.Subscriber(topic, typ, cb, queue_size=400, tcp_nodelay=True)

        sub('/ego_status',       EgoVehicleStatus, self.cb_ego)
        sub('/gps',              GPSMessage,       self.cb_gps)
        sub('/gps/fix',          NavSatFix,        self.cb_fix)
        sub('/imu',              Imu,              self.cb_imu)
        sub('/imu/ekf',          Imu,              self.cb_imue)
        sub('/odom/wheel_speed', Odometry,         self.cb_wheel)
        sub('/odometry/gps',     Odometry,         self.cb_navsat)
        sub('/odom',             Odometry,         self.cb_odom)

        rospy.on_shutdown(self.finish)
        rospy.loginfo('[diag_latency] recording 8 topics to %s (tag=%s)', out_dir, tag)

    # ── 콜백 ──────────────────────────────────────────────────────────
    # 첫 줄에서 시각을 찍는다. 아래 계산을 먼저 하면 그 시간이 지연에 섞인다.

    def cb_ego(self, m):
        t = rospy.Time.now().to_sec()
        self.s_ego.row(t, m.header.stamp.to_sec(),
                       ['%.4f' % m.position.x, '%.4f' % m.position.y,
                        '%.3f' % m.heading, '%.4f' % m.velocity.x])

    def cb_gps(self, m):
        t = rospy.Time.now().to_sec()
        self.s_gps.row(t, m.header.stamp.to_sec(),
                       ['%.8f' % m.latitude, '%.8f' % m.longitude])

    def cb_fix(self, m):
        t = rospy.Time.now().to_sec()
        self.s_fix.row(t, m.header.stamp.to_sec(),
                       ['%.8f' % m.latitude, '%.8f' % m.longitude])

    def cb_imu(self, m):
        t = rospy.Time.now().to_sec()
        self.s_imu.row(t, m.header.stamp.to_sec(),
                       ['%.3f' % yaw_deg(m.orientation),
                        '%.5f' % m.angular_velocity.z])

    def cb_imue(self, m):
        t = rospy.Time.now().to_sec()
        self.s_imue.row(t, m.header.stamp.to_sec(),
                        ['%.3f' % yaw_deg(m.orientation),
                         '%.5f' % m.angular_velocity.z])

    def cb_wheel(self, m):
        t = rospy.Time.now().to_sec()
        self.s_wheel.row(t, m.header.stamp.to_sec(),
                         ['%.4f' % m.twist.twist.linear.x,
                          '%.4f' % m.twist.twist.linear.y])

    def cb_navsat(self, m):
        t = rospy.Time.now().to_sec()
        self.s_navs.row(t, m.header.stamp.to_sec(),
                        ['%.4f' % m.pose.pose.position.x,
                         '%.4f' % m.pose.pose.position.y])

    def cb_odom(self, m):
        t = rospy.Time.now().to_sec()
        self.s_odom.row(t, m.header.stamp.to_sec(),
                        ['%.4f' % m.pose.pose.position.x,
                         '%.4f' % m.pose.pose.position.y,
                         '%.3f' % yaw_deg(m.pose.pose.orientation),
                         '%.4f' % m.twist.twist.linear.x,
                         '%.4f' % m.twist.twist.linear.y])

    # ── 종료 요약 ─────────────────────────────────────────────────────
    def finish(self):
        for s in self.sinks:
            s.close()
        print('')
        print('[diag_latency] 토픽별 수집 결과')
        for s in self.sinks:
            mark = '  ' if s.n else '⚠️'
            print('  %s /%-16s %6d줄  %5.1f Hz' % (mark, s.name, s.n, s.hz()))
        dead = [s.name for s in self.sinks if s.n == 0]
        if dead:
            print('')
            print('  ⚠️ 0줄인 토픽: %s' % ', '.join(dead))
            print('     그 단계가 안 돌고 있다. localization.launch / udp_bridge 확인.')
        print('')
        print('  다음: rosrun path_tracking analyze_latency.py --tag <tag>')


if __name__ == '__main__':
    rospy.init_node('diag_latency', anonymous=True)
    DiagLatency()
    rospy.spin()
