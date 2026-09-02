#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diag_frame : GPS 좌표계와 MORAI 맵 좌표계가 같은 프레임인지 재기 위한 원시 로거

왜 필요한가
  우리 global path 는 /ego_status 의 position(MORAI 맵 로컬좌표)으로 기록했다.
  localization 팀의 /odom 은 GPS 위경도를 origin_file 기준 구면근사로 변환해서
  만든다. 이 둘이 같은 프레임이라는 보장이 없다 (docs/23-localization_node_review.md 2절).
  같은 시각의 (위경도, 맵좌표) 쌍을 충분히 모으면 두 좌표계의 대응관계가 나온다.

이 노드는 아무 판단도 하지 않는다
  /gps 와 /ego_status 를 각각 원시 그대로 CSV 에 쌓기만 한다. 변환도 필터링도
  하지 않는다. 분석은 별도 스크립트가 한다.

왜 두 토픽을 한 줄로 짝지어 기록하지 않나
  주기가 다르다. 짝을 지으면 두 메시지의 시각차만큼 오차가 섞인다. 55km/h
  (=15.3m/s) 에서 50ms 어긋나면 0.76m 다. 우리가 재려는 값과 같은 크기라
  측정이 무의미해진다.
  원시로 남겨두면 분석할 때 ego 위치를 GPS 시각으로 보간할 수 있다. ego 가
  GPS 보다 자주 오므로 보간 구간이 짧아 오차가 거의 사라진다.

시각에 대한 주의
  두 토픽의 header.stamp 는 모두 브릿지가 UDP 패킷을 받은 순간(rospy.Time.now())
  이다. MORAI 가 그 값을 만든 순간이 아니다. 두 스트림의 전송 지연이 서로 다르면
  그 차이는 여기서 알 수 없다. 잔차가 이상하게 크면 이걸 의심할 것.

사용법
  rosrun path_tracking diag_frame.py
  rosrun path_tracking diag_frame.py _out_dir:=/tmp _tag:=lap2

  _out_dir : CSV 저장 폴더. 기본값은 lib/logdir.py 가 환경을 보고 고른다 -
             도커면 마운트된 catkin_ws/logs(호스트에서 바로 열린다),
             아니면 ~/morai_logs
  _tag     : 파일명 꼬리표. frame_gps_<tag>.csv / frame_ego_<tag>.csv

Ctrl+C 로 끝내면 몇 줄씩 쌓였는지 요약한다.
"""
import csv
import os

from lib.logdir import default_log_dir

import rospy
from morai_msgs.msg import EgoVehicleStatus, GPSMessage


class FrameLogger(object):
    def __init__(self):
        out_dir = rospy.get_param('~out_dir', default_log_dir())
        tag = str(rospy.get_param('~tag', 'frame'))

        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)

        self.gps_path = os.path.join(out_dir, 'frame_gps_%s.csv' % tag)
        self.ego_path = os.path.join(out_dir, 'frame_ego_%s.csv' % tag)

        self.gps_f = open(self.gps_path, 'w')
        self.ego_f = open(self.ego_path, 'w')
        self.gps_w = csv.writer(self.gps_f)
        self.ego_w = csv.writer(self.ego_f)
        self.gps_w.writerow(['t', 'lat', 'lon', 'alt'])
        self.ego_w.writerow(['t', 'x', 'y', 'z', 'heading_deg', 'vel_x', 'vel_y'])

        self.gps_n = 0
        self.ego_n = 0

        # queue_size 를 크게 잡는다. 이 노드는 최신값이 아니라 '빠짐없는 기록' 이
        # 목적이라, 콜백이 밀려도 버리지 않고 쌓이는 편이 낫다.
        rospy.Subscriber('/gps', GPSMessage, self.gps_cb, queue_size=200)
        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.ego_cb, queue_size=200)

        rospy.on_shutdown(self.finish)
        rospy.loginfo('[diag_frame] logging -> %s , %s', self.gps_path, self.ego_path)

    @staticmethod
    def _stamp(msg):
        # 브릿지가 header.stamp 를 채운다. 혹시 비어 있으면 수신 시각으로 대체한다.
        t = msg.header.stamp
        return (t if t.to_sec() > 0.0 else rospy.Time.now()).to_sec()

    def gps_cb(self, msg):
        self.gps_w.writerow(['%.6f' % self._stamp(msg),
                             '%.9f' % msg.latitude,
                             '%.9f' % msg.longitude,
                             '%.4f' % msg.altitude])
        self.gps_n += 1

    def ego_cb(self, msg):
        self.ego_w.writerow(['%.6f' % self._stamp(msg),
                             '%.4f' % msg.position.x,
                             '%.4f' % msg.position.y,
                             '%.4f' % msg.position.z,
                             '%.4f' % msg.heading,
                             '%.4f' % msg.velocity.x,
                             '%.4f' % msg.velocity.y])
        self.ego_n += 1

    def finish(self):
        self.gps_f.close()
        self.ego_f.close()
        # ROS_INFO 포맷 문자열에 한글을 쓰면 컨테이너 로케일 때문에 깨진다.
        rospy.loginfo('[diag_frame] gps=%d rows, ego=%d rows', self.gps_n, self.ego_n)
        rospy.loginfo('[diag_frame] %s', self.gps_path)
        rospy.loginfo('[diag_frame] %s', self.ego_path)
        if self.gps_n == 0:
            rospy.logwarn('[diag_frame] no /gps received - check MORAI GPS sensor (port 2503)')
        if self.ego_n == 0:
            rospy.logwarn('[diag_frame] no /ego_status received - check bridge')


if __name__ == '__main__':
    rospy.init_node('diag_frame')
    FrameLogger()
    rospy.spin()
