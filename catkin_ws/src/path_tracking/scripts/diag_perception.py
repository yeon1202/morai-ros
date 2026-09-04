#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diag_perception : 인지 결과(/Object_topic)의 품질을 잴 수 있게 원시 기록으로 남긴다.

왜 필요한가
  팀 YOLO 는 COCO 사전학습본이라 드럼통·라바콘 같은 정적장애물을 car(NPC)로
  오분류하고, 검출이 깜빡이거나 없는 물체를 만들어내기도 한다. planning 은 이걸
  고칠 수 없고(인지는 다른 팀 모듈이다) "견디는" 수밖에 없는데, 견디는 장치를
  만들려면 임계값이 필요하다. 그 임계값을 체감으로 정하지 않으려고 만든 도구다.

  이 프로젝트는 자 없이 체감으로 판단해 틀린 전례가 있다 - 벽시계로 시간을 재고
  "복귀 곡선이 1.6배 나빠졌다" 고 결론냈는데 실제로는 개선이었다(런북 함정 ④).

diag_odom.py 와 같은 원칙: 아무 판단도 하지 않는다
  세 토픽을 원시 그대로 각각 CSV 에 쌓기만 한다. 기록하는 도구가 판단을 섞으면
  결과가 이상할 때 "인지가 이상한 건지 도구가 이상한 건지" 를 못 가른다.

  ⚠️ 세 파일을 한 줄로 짝짓지 않는다. 주기가 달라(/Object_topic 20Hz, /odom
     8~10Hz, /ego_status 20~26Hz) 짝을 지으면 시각차만큼 오차가 섞인다.
     15m/s 에서 25ms 면 0.38m 로, 재려는 값과 같은 크기다. 분석할 때 빠른 쪽을
     느린 쪽 시각으로 보간한다.

왜 /odom 과 /ego_status 를 같이 받나 (2026-08-30)
  인지팀 파이프라인은 물체를 이미 전역좌표로 바꿔서 준다. 그 변환에 EKF 가 소유한
  base_link 를 쓰므로, 우리가 받는 물체 좌표에는 /odom 의 위치 오차가 이미 섞여
  들어와 있다. 자차 위치를 추정치(/odom)와 참값(/ego_status @9111) 둘 다 남겨두면
  나중에 CSV 만 보고 그 둘을 갈라낼 수 있다:

    자차→물체 상대위치 = 물체좌표 − 자차위치(EKF)
    물체좌표(GT)      = 자차위치(GT) + 상대위치
                     ≈ 물체좌표 + (자차위치(GT) − 자차위치(EKF))

  두 번째 줄의 근사는 /odom 의 yaw 오차가 중앙값 0.17도로 거의 없기 때문에 쓴다
  (회전 보정 항이 무시할 만해서 사실상 평행이동 하나로 끝난다).

  ⚠️ 참값은 브릿지가 9111(Ego Vehicle Status)일 때만 온다. 9109(대회 규정 채널)
     에서는 position 이 0,0,0 이다. 이 노드는 시작할 때 어느 쪽인지 찍어준다.
     **제출본은 반드시 9109 여야 한다.**

사용법
  rosrun path_tracking diag_perception.py
  rosrun path_tracking diag_perception.py _tag:=perc_mock

  _out_dir  : CSV 저장 폴더 (기본은 lib/logdir.py 가 환경 보고 고른다)
  _tag      : 파일명 꼬리표. percobj_<tag>.csv / percodom_<tag>.csv / percgt_<tag>.csv
  _max_dets : 요약용으로 메모리에 들고 있을 검출 수 상한 (기본 300000)

  ※ 파일명에 perc 접두사를 붙이는 이유: diag_odom.py 가 odom_<tag>.csv 를 쓰므로
    같은 폴더에서 같은 tag 를 쓰면 서로 덮어쓴다.

Ctrl+C 로 끝내면 요약을 찍는다. 먼저 mock 으로 한 번 돌려볼 것 - mock 장애물은
좌표와 개수를 우리가 알고 있어서(unique_id=1 하나, (-60.610, -142.178)) 도구가
맞는 답을 내는지 그 자리에서 확인된다.
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from nav_msgs.msg import Odometry
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from autonomous_driving.msg import RecognizedObjectArray

from lib.logdir import default_log_dir
from lib.perception_stats import Detection, summarize

KMH_TO_MPS = 1.0 / 3.6


def yaw_deg(q):
    """쿼터니언 -> yaw [deg]. /odom 은 heading 을 각도로 주지 않는다."""
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(s, c))


def load_path(pkg_dir):
    """전역경로 CSV 를 (x, y) 목록으로. path_tracker.py 와 같은 우선순위를 쓴다."""
    smooth = os.path.join(pkg_dir, 'path', 'path_smooth.csv')
    raw = os.path.join(pkg_dir, 'path', 'path.csv')
    csv_path = smooth if os.path.exists(smooth) else raw
    pts = []
    with open(csv_path) as f:
        reader = csv.reader(f)
        next(reader)                       # 헤더(x,y,z)
        for row in reader:
            pts.append((float(row[0]), float(row[1])))
    return csv_path, pts


class DiagPerception:
    def __init__(self):
        out_dir = rospy.get_param('~out_dir', default_log_dir())
        tag = rospy.get_param('~tag', 'perc')
        self.max_dets = int(rospy.get_param('~max_dets', 300000))

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        csv_path, self.path = load_path(pkg_dir)

        self.f_obj = open(os.path.join(out_dir, 'percobj_%s.csv' % tag), 'w')
        self.f_odom = open(os.path.join(out_dir, 'percodom_%s.csv' % tag), 'w')
        self.f_gt = open(os.path.join(out_dir, 'percgt_%s.csv' % tag), 'w')
        # 인지 원본. /Object_topic 에는 없는 필드를 여기서만 얻을 수 있다.
        #
        # 왜 따로 받나 (2026-09-03)
        #   경로 옆 1.5~1.8m 에 0.3~0.9m 짜리 물체가 잡혀 회피가 촉발됐고 차가
        #   인도로 올라갔다. 그게 진짜 연석 위 물체인지 라이다 노이즈인지
        #   크기만으로는 구분이 안 된다. 몇 점으로 이루어진 클러스터인지
        #   (num_points) 알면 갈린다 - lidar_node 의 DBSCAN min_samples 가 5 다.
        #   그런데 num_points·confidence·class_name 은 RecognizedObject 에만 있고
        #   ObjectStatus(=/Object_topic)에는 없다. 어댑터가 옮길 자리가 없어서다.
        self.f_raw = open(os.path.join(out_dir, 'percraw_%s.csv' % tag), 'w')
        self.w_obj = csv.writer(self.f_obj)
        self.w_odom = csv.writer(self.f_odom)
        self.w_gt = csv.writer(self.f_gt)
        self.w_raw = csv.writer(self.f_raw)
        # list = 어느 목록으로 왔는가, type = 메시지의 type 필드.
        # 둘을 따로 남기는 이유: planning 은 목록으로 소비하는데(lattice 는
        # npc_list + obstacle_list) type 필드와 어긋나 있으면 그 자체가 발견이다.
        self.w_obj.writerow(['t', 'frame', 'uid', 'list', 'type', 'name',
                             'x', 'y', 'z', 'sx', 'sy', 'sz',
                             'heading_deg', 'vx_kmh', 'vy_kmh'])
        self.w_odom.writerow(['t', 'x', 'y', 'yaw_deg'])
        self.w_gt.writerow(['t', 'x', 'y', 'heading_deg'])
        self.w_raw.writerow(['t', 'frame', 'uid', 'type', 'class_name',
                             'x', 'y', 'z', 'sx', 'sy', 'sz', 'yaw_rad',
                             'distance', 'num_points', 'confidence'])
        self.raw_frame = 0

        self.frame = 0            # /Object_topic 을 받은 횟수 (0부터)
        self.dets = []            # 종료 요약용. 물체가 없는 프레임도 frame 은 증가한다
        self.n_odom = self.n_gt = 0
        self.capped = False
        self.gt_checked = False

        rospy.Subscriber('/Object_topic', ObjectStatusList, self.obj_cb, queue_size=200)
        rospy.Subscriber('/perception/recognized_objects_global', RecognizedObjectArray,
                         self.raw_cb, queue_size=200)
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=200)
        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.ego_cb, queue_size=200)
        rospy.on_shutdown(self.finish)

        rospy.loginfo('[diag_perception] 경로 %d점 (%s)', len(self.path),
                      os.path.basename(csv_path))
        rospy.loginfo('[diag_perception] recording 3 topics to %s (tag=%s)', out_dir, tag)

    @staticmethod
    def _stamp(msg):
        t = msg.header.stamp.to_sec()
        return t if t > 0.0 else rospy.Time.now().to_sec()

    def raw_cb(self, msg):
        t = self._stamp(msg)
        for o in msg.objects:
            self.w_raw.writerow([
                '%.6f' % t, self.raw_frame, o.unique_id, o.type, o.class_name,
                '%.3f' % o.center.x, '%.3f' % o.center.y, '%.3f' % o.center.z,
                '%.3f' % o.size.x, '%.3f' % o.size.y, '%.3f' % o.size.z,
                '%.4f' % o.yaw, '%.3f' % o.distance,
                o.num_points, '%.3f' % o.confidence])
        self.raw_frame += 1

    def obj_cb(self, msg):
        t = self._stamp(msg)
        lists = (('pedestrian', 0, msg.pedestrian_list),
                 ('npc', 1, msg.npc_list),
                 ('obstacle', 2, msg.obstacle_list))
        for list_name, list_type, items in lists:
            for o in items:
                self.w_obj.writerow([
                    '%.6f' % t, self.frame, o.unique_id, list_name, o.type, o.name,
                    '%.3f' % o.position.x, '%.3f' % o.position.y, '%.3f' % o.position.z,
                    '%.3f' % o.size.x, '%.3f' % o.size.y, '%.3f' % o.size.z,
                    '%.3f' % o.heading,
                    '%.3f' % o.velocity.x, '%.3f' % o.velocity.y])
                if len(self.dets) < self.max_dets:
                    # 요약에는 "planning 이 실제로 소비하는 분류" 인 목록 쪽을 쓴다.
                    self.dets.append(Detection(
                        frame=self.frame, t=t, uid=o.unique_id, type=list_type,
                        x=o.position.x, y=o.position.y,
                        sx=o.size.x, sy=o.size.y,
                        speed=math.hypot(o.velocity.x, o.velocity.y) * KMH_TO_MPS))
                elif not self.capped:
                    self.capped = True
                    rospy.logwarn('[diag_perception] 검출 %d개를 넘어 요약 표본을 '
                                  '멈춘다. CSV 기록은 계속된다.', self.max_dets)
        self.frame += 1

    def odom_cb(self, msg):
        self.w_odom.writerow(['%.6f' % self._stamp(msg),
                              '%.3f' % msg.pose.pose.position.x,
                              '%.3f' % msg.pose.pose.position.y,
                              '%.3f' % yaw_deg(msg.pose.pose.orientation)])
        self.n_odom += 1

    def ego_cb(self, msg):
        p = msg.position
        # 브릿지가 어느 포트로 받고 있는지는 position 으로 드러난다. 9109(대회 채널)
        # 는 0,0,0 을 주고, 9111(GT)은 실제 좌표를 준다. 설정 파일을 읽는 것보다
        # 이쪽이 낫다 - 설정이 아니라 실제로 들어온 값을 보는 것이라 안 속는다.
        if not self.gt_checked:
            self.gt_checked = True
            if abs(p.x) < 1e-6 and abs(p.y) < 1e-6:
                rospy.logwarn('[diag_perception] /ego_status.position 이 0,0,0 이다 '
                              '-> 브릿지가 9109(대회 채널). 참값이 없어 인지 오차와 '
                              '위치 오차를 갈라낼 수 없다. 측정만 할 거면 9111 로 '
                              '되돌릴 것 (제출본은 반드시 9109).')
            else:
                rospy.logwarn('[diag_perception] 참값 위치가 들어온다 -> 브릿지가 '
                              '9111(GT). 측정에는 이게 맞다. **제출 전에 9109 로 '
                              '되돌릴 것 - 안 그러면 실격이다.**')
        self.w_gt.writerow(['%.6f' % self._stamp(msg),
                            '%.3f' % p.x, '%.3f' % p.y, '%.3f' % msg.heading])
        self.n_gt += 1

    def finish(self):
        for f in (self.f_obj, self.f_odom, self.f_gt, self.f_raw):
            f.close()

        print('')
        print('=== diag_perception 요약 ===')
        print('프레임(/Object_topic) %d, /odom %d줄, /ego_status %d줄'
              % (self.frame, self.n_odom, self.n_gt))
        if not self.dets:
            print('검출이 하나도 없었다. 인지(또는 mock)가 떠 있는지 확인할 것.')
            return

        r = summarize(self.dets, self.path, n_frames_total=self.frame)
        print('기록시간 %.1f초, 빈 프레임 %d개 (%.1f%%)'
              % (r['duration'], r['empty_frames'],
                 100.0 * r['empty_frames'] / max(1, r['n_frames'])))
        print('프레임당 물체 수: 평균 %.2f, 최소 %d, 최대 %d'
              % (r['per_frame']['mean'], r['per_frame']['min'], r['per_frame']['max']))

        names = {0: '보행자', 1: 'NPC', 2: '정적장애물'}
        print('')
        print('분류 (목록 기준)      검출건수 / 물체수')
        for k in sorted(set(list(r['type_detections']) + list(r['type_uids']))):
            print('  %-12s %8d / %d' % (names.get(k, str(k)),
                                        r['type_detections'].get(k, 0),
                                        r['type_uids'].get(k, 0)))

        print('')
        print('물체별 (검출 많은 순 20개)')
        print('  %8s %6s %6s %7s %6s %7s %7s %7s' %
              ('uid', '분류', '검출', '생존s', '끊김', 'd최소', 'd중앙', '속도max'))
        for t in sorted(r['tracks'], key=lambda x: -x['n'])[:20]:
            print('  %8s %6s %6d %7.1f %6d %7.2f %7.2f %7.2f' %
                  (t['uid'], names.get(t['type'], t['type']), t['n'], t['life'],
                   t['gaps'], t['d_min'], t['d_med'], t['speed_max']))

        print('')
        print('오검출 후보 (사는 내내 도로 밖) : %s'
              % (r['off_road'] if r['off_road'] else '없음'))
        print('크기 이상치                     : %s'
              % (r['size_outliers'] if r['size_outliers'] else '없음'))
        flips = [t['uid'] for t in r['tracks'] if t['type_flips'] > 0]
        print('도중에 분류가 바뀐 물체         : %s' % (flips if flips else '없음'))
        print('')
        print('※ "오검출 후보" 는 판정이 아니다. 도로 밖에 진짜 물체가 있을 수도 '
              '있다(가드레일, 표지판).')
        print('※ GPS 음영구간에서는 위치 오차가 20m 급이라 인지 평가에서 빼야 한다. '
              'percodom 과 percgt 를 겹쳐 보면 그 구간이 드러난다.')


if __name__ == '__main__':
    rospy.init_node('diag_perception')
    DiagPerception()
    rospy.spin()
