#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""object_topic_adapter : 팀 perception 의 결과를 planning 의 /Object_topic 으로 옮긴다.

  /perception/tracked_objects           ─┐
  (차·사람, 트래킹됨)                    ├─>  [이 노드]  ──>  /Object_topic
  /perception/recognized_objects_global ─┘
  (여기서 type=2 정적장애물만)
  (autonomous_driving)                              (morai_msgs)

팀은 /Object_topic 을 만들지 않는다. object_fusion_node.py 머리말에
"(Planning 쪽 /Object_topic 스펙이 ...)" 이라고 적혀 있듯, 그 변환은 planning 몫이다.

이 노드는 얇게 유지한다 - 실제 변환은 lib/object_convert.py 가 하고 여기서는
구독/발행/타임아웃만 맡는다. 그래야 변환 로직을 ROS 없이 테스트할 수 있다
(test_object_convert.py).

물체별 깜빡임 보정(latch)은 여기서 하지 않는다. tracking_node 가 이미 한다
(MIN_HITS_TO_CONFIRM=3, MAX_MISSES=5). 두 군데서 하면 지연이 겹치고 파라미터가
어긋났을 때 원인을 못 찾는다.

대신 "인지 노드가 통째로 죽는 경우"는 여기서 막는다. lattice_planner::objCb 는
받은 것을 덮어쓰기만 해서, 아무도 안 보내면 마지막 장애물을 영원히 믿는다.
이미 지나간 장애물을 계속 피하려 들게 된다.

사용법
  rosrun path_tracking object_topic_adapter.py
  rosrun path_tracking object_topic_adapter.py _timeout:=1.0

  _timeout : 이 시간[초] 동안 인지가 없으면 빈 목록을 낸다 (기본 0.5)
  _rate    : 발행 주기 [Hz] (기본 20 - mock_obstacle_pub 과 동일)

설계 근거는 docs/24-perception_integration_design.md 5절 참고.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from autonomous_driving.msg import RecognizedObjectArray
from morai_msgs.msg import ObjectStatusList

from lib.object_convert import (TYPE_STATIC_OBSTACLE, empty_object_status_list,
                                to_object_status_list)


class ObjectTopicAdapter:
    def __init__(self):
        self.timeout = rospy.get_param('~timeout', 0.5)
        rate_hz = rospy.get_param('~rate', 20.0)

        self.last_msg = None
        self.last_time = None
        self.static_objs = []
        self.static_time = None
        self.stale_warned = False

        self.pub = rospy.Publisher('/Object_topic', ObjectStatusList, queue_size=1)
        rospy.Subscriber('/perception/tracked_objects', RecognizedObjectArray,
                         self.cb, queue_size=1)
        # 정적장애물(type=2)은 트래킹 대상이 아니라 여기로만 온다 - lib/object_convert.py
        # merge_sources() 주석 참고. 두 토픽을 각각 받고 신선도도 따로 본다:
        # tracking_node 만 죽고 global_transform 은 살아있는 경우가 있을 수 있어서,
        # 한쪽이 끊겼다고 나머지까지 버리면 볼 수 있는 것을 못 보게 된다.
        rospy.Subscriber('/perception/recognized_objects_global', RecognizedObjectArray,
                         self.static_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / rate_hz), self.tick)

        rospy.loginfo('[object_topic_adapter] start - %.1fHz, timeout %.2fs',
                      rate_hz, self.timeout)

    def cb(self, msg):
        if self.stale_warned:
            rospy.loginfo('[object_topic_adapter] perception recovered')
            self.stale_warned = False
        self.last_msg = msg
        self.last_time = rospy.Time.now()

    def static_cb(self, msg):
        # 정적장애물만 고른다. 차/사람은 트래킹된 쪽(unique_id/velocity 가 채워진)을
        # 써야 하므로 여기서 걸러내지 않으면 같은 물체가 두 번 실린다.
        self.static_objs = [o for o in msg.objects if o.type == TYPE_STATIC_OBSTACLE]
        self.static_time = rospy.Time.now()

    def _fresh(self, stamp, now):
        return stamp is not None and (now - stamp).to_sec() <= self.timeout

    def tick(self, _event):
        now = rospy.Time.now()

        tracked_fresh = self._fresh(self.last_time, now)
        static_fresh = self._fresh(self.static_time, now)

        if not tracked_fresh and not static_fresh:
            if self.last_time is None and self.static_time is None:
                # 아직 한 번도 못 받았다. 시작 직후의 정상 상태다 - 경고하지 않는다.
                self.pub.publish(empty_object_status_list(now))
                return
            if not self.stale_warned:
                self.stale_warned = True
                age = min(
                    (now - t).to_sec()
                    for t in (self.last_time, self.static_time) if t is not None)
                # ROS 로그 포맷에 한글을 쓰면 컨테이너 로케일 때문에 깨진다
                rospy.logwarn('[object_topic_adapter] perception stale %.2fs '
                              '- publishing empty list', age)
            self.pub.publish(empty_object_status_list(now))
            return

        self.pub.publish(to_object_status_list(
            self.last_msg if tracked_fresh else None,
            now,
            self.static_objs if static_fresh else None))


if __name__ == '__main__':
    rospy.init_node('object_topic_adapter')
    ObjectTopicAdapter()
    rospy.spin()
