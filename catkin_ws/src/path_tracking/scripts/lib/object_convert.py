# -*- coding: utf-8 -*-
"""팀 perception 의 RecognizedObjectArray 를 planning 의 ObjectStatusList 로 옮긴다.

rospy 를 쓰지 않는 순수 함수다. 시뮬레이터도 ROS 마스터도 없이 테스트할 수 있어야
대회 당일 회귀를 빨리 잡을 수 있다 (acc_core.hpp 를 gtest 로 분리해 둔 것과 같은 이유).

단위가 두 군데 다르다 - 여기서 맞춰주지 않으면 조용히 틀린다:
  velocity  팀은 m/s, /Object_topic 소비자는 km/h (acc_planner.cpp 의 speedKmhToMps)
  방향      팀 yaw 는 라디안, ObjectStatus.heading 은 도

타입도 하나 다르다:
  RecognizedObject.center 는 geometry_msgs/Point,
  ObjectStatus.position   은 geometry_msgs/Vector3.
  필드 이름이 같아서 통째로 대입하고 싶어지지만 다른 타입이라 직렬화가 깨진다.
  x/y/z 를 하나씩 옮긴다.
"""
import math

from morai_msgs.msg import ObjectStatus, ObjectStatusList

MPS_TO_KMH = 3.6

# Planning 스펙 (object_fusion_node.py 의 CLASS_NAME_TO_TYPE 과 같은 정의)
TYPE_PEDESTRIAN = 0
TYPE_NPC = 1
TYPE_STATIC_OBSTACLE = 2


def to_object_status_list(tracked, stamp):
    """RecognizedObjectArray -> ObjectStatusList.

    stamp 가 None 이면 header.stamp 를 건드리지 않는다(테스트용).
    """
    out = ObjectStatusList()
    if stamp is not None:
        out.header.stamp = stamp
    # planning 의 전역경로와 같은 프레임. 팀 노드는 'odom' 으로 채워 보내지만
    # map 과 odom 은 항등 변환으로 이어져 있어 값은 같다(localization.launch 참고).
    out.header.frame_id = 'map'

    for obj in tracked.objects:
        if obj.type not in (TYPE_PEDESTRIAN, TYPE_NPC, TYPE_STATIC_OBSTACLE):
            continue                      # -1 = 자차. 버린다.

        st = ObjectStatus()
        st.unique_id = obj.unique_id
        st.type = obj.type
        st.name = obj.class_name

        st.position.x = obj.center.x      # Point -> Vector3, 통째 대입 금지
        st.position.y = obj.center.y
        st.position.z = obj.center.z

        st.size.x = obj.size.x
        st.size.y = obj.size.y
        st.size.z = obj.size.z

        st.heading = math.degrees(obj.yaw)

        st.velocity.x = obj.velocity.x * MPS_TO_KMH
        st.velocity.y = obj.velocity.y * MPS_TO_KMH
        st.velocity.z = obj.velocity.z * MPS_TO_KMH

        if obj.type == TYPE_PEDESTRIAN:
            out.pedestrian_list.append(st)
        elif obj.type == TYPE_NPC:
            out.npc_list.append(st)
        else:
            out.obstacle_list.append(st)

    out.num_of_pedestrian = len(out.pedestrian_list)
    out.num_of_npcs = len(out.npc_list)
    out.num_of_obstacle = len(out.obstacle_list)
    return out


def empty_object_status_list(stamp):
    """인지가 끊겼을 때 낼 빈 목록. '장애물 없음' 은 정상 상태다."""
    out = ObjectStatusList()
    if stamp is not None:
        out.header.stamp = stamp
    out.header.frame_id = 'map'
    return out
