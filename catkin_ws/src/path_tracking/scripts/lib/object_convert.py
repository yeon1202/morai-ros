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

# ObjectStatusList.msg 주석: "Details of the 20 closest objects".
MAX_OBJECTS = 20

# 정적장애물로 인정할 최대 크기 [m].
#
# 값 출처: 팀 tracking_node.py 의 MAX_PLAUSIBLE_LENGTH_M / MAX_PLAUSIBLE_WIDTH_M.
# 같은 값을 쓰는 이유는 두 경로가 같은 기준으로 판단해야 하기 때문이다 - 트래킹을
# 타는 차/사람은 거기서 걸러지는데 정적물만 안 걸러지면, 같은 클러스터가 어느
# 목록으로 오느냐에 따라 살거나 죽는다.
#
# 왜 필요한가 (2026-09-03 실측, logs/percep5)
#   DBSCAN 이 가드레일·터널벽을 통째로 한 덩어리로 묶는다. 정적물 검출 63,248건의
#   긴 변 길이가 90% 11.28m, 99% 50.52m, 최대 67.09m 였다(num_points 최대 8,474).
#
#   그러면 회전 상자 판정으로도 못 막는다:
#     16.50 x 2.11 물체가 6도 기울면 횡반폭 = |sin6|*8.25 + |cos6|*1.055 = 1.91
#     경로에서 1.50m 떨어져 있어도 1.50 - 1.91 < 0.95 -> ACC 통로 안 -> 앞차 -> 정지
#     기울기가 0 이어도 폭 2.11 의 절반(1.055)만으로 1.50 - 1.055 = 0.445 < 0.95 다.
#   실제로 경로 773m / 791m / 887m / 1534m 에서 이 덩어리들 때문에 차가 섰다.
#
# 왜 버려도 되는가
#   6.5m 를 넘는 덩어리는 이 코스에서 "피해 가거나 뒤를 따라갈 개별 물체" 가 아니다.
#   가드레일·벽·방음벽 같은 구조물이다. 도로 경계는 이제 지도로 안다
#   (road_core.hpp + map/lane_table.csv) - 인지가 벽을 알려줄 필요가 없다.
#
# ⚠️ 최소 크기는 여기서 걸르지 않는다. 라바콘(폭 0.4m 급)이 같이 사라진다.
#    작은 오탐 문제는 num_points 를 본 뒤에 따로 정한다.
MAX_STATIC_LENGTH_M = 6.5
MAX_STATIC_WIDTH_M = 2.8


def is_plausible_static(obj):
    """정적장애물로 인정할 크기인가.

    size 규약: x=length(주축), y=width(부축). 인지가 축을 바꿔 줄 때가 있어
    max/min 으로 정규화해서 본다.
    """
    long_side = max(obj.size.x, obj.size.y)
    short_side = min(obj.size.x, obj.size.y)
    return long_side <= MAX_STATIC_LENGTH_M and short_side <= MAX_STATIC_WIDTH_M


def merge_sources(tracked_objs, static_objs):
    """두 인지 출력을 합쳐 /Object_topic 에 실을 목록을 만든다.

    왜 두 개인가 (2026-09-03)
      tracking_node 가 2026-08-15 부터 카메라+라이다 둘 다 잡힌 것(차 type=1 /
      사람 type=0)만 트래킹한다. 라이다에만 잡힌 정적장애물(type=2)은 트래킹
      대상에서 빠져 /perception/tracked_objects 에 안 들어온다. 그래서
      /perception/recognized_objects_global 을 따로 받아야 정적장애물이 보인다.
      한쪽만 받으면 대회의 "정적장애물 회피" 미션을 아예 못 한다.

    왜 트래킹된 것을 먼저 채우는가
      정적장애물은 DBSCAN 과분할로 조각이 많이 나온다. 거리순으로만 20개를
      자르면 조각들이 자리를 다 먹어서 정작 중요한 차/사람이 밀려난다
      (팀 object_topic_node.py 가 2026-08-21 실측으로 확인한 문제).
      그래서 차/사람을 먼저 다 넣고, 남는 자리만 가까운 정적물로 채운다.
    """
    tracked = sorted(tracked_objs, key=lambda o: o.distance)[:MAX_OBJECTS]
    # 크기 검사는 정적물에만 한다. 차/사람은 tracking_node 가 같은 임계로 이미
    # 걸렀다 - 두 번 걸르면 어느 단계에서 사라졌는지 못 찾는다.
    plausible = [o for o in static_objs if is_plausible_static(o)]
    remaining = max(MAX_OBJECTS - len(tracked), 0)
    nearest_static = sorted(plausible, key=lambda o: o.distance)[:remaining]
    return tracked + nearest_static


def to_object_status_list(tracked, stamp, static_objs=None):
    """RecognizedObjectArray -> ObjectStatusList.

    tracked      : /perception/tracked_objects (차/사람). None 이면 없는 것으로 본다.
    static_objs  : /perception/recognized_objects_global 에서 고른 type=2 목록.
                   None 이면 정적장애물 없이 나간다(예전 동작과 같다).
    stamp 가 None 이면 header.stamp 를 건드리지 않는다(테스트용).
    """
    out = ObjectStatusList()
    if stamp is not None:
        out.header.stamp = stamp
    # planning 의 전역경로와 같은 프레임. 팀 노드는 'odom' 으로 채워 보내지만
    # map 과 odom 은 항등 변환으로 이어져 있어 값은 같다(localization.launch 참고).
    out.header.frame_id = 'map'

    merged = merge_sources(
        list(tracked.objects) if tracked is not None else [],
        list(static_objs) if static_objs else [])

    for obj in merged:
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
