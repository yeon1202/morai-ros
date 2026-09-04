#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""object_convert 단위 테스트. ROS 마스터 없이 돈다 (메시지 타입만 필요).

실행:
  docker exec morai-dev bash -lc \
    'source ~/catkin_ws/devel/setup.bash && rosrun path_tracking test_object_convert.py'
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from autonomous_driving.msg import RecognizedObject, RecognizedObjectArray
from lib.object_convert import (MAX_STATIC_LENGTH_M, MAX_STATIC_WIDTH_M,
                                empty_object_status_list, is_plausible_static,
                                to_object_status_list)


def make_sized(type_, length, width, dist=1.0, uid=7):
    o = RecognizedObject()
    o.type = type_
    o.unique_id = uid
    o.class_name = 'unknown'
    o.center.x, o.center.y, o.center.z = 1.0, 2.0, 3.0
    o.size.x, o.size.y, o.size.z = length, width, 1.5
    o.distance = dist
    return o


def make_obj(type_, x=1.0, y=2.0, yaw=0.0, vx=0.0, vy=0.0, uid=7, name='thing',
             dist=None):
    o = RecognizedObject()
    o.type = type_
    o.unique_id = uid
    o.class_name = name
    o.center.x, o.center.y, o.center.z = x, y, 3.0
    o.size.x, o.size.y, o.size.z = 4.0, 1.8, 1.5
    o.yaw = yaw
    o.velocity.x, o.velocity.y, o.velocity.z = vx, vy, 0.0
    # merge_sources 가 distance 로 정렬한다. 안 주면 원점 거리로 채운다.
    o.distance = math.hypot(x, y) if dist is None else dist
    return o


def wrap(objs):
    arr = RecognizedObjectArray()
    arr.objects = list(objs)
    return arr


def check(name, cond):
    print(('  PASS  ' if cond else '  FAIL  ') + name)
    return cond


def main():
    ok = True

    # 1) 종류별로 올바른 목록에 들어가고 개수가 맞는다
    out = to_object_status_list(wrap([make_obj(0), make_obj(1), make_obj(2), make_obj(2)]), None)
    ok &= check('보행자 1개', len(out.pedestrian_list) == 1 and out.num_of_pedestrian == 1)
    ok &= check('NPC 1개', len(out.npc_list) == 1 and out.num_of_npcs == 1)
    ok &= check('정적장애물 2개', len(out.obstacle_list) == 2 and out.num_of_obstacle == 2)

    # 2) 자차(-1)는 버린다
    out = to_object_status_list(wrap([make_obj(-1), make_obj(2)]), None)
    ok &= check('type -1 은 버려진다',
                len(out.obstacle_list) == 1 and not out.npc_list and not out.pedestrian_list)

    # 3) 속도 m/s -> km/h
    out = to_object_status_list(wrap([make_obj(1, vx=10.0, vy=-2.0)]), None)
    v = out.npc_list[0].velocity
    ok &= check('vx 10 m/s -> 36 km/h', abs(v.x - 36.0) < 1e-6)
    ok &= check('vy -2 m/s -> -7.2 km/h', abs(v.y - (-7.2)) < 1e-6)

    # 4) yaw 라디안 -> heading 도
    out = to_object_status_list(wrap([make_obj(2, yaw=math.pi / 2)]), None)
    ok &= check('yaw pi/2 -> heading 90도', abs(out.obstacle_list[0].heading - 90.0) < 1e-6)

    # 5) center(Point) -> position(Vector3) 값이 보존된다
    out = to_object_status_list(wrap([make_obj(2, x=-60.61, y=-142.178)]), None)
    p = out.obstacle_list[0].position
    ok &= check('position 값 보존', abs(p.x + 60.61) < 1e-6 and abs(p.y + 142.178) < 1e-6)
    ok &= check('position 타입이 Vector3', type(p).__name__ == 'Vector3')

    # 6) size 와 id, name 이 그대로 넘어간다
    o = out.obstacle_list[0]
    ok &= check('size 보존', abs(o.size.x - 4.0) < 1e-6 and abs(o.size.y - 1.8) < 1e-6)
    ok &= check('unique_id 보존', o.unique_id == 7)
    ok &= check('name 보존', o.name == 'thing')

    # 7) 빈 입력 -> 빈 목록, 개수 0
    out = to_object_status_list(wrap([]), None)
    ok &= check('빈 입력이면 개수 전부 0',
                out.num_of_obstacle == 0 and out.num_of_npcs == 0 and out.num_of_pedestrian == 0)

    # 8) frame_id 는 map (planning 의 전역경로와 같은 프레임)
    ok &= check("frame_id 는 'map'", out.header.frame_id == 'map')

    # 9) 인지가 끊겼을 때 낼 빈 목록도 같은 모양이어야 한다
    #    (어댑터가 이 함수를 쓰므로 여기서 같이 지킨다)
    e = empty_object_status_list(None)
    ok &= check('빈 목록도 frame_id 가 map', e.header.frame_id == 'map')
    ok &= check('빈 목록은 세 목록이 다 비어 있다',
                not e.obstacle_list and not e.npc_list and not e.pedestrian_list)
    ok &= check('빈 목록은 개수도 0',
                e.num_of_obstacle == 0 and e.num_of_npcs == 0 and e.num_of_pedestrian == 0)

    # ---- 정적장애물 통로 (2026-09-03 추가) ----

    # 10) static_objs 를 주면 obstacle_list 에 합쳐진다
    out = to_object_status_list(wrap([make_obj(1)]), None,
                                static_objs=[make_obj(2), make_obj(2)])
    ok &= check('tracked 차 1 + static 2 -> npc 1, obstacle 2',
                len(out.npc_list) == 1 and len(out.obstacle_list) == 2)

    # 11) static_objs 를 안 주면 예전과 똑같이 동작한다 (회귀 방지)
    out = to_object_status_list(wrap([make_obj(1)]), None)
    ok &= check('static 없으면 obstacle 0', not out.obstacle_list)

    # 12) tracked 가 None 이어도 (tracking_node 만 죽은 경우) 정적물은 나간다
    out = to_object_status_list(None, None, static_objs=[make_obj(2)])
    ok &= check('tracked=None 이어도 정적물은 발행된다', len(out.obstacle_list) == 1)

    # 13) ★ 정적물이 많아도 차/사람이 밀려나지 않는다
    #     거리순으로만 20개를 자르면 가까운 정적물 조각이 자리를 다 먹는다.
    many_static = [make_obj(2, dist=1.0, uid=100 + i) for i in range(30)]
    far_car = make_obj(1, dist=50.0, uid=1)
    out = to_object_status_list(wrap([far_car]), None, static_objs=many_static)
    ok &= check('먼 차가 가까운 정적물 30개에 밀려나지 않는다', len(out.npc_list) == 1)
    ok &= check('전체는 20개를 넘지 않는다',
                len(out.npc_list) + len(out.obstacle_list) + len(out.pedestrian_list) <= 20)

    # ---- 정적물 크기 상한 (2026-09-03 추가) ----
    #
    # DBSCAN 이 가드레일/터널벽을 통째로 묶어 최대 67m 짜리 물체가 나왔고,
    # 그게 ACC 통로 안으로 판정돼 차가 여러 번 섰다. 6.5 x 2.8 을 넘는 덩어리는
    # 개별 물체가 아니라 구조물이라 버린다.

    # 14) 정상 크기는 통과
    ok &= check('차 크기(4.5x1.9) 는 통과', is_plausible_static(make_sized(2, 4.5, 1.9)))
    ok &= check('라바콘 크기(0.4x0.4) 는 통과', is_plausible_static(make_sized(2, 0.4, 0.4)))

    # 15) 실측에서 차를 세운 크기들은 배제
    ok &= check('16.5x2.11 (경로 791m 에서 세운 것) 배제',
                not is_plausible_static(make_sized(2, 16.5, 2.11)))
    ok &= check('13.13x1.59 (경로 773m) 배제',
                not is_plausible_static(make_sized(2, 13.13, 1.59)))
    ok &= check('67.09x1.0 (최대값) 배제',
                not is_plausible_static(make_sized(2, 67.09, 1.0)))

    # 16) 경계값
    ok &= check('길이 6.5 는 통과(경계 포함)', is_plausible_static(make_sized(2, 6.5, 1.0)))
    ok &= check('길이 6.51 은 배제', not is_plausible_static(make_sized(2, 6.51, 1.0)))
    ok &= check('폭 2.8 은 통과(경계 포함)', is_plausible_static(make_sized(2, 4.0, 2.8)))
    ok &= check('폭 2.81 은 배제', not is_plausible_static(make_sized(2, 4.0, 2.81)))

    # 17) 축이 바뀌어 와도 같은 답 (인지가 x/y 를 뒤집어 줄 때가 있다)
    ok &= check('축이 뒤집혀도(2.11x16.5) 배제',
                not is_plausible_static(make_sized(2, 2.11, 16.5)))

    # 18) 실제 변환 경로에서 걸러지는가
    out = to_object_status_list(
        wrap([]), None,
        static_objs=[make_sized(2, 4.5, 1.9), make_sized(2, 16.5, 2.11)])
    ok &= check('큰 덩어리는 /Object_topic 에 안 실린다', len(out.obstacle_list) == 1)

    # 19) 차/사람은 크기 검사를 안 받는다 (tracking_node 가 이미 걸렀다)
    big_car = make_obj(1)
    big_car.size.x, big_car.size.y = 16.5, 2.11
    out = to_object_status_list(wrap([big_car]), None)
    ok &= check('tracked 는 크기로 안 걸른다', len(out.npc_list) == 1)

    # 20) 임계값 자체를 못 박아둔다 (팀 tracking_node.py 와 같은 값이어야 한다)
    ok &= check('임계 6.5 / 2.8',
                MAX_STATIC_LENGTH_M == 6.5 and MAX_STATIC_WIDTH_M == 2.8)

    print('')
    print('결과: ' + ('전부 통과' if ok else '실패 있음'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
