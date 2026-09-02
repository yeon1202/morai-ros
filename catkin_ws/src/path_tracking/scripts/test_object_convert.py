#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""object_convert 단위 테스트. ROS 마스터 없이 돈다 (메시지 타입만 필요).

실행:
  docker exec morai-dev bash -lc \
    'cd /home/dev/catkin_ws && source devel/setup.bash && \
     python3 src/path_tracking/scripts/test_object_convert.py'
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from autonomous_driving.msg import RecognizedObject, RecognizedObjectArray
from lib.object_convert import empty_object_status_list, to_object_status_list


def make_obj(type_, x=1.0, y=2.0, yaw=0.0, vx=0.0, vy=0.0, uid=7, name='thing'):
    o = RecognizedObject()
    o.type = type_
    o.unique_id = uid
    o.class_name = name
    o.center.x, o.center.y, o.center.z = x, y, 3.0
    o.size.x, o.size.y, o.size.z = 4.0, 1.8, 1.5
    o.yaw = yaw
    o.velocity.x, o.velocity.y, o.velocity.z = vx, vy, 0.0
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

    print('')
    print('결과: ' + ('전부 통과' if ok else '실패 있음'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
