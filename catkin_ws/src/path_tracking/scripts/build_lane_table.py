#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""build_lane_table : 전역경로 웨이포인트마다 "옆 차로가 있는가" 표를 만든다.

  map/mapping_result.csv   웨이포인트 -> MGeo link_id   (팀 global_path_link_mapper.py 산출물)
  map/link_set.json        link_id -> 차로 정보          (대회 제공 MGeo, 9MB)
        │
        └─ 조인 ─> map/lane_table.csv   (약 200KB, 이것만 커밋한다)

왜 미리 만들어 두는가
  link_set.json 이 9MB 라 repo 에 안 넣는다(.gitignore). 그러면 대회장 PC 에서
  받아올 수 없으니, 런타임에 실제로 필요한 열만 뽑아 작은 표로 만들어 커밋한다.
  런타임 조회도 배열 인덱싱이 되어 기하 검색이 사라진다.

왜 이 표가 필요한가 (2026-09-03 실패 기록)
  경로 417.8m 에서 오탐으로 회피가 걸렸고, lattice 가 우측 -3.51m 후보를 골랐다.
  그 지점 링크는 can_move_right_lane=True 라 맞는 선택처럼 보였지만, 12m 앞
  429.9m 에서 링크가 바뀌며 False 가 된다. 차는 인도로 올라가 멈췄고 60초간
  복구하지 못했다.
    -> 자차 위치 하나만 보면 못 잡는다. 후보가 지나갈 "구간 전체" 를 봐야 한다.
    -> 전역경로 4392점 중 우측 차로가 있는 곳은 25.0% 뿐이다. 나머지 75% 에서
       우측 회피는 도로 밖이다.

사용법
  rosrun path_tracking build_lane_table.py
  rosrun path_tracking build_lane_table.py --map-dir <경로>
"""
import argparse
import csv
import json
import os
import sys


def build(map_dir):
    mapping_csv = os.path.join(map_dir, 'mapping_result.csv')
    link_json = os.path.join(map_dir, 'link_set.json')
    out_csv = os.path.join(map_dir, 'lane_table.csv')

    for f in (mapping_csv, link_json):
        if not os.path.exists(f):
            sys.stderr.write('[build_lane_table] 없음: %s\n' % f)
            return 1

    with open(link_json) as f:
        links = json.load(f)
    by_id = {L['idx']: L for L in links}

    with open(mapping_csv) as f:
        rows = list(csv.DictReader(f))

    missing = [r for r in rows if r['link_id'] not in by_id]
    if missing:
        sys.stderr.write('[build_lane_table] link_set 에 없는 link_id %d개 - 중단\n'
                         % len(missing))
        return 1

    with open(out_csv, 'w') as f:
        w = csv.writer(f)
        # x,y 를 같이 넣는 이유: lattice 는 /local_path(전역경로에서 잘린 조각)만
        # 받아서 자기가 전역경로의 몇 번 점에 있는지 모른다. 좌표가 표 안에 있으면
        # road_core 가 스스로 최근접 인덱스를 찾을 수 있어서, lattice 가 전역경로를
        # 따로 구독할 필요가 없어진다.
        w.writerow(['waypoint_idx', 'x', 'y', 'right_ok', 'left_ok', 'ego_lane',
                    'road_width', 'link_id'])
        for r in rows:
            L = by_id[r['link_id']]
            width = 0.5 * (float(L.get('width_start') or 0.0) +
                           float(L.get('width_end') or 0.0))
            w.writerow([r['waypoint_idx'],
                        '%.3f' % float(r['wp_x']),
                        '%.3f' % float(r['wp_y']),
                        1 if L.get('can_move_right_lane') else 0,
                        1 if L.get('can_move_left_lane') else 0,
                        L.get('ego_lane') if L.get('ego_lane') is not None else -1,
                        '%.2f' % width,
                        r['link_id']])

    n = len(rows)
    nr = sum(1 for r in rows if by_id[r['link_id']].get('can_move_right_lane'))
    nl = sum(1 for r in rows if by_id[r['link_id']].get('can_move_left_lane'))
    print('[build_lane_table] %s' % out_csv)
    print('  웨이포인트 %d개' % n)
    print('  우측 차로 있음 %d (%.1f%%)' % (nr, 100.0 * nr / n))
    print('  좌측 차로 있음 %d (%.1f%%)' % (nl, 100.0 * nl / n))
    return 0


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('--map-dir', default=os.path.join(here, '..', 'map'))
    a = ap.parse_args()
    return build(os.path.normpath(a.map_dir))


if __name__ == '__main__':
    sys.exit(main())
