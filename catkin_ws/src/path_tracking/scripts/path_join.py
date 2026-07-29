#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_join : 접근경로 + 코스경로를 하나의 global path 로 잇는다.
------------------------------------------------------------------
읽기:  path/approach.csv  (스폰 지점 -> 코스 진입점)
       path/course.csv    (코스 한 바퀴)
쓰기:  path/path.csv      (이어붙인 결과. 이후 path_smoother.py 로 스무딩)

하는 일 두 가지:
  1) approach 가 코스 진입점을 지나쳐서 끝났으면, 지나친 만큼 잘라낸다.
  2) course 기록이 출발점을 지나쳐서 끝났으면(중복 구간), 그만큼 잘라낸다.
     -> 안 자르면 진입점 부근에서 "가장 가까운 waypoint" 가 경로 끝으로
        잘못 잡혀서, local path 가 텅 비고 조향이 0 이 된다.

실행:  python3 path_join.py   (ROS 불필요)
"""
import os
import csv
from math import hypot


def load(p):
    pts = []
    with open(p) as f:
        r = csv.reader(f)
        next(r)                                   # 헤더 skip
        for row in r:
            pts.append((float(row[0]), float(row[1]), float(row[2])))
    return pts


def nearest(pts, target, lo=0, hi=None):
    """pts[lo:hi] 중 target 에 가장 가까운 인덱스와 거리"""
    hi = len(pts) if hi is None else hi
    best, best_d = lo, float('inf')
    for i in range(lo, hi):
        d = hypot(pts[i][0] - target[0], pts[i][1] - target[1])
        if d < best_d:
            best, best_d = i, d
    return best, best_d


def arclen(pts, i, j):
    return sum(hypot(pts[k+1][0] - pts[k][0], pts[k+1][1] - pts[k][1])
               for k in range(i, j))


def main():
    path_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'path')
    approach = load(os.path.join(path_dir, 'approach.csv'))
    course = load(os.path.join(path_dir, 'course.csv'))
    print('approach %d점, course %d점' % (len(approach), len(course)))

    # 1) approach 에서 코스 진입점(course[0])에 가장 가까운 지점까지만 사용
    a_end, a_gap = nearest(approach, course[0])
    dropped = len(approach) - 1 - a_end
    print('  approach: idx %d 에서 절단 (진입점까지 %.2fm) - 지나친 %d점 %.2fm 제거'
          % (a_end, a_gap, dropped, arclen(approach, a_end, len(approach) - 1)))

    # 2) course 뒤쪽에서 출발점(course[0])으로 되돌아온 중복 구간 제거
    #    뒤 10% 구간에서 출발점에 가장 가까운 점을 찾아 거기서 끊는다
    tail_from = int(len(course) * 0.9)
    c_end, c_gap = nearest(course, course[0], lo=tail_from)
    dropped_c = len(course) - 1 - c_end
    print('  course  : idx %d 에서 절단 (출발점까지 %.2fm) - 중복 %d점 %.2fm 제거'
          % (c_end, c_gap, dropped_c, arclen(course, c_end, len(course) - 1)))

    joined = approach[:a_end + 1] + course[:c_end + 1]

    dst = os.path.join(path_dir, 'path.csv')
    with open(dst, 'w') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'z'])
        w.writerows(joined)

    total = arclen(joined, 0, len(joined) - 1)
    print('저장 완료: %s  (%d점, %.1fm)' % (dst, len(joined), total))
    print('  이음매 위치: idx %d  (%.2f, %.2f)'
          % (a_end, joined[a_end][0], joined[a_end][1]))


if __name__ == '__main__':
    main()
