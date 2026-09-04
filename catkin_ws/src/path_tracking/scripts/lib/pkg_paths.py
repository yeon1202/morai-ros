# -*- coding: utf-8 -*-
"""패키지 안의 파일 경로를 레이아웃에 상관없이 찾는다.

왜 필요한가
  같은 코드가 두 레이아웃에서 돈다.

    개발용   catkin_ws/src/path_tracking/scripts/path_tracker.py
             -> 패키지 루트는 두 단계 위 (path_tracking/)
    팀 repo  autonomous_driving/src/planning/scripts/path_tracker.py
             -> 패키지 루트는 네 단계 위 (autonomous_driving/)

  dirname 을 몇 번 부를지 세는 방식은 한쪽에서만 맞다. 실제로 2026-09-04 에
  네 단계로 고쳤다가 양쪽 다 파일을 못 찾는 상태가 됐다.

  세는 대신 **package.xml 이 있는 디렉터리까지 올라간다.** 그게 catkin 이
  말하는 패키지 루트의 정의라서 레이아웃이 바뀌어도 같이 따라간다.
"""
import os

# 전역경로 파일 이름.
#
# path_smooth_closed.csv 를 쓴다 (2026-09-04). 팀 vehicle_control 의
# waypoint_csv 기본값과 같은 파일이라, planning 과 control 이 같은 전역경로를
# 따르게 된다. map/lane_table.csv 도 이 파일에서 만들어졌으므로 road_core 의
# 웨이포인트 인덱스 조회도 정확히 맞는다.
#
# path_smooth.csv 와는 최대 0.241m 떨어져 있다(0.5m 초과 구간 없음).
GLOBAL_PATH_CSV = 'path_smooth_closed.csv'


def package_root(start_file):
    """start_file 이 속한 catkin 패키지의 루트 디렉터리.

    package.xml 을 찾을 때까지 위로 올라간다. 못 찾으면 FileNotFoundError.
    (조용히 엉뚱한 경로를 돌려주면 "파일이 없다" 가 아니라 "빈 경로로 주행"
     같은 더 나쁜 실패가 된다.)
    """
    d = os.path.dirname(os.path.abspath(start_file))
    while True:
        if os.path.exists(os.path.join(d, 'package.xml')):
            return d
        parent = os.path.dirname(d)
        if parent == d:                      # 루트까지 올라갔다
            raise FileNotFoundError(
                'package.xml 을 못 찾았다 (%s 에서 위로 탐색)' % start_file)
        d = parent


def global_path_csv(start_file):
    """전역경로 CSV 의 절대경로. 파일이 없으면 FileNotFoundError."""
    p = os.path.join(package_root(start_file), 'path', GLOBAL_PATH_CSV)
    if not os.path.exists(p):
        raise FileNotFoundError('전역경로 파일이 없다: %s' % p)
    return p
