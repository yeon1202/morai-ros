#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""perception_stats 단위 테스트. ROS 도 시뮬도 없이 돈다 (순수 파이썬).

test_object_convert.py 와 같은 이유로 분리했다 - 대회 당일 회귀를 몇 초 만에
돌릴 수 있어야 한다.

실행:
  rosrun path_tracking test_perception_stats.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lib.perception_stats import Detection, project_to_path, summarize

# 테스트용 직선 경로: (0,0) -> (10,0) -> (20,0)
STRAIGHT = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)]


def det(frame, uid, x, y, t=None, type_=2, sx=1.0, sy=1.0, speed=0.0):
    """t 를 안 주면 프레임당 0.05초(20Hz)로 친다."""
    return Detection(frame=frame, t=frame * 0.05 if t is None else t,
                     uid=uid, type=type_, x=x, y=y, sx=sx, sy=sy, speed=speed)


def check(name, cond):
    print(('  PASS  ' if cond else '  FAIL  ') + name)
    return cond


def near(a, b, tol=1e-6):
    return abs(a - b) < tol


def main():
    ok = True

    # --- project_to_path -----------------------------------------------------
    s, d = project_to_path(STRAIGHT, 3.0, 2.0)
    ok &= check('직선 위 투영: s=3, d=2', near(s, 3.0) and near(d, 2.0))

    s, d = project_to_path(STRAIGHT, 15.0, -1.5)
    ok &= check('두 번째 구간에서도 s 가 누적된다 (s=15)', near(s, 15.0) and near(d, 1.5))

    s, d = project_to_path(STRAIGHT, 25.0, 0.0)
    ok &= check('경로 끝 너머는 끝점으로 clamp (s=20, d=5)',
                near(s, 20.0) and near(d, 5.0))

    s, d = project_to_path(STRAIGHT, -4.0, 3.0)
    ok &= check('경로 시작 이전도 시작점으로 clamp (s=0, d=5)',
                near(s, 0.0) and near(d, 5.0))

    ok &= check('점이 2개 미만이면 None', project_to_path([(0.0, 0.0)], 1.0, 1.0) is None)

    ok &= check('d 는 부호가 없다 (좌우 같은 값)',
                near(project_to_path(STRAIGHT, 5.0, 2.0)[1],
                     project_to_path(STRAIGHT, 5.0, -2.0)[1]))

    # --- summarize: 프레임당 개수 --------------------------------------------
    dets = [det(0, 1, 5.0, 0.0), det(0, 2, 6.0, 0.0),
            det(1, 1, 5.1, 0.0),
            det(2, 1, 5.2, 0.0), det(2, 2, 6.2, 0.0), det(2, 3, 7.0, 0.0)]
    r = summarize(dets, STRAIGHT)
    ok &= check('프레임 수 3', r['n_frames'] == 3)
    ok &= check('프레임당 최대 3개', r['per_frame']['max'] == 3)
    ok &= check('프레임당 평균 2.0', near(r['per_frame']['mean'], 2.0))

    # 검출이 하나도 없는 프레임은 dets 만 봐서는 안 보인다. 그게 곧 "인지가
    # 아무것도 못 본 구간"(누락)이라 노드가 총 프레임 수를 따로 알려준다.
    r = summarize(dets, STRAIGHT, n_frames_total=5)
    ok &= check('빈 프레임을 알려주면 프레임 수 5', r['n_frames'] == 5)
    ok &= check('빈 프레임 2개', r['empty_frames'] == 2)
    ok &= check('빈 프레임을 0 으로 친 평균 1.2', near(r['per_frame']['mean'], 1.2))
    ok &= check('빈 프레임이 있으면 최소 0', r['per_frame']['min'] == 0)

    # --- summarize: 깜빡임 ---------------------------------------------------
    # uid 9 는 프레임 0,1,3,4 에 있다 -> 2 번 프레임에서 한 번 끊겼다.
    dets = [det(f, 9, 5.0, 0.0) for f in (0, 1, 3, 4)]
    r = summarize(dets, STRAIGHT)
    trk = r['tracks'][0]
    ok &= check('끊김 1회', trk['gaps'] == 1)
    ok &= check('검출 4회', trk['n'] == 4)
    ok &= check('생존시간 = 마지막-처음 (0.2초)', near(trk['life'], 0.2))

    dets = [det(f, 9, 5.0, 0.0) for f in (0, 1, 2)]
    ok &= check('연속이면 끊김 0회', summarize(dets, STRAIGHT)['tracks'][0]['gaps'] == 0)

    # --- summarize: 도로 밖 (오검출 후보) ------------------------------------
    # lane_half=1.755 (LANE_WIDTH 3.51 의 절반)
    dets = ([det(f, 1, 5.0, 0.5) for f in range(3)] +       # 도로 위
            [det(f, 2, 5.0, 9.0) for f in range(3)])        # 한 번도 도로 근처가 아니다
    r = summarize(dets, STRAIGHT)
    ok &= check('도로 밖 uid 만 걸린다', r['off_road'] == [2])

    # 잠깐이라도 도로 위였으면 오검출로 몰지 않는다 (d_min 으로 판정)
    dets = [det(0, 3, 5.0, 0.5), det(1, 3, 5.0, 9.0), det(2, 3, 5.0, 9.0)]
    ok &= check('한 번이라도 도로 위면 제외', summarize(dets, STRAIGHT)['off_road'] == [])

    # --- summarize: type 분포 -------------------------------------------------
    dets = [det(0, 1, 5.0, 0.0, type_=1), det(1, 1, 5.0, 0.0, type_=1),
            det(0, 2, 6.0, 0.0, type_=2)]
    r = summarize(dets, STRAIGHT)
    ok &= check('type 별 검출 수 (1:2건, 2:1건)',
                r['type_detections'][1] == 2 and r['type_detections'][2] == 1)
    ok &= check('type 별 물체 수 (각 1개)',
                r['type_uids'][1] == 1 and r['type_uids'][2] == 1)

    # --- summarize: 크기 이상치 ----------------------------------------------
    dets = [det(0, 1, 5.0, 0.0, sx=4.0, sy=1.8),    # 정상 (승용차)
            det(0, 2, 6.0, 0.0, sx=30.0, sy=2.0),   # 너무 큼
            det(0, 3, 7.0, 0.0, sx=0.05, sy=0.05)]  # 너무 작음
    r = summarize(dets, STRAIGHT)
    ok &= check('크기 이상치 2개 (너무 큼/너무 작음)', sorted(r['size_outliers']) == [2, 3])

    # --- summarize: 투영 예산 ---------------------------------------------
    # 인지가 깜빡여 물체가 수백 개가 되어도 종료 요약이 오래 걸리면 안 된다.
    # 투영 횟수를 세어 예산 안에 묶이는지 직접 확인한다.
    import lib.perception_stats as ps
    calls = [0]
    orig = ps.project_to_path

    def counting(path, x, y):
        calls[0] += 1
        return orig(path, x, y)

    ps.project_to_path = counting
    try:
        dets = [det(f, uid, 5.0, 0.0) for uid in range(100) for f in range(50)]
        ps.summarize(dets, STRAIGHT, projection_budget=300)
    finally:
        ps.project_to_path = orig
    ok &= check('투영이 예산(300)을 넘지 않는다', calls[0] <= 300)
    ok &= check('물체가 많아도 물체당 최소 2표본', calls[0] >= 2 * 100)

    # --- summarize: 빈 입력 ---------------------------------------------------
    r = summarize([], STRAIGHT)
    ok &= check('빈 입력에도 안 죽는다', r['n_frames'] == 0 and r['tracks'] == [])

    print('\n' + ('전부 통과' if ok else '실패 있음'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
