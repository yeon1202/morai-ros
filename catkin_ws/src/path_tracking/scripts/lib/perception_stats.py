# -*- coding: utf-8 -*-
"""/Object_topic 에 실린 인지 결과의 품질을 수치로 만든다.

rospy 를 쓰지 않는 순수 함수다 (lib/object_convert.py, acc_core.hpp 와 같은 이유).
시뮬도 ROS 마스터도 없이 테스트할 수 있어야 대회 당일 회귀를 빨리 잡는다.

무엇을 재는가 - 인지의 세 가지 실패 모드에 하나씩 대응한다:
  오분류    type_detections / type_uids. 정적장애물이 NPC(type 1)로 오면 여기 보인다.
  깜빡임    tracks[].gaps. 같은 unique_id 가 사라졌다 다시 나타난 횟수.
  누락      empty_frames. 인지가 한 물체도 못 낸 프레임 수.
  오검출    off_road. 전역경로에서 차로 반폭보다 멀리 떨어진 채로만 살다 간 물체.

⚠️ 여기서 "오검출" 은 후보일 뿐 판정이 아니다. 도로 밖에 진짜 물체가 있을 수도
   있다(가드레일, 표지판). 게이트 임계값을 정하기 위한 재료로만 쓴다.
"""
import math
from collections import namedtuple

# 한 프레임에서 본 물체 하나. 노드가 /Object_topic 을 풀어서 이 모양으로 넘긴다.
#   frame : 프레임 일련번호 (0부터). 같은 프레임의 물체들은 같은 번호를 갖는다
#   t     : 그 프레임의 시각 [s]
#   uid   : unique_id (tracking_node 가 붙인다)
#   sx,sy : 바운딩박스 크기 [m]
#   speed : 속도 스칼라 [m/s]  (/Object_topic 은 km/h 라 노드가 바꿔서 넘긴다)
Detection = namedtuple('Detection', 'frame t uid type x y sx sy speed')

# 차로 반폭 [m]. lattice_planner.cpp 의 LANE_WIDTH = 3.51 의 절반이다.
LANE_HALF = 1.755

# 크기 이상치 기준 [m]. 대회 NPC 에는 버스도 있어 넉넉히 잡았다 - 여기 걸리는 건
# "차량으로 설명이 안 되는 크기" 라는 뜻이다.
SIZE_MAX = 15.0
SIZE_MIN = 0.2

# uid 하나당 경로 투영을 몇 번까지 할지. 전 프레임을 다 투영하면 4400점 경로에
# 대해 수만 번이 되어 종료 요약이 몇 분씩 걸린다. 생존구간에 고르게 뿌린
# 표본이면 "도로 위였나" 를 가리는 데 충분하다.
SAMPLES_PER_UID = 20

# 투영 총 횟수 상한. 인지가 깜빡이면 unique_id 가 계속 새로 생겨 물체 수가 수백~수천
# 개가 된다. uid 당 20표본을 고집하면 종료 요약이 30초까지 걸리는데(실측), Ctrl+C
# 뒤에 그만큼 멈춰 있으면 못 쓰고 roslaunch 로 띄웠을 땐 요약을 찍기 전에 강제
# 종료될 수도 있다. 물체가 많아지면 물체당 표본을 줄여 총량을 묶는다.
# 실측 1.4ms/투영 기준으로 3000회 ~= 4초.
PROJECTION_BUDGET = 3000
MIN_SAMPLES_PER_UID = 2   # 아무리 많아도 처음과 끝은 본다


def project_to_path(path, x, y):
    """점을 경로(폴리라인)에 투영해 (s, d) 를 낸다. 점이 2개 미만이면 None.

    acc_core.hpp 의 projectToPath 와 같은 정의를 쓴다 - 진단이 planning 과 다른
    자를 쓰면 비교가 안 된다.
      s  경로 시작부터의 종방향 거리 [m]
      d  경로에서의 횡방향 거리 [m], **부호 없음**
    선분 밖으로 떨어지는 점은 끝점으로 clamp 한다(경로 앞뒤로 벗어난 물체도
    거리가 발산하지 않고 끝점까지의 거리로 나온다).
    """
    if len(path) < 2:
        return None

    best_d = float('inf')
    best_s = 0.0
    s_acc = 0.0
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        vx, vy = bx - ax, by - ay
        len2 = vx * vx + vy * vy
        seg_len = math.sqrt(len2)
        if len2 < 1e-12:
            t, dist = 0.0, math.hypot(x - ax, y - ay)
        else:
            t = ((x - ax) * vx + (y - ay) * vy) / len2
            t = max(0.0, min(1.0, t))          # 선분 밖이면 끝점으로
            dist = math.hypot(x - (ax + t * vx), y - (ay + t * vy))
        if dist < best_d:
            best_d = dist
            best_s = s_acc + t * seg_len
        s_acc += seg_len
    return best_s, best_d


def _sample(values, k):
    """리스트에서 최대 k 개를 고르게 뽑는다 (양 끝 포함)."""
    n = len(values)
    if n <= k:
        return values
    step = (n - 1) / float(k - 1)
    return [values[int(round(i * step))] for i in range(k)]


def _median(v):
    if not v:
        return float('nan')
    s = sorted(v)
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


def summarize(dets, path, lane_half=LANE_HALF, n_frames_total=None,
              samples_per_uid=SAMPLES_PER_UID, projection_budget=PROJECTION_BUDGET,
              size_max=SIZE_MAX, size_min=SIZE_MIN):
    """검출 목록을 요약한다.

    n_frames_total 을 주면 검출이 하나도 없던 프레임까지 세어 넣는다. 안 주면
    검출이 있었던 프레임만 센다 - 그러면 "인지가 아무것도 못 본 구간" 이 통계에서
    통째로 사라져 평균이 실제보다 좋아 보인다.
    """
    # 프레임별 개수
    per_frame = {}
    for d in dets:
        per_frame[d.frame] = per_frame.get(d.frame, 0) + 1

    n_frames = n_frames_total if n_frames_total is not None else len(per_frame)
    empty_frames = max(0, n_frames - len(per_frame))
    counts = list(per_frame.values()) + [0] * empty_frames

    # uid 별로 모은다
    by_uid = {}
    for d in dets:
        by_uid.setdefault(d.uid, []).append(d)

    # 물체가 많으면 물체당 표본을 줄여 총 투영 횟수를 예산 안에 묶는다.
    n_uid = len(by_uid)
    if n_uid:
        samples_per_uid = max(MIN_SAMPLES_PER_UID,
                              min(samples_per_uid, projection_budget // n_uid))

    tracks = []
    off_road = []
    size_outliers = []
    for uid in sorted(by_uid):
        seq = sorted(by_uid[uid], key=lambda d: d.frame)
        frames = [d.frame for d in seq]

        # 깜빡임: 프레임 번호가 1 보다 크게 뛴 횟수. 프레임 5개를 건너뛰어도
        # "한 번 끊겼다" 로 센다 - 끊긴 길이는 life 와 n 으로 따로 보인다.
        gaps = sum(1 for a, b in zip(frames, frames[1:]) if b - a > 1)

        ds = []
        for d in _sample(seq, samples_per_uid):
            pr = project_to_path(path, d.x, d.y)
            if pr is not None:
                ds.append(pr[1])

        d_min = min(ds) if ds else float('nan')
        tracks.append({
            'uid': uid,
            'type': seq[-1].type,          # 마지막 판정. 도중에 바뀌면 아래 type_flips
            'n': len(seq),
            'life': seq[-1].t - seq[0].t,
            'gaps': gaps,
            'type_flips': len({d.type for d in seq}) - 1,
            'd_min': d_min,
            'd_med': _median(ds),
            'd_max': max(ds) if ds else float('nan'),
            'speed_max': max(d.speed for d in seq),
        })

        # 오검출 후보: 사는 내내 도로에서 멀었던 것. 한 번이라도 도로 위였으면
        # (d_min <= lane_half) 실제 물체일 가능성이 있으니 빼지 않는다.
        if ds and d_min > lane_half:
            off_road.append(uid)

        # 크기 이상치. 두 판정 다 "생존구간 최대 변" 하나로 본다:
        #   너무 큼   한 번이라도 size_max 를 넘었다 (그 프레임이 곧 이상이다)
        #   너무 작음 끝까지 size_min 을 못 넘었다 (잠깐 작게 보이는 건 정상이라
        #             최대값으로 봐야 한 프레임짜리 흔들림에 안 속는다)
        max_side = max(max(d.sx, d.sy) for d in seq)
        if max_side > size_max or max_side < size_min:
            size_outliers.append(uid)

    # type 분포. 검출 건수와 물체 수를 따로 낸다 - 오래 살아남은 오분류 하나가
    # 건수로는 수백 건이라 물체 수로도 봐야 규모를 안 속는다.
    type_detections = {}
    for d in dets:
        type_detections[d.type] = type_detections.get(d.type, 0) + 1
    type_uids = {}
    for t in tracks:
        type_uids[t['type']] = type_uids.get(t['type'], 0) + 1

    return {
        'n_frames': n_frames,
        'empty_frames': empty_frames,
        'duration': (max(d.t for d in dets) - min(d.t for d in dets)) if dets else 0.0,
        'per_frame': {
            'mean': (sum(counts) / float(len(counts))) if counts else 0.0,
            'max': max(counts) if counts else 0,
            'min': min(counts) if counts else 0,
        },
        'type_detections': type_detections,
        'type_uids': type_uids,
        'tracks': tracks,
        'off_road': off_road,
        'size_outliers': size_outliers,
    }
