# -*- coding: utf-8 -*-
"""궤적 CSV 로 "추정이 떠는가 / 차가 실제로 어디를 지났나" 를 재는 순수 계산.

rospy·numpy 를 안 쓴다. 호스트에 numpy 가 없어서(2026-09-01 확인) 순수 파이썬으로
짰다. 4392점 경로 × 1500표본이 몇 초면 끝나므로 충분하다.

무엇을 재는가
  ① lateral_jitter  - "추정이 물리적으로 불가능하게 떠는 정도"
     차는 0.5~1초 안에 옆으로 홱 못 움직인다. 그래서 짧은 창에 2차곡선을 맞추고
     거기서 벗어나는 가로 성분을 보면, 그건 차가 그렇게 움직인 게 아니라
     추정이 떤 것이다. 참값(/ego_status GT)에 같은 잣대를 대면 거의 0 이 나와야
     하고, 안 나오면 잣대나 창 크기가 잘못된 것이다. 반드시 같이 재서 대조할 것.

     실측 (novy1, 창 1.0초): /odom RMS 7.33cm / 95% 3.84cm  vs  GT RMS 0.13cm

  ② cross_track    - "차가 경로(=차선 중심선) 에서 옆으로 얼마나 벗어났나"
     path_smooth.csv 가 MGeo 차선 중심선 위에 0.000m 로 놓여 있음이 확인돼 있어
     (2026-08-29), 이 값이 곧 "차선 중심에서 얼마" 다.

     실측 (novy1, GT 기준): 중앙 16.8cm / 90% 27.6cm / 0.81m 초과 1.77%

⚠️ 최근접점을 "직전 인덱스 ± 창" 으로 좇지 말 것. 창이 경로를 한 번 놓치면 못
   돌아와서 13.5m 같은 값이 나온다(2026-09-01 실제로 당했다). cross_track 은
   전역 탐색이다. 느리면 표본을 솎을 것(step 인자).
"""
import bisect
import csv
import math

# 차체가 차선을 밟기 시작하는 횡오차 [m].
#   (LANE_WIDTH 3.51 - 아이오닉5 차폭 1.89) / 2 = 0.81
# ⚠️ LANE_WIDTH 는 lattice_planner.cpp:38 의 복사본이다. 한쪽만 고치면 조용히 어긋난다.
LANE_TOUCH_MARGIN = 0.81


# ---------------------------------------------------------------- 읽기

def load_xy(path, tcol='t_stamp'):
    """diag_latency 가 쓴 CSV 에서 (시각, x, y) 를 꺼낸다.

    t_arr(도착 벽시계) 가 아니라 t_stamp(메시지 스탬프) 를 쓴다. 궤적의 모양을
    보는 것이므로 "언제의 상태인가" 가 맞다. 도착 주기를 볼 때만 t_arr 를 쓴다.
    """
    ts, xs, ys = [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            ts.append(float(row[tcol])); xs.append(float(row['x'])); ys.append(float(row['y']))
    return ts, xs, ys


def load_arrival(path):
    """도착 벽시계만 꺼낸다 (발행 주기·구멍을 볼 때)."""
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            out.append(float(row['t_arr']))
    return out


def load_path(path_csv):
    """path_smooth.csv (x,y,z 헤더) 를 읽는다."""
    px, py = [], []
    with open(path_csv) as f:
        for row in csv.DictReader(f):
            px.append(float(row['x'])); py.append(float(row['y']))
    return px, py


# ---------------------------------------------------------------- 적합

def quad_fit(tt, vv):
    """v = a + b·t + c·t² 최소제곱. 3x3 정규방정식을 Cramer 로 푼다.

    2차인 이유: 0.5~1초 창이면 실제 차량 궤적은 등가속 곡선으로 충분히 표현된다.
    1차(직선)로 하면 정상적인 선회까지 잔차로 잡혀 GT 도 크게 나온다.
    반환 (a, b, c). b 는 t=0 에서의 속도벡터 성분이라 진행방향을 준다.
    """
    n = len(tt)
    s1 = s2 = s3 = s4 = 0.0
    b0 = b1 = b2 = 0.0
    for t, v in zip(tt, vv):
        t2 = t * t
        s1 += t; s2 += t2; s3 += t2 * t; s4 += t2 * t2
        b0 += v; b1 += v * t; b2 += v * t2
    M = [[n, s1, s2], [s1, s2, s3], [s2, s3, s4]]
    B = [b0, b1, b2]

    def det3(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))

    D = det3(M)
    if abs(D) < 1e-18:          # 표본이 한 점에 몰렸다
        return None
    out = []
    for k in range(3):
        Mk = [r[:] for r in M]
        for i in range(3):
            Mk[i][k] = B[i]
        out.append(det3(Mk) / D)
    return out


def lateral_jitter(ts, xs, ys, win=1.0, min_pts=10, min_speed=0.5):
    """국소 2차 적합에서 벗어나는 '가로' 잔차. [(시각, 잔차[m]), ...]

    win       창 길이[초]. 넓힐수록 실제 선회까지 잔차로 잡혀 GT 도 커진다.
              여러 값으로 재서 GT 가 계속 작게 나오는지 확인할 것(민감도 점검).
    min_pts   창 안 최소 표본. 2차 적합이 계수 3개라 최소 7 은 있어야 안정적이다.
    min_speed 진행방향 벡터 크기의 하한[m/s]. 정지 중에는 방향이 정의가 안 된다.
    """
    res = []
    n = len(ts)
    for i in range(n):
        lo = bisect.bisect_left(ts, ts[i] - win / 2.0)
        hi = bisect.bisect_right(ts, ts[i] + win / 2.0)
        if hi - lo < min_pts:
            continue
        tt = [ts[j] - ts[i] for j in range(lo, hi)]
        if tt[-1] - tt[0] < win * 0.6:   # 구멍 때문에 창이 실제로 안 찼다
            continue
        cx = quad_fit(tt, [xs[j] for j in range(lo, hi)])
        cy = quad_fit(tt, [ys[j] for j in range(lo, hi)])
        if cx is None or cy is None:
            continue
        ex = xs[i] - cx[0]           # t=0 에서의 적합값과 실제값의 차
        ey = ys[i] - cy[0]
        hx, hy = cx[1], cy[1]        # 1차항 = 진행방향
        nrm = math.hypot(hx, hy)
        if nrm < min_speed:
            continue
        # 진행방향에 수직인 성분만 (앞뒤로 밀린 건 여기서 관심 밖이다)
        res.append((ts[i], abs(-ex * hy / nrm + ey * hx / nrm)))
    return res


# ---------------------------------------------------------------- 투영

def cross_track(px, py, qx, qy):
    """경로 전체에서 가장 가까운 선분까지의 부호있는 수직거리.

    반환 (거리[m], 부호있는거리[m], 선분인덱스). 부호 +는 경로 진행방향 기준 왼쪽.
    ⚠️ 전역 탐색이다. 창으로 좁히면 한 번 놓쳤을 때 못 돌아온다(모듈 주석 참고).
    """
    best2 = None
    bi = -1
    bsign = 1.0
    for i in range(len(px) - 1):
        ax, ay = px[i], py[i]
        dx, dy = px[i + 1] - ax, py[i + 1] - ay
        L2 = dx * dx + dy * dy
        if L2 < 1e-12:
            continue
        u = ((qx - ax) * dx + (qy - ay) * dy) / L2
        u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
        ddx = qx - (ax + u * dx)
        ddy = qy - (ay + u * dy)
        d2 = ddx * ddx + ddy * ddy
        if best2 is None or d2 < best2:
            best2 = d2
            bi = i
            bsign = 1.0 if (dx * (qy - ay) - dy * (qx - ax)) > 0 else -1.0
    if best2 is None:
        return None
    d = math.sqrt(best2)
    return d, bsign * d, bi


def cross_track_series(px, py, ts, xs, ys, step=1):
    """궤적 전체를 경로에 투영한다. [(시각, 부호있는거리[m], 선분인덱스), ...]

    step 으로 표본을 솎는다. 전역 탐색이라 표본×경로점 만큼 돌기 때문이다
    (4392점 × 1469표본 ≈ 640만회, 순수 파이썬으로 수 초).
    """
    out = []
    for k in range(0, len(ts), step):
        r = cross_track(px, py, xs[k], ys[k])
        if r is None:
            continue
        out.append((ts[k], r[1], r[2]))
    return out


# ---------------------------------------------------------------- 집계

def summary(values):
    """RMS 와 분위수. 표본이 없으면 None."""
    if not values:
        return None
    v = sorted(values)
    n = len(v)
    return {
        'n': n,
        'rms': math.sqrt(sum(q * q for q in v) / n),
        'p50': v[n // 2],
        'p90': v[int(0.90 * n)],
        'p95': v[int(0.95 * n)],
        'p99': v[int(0.99 * n)],
        'max': v[-1],
    }


def steering_from_lateral(e, wheelbase=3.0, lfd=4.0):
    """횡방향 위치오차 e[m] 가 pure pursuit 에서 만드는 조향 흔들림 [rad].

        δ ≈ 2·L·e / lfd²

    lfd 가 제곱으로 나눗셈에 들어가서 짧게 볼수록 급격히 증폭된다.
    기본값은 path_tracker.py 의 WHEELBASE=3.0, MIN_LFD=4.0 (순항 20km/h 에서는
    LFD_GAIN*v = 2.78 < MIN_LFD 라 lfd 가 4.0 에 고정된다).
    """
    return 2.0 * wheelbase * e / (lfd * lfd)


def rate_report(t_arr, timeout=0.2):
    """도착 주기. 반환 dict(hz, dt_p50, dt_p99, dt_max, gaps) - gaps 는 timeout 초과 횟수."""
    if len(t_arr) < 3:
        return None
    d = sorted(t_arr[i + 1] - t_arr[i] for i in range(len(t_arr) - 1))
    n = len(d)
    dur = t_arr[-1] - t_arr[0]
    return {
        'n': len(t_arr), 'dur': dur, 'hz': len(t_arr) / dur if dur > 0 else 0.0,
        'dt_p50': d[n // 2], 'dt_p99': d[int(0.99 * n)], 'dt_max': d[-1],
        'gaps': sum(1 for v in d if v > timeout),
    }
