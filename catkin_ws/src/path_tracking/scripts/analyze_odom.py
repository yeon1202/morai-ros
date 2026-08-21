#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_odom : diag_odom.py 가 남긴 두 CSV 를 겹쳐 /odom 이 MORAI 맵 프레임과
               같은지 판정한다.

analyze_frame.py 와 같은 방식이지만 대상이 다르다. 거기서는 GPS 위경도를 평면으로
펴서 비교했고, 여기서는 /odom 이 이미 미터 좌표라 바로 비교한다.

가장 중요한 숫자는 **1) 보정 없는 잔차** 다. planning 은 /odom 을 그대로 쓸
것이므로, 맞추기를 허용한 2)·3) 이 아무리 좋아도 1) 이 크면 못 쓴다.
2)·3) 은 "왜 어긋나는가" 를 알려주는 진단이지 합격 기준이 아니다.

판정 기준 (차로 폭 3.51m, 차로 안 편도 여유 0.809m, MAX_CTE 6.0m)
  RMS < 0.30m   그대로 전환 가능
  RMS < 0.80m   전환은 되지만 회피 여유(1.4m)를 갉아먹는다. 원인을 봐야 한다
  RMS > 0.80m   전환 불가

사용법
  python3 analyze_odom.py                 # logs/odom_frame.csv 기본
  python3 analyze_odom.py lap2            # logs/odom_lap2.csv
"""
import csv
import math
import os
import sys

TAG = sys.argv[1] if len(sys.argv) > 1 else 'frame'
LOGS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    '..', '..', 'logs')
LOGS = os.path.normpath(os.environ.get('LOG_DIR', LOGS))


def load(name, cols):
    path = os.path.join(LOGS, name)
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            out.append(tuple(float(row[c]) for c in cols))
    return out


odom = load('odom_%s.csv' % TAG,    ['t', 'x', 'y', 'yaw_deg', 'vx', 'vy'])
ego  = load('odomego_%s.csv' % TAG, ['t', 'x', 'y', 'heading_deg', 'vel_x', 'vel_y'])
print('표본  /odom %d줄  /ego_status %d줄' % (len(odom), len(ego)))
if len(odom) < 50 or len(ego) < 50:
    print('표본이 너무 적다. 주행을 더 길게 기록할 것.')
    sys.exit(1)

# ---- 촘촘한 쪽을 성긴 쪽 시각으로 보간한다 ----
# 반대로 하면 보간 구간이 길어져 그만큼 오차가 섞인다. 어느 쪽이 빠른지는
# 실행마다 다르므로(부하에 따라 EKF 가 밀린다) 표본 수를 보고 정한다.
def interp_on(series, t):
    if t < series[0][0] or t > series[-1][0]:
        return None
    lo, hi = 0, len(series) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    a, b = series[lo], series[hi]
    dt = b[0] - a[0]
    if dt <= 0:
        return a
    r = (t - a[0]) / dt
    return tuple(a[i] + (b[i] - a[i]) * r for i in range(len(a)))


# 구간 제외 (충돌 등 교란 구간). 기록 시작으로부터의 경과초로 준다.
#   python3 analyze_odom.py frame --exclude 54,62
EXCLUDE = []
for i, a in enumerate(sys.argv):
    if a == '--exclude' and i + 1 < len(sys.argv):
        lo, hi = sys.argv[i + 1].split(',')
        EXCLUDE.append((float(lo), float(hi)))

T0 = min(odom[0][0], ego[0][0])
if len(ego) >= len(odom):
    base, other, ego_is_base = odom, ego, False
else:
    base, other, ego_is_base = ego, odom, True
print('보간: %s 를 %s 시각으로 (촘촘한 쪽 -> 성긴 쪽)'
      % ('ego' if not ego_is_base else 'odom', 'odom' if not ego_is_base else 'ego'))

pairs, dropped = [], 0
for b in base:
    t = b[0]
    if any(lo <= t - T0 <= hi for lo, hi in EXCLUDE):
        dropped += 1
        continue
    o = interp_on(other, t)
    if o is None:
        continue
    pairs.append((b, o) if ego_is_base else (b, o))
# pairs 를 (odom, ego) 순서로 통일한다
if ego_is_base:
    pairs = [(o, e) for e, o in pairs]
print('짝지어진 표본 %d개' % len(pairs), end='')
print('  (제외 %d개)' % dropped if dropped else '')

src = [(o[1], o[2]) for o, e in pairs]      # odom  (x, y)
dst = [(e[1], e[2]) for o, e in pairs]      # ego   (x, y) = MORAI 맵 좌표 = 정답


def rms(res):
    return math.sqrt(sum(x * x + y * y for x, y in res) / len(res))


def worst(res):
    return max(math.hypot(x, y) for x, y in res)


print()
print('=' * 68)
print('1) 보정 없이 그대로 겹치기  ← planning 이 실제로 겪을 오차')
print('=' * 68)
res_raw = [(d[0] - s[0], d[1] - s[1]) for s, d in zip(src, dst)]
mx = sum(r[0] for r in res_raw) / len(res_raw)
my = sum(r[1] for r in res_raw) / len(res_raw)
print('  평균 어긋남 (%+.3f, %+.3f) m   크기 %.3f m' % (mx, my, math.hypot(mx, my)))
print('  잔차 RMS %.3f m   최대 %.3f m' % (rms(res_raw), worst(res_raw)))
r = rms(res_raw)
verdict = ('전환 가능' if r < 0.30 else
           '조건부 - 회피 여유를 갉아먹는다' if r < 0.80 else '전환 불가')
print('  판정: %s' % verdict)


def fit(allow_rot_scale):
    """analyze_frame.py 의 것과 같은 Umeyama 유사변환."""
    n = len(src)
    px = sum(p[0] for p in src) / n
    py = sum(p[1] for p in src) / n
    qx = sum(q[0] for q in dst) / n
    qy = sum(q[1] for q in dst) / n
    if not allow_rot_scale:
        res = [(q[0] - (p[0] + qx - px), q[1] - (p[1] + qy - py)) for p, q in zip(src, dst)]
        return 0.0, 1.0, (qx - px, qy - py), res
    sdot = scross = snorm = 0.0
    for p, q in zip(src, dst):
        ax, ay = p[0] - px, p[1] - py
        bx, by = q[0] - qx, q[1] - qy
        sdot += ax * bx + ay * by
        scross += ax * by - ay * bx
        snorm += ax * ax + ay * ay
    th = math.atan2(scross, sdot)
    s = (sdot * math.cos(th) + scross * math.sin(th)) / snorm
    ct, st = math.cos(th) * s, math.sin(th) * s
    tx = qx - (ct * px - st * py)
    ty = qy - (st * px + ct * py)
    res = [(q[0] - (ct * p[0] - st * p[1] + tx),
            q[1] - (st * p[0] + ct * p[1] + ty)) for p, q in zip(src, dst)]
    return th, s, (tx, ty), res


print()
print('=' * 68)
print('2) 왜 어긋나는가 (진단용 - 합격 기준 아님)')
print('=' * 68)
_, _, t0, res0 = fit(False)
print('  평행이동만  offset=(%+.3f, %+.3f)   잔차 RMS %.3f m  최대 %.3f m'
      % (t0[0], t0[1], rms(res0), worst(res0)))
th, s, _, res1 = fit(True)
print('  회전+축척   회전 %+.4f°  축척 %.6f (%+.0f ppm)' % (
    math.degrees(th), s, (s - 1.0) * 1e6))
print('              잔차 RMS %.3f m  최대 %.3f m' % (rms(res1), worst(res1)))
if abs(math.degrees(th)) > 0.1 or abs(s - 1.0) * 1e6 > 500:
    print('  ⚠️ 회전이나 축척이 남아 있다. 투영 방식이 아직 다르다는 뜻이다')
    print('     (2026-08-03 구면근사 때: 회전 -1.2595°, 축척 -2837 ppm)')

print()
print('=' * 68)
print('3) 헤딩과 속도')
print('=' * 68)
dh = []
for o, e in pairs:
    d = (o[3] - e[3] + 180.0) % 360.0 - 180.0
    dh.append(d)
mh = sum(dh) / len(dh)
sd = math.sqrt(sum((d - mh) ** 2 for d in dh) / len(dh))
print('  헤딩차 (odom - ego) 평균 %+.2f°  표준편차 %.2f°  최대 %.2f°'
      % (mh, sd, max(abs(d) for d in dh)))
if abs(mh) > 2.0:
    print('  ⚠️ 상수 오프셋이 있다. 두 헤딩 정의가 다를 수 있다')

dv = []
for o, e in pairs:
    v_odom = math.hypot(o[4], o[5])            # /odom twist 는 m/s
    v_ego = math.hypot(e[4], e[5]) / 3.6       # /ego_status 는 km/h
    dv.append(v_odom - v_ego)
mv = sum(dv) / len(dv)
print('  속도차 (odom - ego) 평균 %+.3f m/s  최대 %+.3f m/s'
      % (mv, max(dv, key=abs)))

# 속도 구간별 잔차 - 크게 갈리면 프레임이 아니라 시각 동기 문제다
slow = [r for (o, e), r in zip(pairs, res_raw) if math.hypot(e[4], e[5]) / 3.6 < 5.0]
fast = [r for (o, e), r in zip(pairs, res_raw) if math.hypot(e[4], e[5]) / 3.6 > 8.0]
print()
if slow:
    print('  저속(<5m/s)  표본 %4d  잔차 RMS %.3f m' % (len(slow), rms(slow)))
if fast:
    print('  고속(>8m/s)  표본 %4d  잔차 RMS %.3f m' % (len(fast), rms(fast)))
if slow and fast and rms(fast) > rms(slow) * 2.0:
    print('  ⚠️ 속도에 비례해 커진다 = 프레임 문제가 아니라 시각 지연일 수 있다')
