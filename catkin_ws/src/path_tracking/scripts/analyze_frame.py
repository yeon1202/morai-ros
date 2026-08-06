#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""frame_gps/frame_ego 로그로 두 좌표계의 대응관계를 찾는다. 의존성 없음."""
import csv
import math
import sys

LOGDIR = sys.argv[1] if len(sys.argv) > 1 else '/home/yeon/morai-ros/catkin_ws/logs'
TAG = sys.argv[2] if len(sys.argv) > 2 else 'frame'
R_EARTH = 6378137.0


def load(path, cols):
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            out.append(tuple(float(row[c]) for c in cols))
    return out


gps = load('%s/frame_gps_%s.csv' % (LOGDIR, TAG), ['t', 'lat', 'lon', 'alt'])
ego = load('%s/frame_ego_%s.csv' % (LOGDIR, TAG),
           ['t', 'x', 'y', 'z', 'heading_deg', 'vel_x', 'vel_y'])
gps.sort(key=lambda r: r[0])
ego.sort(key=lambda r: r[0])

print('=' * 66)
print('1) 기본')
print('=' * 66)
dur = ego[-1][0] - ego[0][0]
print('주행시간        %.1f s (%.1f 분)' % (dur, dur / 60.0))
print('ego  %5d 행  %.1f Hz' % (len(ego), (len(ego) - 1) / (ego[-1][0] - ego[0][0])))
print('gps  %5d 행  %.1f Hz' % (len(gps), (len(gps) - 1) / (gps[-1][0] - gps[0][0])))

# GPS 실제 갱신 주기: 같은 좌표가 반복되면 그건 새 측정이 아니다
uniq = [g for i, g in enumerate(gps) if i == 0 or (g[1], g[2]) != (gps[i - 1][1], gps[i - 1][2])]
print('gps  %5d 행  %.1f Hz  <- 좌표가 실제로 바뀐 것만' % (
    len(uniq), (len(uniq) - 1) / (uniq[-1][0] - uniq[0][0])))

lats = sorted(set(g[1] for g in gps))
steps = [b - a for a, b in zip(lats, lats[1:]) if b - a > 1e-12]
if steps:
    q = min(steps)
    print('lat 최소 변화량 %.3e deg  ≈ %.3f m  <- 양자화 단위' % (q, q * R_EARTH * math.pi / 180.0))
print('alt 범위        %.3f ~ %.3f m   (ego z: %.3f ~ %.3f m)' % (
    min(g[3] for g in gps), max(g[3] for g in gps),
    min(e[3] for e in ego), max(e[3] for e in ego)))

dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(ego, ego[1:]))
print('주행거리        %.1f m' % dist)
print('시작 ego        (%.2f, %.2f)' % (ego[0][1], ego[0][2]))
print('끝   ego        (%.2f, %.2f)' % (ego[-1][1], ego[-1][2]))


# ---- ego 를 GPS 시각으로 보간 ----
def interp(t):
    lo, hi = 0, len(ego) - 1
    if t < ego[0][0] or t > ego[-1][0]:
        return None
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ego[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    a, b = ego[lo], ego[hi]
    dt = b[0] - a[0]
    if dt <= 0 or dt > 0.5:
        return None
    w = (t - a[0]) / dt
    x = a[1] + w * (b[1] - a[1])
    y = a[2] + w * (b[2] - a[2])
    # heading 은 각도라 선형보간하면 359->1 에서 깨진다. sin/cos 로 돌린다
    ha, hb = math.radians(a[4]), math.radians(b[4])
    s = (1 - w) * math.sin(ha) + w * math.sin(hb)
    c = (1 - w) * math.cos(ha) + w * math.cos(hb)
    v = math.hypot(a[5], b[5])
    return x, y, math.atan2(s, c), v


pairs = []
for t, lat, lon, alt in uniq:
    e = interp(t)
    if e is not None:
        pairs.append((lat, lon, e[0], e[1], e[2], e[3]))
print('짝지어진 표본    %d 개' % len(pairs))

# ---- 위경도 -> 평면 (localization 팀과 동일한 구면근사, 원점은 표본 평균) ----
lat0 = sum(p[0] for p in pairs) / len(pairs)
lon0 = sum(p[1] for p in pairs) / len(pairs)
cos0 = math.cos(math.radians(lat0))
src = [((math.radians(p[1] - lon0)) * cos0 * R_EARTH,
        (math.radians(p[0] - lat0)) * R_EARTH) for p in pairs]
dst = [(p[2], p[3]) for p in pairs]


def rms(res):
    return math.sqrt(sum(dx * dx + dy * dy for dx, dy in res) / len(res))


def fit(allow_rot_scale):
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
print('=' * 66)
print('2) 두 좌표계 맞추기')
print('=' * 66)
_, _, t0, res0 = fit(False)
print('[평행이동만]   offset=(%.3f, %.3f) m   잔차 RMS = %.3f m   최대 %.3f m' % (
    t0[0], t0[1], rms(res0), max(math.hypot(*r) for r in res0)))
th, s, t1, res1 = fit(True)
print('[회전+축척+이동]')
print('   회전  %+.4f deg' % math.degrees(th))
print('   축척  %.7f   (1 에서 %+.1f ppm, 1km 당 %+.2f m)' % (s, (s - 1) * 1e6, (s - 1) * 1000))
print('   이동  (%.3f, %.3f) m' % (t1[0], t1[1]))
print('   잔차 RMS = %.3f m   최대 %.3f m' % (rms(res1), max(math.hypot(*r) for r in res1)))

# ---- 잔차가 차량 heading 과 함께 도는가 = GPS 안테나 장착 오프셋 ----
print()
print('=' * 66)
print('3) 잔차를 차체 좌표로 돌려보기 (GPS 안테나 장착 위치 확인)')
print('=' * 66)
fwd = lat_ = 0.0
for (dx, dy), p in zip(res1, pairs):
    c, sn = math.cos(p[4]), math.sin(p[4])
    fwd += dx * c + dy * sn
    lat_ += -dx * sn + dy * c
fwd /= len(res1)
lat_ /= len(res1)
print('잔차 평균(차체기준)  전방 %+.3f m / 좌측 %+.3f m' % (fwd, lat_))
bias = math.hypot(fwd, lat_)
print('  -> 크기 %.3f m  (잔차 RMS %.3f m 의 %.0f%%)' % (bias, rms(res1), 100 * bias / rms(res1)))

# 속도 구간별 잔차 = 시각 동기 오차인지 보는 지표
slow = [r for r, p in zip(res1, pairs) if p[5] < 5.0]
fast = [r for r, p in zip(res1, pairs) if p[5] > 30.0]
if slow:
    print('정지/저속(<5)  표본 %4d  잔차 RMS %.3f m' % (len(slow), rms(slow)))
if fast:
    print('고속(>30)      표본 %4d  잔차 RMS %.3f m' % (len(fast), rms(fast)))

# ---- 안테나 오프셋을 빼면 얼마나 남나 ----
res2 = []
for (dx, dy), p in zip(res1, pairs):
    c, sn = math.cos(p[4]), math.sin(p[4])
    res2.append((dx - (fwd * c - lat_ * sn), dy - (fwd * sn + lat_ * c)))
print()
print('=' * 66)
print('4) 안테나 오프셋까지 보정하면')
print('=' * 66)
print('잔차 RMS = %.3f m   최대 %.3f m' % (rms(res2), max(math.hypot(*r) for r in res2)))
print('(GPS 양자화 %.3f m 단독으로 만드는 RMS ≈ %.3f m)' % (0.185, 0.185 / math.sqrt(12) * math.sqrt(2)))

# ---- 이론값 대조: UTM 52N 격자수렴각 / 타원체 반지름 ----
print()
print('=' * 66)
print('5) 이론값 대조')
print('=' * 66)
phi = math.radians(lat0)
gamma = math.degrees(math.atan(math.tan(math.radians(lon0 - 129.0)) * math.sin(phi)))
print('UTM 52N 격자수렴각(중앙자오선 129E, lon=%.4f)  %+.4f deg' % (lon0, gamma))
print('  측정된 회전                                  %+.4f deg' % math.degrees(th))
e2 = 0.00669437999014
M = 6378137.0 * (1 - e2) / (1 - e2 * math.sin(phi) ** 2) ** 1.5
N = 6378137.0 / math.sqrt(1 - e2 * math.sin(phi) ** 2)
print('타원체 자오선반지름 M = %.0f m  -> 구형근사 대비 %.6f (%.0f ppm)'
      % (M, M / 6378137.0, (M / 6378137.0 - 1) * 1e6))
print('타원체 묘유선반지름 N = %.0f m  -> 구형근사 대비 %.6f (%.0f ppm)'
      % (N, N / 6378137.0, (N / 6378137.0 - 1) * 1e6))
print('  측정된 축척                                  %.6f (%.0f ppm)' % (s, (s - 1) * 1e6))
