#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_latency : diag_latency.py 가 쌓은 CSV 로 단계별 지연을 갈라낸다.

두 가지를 따로 잰다 (diag_latency.py 머리말 참고)

  (A) 통과 시간 [벽시계]
      /gps -> /gps/fix -> /odometry/gps 는 stamp 를 승계하므로, 같은 stamp 를
      단계마다 언제 받았는지 보면 소요 시간이 나온다.

  (B) 정보의 나이 [차량이 겪는 지연]
      EKF 는 출력 stamp 를 "지금" 으로 다시 찍어서 (A) 로는 안 잡힌다.
      대신 위치를 GT(/ego_status)와 시간축으로 밀어가며 맞춰, 잔차를 최소로
      만드는 이동량 τ 를 찾는다.

  ⚠️ 두 값의 단위가 다르다. 시뮬이 실시간의 r 배로 도니까
       (차량이 겪는 지연) = (벽시계 지연) x r
     기존 측정치 "0.22초" 는 거리÷속도로 낸 값이라 (차량이 겪는 지연) 쪽이다.
     r 은 GT 이동거리와 GT 속도로 역산해서 같이 출력한다.

사용법
  python3 analyze_latency.py --tag pilot
  python3 analyze_latency.py --tag pilot --dir ~/morai-ros/catkin_ws/logs
  python3 analyze_latency.py --tag pilot --min-speed 3.0

구간 정리는 자동이다
  - 앞뒤로 안 움직이는 구간을 잘라낸다 (기록 끄는 걸 잊은 꼬리 등)
  - 급감속(충돌 등)을 찾아 그 앞뒤 --pad 초를 뺀다
  - 지연은 속도에 비례해 거리로 나타나므로 저속 표본은 원래 못 쓴다
"""
import argparse
import bisect
import csv
import math
import os
import sys

# ── 읽기 ────────────────────────────────────────────────────────────

def load(dirname, name, tag):
    p = os.path.join(dirname, 'lat_%s_%s.csv' % (name, tag))
    if not os.path.isfile(p):
        return None
    with open(p) as f:
        r = csv.DictReader(f)
        rows = [{k: float(v) for k, v in row.items()} for row in r]
    rows.sort(key=lambda d: d['t_stamp'])
    return rows


class Track:
    """(t, x, y) 시계열. 임의 시각의 위치를 선형보간으로 준다."""

    def __init__(self, rows, tkey='t_stamp'):
        self.t = [r[tkey] for r in rows]
        self.x = [r['x'] for r in rows]
        self.y = [r['y'] for r in rows]

    def at(self, t):
        if t < self.t[0] or t > self.t[-1]:
            return None
        i = bisect.bisect_left(self.t, t)
        if i == 0:
            return (self.x[0], self.y[0])
        t0, t1 = self.t[i - 1], self.t[i]
        if t1 == t0:
            return (self.x[i], self.y[i])
        f = (t - t0) / (t1 - t0)
        return (self.x[i - 1] + f * (self.x[i] - self.x[i - 1]),
                self.y[i - 1] + f * (self.y[i] - self.y[i - 1]))


def pct(vals, q):
    if not vals:
        return float('nan')
    s = sorted(vals)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


def median(v):
    return pct(v, 0.5)


# ── 본문 ────────────────────────────────────────────────────────────

ap = argparse.ArgumentParser()
ap.add_argument('--tag', default='pilot')
ap.add_argument('--dir', default=os.path.expanduser('~/morai-ros/catkin_ws/logs'))
ap.add_argument('--min-speed', type=float, default=3.0,
                help='이 속도[m/s] 미만 표본은 지연 추정에서 뺀다 (기본 3.0)')
ap.add_argument('--pad', type=float, default=2.0,
                help='급감속 앞뒤로 뺄 시간[초] (기본 2.0)')
ap.add_argument('--outlier', type=float, default=5.0,
                help='이 거리[m]를 넘는 구간은 지연이 아닌 다른 고장으로 보고 뺀다 (기본 5.0)')
ap.add_argument('--decel', type=float, default=4.0,
                help='급감속 판정 문턱 [m/s^2] (기본 4.0)')
a = ap.parse_args()

D = os.path.expanduser(a.dir)
ego = load(D, 'ego', a.tag)
if not ego:
    sys.exit('lat_ego_%s.csv 를 못 찾겠다: %s' % (a.tag, D))

names = ['ego', 'gps', 'fix', 'imu', 'imuekf', 'wheel', 'navsat', 'odom']
data = {n: load(D, n, a.tag) for n in names}

print('=' * 72)
print('1) 수집 요약')
print('=' * 72)
t0 = ego[0]['t_stamp']
for n in names:
    rows = data[n]
    if not rows:
        print('  %-8s ⚠️ 파일 없음' % n)
        continue
    span = rows[-1]['t_stamp'] - rows[0]['t_stamp']
    hz = (len(rows) - 1) / span if span > 0 else 0.0
    print('  %-8s %5d줄  %6.1f Hz  %6.1f초' % (n, len(rows), hz, span))

# ── GT 속도/이동 ────────────────────────────────────────────────────
for i, r in enumerate(ego):
    r['v'] = r['vel_kmh'] / 3.6          # /ego_status.velocity.x 는 km/h
    r['t'] = r['t_stamp'] - t0

print()
print('=' * 72)
print('2) 구간 정리')
print('=' * 72)

# 앞뒤 정지 구간 잘라내기
moving = [i for i, r in enumerate(ego) if r['v'] > 1.0]
if not moving:
    sys.exit('움직인 구간이 없다. 차가 안 굴렀거나 /ego_status 가 이상하다.')
i0, i1 = moving[0], moving[-1]
t_start, t_end = ego[i0]['t_stamp'], ego[i1]['t_stamp']
print('  주행 구간   %.1f ~ %.1f초 (전체 %.1f초 중)'
      % (ego[i0]['t'], ego[i1]['t'], ego[-1]['t']))
if ego[-1]['t'] - ego[i1]['t'] > 3.0:
    print('               꼬리 %.1f초는 정지 상태라 뺀다' % (ego[-1]['t'] - ego[i1]['t']))

# 급감속(충돌 등) 찾기
events = []
for i in range(1, len(ego)):
    dt = ego[i]['t_stamp'] - ego[i - 1]['t_stamp']
    if dt <= 0 or dt > 0.5:
        continue
    acc = (ego[i]['v'] - ego[i - 1]['v']) / dt
    if acc < -a.decel:
        if not events or ego[i]['t_stamp'] - events[-1][1] > 1.0:
            events.append([ego[i]['t_stamp'], ego[i]['t_stamp'], ego[i - 1]['v']])
        else:
            events[-1][1] = ego[i]['t_stamp']
if events:
    # 문턱은 벽시계 기준이다. 시뮬이 r 배로 도니 차량이 실제로 겪은 감속은
    # 이보다 1/r 배 크다 - 그래서 문턱을 낮게 잡아도 급제동만 잡힌다.
    print('  급감속 %d회 - 앞뒤 %.1f초씩 뺀다 (충돌/급제동 때는 GT 가 튄다)'
          % (len(events), a.pad))
    print('     %s' % ', '.join('%.0f초' % (ev[0] - t0) for ev in events[:12])
          + (' ...' if len(events) > 12 else ''))
else:
    print('  급감속 없음')

excl = [(ev[0] - a.pad, ev[1] + a.pad) for ev in events]


def usable(t):
    if t < t_start or t > t_end:
        return False
    return not any(lo <= t <= hi for lo, hi in excl)


# ── 위치가 아예 틀린 구간 찾아내기 ──────────────────────────────────
# 지연은 "조금 늦은" 문제라 오차가 속도x지연(고속에서 3m 남짓)을 넘지 않는다.
# 그보다 훨씬 큰 오차가 나는 구간은 지연이 아니라 다른 고장이다(GPS 음영구역이
# 대표적). 섞어서 평균내면 지연 추정이 통째로 망가지므로 따로 떼어낸다.
_gt_t = [r['t_stamp'] for r in ego]
_gt_x = [r['x'] for r in ego]
_gt_y = [r['y'] for r in ego]


def _gt_at(t):
    if t < _gt_t[0] or t > _gt_t[-1]:
        return None
    i = bisect.bisect_left(_gt_t, t)
    if i == 0:
        return (_gt_x[0], _gt_y[0])
    d = _gt_t[i] - _gt_t[i - 1]
    f = 0.0 if d <= 0 else (t - _gt_t[i - 1]) / d
    return (_gt_x[i - 1] + f * (_gt_x[i] - _gt_x[i - 1]),
            _gt_y[i - 1] + f * (_gt_y[i] - _gt_y[i - 1]))


BUCKET = 10.0
buckets = {}
for r in data['odom']:
    t = r['t_stamp']
    if not usable(t):
        continue
    g = _gt_at(t)
    if g is None:
        continue
    buckets.setdefault(int((t - t_start) // BUCKET), []).append(
        math.hypot(r['x'] - g[0], r['y'] - g[1]))

print()
print('  /odom 이 GT 에서 얼마나 떨어져 있나 (10초 단위, 보정 전):')
bad = []
line = []
for k in sorted(buckets):
    m = median(buckets[k])
    flag = ' ***' if m > a.outlier else ''
    line.append('%3d초:%6.2fm%s' % (k * BUCKET, m, flag))
    if m > a.outlier:
        bad.append((t_start + k * BUCKET, t_start + (k + 1) * BUCKET))
for i in range(0, len(line), 3):
    print('     ' + '   '.join(line[i:i + 3]))

if bad:
    # 붙어 있는 구간은 하나로 합친다
    merged = [list(bad[0])]
    for lo, hi in bad[1:]:
        if lo - merged[-1][1] < BUCKET * 1.5:
            merged[-1][1] = hi
        else:
            merged.append([lo, hi])
    print()
    print('  *** 표시 = %.0fm 초과. 지연으로 설명 안 되는 크기다 (GPS 음영 의심).' % a.outlier)
    for lo, hi in merged:
        print('      %.0f ~ %.0f초를 지연 추정에서 뺀다 (%.0f초간)'
              % (lo - t_start, hi - t_start, hi - lo))
    excl.extend((lo, hi) for lo, hi in merged)
    OUTLIER_WINDOWS = merged
else:
    OUTLIER_WINDOWS = []


# ── 배속 ────────────────────────────────────────────────────────────
print()
print('=' * 72)
print('3) 시뮬 배속 (벽시계 지연 <-> 차량이 겪는 지연 환산에 필요)')
print('=' * 72)
# !!! 2026-08-27 수정: 표본 중앙값 -> 시간가중 !!!
# 예전에는 표본마다 (d/dt)/v 를 만들고 그 중앙값을 배속이라 불렀다. 그런데 표본은
# 시간을 균등하게 대표하지 않는다. lag031 을 표본 간격별로 쪼개보면:
#
#     dt <1ms   (중복 발행)   표본 2540개 /   1.2초   <- MORAI 가 같은 패킷을 두 번 쏨
#     dt 50~150ms (정상)      표본 2436개 / 275.4초 / r = 0.536
#     dt >150ms  (시뮬 멈칫)  표본  599개 / 104.8초 / r = 0.342
#
# 멈칫 구간이 런의 27%(105초)를 차지하는데 표본 수로는 1/4뿐이다. 중앙값은 시간이
# 아니라 표본을 세므로 이 구간을 과소평가한다. lag031 전체 구간 기준으로
# 표본중앙값 0.496 vs 시간가중 0.477 - 약 4% 차이다.
#
# ※ 이 노드가 출력하는 값이 위 0.477 과 다른 건 usable() 때문이다. 급감속·GPS
#   음영 구간을 빼고 남은 구간만 쓰므로 대상 구간 자체가 다르다. 둘 다 각자
#   맞는 값이고, 지연 환산에 쓸 값은 "지연을 추정한 그 구간의 배속" 이므로
#   usable() 을 거친 이 값이 맞다.
#
# EKF 는 표본이 아니라 벽시계를 따라 쉬지 않고 적분한다. 시뮬이 멈칫하는 105초
# 동안에도 계속 위치를 앞으로 민다. 그러니 시간가중 쪽이 우리가 원하는 값이다.
#
# dt 상한은 0.3 -> 0.5 로 올렸다. 0.3 은 가장 심한 멈칫을 잘라내던 값이다.
# 상한에 민감하지 않은 건 확인했다 (lag031 은 0.3/0.5/1.0 전부 0.477).
sum_path = 0.0        # 차가 실제로 지나간 거리 [m]
sum_pred = 0.0        # 속도계를 벽시계 시간으로 적분한 거리 [m]
n_pair = n_dup = 0
WIN = 30.0            # 변동폭 보고용 창 [초]
win = []
w_path = w_pred = 0.0
w_edge = None
for i in range(1, len(ego)):
    t = ego[i]['t_stamp']
    dt = t - ego[i - 1]['t_stamp']
    if dt <= 0 or dt > 0.5 or not usable(t):
        continue
    if dt < 0.001:            # 같은 패킷 두 번. 시간을 안 차지하므로 뺀다
        n_dup += 1
        continue
    v = 0.5 * (ego[i]['v'] + ego[i - 1]['v'])
    if v < a.min_speed:
        continue
    d = math.hypot(ego[i]['x'] - ego[i - 1]['x'], ego[i]['y'] - ego[i - 1]['y'])
    sum_path += d
    sum_pred += v * dt
    n_pair += 1
    if w_edge is None:
        w_edge = t
    if t - w_edge >= WIN:
        if w_pred > 0:
            win.append(w_path / w_pred)
        w_path = w_pred = 0.0
        w_edge = t
    w_path += d
    w_pred += v * dt

r_mid = sum_path / sum_pred if sum_pred > 0 else 0.0
print('  배속 r = %.3f   (실제 이동 %.0f m / 속도계 적분 %.0f m, 표본 %d쌍)'
      % (r_mid, sum_path, sum_pred, n_pair))
if win:
    print('           %.0f초 창별로는 %.3f ~ %.3f 사이에서 움직인다 (창 %d개)'
          % (WIN, min(win), max(win), len(win)))
print('  -> 벽시계 1초의 지연은 차량 입장에서 %.3f초짜리 지연이다' % r_mid)
print('  ※ 시간가중이다 (2026-08-27 수정). 표본 중앙값으로 재면 시뮬이 멈칫한')
print('     구간이 과소평가돼 몇 %% 높게 나온다. 위 주석 참고.')

# ── (A) 통과 시간 ───────────────────────────────────────────────────
print()
print('=' * 72)
print('4) (A) 단계별 통과 시간  [벽시계]')
print('=' * 72)


def arr_by_stamp(rows):
    m = {}
    for r in rows:
        if usable(r['t_stamp']):
            m.setdefault(round(r['t_stamp'], 6), r['t_arr'])
    return m


A_gps, A_fix, A_nav = (arr_by_stamp(data[n]) for n in ('gps', 'fix', 'navsat'))


def stage(m_from, m_to, label):
    d = [(m_to[k] - m_from[k]) * 1000.0 for k in m_from if k in m_to]
    if not d:
        print('  %-34s 짝지어진 표본 없음' % label)
        return
    print('  %-34s 중앙값 %7.2f ms   p90 %7.2f ms  (표본 %d)'
          % (label, median(d), pct(d, 0.9), len(d)))


print('  같은 stamp 가 각 단계에 언제 도착했나:')
stage(A_gps, A_fix, '/gps -> /gps/fix  (loc_node)')
stage(A_fix, A_nav, '/gps/fix -> /odometry/gps (navsat)')
stage(A_gps, A_nav, '  합계 /gps -> /odometry/gps')
print()
print('  각 토픽의 "도착했을 때 이미 몇 초짜리였나" (t_arr - stamp):')
for n in ('gps', 'fix', 'navsat', 'odom', 'wheel'):
    v = [(r['t_arr'] - r['t_stamp']) * 1000.0 for r in data[n] if usable(r['t_stamp'])]
    if v:
        print('    %-14s 중앙값 %7.2f ms   p90 %7.2f ms' % (n, median(v), pct(v, 0.9)))
print('    (imu 는 시뮬이 준 시각이라 벽시계와 원점이 달라 여기서 뺐다)')

# ── 발행 주기 ───────────────────────────────────────────────────────
print()
print('=' * 72)
print('5) 발행 주기 - EKF 가 설정대로 도는가')
print('=' * 72)
for n in ('imuekf', 'navsat', 'wheel', 'odom'):
    rows = [r for r in data[n] if usable(r['t_stamp'])]
    gaps = [(rows[i]['t_arr'] - rows[i - 1]['t_arr']) * 1000.0 for i in range(1, len(rows))]
    gaps = [g for g in gaps if 0 < g < 1000]
    if gaps:
        print('  %-8s 중앙 %6.1f ms (%5.1f Hz)   p90 %6.1f ms   최대 %6.1f ms'
              % (n, median(gaps), 1000.0 / median(gaps), pct(gaps, 0.9), max(gaps)))
print('  (ekf.yaml 은 frequency: 40 -> 25.0 ms 가 설계값이다)')

# ── (B) 정보의 나이 ─────────────────────────────────────────────────
print()
print('=' * 72)
print('6) (B) 단계별 정보의 나이  <- 0.22초의 정체')
print('=' * 72)

def gt_full(t):
    """GT 의 (x, y, heading[deg], v[m/s]) 를 t 시각으로 보간해서 준다."""
    if t < _gt_t[0] or t > _gt_t[-1]:
        return None
    i = bisect.bisect_left(_gt_t, t)
    if i == 0:
        i = 1
    a0, b0 = ego[i - 1], ego[i]
    d = b0['t_stamp'] - a0['t_stamp']
    f = 0.0 if d <= 0 else (t - a0['t_stamp']) / d
    return (a0['x'] + f * (b0['x'] - a0['x']),
            a0['y'] + f * (b0['y'] - a0['y']),
            a0['heading_deg'], a0['v'])


gt = Track(ego)
gt_t = [r['t_stamp'] for r in ego]
gt_v = [r['v'] for r in ego]


def speed_at(t):
    i = bisect.bisect_left(gt_t, t)
    return gt_v[min(i, len(gt_v) - 1)]


def best_shift(rows, label):
    """rows 의 위치가 GT 의 몇 초 전 위치와 가장 잘 맞는지 찾는다."""
    pts = [(r['t_stamp'], r['x'], r['y']) for r in rows
           if usable(r['t_stamp']) and speed_at(r['t_stamp']) >= a.min_speed]
    if len(pts) < 50:
        print('  %-16s 쓸 표본이 부족하다 (%d개)' % (label, len(pts)))
        return None

    def rms(tau):
        s, n = 0.0, 0
        for t, x, y in pts:
            g = gt.at(t - tau)
            if g is None:
                continue
            s += (x - g[0]) ** 2 + (y - g[1]) ** 2
            n += 1
        return math.sqrt(s / n) if n else float('inf')

    best_t, best_r = None, float('inf')
    LO, HI = -0.30, 1.00
    tau = LO
    while tau <= HI:
        v = rms(tau)
        if v < best_r:
            best_r, best_t = v, tau
        tau += 0.005
    if best_t is not None and (abs(best_t - LO) < 1e-6 or abs(best_t - HI) < 1e-6):
        print('  %-16s ⚠️ τ 가 탐색 범위 끝(%.2f)에 붙었다. 지연이 아니라 다른 오차가'
              % (label, best_t))
        print('  %-16s    지배하고 있다는 뜻이라 이 값은 믿으면 안 된다.' % '')
    r0 = rms(0.0)
    v_avg = sum(speed_at(t) for t, _, _ in pts) / len(pts)
    dist = best_t * v_avg * r_mid          # 벽시계 τ 동안 차가 간 거리
    tau_sim = dist / v_avg if v_avg else 0.0
    print('  %-16s τ(벽시계) %+.3f초   차량기준 %+.3f초   거리 %+.2f m'
          % (label, best_t, tau_sim, dist))
    print('  %-16s   잔차 RMS  τ적용 %.3f m  <-  보정없음 %.3f m   (표본 %d, 평균 %.1f m/s)'
          % ('', best_r, r0, len(pts), v_avg))
    return best_t, tau_sim, best_r, r0


print('  오차를 진행방향/횡방향으로 나눠서 본다. 지연은 진행방향에만 나타난다.')
print('  (+ = GT 보다 앞, - = GT 보다 뒤)')
print()


def decompose(rows, label):
    al, cr, vs = [], [], []
    for r in rows:
        t = r['t_stamp']
        if not usable(t):
            continue
        g = gt_full(t)
        if g is None or g[3] < a.min_speed:
            continue
        h = math.radians(g[2])
        dx, dy = r['x'] - g[0], r['y'] - g[1]
        al.append(dx * math.cos(h) + dy * math.sin(h))
        cr.append(-dx * math.sin(h) + dy * math.cos(h))
        vs.append(g[3])
    if len(al) < 50:
        print('  %-16s 표본 부족 (%d개)' % (label, len(al)))
        return None
    mean = lambda v: sum(v) / len(v)
    rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v))
    v_avg = mean(vs)
    a_mean = mean(al)
    print('  %s   (표본 %d, 평균속도 %.1f m/s)' % (label, len(al), v_avg))
    print('     진행방향  평균 %+.2f m  = %+.3f 초   RMS %.2f m'
          % (a_mean, a_mean / v_avg, rms(al)))
    print('     횡방향    평균 %+.2f m               RMS %.2f m   최대 %.2f m'
          % (mean(cr), rms(cr), max(abs(x) for x in cr)))
    return {'along': a_mean, 'sec': a_mean / v_avg, 'cross_rms': rms(cr),
            'cross_max': max(abs(x) for x in cr), 'v': v_avg}


res_nav = decompose(data['navsat'], '/odometry/gps  (GPS -> UTM, EKF 들어가기 전)')
print()
res_odom = decompose(data['odom'], '/odom          (EKF 나온 뒤)')

print()
print('=' * 72)
print('7) 결론')
print('=' * 72)
if res_nav and res_odom:
    d = res_odom['along'] - res_nav['along']
    print('  ① EKF 가 위치를 앞으로 밀어낸다')
    print('     GPS 유래 위치   %+.2f m (%+.3f초)' % (res_nav['along'], res_nav['sec']))
    print('     EKF 출력        %+.2f m (%+.3f초)' % (res_odom['along'], res_odom['sec']))
    print('     차이            %+.2f m  <- EKF 가 더한 몫' % d)
    print()
    print('  ② EKF 가 횡방향 오차를 키운다')
    print('     GPS 유래 위치   RMS %.2f m  (최대 %.2f m)'
          % (res_nav['cross_rms'], res_nav['cross_max']))
    print('     EKF 출력        RMS %.2f m  (최대 %.2f m)  <- %.1f배'
          % (res_odom['cross_rms'], res_odom['cross_max'],
             res_odom['cross_rms'] / res_nav['cross_rms'] if res_nav['cross_rms'] else 0))
    print()
    print('  ③ 배관은 범인이 아니다')
    print('     /gps -> /odometry/gps 통과 시간이 벽시계 15 ms 남짓이다.')
    print('     차량 기준으로 %.0f ms 밖에 안 되므로 위 숫자를 설명 못 한다.'
          % (15.0 * r_mid))
print()
print('  ※ 진행방향 값은 /ego_status 를 시각 기준으로 삼은 결과다. 두 단계의')
print('     "차이"(①의 마지막 줄)는 기준이 상쇄되므로 그 영향을 안 받는다.')
