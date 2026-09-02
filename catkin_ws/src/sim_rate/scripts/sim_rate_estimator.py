#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_rate_estimator : 시뮬 배속 r 을 GT 없이 온라인으로 추정한다.

왜 필요한가
  MORAI 는 물리를 "시뮬 시간" 으로 돌린다. 머신이 버거우면 벽시계 1초 동안
  시뮬은 r초분만 진행한다(실측 0.48~0.87, 머신·부하·센서구성에 따라 변한다).
  그런데 우리 스택엔 /clock 이 없어 모든 스탬프가 벽시계다. 그래서 이런 곱셈이
  곳곳에서 벌어진다:

      속도 11.2 [시뮬 m / 시뮬초]  ×  dt 0.1 [벽시계 초]  =  1.12 m 갔다고 계산
      실제로 간 거리는 1.12 × r = 0.53 m   (r=0.477)

  단위가 다른 둘을 곱하니 위치가 1/r 배로 과전진한다. r=0.48 이면 110% 과전진이다.

  고치려면 r 을 알아야 하는데, r 은 머신마다·순간마다 다르므로 값을 박으면
  안 된다. 이 노드는 지금 이 순간의 r 을 재서 발행만 한다.

  ※ 발행만 하고 아무것도 안 고친다. 적용은 별도 스케일러 노드가
    /odom/wheel_speed 를 스케일하는 방식으로 한다 - localization_node.cpp 는
    팀 코드라 건드리지 않는다.

어떻게 재는가
  같은 움직임에 대해 서로 다른 두 시계의 답이 있다.

    실제 변위 = GPS 두 점 사이 벡터                      (벽시계 기준, 진짜)
    예측 변위 = ∫ v·[cos ψ, sin ψ] dt_wall               (시뮬 속도 × 벽시계 dt)

    r = |실제 변위| / |예측 변위|

  둘은 **같은 모양의 경로**이고 예측만 1/r 배 크다(같은 헤딩 궤적을 따라 속도만
  1/r 배로 간 것이므로). 그래서 곡률이 얼마든 크기 비율은 곧 r 이다.
  IMU 헤딩에 상수 오프셋이 있어도 두 벡터가 같이 회전할 뿐 크기비는 안 변한다.

  GPS·속도계·IMU 전부 합법 센서다. ground truth 를 안 쓰므로 대회 중에도 돈다.

  ⚠️ /odom 이 아니라 /odometry/gps 를 쓴다. /odom 은 지금 고치려는 그 오차를
     품고 있어서 그걸로 r 을 재면 순환논법이 된다. EKF 이전 신호여야 한다.

  ⚠️ stamp 를 쓴다(도착시각 아님). udp_bridge 가 GPS 전송지연 0.30초만큼
     스탬프를 과거로 찍어준 뒤에야 이 계산이 성립한다.

GPS 가 끊기면 (터널·음영구간)
  GPS 변위로는 못 잰다. 그런데 MORAI 는 IMU 를 **시뮬 시간 주기**로 내보내므로
  메시지 도착 빈도 자체가 배속에 비례한다. 실측(2026-08-29, novy1 13개 구간):

      r 과 /imu 도착 Hz 의 상관계수 0.963,  imuHz/r = 25.2 +- 10%

  그래서 GPS 가 있을 때 N = imuHz / r̂ 을 배워 두고, 끊기면 r̂ = imuHz / N 으로
  이어간다. N 을 상수로 박지 않는 게 중요하다 - MORAI 센서 설정이나 머신이
  달라도 알아서 맞는다.

  왜 이게 필요한가: 음영구간 드리프트의 지배항이 r 오차다. 속도계는 시뮬이 주는
  실측값이라 정확하고 헤딩도 0.17도다. 실측에서 두절 직전 r̂ 0.758 이 얼어붙은
  채 실제 0.640 인 구간을 100m 달려 종방향 23m(=18.5%)가 밀렸다.

왜 비율을 평균하지 않고 분자·분모를 따로 평균하는가
  창마다 나온 r 을 EMA 하면 편향된다. 각 창이 대표하는 이동거리가 다른데
  비율은 그 무게를 안 담기 때문이다(analyze_latency 가 표본 중앙값으로 배속을
  재다가 같은 함정에 빠졌다). 분자(실제 변위)와 분모(예측 변위)를 각각 EMA 하고
  마지막에 나누면 자연히 이동거리로 무게가 실린다.

검증 (2026-08-27, lag031 로그로 이 로직을 그대로 재현)
    GT 실제값 대비 평균 +4.1%, RMS 7.2%, 최대 홀드 17.3초, 갱신 908회/383초
  직선 구간에서만 재던 초안은 RMS 17.2%, 최대 홀드 61.3초였다. 곡선을 버리면
  대회 코스에서 1분씩 얼어붙는다.

사용법
  rosrun sim_rate sim_rate_estimator.py
  rosrun sim_rate sim_rate_estimator.py _window_sec:=6.0

발행
  /sim_rate  (std_msgs/Float64)  - r̂. 새 표본이 없으면 직전 값을 유지한다.
"""
import math
import statistics
from collections import deque

import rospy
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimRateEstimator:
    def __init__(self):
        # 창이 길수록 GPS 노이즈가 희석되지만 r 변화를 늦게 따라간다. 4초면
        # 11m/s 에서 44m 이동이라 GPS 노이즈(약 0.9m)가 2% 수준으로 묻힌다.
        self.window = rospy.get_param('~window_sec', 4.0)
        self.min_speed = rospy.get_param('~min_speed', 3.0)   # m/s. 저속은 비율이 불안정
        self.alpha = rospy.get_param('~alpha', 0.1)
        self.clamp_lo = rospy.get_param('~clamp_lo', 0.2)
        self.clamp_hi = rospy.get_param('~clamp_hi', 1.5)
        self.max_gap = rospy.get_param('~max_gps_gap', 1.0)   # 초. GPS 음영 배제

        self.gps = deque()      # (t_stamp, x, y)
        self.motion = deque()   # (t_stamp, v_mps, yaw_rad)
        self.yaw = 0.0
        self.has_yaw = False

        self.ema_n = None       # 실제 변위 EMA
        self.ema_d = None       # 예측 변위 EMA
        self.raw = []
        self.reject = {'짧은창': 0, 'GPS끊김': 0, '저속': 0, '범위밖': 0}

        # --- GPS 두절 폴백 (IMU 도착 빈도) ---
        self.gps_stale = rospy.get_param('~gps_stale_sec', 1.5)   # 이만큼 새 표본이 없으면 폴백
        self.imu_window = rospy.get_param('~imu_window_sec', 2.0)
        self.imu_times = deque()    # IMU 도착 시각
        self.n_ratio = None         # imuHz / r  (GPS 있을 때 배운다)
        self.last_est = None        # 마지막으로 GPS 기반 추정에 성공한 시각
        self.fallback = False
        self.fallback_secs = 0.0

        self.pub = rospy.Publisher('/sim_rate', Float64, queue_size=1)
        rospy.Subscriber('/imu', Imu, self.cb_imu)
        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.cb_ego)
        rospy.Subscriber('/odometry/gps', Odometry, self.cb_gps)
        rospy.Timer(rospy.Duration(0.1), self.cb_pub)
        rospy.on_shutdown(self.finish)
        rospy.loginfo('[sim_rate] start - window %.1fs, 최저속도 %.1f m/s',
                      self.window, self.min_speed)

    # ── 입력 ──────────────────────────────────────────────────────────
    def cb_imu(self, m):
        self.yaw = yaw_from_quat(m.orientation)
        self.has_yaw = True
        # 도착 시각만 쌓는다. 내용은 안 본다 - 빈도 자체가 배속 신호다.
        now = rospy.Time.now()
        self.imu_times.append(now.to_sec())
        cutoff = self.imu_times[-1] - self.imu_window
        while len(self.imu_times) > 2 and self.imu_times[0] < cutoff:
            self.imu_times.popleft()

    def _imu_hz(self):
        if len(self.imu_times) < 5:
            return None
        span = self.imu_times[-1] - self.imu_times[0]
        return (len(self.imu_times) - 1) / span if span > 0.5 else None

    def cb_ego(self, m):
        if not self.has_yaw:
            return
        # velocity 는 km/h 다 (udp_bridge 주석 참고 - 팀 원본이 여기서 한 번 틀렸다).
        self.motion.append((m.header.stamp.to_sec(), m.velocity.x / 3.6, self.yaw))
        self._prune(self.motion)

    def cb_gps(self, m):
        self.gps.append((m.header.stamp.to_sec(),
                         m.pose.pose.position.x, m.pose.pose.position.y))
        self._prune(self.gps)
        self._estimate()

    def _prune(self, dq):
        cutoff = dq[-1][0] - self.window * 1.5
        while len(dq) > 2 and dq[0][0] < cutoff:
            dq.popleft()

    # ── 추정 ──────────────────────────────────────────────────────────
    def _estimate(self):
        if len(self.gps) < 3 or len(self.motion) < 3:
            return
        t2, x2, y2 = self.gps[-1]

        left = None
        for i, (t, _, _) in enumerate(self.gps):
            if t >= t2 - self.window:
                left = i
                break
        if left is None or left >= len(self.gps) - 1:
            return
        t1, x1, y1 = self.gps[left]
        if t2 - t1 < self.window * 0.7:
            self.reject['짧은창'] += 1
            return

        win = list(self.gps)[left:]
        for a, b in zip(win, win[1:]):
            if b[0] - a[0] > self.max_gap:
                self.reject['GPS끊김'] += 1
                return

        seg = [s for s in self.motion if t1 <= s[0] <= t2]
        if len(seg) < 3 or seg[0][0] - t1 > 0.5 or t2 - seg[-1][0] > 0.5:
            self.reject['짧은창'] += 1
            return
        if min(v for _, v, _ in seg) < self.min_speed:
            self.reject['저속'] += 1
            return

        # 예측 변위: 시뮬 속도를 IMU 헤딩 방향으로 벽시계 시간만큼 적분
        px = py = 0.0
        for (ta, va, ya), (tb, vb, yb) in zip(seg, seg[1:]):
            dt = tb - ta
            px += 0.5 * (va * math.cos(ya) + vb * math.cos(yb)) * dt
            py += 0.5 * (va * math.sin(ya) + vb * math.sin(yb)) * dt
        d_pred = math.hypot(px, py)
        if d_pred <= 0.1:
            return
        d_real = math.hypot(x2 - x1, y2 - y1)

        if not (self.clamp_lo <= d_real / d_pred <= self.clamp_hi):
            self.reject['범위밖'] += 1
            return

        # 비율이 아니라 분자·분모를 각각 EMA 한다 (머리말 참고)
        a = self.alpha
        self.ema_n = d_real if self.ema_n is None else a * d_real + (1 - a) * self.ema_n
        self.ema_d = d_pred if self.ema_d is None else a * d_pred + (1 - a) * self.ema_d
        r_now = self.ema_n / self.ema_d
        self.raw.append(r_now)
        self.last_est = rospy.Time.now()

        # GPS 가 있는 동안 "IMU 몇 Hz 가 배속 1 에 해당하는가" 를 배워 둔다.
        hz = self._imu_hz()
        if hz and r_now > 0.05:
            n = hz / r_now
            self.n_ratio = n if self.n_ratio is None else a * n + (1 - a) * self.n_ratio

    # ── 출력 ──────────────────────────────────────────────────────────
    def cb_pub(self, _):
        """GPS 기반 값이 신선하면 그것, 아니면 IMU 빈도 폴백.

        표본이 아예 없으면 아무것도 안 낸다. 소비자(스케일러)는 값이 없으면
        1.0(무보정)을 써야 한다 - 0 을 쓰면 속도를 0 으로 만들어 버린다.
        """
        if not self.ema_d:
            return
        r_gps = self.ema_n / self.ema_d
        stale = (self.last_est is None or
                 (rospy.Time.now() - self.last_est).to_sec() > self.gps_stale)

        r_out = r_gps
        if stale and self.n_ratio:
            hz = self._imu_hz()
            if hz:
                r_fb = hz / self.n_ratio
                if self.clamp_lo <= r_fb <= self.clamp_hi:
                    r_out = r_fb
                    self.fallback_secs += 0.1
                    if not self.fallback:
                        rospy.logwarn('[sim_rate] GPS 두절. IMU 빈도로 전환 '
                                      '(r %.3f -> %.3f)', r_gps, r_fb)
                    self.fallback = True
        if self.fallback and not stale:
            rospy.loginfo('[sim_rate] GPS 복귀. 다시 GPS 기반으로.')
            self.fallback = False

        self.pub.publish(Float64(data=r_out))

    def finish(self):
        print('')
        print('[sim_rate] 결과')
        if self.raw:
            s = sorted(self.raw)
            print('  갱신 %d회   r̂ 중앙값 %.3f   (10%%=%.3f  90%%=%.3f)   마지막 %.3f'
                  % (len(s), statistics.median(s), s[len(s) // 10], s[len(s) * 9 // 10],
                     self.raw[-1]))
        else:
            print('  ⚠️ 갱신이 하나도 없다.')
        print('  버린 창: %s' % ', '.join('%s %d' % kv for kv in self.reject.items()))
        if self.n_ratio:
            print('  IMU 빈도 환산계수 N = %.1f Hz (배속 1 기준)' % self.n_ratio)
        print('  GPS 두절 폴백 사용 시간: %.1f초' % self.fallback_secs)
        print('')
        print('  검증: analyze_latency.py --tag <tag> 의 "배속 r" 과 비교할 것.')


if __name__ == '__main__':
    rospy.init_node('sim_rate_estimator')
    SimRateEstimator()
    rospy.spin()
