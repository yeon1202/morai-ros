#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wheel_speed_scaler : /odom/wheel_speed 의 속도를 시뮬 배속만큼 줄여 EKF 에 넘긴다.

왜 필요한가
  EKF 는 위치를 이렇게 예측한다:  위치 += 속도 × dt

  그런데 두 항의 시간 단위가 다르다.
    속도 : MORAI 가 주는 값이라 "시뮬 1초당 m"
    dt   : ros::Time 차이라 "벽시계 초"     (우리 스택엔 /clock 이 없다)

  시뮬이 벽시계보다 r배 느리게 흐르므로, 이 곱셈은 1/r 배 과전진한다.
  r=0.48 이면 EKF 가 실제의 2.1배를 간다. 실측(2026-08-27 lag031)에서
  /odom 이 GT 대비 진행방향 +4 m, 구간에 따라 수십 m 까지 앞섰다.

  근본 해법은 /clock 을 세우고 use_sim_time 을 쓰는 것이지만, MORAI 는 센서
  스탬프를 벽시계로 주므로(2026-08-27 확인: /imu 의 시뮬 스탬프도 벽시계와
  같은 속도로 흐른다) 믿을 시뮬 시계가 없다. 대신 배속 r 을 재서
  (sim_rate_estimator) 속도 쪽을 벽시계 단위로 맞춘다.

  ※ localization_node.cpp 는 localization 팀 코드라 건드리지 않는다. 그쪽이
    파일을 업데이트할 때마다 다시 고쳐야 하기 때문이다. 토픽 사이에 이 노드를
    끼우고 ekf.yaml 의 odom1 만 이쪽으로 돌린다.

왜 vy 만 제곱인가
  vx 는 속도계 실측값이라 단위 환산 한 번이면 된다:   vx_out = r · vx

  vy 는 실측이 아니다. localization_node.cpp:441 이
      vy += (ay - vx·ω) · dt
  로 만드는 적분값인데, ay·ω·vx 가 전부 시뮬 단위인 걸 벽시계 dt 로 적분한다.
  그래서 vy 자체가 이미 1/r 배 부풀어 있다. 벽시계 기준 횡속도는

      d(vy_wall)/dt_wall = a_wall - vx_wall·ω_wall
                         = r²·(a_sim - vx_sim·ω_sim)

  이므로 vy_out = r² · vy 다. 적분 오차를 되돌리는 데 r 하나, 단위 환산에 r
  하나다. 횡오차가 종오차보다 심했던 것도 이 제곱 때문이다
  (실측 EKF 전 0.04 m -> EKF 후 0.71 m).

공분산도 같이 조정한다
  값만 줄이고 분산을 놔두면 EKF 가 이 속도를 실제보다 덜 믿게 된다(값은 r배
  작아졌는데 불확실성은 그대로니까). 게다가 r̂ 자체에 오차가 있어서(실측 RMS
  7.2%) 그 몫이 새로 더해진다. 델타법으로:

      var(r·vx)  = r²·var(vx) + vx²·var(r)
      var(r²·vy) = r⁴·var(vy) + (2r·vy)²·var(r)

  var(r) 은 ~sim_rate_rel_std(기본 0.072, 실측 RMS)로 잡는다. 0 으로 두면
  이 항을 끄고 순수 단위 환산만 한다(원인 격리용).

안전장치
  /sim_rate 가 아직 없으면 r=1.0 으로 그냥 통과시킨다. 0 을 쓰면 속도를 0 으로
  만들어 EKF 가 "차가 멈췄다" 고 믿는다. 대회장 머신이 빨라서 r≈1 이면 곱하기
  1 이라 이 노드는 자동으로 무동작이 된다.

  /sim_rate 가 끊기면 직전 값을 유지한다(1.0 으로 되돌리면 주행 중에 튄다).
  ~stale_warn_sec 마다 경고만 남긴다.

사용법
  rosrun sim_rate wheel_speed_scaler.py
  rosrun sim_rate wheel_speed_scaler.py _sim_rate_rel_std:=0.0   # 분산 항 끄기

구독 /odom/wheel_speed (nav_msgs/Odometry), /sim_rate (std_msgs/Float64)
발행 /odom/wheel_speed_scaled (nav_msgs/Odometry)   <- ekf.yaml 의 odom1
"""
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class WheelSpeedScaler:
    def __init__(self):
        self.rel_std = rospy.get_param('~sim_rate_rel_std', 0.072)
        self.stale_warn = rospy.get_param('~stale_warn_sec', 5.0)
        self.lo = rospy.get_param('~clamp_lo', 0.2)
        self.hi = rospy.get_param('~clamp_hi', 1.5)

        self.r = None              # None = 아직 한 번도 못 받음 -> 통과
        self.r_time = None
        self.warned = None
        self.n_in = self.n_scaled = 0

        self.pub = rospy.Publisher('/odom/wheel_speed_scaled', Odometry, queue_size=1)
        rospy.Subscriber('/sim_rate', Float64, self.cb_rate)
        rospy.Subscriber('/odom/wheel_speed', Odometry, self.cb_speed)
        rospy.on_shutdown(self.finish)
        rospy.loginfo('[wheel_scaler] start - r̂ 오면 vx*r, vy*r^2 로 내보낸다 '
                      '(r̂ 없으면 그대로 통과)')

    def cb_rate(self, m):
        if self.lo <= m.data <= self.hi:
            self.r = m.data
            self.r_time = rospy.Time.now()

    def cb_speed(self, m):
        self.n_in += 1
        out = m                       # 같은 메시지를 고쳐서 내보낸다
        r = self.r

        if r is None:
            # 아직 배속을 모른다. 손대지 않고 그대로 흘린다.
            self.pub.publish(out)
            return

        age = (rospy.Time.now() - self.r_time).to_sec()
        if age > self.stale_warn:
            # 끊겨도 직전 값을 유지한다 - 1.0 으로 되돌리면 주행 중에 튄다.
            if self.warned is None or (rospy.Time.now() - self.warned).to_sec() > 10.0:
                rospy.logwarn('[wheel_scaler] /sim_rate 가 %.1f초째 없다. '
                              'r=%.3f 을 유지한다 (sim_rate_estimator 확인)', age, r)
                self.warned = rospy.Time.now()

        vx = m.twist.twist.linear.x
        vy = m.twist.twist.linear.y
        var_r = (r * self.rel_std) ** 2

        out.twist.twist.linear.x = r * vx
        out.twist.twist.linear.y = r * r * vy
        out.twist.covariance = list(m.twist.covariance)
        out.twist.covariance[0] = r * r * m.twist.covariance[0] + vx * vx * var_r
        out.twist.covariance[7] = (r ** 4) * m.twist.covariance[7] \
            + (2.0 * r * vy) ** 2 * var_r

        self.pub.publish(out)
        self.n_scaled += 1

    def finish(self):
        print('')
        print('[wheel_scaler] 받은 %d개 중 %d개를 스케일했다 (마지막 r=%s)'
              % (self.n_in, self.n_scaled,
                 '%.3f' % self.r if self.r is not None else '없음'))
        if self.n_scaled == 0:
            print('  ⚠️ 하나도 스케일 안 됐다. /sim_rate 가 안 왔다는 뜻이다.')
            print('     sim_rate_estimator.py 가 떠 있는지, 3 m/s 이상으로 달렸는지 확인.')


if __name__ == '__main__':
    rospy.init_node('wheel_speed_scaler')
    WheelSpeedScaler()
    rospy.spin()
