#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
조향 포화 진단기 (읽기 전용 성격 - 차는 브레이크로 세워둔다)
------------------------------------------------------------------
목적: "우리가 명령한 조향각이 MORAI 에서 실제로 얼마가 되는가?"

배경
  - path_tracker 는 MAX_STEER = 0.5 rad (28.6도) 로 조향을 잘라서 내보낸다.
  - 그런데 udp_bridge 가 STEER_RATIO_CORRECTION(0.70) 으로 나눠 1.43배 증폭한다.
    -> 0.5 rad 를 명령하면 패킷에는 0.714 rad (40.9도) 가 실린다.
  - 규정집상 차량 최대 조향은 40도. 즉 이미 천장에 닿아 있을 수 있다.
    그렇다면 MAX_STEER 를 올려도 MORAI 가 잘라서 아무 변화가 없다.

방법
  브레이크를 밟은 채(차는 정지) 조향 명령을 단계별로 올리면서,
  /ego_status 로 되돌아오는 front_steer_angle 을 읽는다.
  되돌아온 값이 평평해지는 지점이 진짜 천장이다.

  덤으로 front_steer_angle 의 단위(deg 인지 rad 인지)도 여기서 판명된다.
  브릿지는 "front_steer_deg" 라고 이름을 붙였지만 확인된 적이 없다.

사용법
  rosrun path_tracking diag_steer.py
"""
import time
from math import degrees

import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

# 시험할 조향 명령 [rad]. 0.5 가 지금 MAX_STEER, 0.698 이 규정상 차량 한계(40도).
STEER_STEPS = [0.0, 0.1, 0.2, 0.3, 0.4, 0.45, 0.5, 0.55, 0.6, 0.698]

HOLD_SEC = 2.0        # 각 단계를 유지하는 시간 (조향이 그 각까지 돌아갈 시간을 준다)
SETTLE_SEC = 1.0      # 앞쪽 이 시간은 버리고, 뒤쪽만 평균 낸다 (과도응답 제외)
RATE_HZ = 20.0

STEER_RATIO_CORRECTION = 0.70   # 브릿지와 같은 값 (표에 패킷값을 같이 보여주려고)


class SteerDiag:
    def __init__(self):
        self.readback = None      # /ego_status 의 front_steer_angle (단위 미상)
        self.samples = []         # 현재 단계에서 모은 되돌아온 값들

        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.ego_callback)

    def ego_callback(self, msg):
        self.readback = msg.front_steer_angle

    def send(self, steer_rad):
        """브레이크를 밟은 채 조향만 명령한다."""
        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.accel = 0.0
        cmd.brake = 1.0            # 차를 세워둔다
        cmd.front_steer = steer_rad
        self.cmd_pub.publish(cmd)

    def run(self):
        # /ego_status 가 올 때까지 기다린다 (브릿지가 떠 있어야 한다)
        rospy.loginfo('[diag_steer] /ego_status 대기중...')
        while self.readback is None and not rospy.is_shutdown():
            self.send(0.0)
            time.sleep(0.1)
        rospy.loginfo('[diag_steer] 연결됨. 시험 시작 (차는 브레이크로 정지 상태)')

        print('')
        print(' 명령(rad)  명령(deg)  패킷(deg)  되돌아온값   샘플수')
        print(' ---------  ---------  ---------  ----------  ------')

        results = []
        for target in STEER_STEPS:
            if rospy.is_shutdown():
                break
            self.samples = []
            t0 = time.time()
            while time.time() - t0 < HOLD_SEC and not rospy.is_shutdown():
                self.send(target)
                # 과도응답 구간을 지난 뒤부터만 샘플로 인정
                if time.time() - t0 > SETTLE_SEC and self.readback is not None:
                    self.samples.append(self.readback)
                time.sleep(1.0 / RATE_HZ)

            avg = sum(self.samples) / len(self.samples) if self.samples else float('nan')
            packet_deg = degrees(target) / STEER_RATIO_CORRECTION
            print('   %6.3f     %6.2f     %6.2f     %8.3f  %6d'
                  % (target, degrees(target), packet_deg, avg, len(self.samples)))
            results.append((target, avg))

        self.report(results)

    def report(self, results):
        """되돌아온 값이 어디서 평평해지는지 = 천장을 찾는다."""
        print('')
        valid = [(c, r) for c, r in results if r == r]      # NaN 제외
        if len(valid) < 2:
            print('  판정 불가 - 샘플이 모이지 않았다. 브릿지/시뮬 상태를 확인할 것.')
            return

        peak = max(r for _, r in valid)
        # 최대치의 99% 이상에 처음 도달한 명령값 = 그 위로는 올려도 소용없는 지점
        saturate_at = next(c for c, r in valid if abs(r) >= abs(peak) * 0.99)

        print('  되돌아온 최대값 : %.3f' % peak)
        print('  포화 시작 명령  : %.3f rad (%.2f deg)' % (saturate_at, degrees(saturate_at)))
        if saturate_at < max(STEER_STEPS) - 1e-6:
            print('  -> 이 위로는 명령을 올려도 실제 조향이 안 늘어난다.')
            print('     MAX_STEER 를 %.3f 보다 크게 잡는 것은 의미가 없다.' % saturate_at)
        else:
            print('  -> 시험 범위 안에서 포화가 안 보였다. MAX_STEER 를 올릴 여지가 있다.')

        # 단위 판정: 명령(rad) 대비 되돌아온 값의 비율을 본다
        ref = [(c, r) for c, r in valid if 0.05 < c < saturate_at - 1e-9]
        if ref:
            ratio_rad = sum(r / c for c, r in ref) / len(ref)
            print('')
            print('  되돌아온값 / 명령(rad) 평균 비율 = %.3f' % ratio_rad)
            print('   ~1.0  이면 되돌아온 값의 단위는 rad (이름과 달리 deg 아님)')
            print('   ~57   이면 deg  (이름대로)')
            print('   ~1.43 이면 rad 인데 0.70 보정이 실제로는 안 걸린 것')


def main():
    rospy.init_node('diag_steer', anonymous=True)
    diag = SteerDiag()
    try:
        diag.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 끝나면 조향 0 + 브레이크로 마무리 (MORAI 는 마지막 명령을 계속 물고 있다)
        for _ in range(20):
            diag.send(0.0)
            time.sleep(0.05)


if __name__ == '__main__':
    main()
