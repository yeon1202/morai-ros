#!/usr/bin/env python3
"""
키보드로 /ctrl_cmd (morai_msgs/CtrlCmd) 발행 - 제어 테스트용
------------------------------------------------------------
  W : 가속(accel +)      S : 브레이크(brake +)
  A : 좌조향             D : 우조향
  Space : 정지(brake=1, steer=0)      X : 조향 중립
  Q : 종료
front_steer 단위는 라디안. 브릿지가 0.70 보정 후 MORAI로 전송.
"""
import sys
import termios
import tty
import select
import rospy
from morai_msgs.msg import CtrlCmd

HELP = __doc__


def get_key(timeout=0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key


def main():
    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
    accel = brake = steer = 0.0
    print(HELP)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        k = get_key().lower()
        if k == 'w':
            accel = min(accel + 0.1, 1.0); brake = 0.0
        elif k == 's':
            brake = min(brake + 0.1, 1.0); accel = 0.0
        elif k == 'a':
            steer = min(steer + 0.03, 0.5)
        elif k == 'd':
            steer = max(steer - 0.03, -0.5)
        elif k == 'x':
            steer = 0.0
        elif k == ' ':
            accel = 0.0; brake = 1.0; steer = 0.0
        elif k == 'q':
            break

        cmd = CtrlCmd()
        cmd.longlCmdType = 1      # Throttle (대회 규정)
        cmd.accel = accel
        cmd.brake = brake
        cmd.front_steer = steer
        pub.publish(cmd)
        print(f"\raccel={accel:.2f} brake={brake:.2f} steer={steer:+.2f}rad   ", end='')
        sys.stdout.flush()
        rate.sleep()

    # 종료 시 정지 명령
    stop = CtrlCmd(); stop.longlCmdType = 1; stop.brake = 1.0
    pub.publish(stop)
    print("\n[teleop] 종료")


if __name__ == '__main__':
    main()
