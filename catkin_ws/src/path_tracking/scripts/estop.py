#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
비상정지 - /ctrl_cmd 에 brake=1 을 잠깐 퍼부어 차를 세운다.

왜 필요한가
  MORAI 는 마지막으로 받은 /ctrl_cmd 를 계속 물고 있는다. path_tracker 를 그냥
  죽이면 차는 마지막 accel 명령으로 계속 가속한다(실측 5.4 -> 17.4 m/s).
  path_tracker 에는 SIGINT 를 가로채 제동하는 장치가 들어있지만, kill -9 나
  노드 crash 로 그 경로를 못 타는 경우가 있어 별도 수단을 남겨둔다.

  예전에는 이걸 컨테이너 /tmp/stop.py 에 만들어 썼는데, GPU 복구용으로
  docker compose down/up 을 하면 매번 날아가서 다시 만들어야 했다.
  catkin_ws 는 호스트 마운트라 여기 두면 컨테이너를 다시 만들어도 남는다.

사용법
  rosrun path_tracking estop.py
  rosrun path_tracking estop.py _sec:=10        # 더 오래 밟기
"""
import time

import rospy
from morai_msgs.msg import CtrlCmd

RATE_HZ = 20.0


def main():
    rospy.init_node('estop', anonymous=True)
    duration = rospy.get_param('~sec', 6.0)

    pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
    time.sleep(0.5)          # 구독자(브릿지) 연결이 붙을 시간을 준다

    cmd = CtrlCmd()
    cmd.longlCmdType = 1
    cmd.accel = 0.0
    cmd.brake = 1.0
    cmd.front_steer = 0.0

    rospy.loginfo('[estop] %.1f초 동안 제동', duration)
    n = int(duration * RATE_HZ)
    for _ in range(n):
        pub.publish(cmd)
        # rospy.Rate/rospy.sleep 은 종료 중에 ROSInterruptException 을 던져
        # 제동이 도중에 끊긴 적이 있다. 여기서는 time.sleep 을 쓴다.
        time.sleep(1.0 / RATE_HZ)
    rospy.loginfo('[estop] 완료')


if __name__ == '__main__':
    main()
