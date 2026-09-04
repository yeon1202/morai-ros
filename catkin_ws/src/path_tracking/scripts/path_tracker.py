#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_tracker : 기록한 path.csv 를 따라가는지 확인하는 (검증용) 노드
------------------------------------------------------------------
흐름:  path.csv 로드
       -> PathManager 로 현재 위치 앞 구간(local path) 추출
       -> PurePursuit 로 조향각 계산
       -> 아주 단순한 목표속도 제어로 accel/brake
       -> /ctrl_cmd 발행
RViz 확인용:  /global_path (전체 경로),  /local_path (지금 따라가는 구간)

※ 조향 정밀 튜닝(lfd, 게인 등)은 control팀 몫. 여기선 "경로 잘 따라가나" 확인용.
"""
import os
import csv
import time
import signal
from math import hypot, atan2

import rospy
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from lib.point import Point
from lib.pkg_paths import global_path_csv
from lib.vehicle_state import VehicleState
from lib.path_manager import PathManager
from lib.pure_pursuit import PurePursuit

# ---- 파라미터 (control팀이 나중에 조정할 값들) ----
TARGET_SPEED_KMH = 20.0     # 검증용 (더 올리려면 이 값만 키우기)
WHEELBASE        = 3.0      # Ioniq5 축거 [m]
LFD_GAIN         = 0.5      # 전방주시거리 = LFD_GAIN * 속도 (속도 빠를수록 멀리 봄)
                            #   0.7 -> 0.5 로 낮춤. 경로에 기록된 차선 변경(횡 3.8m를
                            #   44m 에 걸쳐 이동)에서 0.7 이면 lfd 가 9.3m 나 되어 S자를
                            #   가로질러 버린다. 오프라인 측정(수직투영 기준 횡오차):
                            #     0.7 -> 0.360m | 0.6 -> 0.277m | 0.5 -> 0.189m | 0.4 -> 0.111m
                            #   더 줄이면 더 붙지만 실제 차량은 조향 지연·타이어 특성이
                            #   있어 너무 짧으면 사행(weaving)한다. 0.5 에서 확인 후 조정.
MIN_LFD          = 4.0      # 3.0 과 5.0 을 모두 시험해 4.0 이 최적임을 확인했다.
                            #   유턴 구간은 곡률 리미터가 속도를 4 m/s 아래로
                            #   낮추므로 LFD_GAIN*v = 0.5*4 = 2.0 < MIN_LFD 가 되어
                            #   lfd 가 내내 이 값에 고정된다. 즉 이 구간에서 실제로
                            #   작동하는 파라미터는 LFD_GAIN 이 아니라 MIN_LFD 다.
                            #
                            #   유턴 실측 (2026-07-29):
                            #                    3.0     4.0     5.0
                            #     최악 CTE      1.90m   1.79m   1.87m
                            #     탈출후 진동   0.53m   0.21m   0.22m
                            #     최대 조향요구 0.968   0.808   0.709 rad
                            #
                            #   두 효과가 상충한다. 짧게 보면 경로에 밀착하려 하지만
                            #   조향 요구가 폭증해 MAX_STEER(0.65)에 잘리고 예민해진다.
                            #   길게 보면 포화는 풀리지만 코너를 가로질러 버린다.
                            #   양쪽 다 4.0 보다 나빴으므로 이 파라미터는 닫힌 문제다.
                            #   남은 개선은 유턴 진입 속도를 낮추거나(lat_accel_limit)
                            #   접근구간을 다시 기록하는 쪽이다. docs/42 7.2 참고.
MAX_LFD          = 20.0
LOCAL_PATH_SIZE  = 140      # 앞으로 볼 waypoint 개수 (약 0.6m 간격 → 약 84m)
                            #   ACC 가 이 경로 위에서 앞차를 찾으므로 ACC 지평선을 결정한다.
                            #   60km/h(16.67m/s) 기준:
                            #     평형 간격 = 16.67*1.0 + 5 + 4.635 = 26.3m
                            #     제동거리  = v^2/2a = 46m (감속 3m/s^2)
                            #     반응 여유 = 4초 * 16.67 = 67m
                            #   → 84m. lattice 는 자체 end_idx 로 앞 30m 만 쓰므로 영향 없음.
IS_CLOSED_PATH   = True    # 경로가 안 닫혀있음(시작≠끝). 폐곡선 코스로 다시 따면 True
MAX_STEER        = 0.65     # 안전 클램프 [rad] (약 37도) - 큰 조향각 튐 방지
                            #   0.5 -> 0.65. 0.5(28.6도)는 우리가 스스로 채운 족쇄였다.
                            #   diag_steer.py 실측: 명령한 각이 그대로 실제 조향각이 되고
                            #   (브릿지의 /0.70 보정이 정확히 상쇄된다) 0.698rad(40도)까지
                            #   잘림 없이 들어간다. 즉 차가 낼 수 있는 40도 중 28.6도만
                            #   쓰고 있었다.
                            #   문제가 된 곳: 접근구간 유턴(idx 150~170, 최소 R=6.03m)은
                            #   필요 조향이 26.4도라 0.5 에서 여유가 2.2도뿐이었다. 조금만
                            #   바깥으로 밀리면 곧바로 포화 -> 더 밀림 -> 회복 불가 ->
                            #   MAX_CTE 가드로 정차. 0.65 면 여유 10.8도.
                            #   차량 한계인 0.698 을 다 쓰지 않는 이유: 시뮬이 잘라내는
                            #   지점에 명령을 걸치지 않도록 약간 남겨둔다.
ACC_TIMEOUT      = 0.5      # /target_velocity 가 이보다 오래 끊기면 자체 속도로 폴백 [s]

# 종방향 가감속 게인 (control팀 정식 PID로 대체될 임시값).
#   출력 = BASE + GAIN * |속도오차[m/s]|  , 0~1 로 clamp
#   제동을 가속보다 세게 잡는다: 곡선 진입 전 목표속도까지 확실히 떨어뜨려야
#   커브에서 조향이 포화되지 않는다. 감속이 굼뜨면 진입속도가 높아져 선을 밟는다.
ACCEL_BASE       = 0.3
ACCEL_GAIN       = 0.2
BRAKE_BASE       = 0.2      # 0.1 -> 0.2
BRAKE_GAIN       = 0.6      # 0.3 -> 0.6
MAX_CTE          = 6.0      # 경로 이탈 한계 [m]. 넘으면 정지.
                            #   pure_pursuit 은 차가 경로 위에 있다고 가정하는 알고리즘이라,
                            #   크게 벗어나면 목표점이 옆으로 90도 방향에 잡혀 최대조향으로
                            #   폭주한다(2m만 벗어나도 이미 포화). lattice 최대 offset 이
                            #   3.0m 이므로 회피 중 정상 이탈과는 겹치지 않게 6m 로 둔다.


class PathTracker:
    def __init__(self):
        # 1) 전역경로 로드.
        #    파일 위치는 lib/pkg_paths 가 package.xml 을 찾아 올라가서 정한다
        #    (개발용과 팀 repo 의 폴더 깊이가 달라서 dirname 횟수를 세면 틀린다).
        csv_path = global_path_csv(__file__)
        rospy.loginfo('[path_tracker] 경로 파일: %s', csv_path)
        self.path = self._load_path(csv_path)
        if len(self.path) < LOCAL_PATH_SIZE:
            rospy.logwarn('waypoint가 %d개뿐. LOCAL_PATH_SIZE보다 적음', len(self.path))
        rospy.loginfo('[path_tracker] waypoint %d개 로드', len(self.path))

        # 2) 플래너 준비
        self.path_manager = PathManager(self.path, IS_CLOSED_PATH, LOCAL_PATH_SIZE)
        # 검증용: 일정 목표속도(곡률 기반 속도프로파일은 control팀이 나중에)
        self.target_mps = TARGET_SPEED_KMH / 3.6
        self.path_manager.velocity_profile = [self.target_mps] * len(self.path)

        self.pure_pursuit = PurePursuit(LFD_GAIN, WHEELBASE, MIN_LFD, MAX_LFD)

        # 3) ROS 입출력
        # 위치·헤딩은 /odom(GPS+IMU 융합), 속도는 /ego_status(속도계).
        # 왜 나눴는지는 odom_callback 주석 참고.
        self.odom_xy = None              # (x, y) [m]
        self.odom_yaw = None             # [rad]
        self.odom_stamp = None
        self.odom_timeout = rospy.get_param('~odom_timeout', 0.5)   # 초
        self._odom_warned = None

        self.lattice_points = None       # lattice가 준 회피경로 (list of Point)
        self.lattice_stamp = None
        self.acc_velocity = None         # ACC가 준 목표속도 [m/s]
        self.acc_stamp = None

        # 제어 출력을 낼지 (기본 켬). 팀 vehicle_control 을 쓸 때 false 로 둔다.
        #   rosrun  : _control:=false
        #   launch  : <param name="control" value="false"/>
        #
        # ⚠️ 노드 자체를 끄면 안 된다. 이 노드는 제어기이기 전에 경로 공급원이다
        #    (/global_path, /local_path -> lattice_planner). 끄면 회피가 통째로
        #    멈춘다. 그래서 노드는 살리고 제어 출력만 끄는 스위치를 둔다.
        #
        # ⚠️ 제어기 둘이 동시에 /ctrl_cmd 를 내면 서로 덮어쓰는데 양쪽 로그는
        #    정상으로 보인다. 반드시 한쪽만 켤 것.
        self.control_enabled = rospy.get_param('~control', True)
        if not self.control_enabled:
            rospy.loginfo('[path_tracker] 제어 출력 꺼짐 - 경로 공급만 한다 '
                          '(/ctrl_cmd 는 다른 제어기 몫)')
        self.cmd_pub = (rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
                        if self.control_enabled else None)
        self.gpath_pub = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        self.lpath_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        rospy.Subscriber('/lattice_path', Path, self.lattice_callback)
        rospy.Subscriber('/target_velocity', Float64, self.acc_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.callback)

        self.gpath_pub.publish(self._to_path_msg(self.path))   # 전체 경로 1회 발행

        # MORAI 는 마지막 /ctrl_cmd 를 계속 물고 있으므로, 노드를 그냥 끄면
        # 차가 마지막 accel 명령으로 계속 가속한다. 종료 시 반드시 제동을 걸어준다.
        # (실제 제동은 SIGINT 핸들러에서 - brake_now() 주석 참고)
        self._braked = False
        self._finish_logged = False            # 완주 로그를 한 번만 찍기 위한 플래그
        rospy.on_shutdown(self.brake_now)      # kill -TERM 등 최후 수단

        rospy.loginfo('[path_tracker] 시작 - 목표속도 %.0f km/h', TARGET_SPEED_KMH)

    def brake_now(self):
        """제동을 걸고 확실히 전달될 때까지 반복 발행한다.

        ※ 반드시 노드가 살아있는 동안(=SIGINT 핸들러에서) 호출할 것.
          rospy.on_shutdown 훅 안에서 publish 하면 연결이 이미 정리되기 시작해
          메시지가 조용히 버려진다. 실측으로 확인했다 - 훅 로그는 정상적으로
          찍히는데 차는 마지막 accel 명령으로 2.8m/s -> 11.4m/s 까지 가속했다.
        """
        if self._braked:
            return
        self._braked = True
        if self.cmd_pub is None:
            rospy.loginfo('[path_tracker] 종료 - 제어 출력이 꺼져 있어 제동은 건너뛴다')
            return
        rospy.logwarn('[path_tracker] 종료 - 제동 명령 전송')
        for _ in range(30):          # 1.5초간 반복 전송 (UDP 유실 대비)
            try:
                self._publish_stop()
            except Exception as e:
                rospy.logerr('[path_tracker] 제동 발행 실패: %s', e)
                break
            time.sleep(0.05)

    def _load_path(self, csv_path):
        points = []
        with open(csv_path) as f:
            reader = csv.reader(f)
            next(reader)                       # 헤더(x,y,z) 건너뛰기
            for row in reader:
                points.append(Point(float(row[0]), float(row[1])))
        return points

    def odom_callback(self, msg):
        """위치와 헤딩을 /odom 에서 받는다 (2026-08-29 전환).

        왜 /ego_status 를 안 쓰나
          대회 규정 채널(9109 Competition Vehicle Status)은 position 을 0,0,0 으로
          준다. 개발 중에는 9111(Ego Vehicle Status = ground truth)로 받아 썼지만
          제출본에 그걸 쓰면 실격이다. 그래서 위치는 GPS+IMU 융합 결과인 /odom
          에서 받는다.

        왜 속도는 여기서 안 받나
          /odom.twist 는 wheel_speed_scaler 가 시뮬 배속(r)을 곱해 벽시계 단위로
          바꿔놓은 값이라 실제 주행속도가 아니다(r=0.5 면 절반으로 보인다).
          속도계는 대회 채널에서도 그대로 오므로 /ego_status 에서 계속 받는다.
        """
        q = msg.pose.pose.orientation
        self.odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.odom_yaw = atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_stamp = rospy.Time.now()

    def _odom_fresh(self):
        """/odom 이 살아 있는지. 죽었으면 조향을 멈춘다.

        /ego_status 는 20Hz 넘게 오는데 /odom 은 8~10Hz 다. 그리고 EKF 가 멈추면
        (노드 죽음, GPS 음영 장기화) 위치가 그대로 얼어붙는데, 그걸 모르고 계속
        조향하면 옛 위치 기준으로 핸들을 꺾는다.
        """
        if self.odom_stamp is None:
            return False
        age = (rospy.Time.now() - self.odom_stamp).to_sec()
        if age > self.odom_timeout:
            if self._odom_warned is None or (rospy.Time.now() - self._odom_warned).to_sec() > 2.0:
                rospy.logwarn('[path_tracker] /odom 이 %.2f초째 없다. 조향을 멈춘다.', age)
                self._odom_warned = rospy.Time.now()
            return False
        return True

    def callback(self, msg):
        # 현재 차량 상태
        #   위치·헤딩 : /odom (GPS+IMU 융합)
        #   속도      : /ego_status 의 velocity. MORAI UDP 원본 그대로라 단위가
        #               km/h 다(브릿지가 변환하지 않음). m/s 로 바꿔서 쓸 것 -
        #               안 그러면 목표속도 비교가 "km/h vs m/s" 가 되어 차가
        #               20km/h 가 아니라 20/3.6 = 5.6km/h 로 기어간다.
        if not self._odom_fresh():
            return
        speed = hypot(msg.velocity.x, msg.velocity.y) / 3.6    # km/h -> m/s
        vs = VehicleState(self.odom_xy[0], self.odom_xy[1],
                          self.odom_yaw, speed)

        # 앞 구간 + (ACC 가 없을 때 쓸) 자체 목표속도
        local_path, fallback_velocity = self.path_manager.get_local_path(vs)
        target_velocity = self._target_velocity(fallback_velocity)
        self.lpath_pub.publish(self._to_path_msg(local_path))  # lattice가 이걸 받아 회피경로 생성

        # 완주했으면 정지한다.
        #
        # 우리 경로는 완주 지점과 코스 진입 지점이 물리적으로 같은 자리라, 이게
        # 없으면 경로 끝을 지나자마자 진입부 인덱스를 다시 잡아 두 바퀴째를 돈다
        # (실측: 완주 0.7초 뒤 idx 4633 -> 909 로 뛰어 계속 주행).
        # 제동거리가 있어 결승선을 지나 멈추는 것은 정상이다.
        if self.path_manager.finished:
            if not self._finish_logged:
                rospy.loginfo('[path_tracker] 완주 - 정지한다 (waypoint %d/%d)',
                              self.path_manager.current_waypoint, len(self.path))
                self._finish_logged = True
            self._publish_stop()
            return

        # 경로에서 너무 벗어났으면 조향하지 말고 정지 (근거는 MAX_CTE 주석 참고)
        if self.path_manager.cte > MAX_CTE:
            rospy.logwarn_throttle(
                1.0, '[path_tracker] 경로에서 %.1fm 이탈 (한계 %.1fm) - 정지',
                self.path_manager.cte, MAX_CTE)
            self._publish_stop()
            return

        # lattice 회피경로가 최근(0.3초 내)에 왔으면 그걸 따라감, 아니면 기준 local_path
        follow_path = local_path
        if (self.lattice_points and self.lattice_stamp and
                (rospy.Time.now() - self.lattice_stamp).to_sec() < 0.3 and
                len(self.lattice_points) > 1):
            follow_path = self.lattice_points

        # 조향 (pure pursuit)
        self.pure_pursuit.path = follow_path
        self.pure_pursuit.vehicle_state = vs
        steering = self.pure_pursuit.calculate_steering_angle()   # rad
        steering = max(-MAX_STEER, min(MAX_STEER, steering))      # 안전 클램프

        # 종방향: 목표속도까지의 오차에 비례해서 가감속 (control팀 정식 PID로 대체 예정).
        #   예전엔 accel 0.3 / brake 0.1 고정이었는데, ACC가 "정지(0.0)"를 명령해도
        #   brake 0.1 로는 거의 안 줄어서 ACC가 일하는지 확인할 수가 없었다.
        err = target_velocity - speed
        if err >= 0.0:
            accel, brake = min(1.0, ACCEL_BASE + ACCEL_GAIN * err), 0.0
        else:
            accel, brake = 0.0, min(1.0, BRAKE_BASE + BRAKE_GAIN * (-err))

        if self.cmd_pub is None:
            return                        # 제어 출력 꺼짐 (경로 공급만 한다)

        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.accel = accel
        cmd.brake = brake
        cmd.front_steer = steering        # 브릿지가 0.70 보정 적용
        self.cmd_pub.publish(cmd)

    def acc_callback(self, msg):
        self.acc_velocity = float(msg.data)      # [m/s]
        self.acc_stamp = rospy.Time.now()

    def _target_velocity(self, fallback):
        """종방향 목표속도를 고른다.

        /target_velocity 가 종방향의 단일 권한이다. ACC(그리고 나중에 behavior)가
        크루즈·곡률한계·앞차추종을 모두 반영해 최종값 하나로 내보내고, 여기서는
        그걸 그대로 따른다. 이 노드는 control 팀 정식 노드로 대체될 임시 노드이므로
        속도 정책을 여기 두면 그때 버려진다.

        ACC 가 끊기면(노드 죽음/미실행) 자체 프로파일로 폴백하되 경고를 남긴다.
        """
        if self.acc_stamp is not None and self.acc_velocity is not None:
            age = (rospy.Time.now() - self.acc_stamp).to_sec()
            if age < ACC_TIMEOUT:
                return self.acc_velocity
            rospy.logwarn_throttle(
                2.0, '[path_tracker] /target_velocity 끊김 (%.1fs) - 자체 속도로 폴백', age)
        else:
            rospy.logwarn_throttle(
                5.0, '[path_tracker] /target_velocity 수신 없음 - 자체 속도 %.1f m/s 사용',
                fallback)
        return fallback

    def _publish_stop(self):
        if self.cmd_pub is None:
            return                        # 제어 출력 꺼짐 - 정지도 다른 제어기 몫
        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.accel = 0.0
        cmd.brake = 1.0
        cmd.front_steer = 0.0
        self.cmd_pub.publish(cmd)

    def lattice_callback(self, msg):
        # lattice가 준 회피경로를 Point 리스트로 저장
        self.lattice_points = [Point(p.pose.position.x, p.pose.position.y)
                               for p in msg.poses]
        self.lattice_stamp = rospy.Time.now()

    def _to_path_msg(self, points):
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = rospy.Time.now()
        for pt in points:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(pt.x)
            ps.pose.position.y = float(pt.y)
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        return p


if __name__ == '__main__':
    rospy.init_node('path_tracker')
    tracker = PathTracker()

    # rospy 의 기본 SIGINT 핸들러를 가로챈다. 그래야 노드가 아직 살아있는 상태에서
    # 제동을 발행할 수 있다. rospy.on_shutdown 훅은 이미 늦다(brake_now 주석 참고).
    def _on_signal(signum, frame):
        tracker.brake_now()
        rospy.signal_shutdown('signal %d' % signum)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    rospy.spin()
