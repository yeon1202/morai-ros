#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_lattice : lattice_planner 회피 선택을 시뮬 없이 검증한다.

가짜 /local_path(직선), /ego_status, /Object_topic 을 쏘고 /lattice_path 를 받아
"기준경로에서 횡으로 얼마나 벗어난 경로를 골랐는가" 를 잰다. 시뮬레이터도
주행도 필요 없다. roscore 만 있으면 된다.

사용법
  roscore &
  rosrun path_tracking test_lattice.py
"""
import sys
import time
from math import hypot

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, ObjectStatus
from visualization_msgs.msg import MarkerArray

LANE_WIDTH = 3.51          # lattice_planner.cpp 와 같은 값
PATH_LEN   = 140           # LOCAL_PATH_SIZE
STEP       = 0.6           # waypoint 간격 [m]


def straight_path(origin=(0.0, 0.0)):
    """+x 방향 직선 기준경로. 횡오차를 y 로 바로 읽을 수 있어 검증이 쉽다.

    origin 을 주면 그 좌표에서 시작한다. 정적장애물 미션 예외는 장애물의 절대
    좌표로 판정하므로, 그 근처로 경로를 옮겨야 게이트가 열린다.
    """
    p = Path()
    p.header.frame_id = 'map'
    for i in range(PATH_LEN):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = origin[0] + i * STEP
        ps.pose.position.y = origin[1]
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def ego(speed_kmh=20.0, y=0.0, origin=(0.0, 0.0)):
    """y 는 기준경로에서의 횡오차. 좌측이 +, 우측이 - 다."""
    e = EgoVehicleStatus()
    e.position.x = origin[0]
    e.position.y = origin[1] + y
    e.heading = 0.0
    e.velocity.x = speed_kmh        # 브릿지가 km/h 원본을 그대로 넘긴다
    return e


def objects(items, pedestrians=()):
    """items/pedestrians = [(x, y, size)] -> ObjectStatusList

    size 는 한 변 길이다. lattice 는 반경을 0.5*max(size.x, size.y) 로 잡고
    최소 0.3m 를 보장한다.
    """
    lst = ObjectStatusList()

    def make(x, y, size):
        o = ObjectStatus()
        o.position.x = x
        o.position.y = y
        o.size.x = size
        o.size.y = size
        return o

    for x, y, size in items:
        lst.obstacle_list.append(make(x, y, size))
    for x, y, size in pedestrians:
        lst.pedestrian_list.append(make(x, y, size))
    return lst


class Harness:
    def __init__(self):
        self.got = None
        self._origin = (0.0, 0.0)
        self.pub_path = rospy.Publisher('/local_path', Path, queue_size=1)
        self.pub_ego  = rospy.Publisher('/ego_status', EgoVehicleStatus, queue_size=1)
        self.pub_obj  = rospy.Publisher('/Object_topic', ObjectStatusList, queue_size=1)
        self.cands = None
        self._obs = []
        self.avoid = None
        rospy.Subscriber('/lattice_path', Path, self.cb)
        rospy.Subscriber('/lattice_candidates', MarkerArray, self.cand_cb)
        rospy.Subscriber('/speed_limit/avoid', Float64, self.avoid_cb)
        time.sleep(1.0)                     # 연결이 붙을 시간

    def cb(self, msg):
        self.got = msg

    def cand_cb(self, msg):
        self.cands = msg

    def avoid_cb(self, msg):
        self.avoid = msg.data

    def dump_candidates(self):
        """후보별 최종 offset 과 판정을 찍는다.

        lattice 가 /lattice_candidates 에 색으로 알려준다.
        초록=선택, 빨강=충돌, 회색=여유.
        """
        if not getattr(self, 'cands', None):
            print('    (후보 마커 없음)')
            return
        print('    후보   최종offset  판정')
        for m in self.cands.markers:
            if not m.points:
                continue
            off = max((p.y for p in m.points), key=abs)
            if m.color.g > 0.9 and m.color.r < 0.5:
                verdict = '선택'
            elif m.color.r > 0.9 and m.color.g < 0.5:
                verdict = '충돌'
            else:
                verdict = '여유'
            # 장애물이 놓인 x=15m 지점에서 실제로 얼마나 벌어져 있는지
            near = min(m.points, key=lambda q: abs(q.x - 15.0))
            # 마커 점들로부터 장애물까지의 실제 최소거리 (판정의 근거)
            dmin, dat = None, None
            for ox, oy, osz in (self._obs or []):
                thr = max(0.3, 0.5 * osz) + 0.95 + 0.5
                for q in m.points:
                    d = hypot(q.x - ox, q.y - oy)
                    if dmin is None or d < dmin:
                        dmin, dat = d, (q.x, q.y, thr)
            extra = ''
            if dmin is not None:
                extra = '  최소거리 %.3f (임계 %.3f, x=%.1f y=%+.3f)' % (
                    dmin, dat[2], dat[0], dat[1])
            print('    %2d   %+8.2f m   %s   (x=%.1f 에서 y=%+.3f)%s'
                  % (m.id, off, verdict, near.x, near.y, extra))

    def lateral_g(self, poses, v_kmh):
        """고른 경로의 최대 횡가속도를 G 로 환산한다.

        폴리라인의 곡률을 약 2m 간격 세 점의 외접원으로 잰다.
        (외접원 반지름 R = abc / (2*|외적|). abc/|외적| 로 쓰면 2R 이 나온다.)
        횡가속도 = v^2 / R.
        """
        pts = [(q.pose.position.x, q.pose.position.y) for q in poses]
        if len(pts) < 9:
            return 0.0
        v = v_kmh / 3.6
        worst = 0.0
        step = 4                      # 0.5m 간격 * 4 = 2m
        for i in range(step, len(pts) - step):
            (x1, y1), (x2, y2), (x3, y3) = pts[i-step], pts[i], pts[i+step]
            a = hypot(x2-x1, y2-y1); b = hypot(x3-x2, y3-y2); c = hypot(x3-x1, y3-y1)
            cr = abs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1))
            if cr < 1e-9 or a*b*c < 1e-12:
                continue
            R = a*b*c / (2*cr)
            worst = max(worst, v*v / R)
        return worst / 9.8

    def first_touch_x(self, poses):
        """고른 경로가 차선을 처음 밟는 x [m]. 끝까지 안 밟으면 None.

        차로 안 여유는 편도 (3.51 - 1.892)/2 = 0.809m 다. 경로의 횡변위가 이를
        넘는 지점부터 바퀴가 실선에 닿는다. 규정은 접촉 3초당 5초라 "얼마나
        비켜났나" 보다 "얼마나 오래 밟았나" 가 점수를 정한다.
        """
        LANE_EDGE = 0.809
        for p in poses:
            if abs(p.pose.position.y - self._origin[1]) > LANE_EDGE:
                return p.pose.position.x - self._origin[0]
        return None

    def run_case(self, name, obs, expect, tol=0.25, peds=(), speed=20.0, max_g=None,
                 touch_x_min=None, ego_y=0.0, origin=(0.0, 0.0), expect_avoid_kmh=None):
        """obs 를 놓고 lattice 가 고른 경로의 최대 횡변위를 잰다.

        touch_x_min 을 주면 "차선을 이보다 앞에서 밟지 않는다" 까지 함께 본다.
        ego_y 를 주면 차를 경로에서 옆으로 띄운 채 시작한다(복귀 검증용).
        """
        path = straight_path(origin)
        e = ego(speed, ego_y, origin)
        o = objects(obs, peds)
        self._obs = list(obs)
        self._origin = origin
        self.got = None

        # 입력을 충분히 쏜 뒤 "마지막" 결과를 쓴다.
        #   lattice 는 30Hz 타이머로 돌기 때문에, 새 장애물을 쏜 직후 도착하는
        #   첫 /lattice_path 는 이전 케이스 입력으로 계산된 것일 수 있다.
        #   첫 메시지를 그대로 믿으면 케이스가 서로 오염된다.
        t0 = time.time()
        while time.time() - t0 < 1.2:
            self.pub_path.publish(path)
            self.pub_ego.publish(e)
            self.pub_obj.publish(o)
            time.sleep(0.05)

        if self.got is None or not self.got.poses:
            print('  %-34s 결과 없음 (lattice_planner 가 떠 있나?)' % name)
            return False

        # 기준경로가 y=0 직선이므로 횡변위 = |y|. 부호도 같이 본다.
        ys = [p.pose.position.y - origin[1] for p in self.got.poses]
        far = max(ys, key=abs)
        ok = abs(abs(far) - abs(expect)) <= tol and (expect == 0 or far * expect > 0)
        gtxt = ''
        if max_g is not None:
            g = self.lateral_g(self.got.poses, speed)
            gtxt = '  횡가속 %.2fG(<=%.2f)' % (g, max_g)
            if g > max_g:
                ok = False
        if expect_avoid_kmh is not None:
            # expect_avoid_kmh = 'inf' 이면 "제한 없음(1e6)" 을 기대한다는 뜻이다.
            got_kmh = None if self.avoid is None else self.avoid * 3.6
            shown = '없음' if got_kmh is None else (
                '무제한' if got_kmh > 999 else '%.1f km/h' % got_kmh)
            want = '무제한' if expect_avoid_kmh == 'inf' else '%.1f km/h' % expect_avoid_kmh
            gtxt += '  avoid=%s(기대 %s)' % (shown, want)
            if got_kmh is None:
                ok = False
            elif expect_avoid_kmh == 'inf':
                ok = ok and got_kmh > 999
            else:
                ok = ok and got_kmh <= 999 and abs(got_kmh - expect_avoid_kmh) <= 2.0
        if touch_x_min is not None:
            tx = self.first_touch_x(self.got.poses)
            gtxt += '  차선접촉 x=%s(>=%.1f)' % (
                '%.1f' % tx if tx is not None else '없음', touch_x_min)
            if tx is not None and tx < touch_x_min:
                ok = False
        print('  %-30s 선택 %+6.2f m  기대 %+6.2f m  %s%s'
              % (name, far, expect, 'OK' if ok else '** 불일치 **', gtxt))
        if not ok:
            self.dump_candidates()
        return ok


def main():
    rospy.init_node('test_lattice', anonymous=True)
    h = Harness()

    print('')
    print('lattice 후보 선택 검증 (LANE_WIDTH=%.2f)' % LANE_WIDTH)
    print('  충돌 판정 임계 = 장애물반경 + CAR_HALF_WIDTH(0.95) + SAFE_MARGIN(0.5)')
    print('')

    results = []

    # 장애물 없음 -> 기준경로 그대로
    results.append(h.run_case('장애물 없음', [], 0.0))

    # 작은 상자(r=0.5, 임계 1.95m).
    #   후보는 S자라 장애물 위치(x=15m)에서는 아직 최종 offset 에 도달하지 않는다.
    #   20km/h 에서 전이 길이 xf=24m 이므로 x=15 에서의 실제 횡변위는 offset*0.684 다.
    #     -2.0 -> 1.37m (임계 미달, 충돌)   -3.0 -> 2.05m (통과)
    #   그래서 충돌을 면하는 가장 작은 회피는 -3.0 이다.
    results.append(h.run_case('작은 상자 r=0.5 -> 최소 회피', [(15.0, 0.0, 1.0)], -2.0,
                              max_g=0.35))

    # 큰 상자(r=1.0, 임계 2.45m) -> 어떤 후보도 못 피한다.
    #   최대 후보 -3.51 조차 x=14.5 에서 최소거리가 2.35m 로 임계에 못 미친다.
    #   전부 막히면 가장 싼 후보(제자리)를 내보내고 경고를 띄운다. 실제 정지는
    #   behavior/ACC 몫이다. 즉 이 상황은 lattice 가 아니라 종방향이 풀어야 한다.
    results.append(h.run_case('큰 상자 r=1.0 -> 최소 회피', [(15.0, 0.0, 2.0)], -3.0,
                              max_g=0.35))

    # 우측이 전부 막히면 마지막 수단으로 좌측(중앙선 너머)을 쓴다.
    #   피할 수 있는 크기(r=0.5)여야 의미가 있다. 경로 위 상자 하나로 0/-1/-2 를
    #   막고, 우측 -2.0m 지점의 상자로 -3.0/-3.51 까지 막는다. 남는 것은 +3.51 뿐.
    results.append(h.run_case('우측 전부 막힘 -> 좌측 최후수단',
                              [(15.0, 0.0, 1.0), (15.0, -2.0, 1.0)], +LANE_WIDTH))

    # 보행자는 회피 대상이 아니다. 경로 한복판에 있어도 꺾지 않는다(정지는 FSM 몫).
    results.append(h.run_case('보행자 경로 위 -> 꺾지 않음', [], 0.0,
                              peds=[(15.0, 0.0, 1.0)]))

    # --- 전이 길이가 속도 비례인지 ---
    #
    # 예전에는 하한 20점(약 24m) 이 50km/h 이하를 전부 덮어써서 저속에서 너무
    # 느긋했다. 15m 앞 큰 상자(r=1.0, 임계 2.45m)에 닿았을 때 2.40m 밖에 못
    # 비켜나 전부 막힘 처리됐다. 시간 기준(2.68초)으로 바꾸면 20km/h 에서
    # 전이 길이가 14.9m 라 같은 상자를 피할 수 있어야 한다.
    #   -3.0 으로 이미 3.00m 가 확보되므로(임계 2.45m) 그보다 큰 회피는 불필요하다.
    results.append(h.run_case('저속 20km/h 큰 상자 -> 회피 가능',
                              [(15.0, 0.0, 2.0)], -3.0, speed=20.0, max_g=0.35))

    # 고속에서는 반대로 전이 길이가 길어져 횡가속도가 한계 안에 들어와야 한다.
    # 예전에는 26m 로 짧아 0.72G 를 요구했다(우리 한계 0.3G 의 2.4배).
    results.append(h.run_case('고속 55km/h 큰 상자 -> 0.3G 이내',
                              [(35.0, 0.0, 2.0)], -3.0, speed=55.0, max_g=0.35))

    # 트리거는 되지만 후보 범위 밖에 있는 장애물.
    #   objectOnPath() 는 local_path 전체(140점, 약 84m)를 훑는데, 후보 경로는
    #   속도로 정해지는 end_idx(20km/h 에서 약 24m) + tail 12점까지만 뻗는다.
    #   그래서 60m 앞 장애물은 트리거는 시키지만 어떤 후보와도 충돌하지 않는다.
    #   예전에는 0 후보가 없어서, 이럴 때도 가장 싼 +-1.0 을 골라 60m 나 앞의
    #   장애물 때문에 즉시 차선을 침범했다. 지금은 0 이 가장 싸므로 제자리를 지킨다.
    results.append(h.run_case('60m 앞 장애물 (후보 범위 밖)', [(60.0, 0.0, 2.0)], 0.0))

    # --- 기동을 늦게 시작하는가 (2026-08-19) ---
    #
    # 예전에는 후보 곡선이 항상 차 바로 앞(x=0)에서 시작했다. 후보 0 이 막히는
    # 순간 곧바로 옆으로 나가기 시작하는데, 55km/h 에서 그 순간은 장애물 약 47m
    # 앞이고 -2.0m 회피에 실제로 필요한 길이는 30.9m 뿐이다. 남는 16m 를 차선
    # 밟은 채로 흘려보내고 있었다.
    #
    # 지금은 "장애물 5m 앞에서 기동이 끝나도록" 역산해 시작을 미룬다.
    #   45m 앞 작은 상자(r=0.5, 임계 1.95m) -> -2.0m 로 통과 (2.0 > 1.95)
    #   전이 30.9m, 시작 x≈7.8m  ->  차선 접촉은 x≈21m 부터
    #   예전에는 전이 40.9m 를 x=0 부터 써서 접촉이 x≈17.9m 에서 시작됐다.
    #
    # 앞의 케이스들은 장애물이 15~35m 로 가까워 시작점이 0 으로 잘린다. 지연
    # 경로를 실제로 타는 것은 이 케이스뿐이라, 이걸 빼면 회귀가 안 잡힌다.
    results.append(h.run_case('45m 앞 상자 -> 늦게 시작', [(45.0, 0.0, 1.0)], -2.0,
                              speed=55.0, max_g=0.35, touch_x_min=19.0))

    # 경로에서 충분히 비껴 있어 트리거조차 안 되는 장애물 -> 기준경로 그대로.
    #   임계 = 0.3 + 0.95 + 0.5 = 1.75m 이고 장애물은 2.6m 떨어져 있다.
    results.append(h.run_case('경로 옆 2.6m 장애물 (트리거 안 됨)', [(15.0, 2.6, 0.6)], 0.0))

    # --- 회피가 끝난 뒤: 기준경로를 그대로 낸다 (2026-08-19) ---
    #
    # 장애물이 판정에서 빠지는 순간을 재현한다. 차는 아직 우측 2.3m 에 나가 있고
    # 장애물은 없는 상태다. 이때 발행되는 것은 기준경로 그대로여야 한다(횡변위 0).
    #
    # ⚠️ 여기에 "복귀 곡선"(차의 현재 위치에서 시작해 기준경로로 수렴하는 경로)을
    # 넣어봤다가 되돌렸다. 실차에서 차선 이탈이 31m/8.6초 -> 103m/12.9초 로
    # 크게 나빠졌다. 복귀 곡선의 끝점이 "차에서 L 앞" 이라 차를 따라 도망갔고,
    # pure_pursuit 이 보는 오차가 늘 0 에 가까워 되돌아오질 않았다.
    # 다시 시도한다면 끝점을 도로 위 한 지점에 고정해야 한다(lattice_planner.cpp
    # run() 주석 참고). 이 케이스는 그 회귀를 막는다.
    results.append(h.run_case('회피 직후 -> 기준경로 그대로', [], 0.0, ego_y=-2.3, speed=38.0))

    # --- 정적장애물 미션 전용 예외 (2026-08-19) ---
    #
    # 장애물 좌표가 시나리오의 그 정적장애물(-60.610, -142.178)일 때만
    # "감속 요청 + 횡가속 0.5G 완화" 가 켜진다. 고주로 등 다른 구간에서 같은
    # 처리를 하면 고속 주행 중 24km/h 상한이 걸려 훨씬 위험하기 때문이다.
    MX, MY = -60.610, -142.178
    MOBS = (MX, MY, 3.0)          # size 3.0 -> 반경 1.5, 임계 2.95m

    # 일반 장애물에는 제한을 걸지 않는다. 이 케이스가 고주로 안전장치다.
    results.append(h.run_case('일반 장애물 -> 감속 요청 없음', [(15.0, 0.0, 2.0)], -3.0,
                              expect_avoid_kmh='inf'))

    # 미션 장애물이 45m 앞 - 아직 멀어서 상한이 느슨하다.
    #   d = 45.0 - 13.0(span) = 32m -> sqrt(6.78^2 + 2*2*32) = 13.2 m/s
    results.append(h.run_case('미션 장애물 45m -> 완만한 상한', [MOBS], -3.0,
                              speed=50.0, origin=(MX - 45.0, MY),
                              expect_avoid_kmh=47.5))

    # 미션 장애물이 13m 앞 - 기동 시작점에 닿았으므로 하한값이 나온다.
    #   span 13m 안에 3.0m 를 0.5G 로 옮길 수 있는 속도 = sqrt(4.9*13^2/(6*3.0))
    results.append(h.run_case('미션 장애물 13m -> 24.4km/h 하한', [MOBS], -3.0,
                              speed=25.0, origin=(MX - 13.0, MY),
                              expect_avoid_kmh=24.4))

    print('')
    ok = all(results)
    print('%s  (%d/%d)' % ('PASS' if ok else 'FAIL', sum(results), len(results)))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
