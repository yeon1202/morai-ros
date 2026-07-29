#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .point import Point
import numpy as np
from math import sqrt,pi 


class PathManager:
    # --- 최근접 waypoint 탐색 범위 ---
    # 차는 경로를 순서대로 지나가므로, 매번 전체를 훑을 필요가 없다.
    # 직전 인덱스 주변만 보면 경로가 자기 자신과 겹쳐도 엉뚱한 구간으로 튀지 않는다.
    SEARCH_BACK = 10         # 직전 인덱스에서 뒤로 볼 범위 [점] (정지·후진·노이즈 대비)
    SEARCH_AHEAD = 100       # 앞으로 볼 범위 [점] (0.5m 간격 기준 약 60m)
    RESET_DISTANCE = 10.0    # 창 안에서 이보다 멀면 창을 잘못 물고 있는 것 -> 전역 재탐색 [m]

    def __init__(self, path, is_closed_path, local_path_size):
        self.path = path
        self.is_closed_path = is_closed_path
        self.local_path_size = local_path_size
        self.velocity_profile = []

        self.current_waypoint = None  # 직전 최근접 인덱스 (None이면 전역탐색)
        self.cte = 0.0                # 최근접 waypoint 까지 거리 = 경로 이탈량 [m]
        self.relocated = 0            # 전역 재탐색이 일어난 횟수 (진단용)

    def set_velocity_profile(self, max_velocity, road_friction, window_size):
        # TODO: moving window를 설정하는 방식 개선.
        max_velocity = max_velocity / 3.6
        velocity_profile = []
        for i in range(0, window_size):
            velocity_profile.append(max_velocity)

        target_velocity = max_velocity

        for i in range(window_size, len(self.path)-window_size):
            x_list = []
            y_list = []
            for window in range(-window_size, window_size):
                x = self.path[i+window].x
                y = self.path[i+window].y
                x_list.append(x)
                y_list.append(y)

            x_start  = x_list[0]
            x_end    = x_list[-1]
            x_mid    = x_list[int(len(x_list)/2)]

            y_start  = y_list[0]
            y_end    = y_list[-1]
            y_mid    = y_list[int(len(y_list)/2)]

            dSt = np.array([x_start - x_mid, y_start - y_mid])
            dEd = np.array([x_end - x_mid, y_end - y_mid])

            Dcom = 2 * (dSt[0]*dEd[1] - dSt[1]*dEd[0])

            dSt2 = np.dot(dSt,dSt)
            dEd2 = np.dot(dEd,dEd)

            U1 = (dEd[1] * dSt2 - dSt[1] * dEd2)/Dcom
            U2 = (dSt[0] * dEd2 - dEd[0] * dSt2)/Dcom

            tmp_r = sqrt(pow(U1, 2)+ pow(U2, 2))

            if np.isnan(tmp_r):
                tmp_r = float('inf')

            target_velocity = sqrt(tmp_r*9.8*road_friction)

            if target_velocity > max_velocity:
                target_velocity = max_velocity

            velocity_profile.append(target_velocity)

        for i in range(len(self.path)-window_size, len(self.path) - 10):
            velocity_profile.append(max_velocity)

        for i in range(len(self.path) - 10,len(self.path)):
            velocity_profile.append(0.)

        self.velocity_profile = velocity_profile

        print('velocity_profile',velocity_profile)

    def _scan(self, position, indices):
        """indices 중 position 에 가장 가까운 (인덱스, 거리[m]) 를 찾는다."""
        best_i, best_d2 = None, float('inf')
        for i in indices:
            dx = self.path[i].x - position.x
            dy = self.path[i].y - position.y
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2, best_i = d2, i
        return best_i, sqrt(best_d2)

    def find_nearest_waypoint(self, vehicle_state):
        """직전 인덱스 주변만 탐색해서 최근접 waypoint 를 찾는다.

        전체를 훑으면, 경로가 자기 자신과 겹치는 곳(코스 진입/완주 지점,
        왕복 구간)에서 수백 인덱스 떨어진 엉뚱한 점이 뽑힌다. 그러면
        local path 가 텅 비거나 역주행 경로가 나온다.
        """
        position = vehicle_state.position
        n = len(self.path)

        if self.current_waypoint is None:      # 첫 호출 - 기억이 없으니 전체 탐색
            return self._scan(position, range(n))

        lo = self.current_waypoint - self.SEARCH_BACK
        hi = self.current_waypoint + self.SEARCH_AHEAD
        if self.is_closed_path:
            window = [i % n for i in range(lo, hi + 1)]
        else:
            window = range(max(0, lo), min(n, hi + 1))

        index, distance = self._scan(position, window)

        if distance > self.RESET_DISTANCE:
            # 창 안에 가까운 점이 없다 = 경로에서 크게 벗어났거나 창을 놓쳤다.
            # 전역 재탐색으로 한 번 복구를 시도한다.
            index, distance = self._scan(position, range(n))
            self.relocated += 1

        return index, distance

    def _perpendicular_distance(self, position, index):
        """경로 '선분'에 수직 투영한 진짜 횡오차.

        최근접 waypoint 까지의 거리로 재면 안 된다. waypoint 간격이 최대 1.0m 라
        차가 경로 위에 정확히 있어도 최대 0.5m 로 측정된다. 실제로 이 오차 때문에
        "차선 변경 구간 CTE 0.69m" 로 잘못 읽고 튜닝 방향을 헤맨 적이 있다.
        """
        n = len(self.path)
        best = float('inf')
        for i in range(max(0, index - 3), min(n - 1, index + 3)):
            ax, ay = self.path[i].x, self.path[i].y
            vx = self.path[i+1].x - ax
            vy = self.path[i+1].y - ay
            len2 = vx*vx + vy*vy
            if len2 < 1e-12:
                continue
            t = ((position.x - ax)*vx + (position.y - ay)*vy) / len2
            t = max(0.0, min(1.0, t))          # 선분 밖이면 끝점으로
            d = sqrt((position.x - (ax + t*vx))**2 + (position.y - (ay + t*vy))**2)
            if d < best:
                best = d
        return best if best < float('inf') else 0.0

    def get_local_path(self, vehicle_state):
        current_waypoint, _ = self.find_nearest_waypoint(vehicle_state)
        self.current_waypoint = current_waypoint
        self.cte = self._perpendicular_distance(vehicle_state.position, current_waypoint)

        if current_waypoint + self.local_path_size < len(self.path):
            local_path = self.path[current_waypoint:current_waypoint + self.local_path_size]
        else:
            local_path = self.path[current_waypoint:]
            # 연결된 경로 (closed path) 일 경우, 경로 끝과 처음을 이어준다.
            if self.is_closed_path:
                local_path += self.path[:self.local_path_size + len(self.path) - current_waypoint]

        return local_path, self.velocity_profile[current_waypoint]