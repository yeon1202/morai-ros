# ACC (Adaptive Cruise Control) 설계문서

작성일: 2026-07-15
작성: planning (solo)
상태: 설계 승인됨 → 구현 대기

관련 문서: [31-lattice_code_review.md](31-lattice_code_review.md), [20-perception_interface.md](20-perception_interface.md), [21-localization_interface.md](21-localization_interface.md)

---

## 1. 목적과 범위

lattice가 **횡방향(조향) 회피**를 담당하듯, ACC는 **종방향(속도)** 을 앞차/장애물 기반으로 지능형 제어한다. 현재 종제어는 `path_tracker.py` 안의 임시 단순 로직(`speed<target`이면 accel, 아니면 brake)뿐이다.

### 담당 범위 (이번 구현)
- **NPC 앞차 추종**: 앞 차량과 안전거리 유지, 상대속도 기반 감속/추종
- **정적장애물 대응**: 경로 막힘 시 감속/정지
- **60kph 하드캡**: 대회 제한

### 범위 밖 (behavior FSM으로 미룸)
- 신호등 정지 (traffic light, object type 3)
- 보행자 급정지 (트랙 갓 잡혀도 즉발동 — behavior 몫)
- 곡률 기반 속도 프로파일 최적화 (Phase6)

---

## 2. 아키텍처

lattice와 대칭인 **순수 planning 노드**. `/ctrl_cmd`는 절대 건드리지 않는다(충돌 방지).

```
                    /Object_topic (perception, 지금은 mock)
                         │
  /local_path ──┐        │
  /lattice_path ─┼──▶ [ acc_planner.cpp ] ──▶ /target_velocity (Float64, m/s)
  /ego_status ──┘   (C++, path_tracking 패키지)          │
                                                          ▼
                                    control팀 추종기(정식) / path_tracker.py(임시 검증)
                                                → /ctrl_cmd
```

- 새 노드: `path_tracking/src/acc_planner.cpp` (lattice_planner.cpp 옆, C++ 원칙 준수)
- 출력 `/target_velocity` = **planning→control 인터페이스** (§4)
- `path_tracker.py`는 임시 검증용으로 이 토픽을 구독하도록 수정 (control팀 정식 노드가 나오면 대체됨)

### 왜 별도 노드인가
`path_tracker.py`는 control팀 개발이 덜 되어 임시로 만든 스탠드인이다. waypoint 추종(조향+페달)은 원래 control팀 몫이므로, ACC(planning 산출물)를 거기 박으면 안 된다. lattice가 `/lattice_path`를 내듯 ACC는 `/target_velocity`를 내고, control이 소비한다.

---

## 3. 데이터 흐름 & 제어 로직 (30Hz 타이머)

### 3.1 추종경로 선택
`path_tracker`와 **동일 규칙**: `/lattice_path`가 최근 0.3초 내 수신되었고 점이 2개 이상이면 그것을, 아니면 `/local_path`를 기준경로로 사용. → lattice가 횡으로 피하는 중이면 그 경로 기준으로 객체를 보므로 **ACC와 lattice가 협력**(피한 장애물엔 불필요한 브레이크 안 밟음, 못 피하면 자연 정지).

### 3.2 전방객체(lead) 탐색
- 대상: `objs_.npc_list`(움직이는 차) + `objs_.obstacle_list`(정적)만. `pedestrian_list`·신호는 제외.
- 각 객체를 기준경로 각 점과 비교해 **횡거리 < distance_threshold**(경로 위, `distance_threshold`는 파라미터)인 것만 후보.
- 후보 중 **ego 기준 상대거리(직선거리) 최소**인 하나를 lead로 선택 (레퍼런스 `local_position.distance()`와 동일).

### 3.3 목표속도 계산 — 레퍼런스 SSAFY 표준식 그대로
(`MORAI-RoboticsExample/AD/.../planning/adaptive_cruise_control.py` 재사용)

```
gap        = (lead 상대거리) − vehicle_length
velocity_error = ego_vel − lead_vel
distance_error = safe_distance − gap
safe_distance  = ego_vel × time_gap + default_space

acceleration = −(velocity_gain × velocity_error + distance_gain × distance_error)
target_vel   = min(ego_vel + acceleration, cruise_speed)

if gap < default_space:   target_vel = 0     # 정지
```

- `default_space`: 차량 5m (보행자/신호는 범위 밖이라 단일 값으로 단순화 가능)
- 파라미터: `velocity_gain`, `distance_gain`, `time_gap`, `vehicle_length`, `cruise_speed`

### 3.4 캡 & 클램프
```
target_vel = min(target_vel, 60 km/h(=16.67 m/s))
target_vel = max(target_vel, 0)
```

### 3.5 객체 없을 때
`target_vel = cruise_speed` (캡 적용) — 순수 크루즈.

---

## 4. 메시지 인터페이스 `/target_velocity`

- 타입: **`std_msgs/Float64`**
- 단위: **m/s** (localization `/odom` twist.linear와 동일 단위계)
- 발행 주기: 30Hz
- **control팀 계약**: 미수신 또는 오래됨(>0.3s)이면 ACC 미동작으로 간주하고 **안전하게 정지로 폴백**할 것. (별도 `docs/22-target_velocity_interface.md`로 명세 예정)

---

## 5. 엣지/안전 처리

| 상황 | 처리 |
|------|------|
| 경로 완전 막힘(lattice 후보 전부 충돌) | lead가 코앞 → `gap < default_space` → **target_vel = 0** |
| `/ego_status`·기준경로 미수신 | 발행 보류 → control이 폴백 정지 |
| 객체 velocity 노이즈 | perception Kalman 적용분 신뢰(메모리 합의), ACC 추가 필터 없음 |
| 정적장애물(velocity ≈ 0) | lead_vel=0으로 자연 처리 → 거리 유지 후 정지 |
| lead 없음 | 크루즈 속도 |

---

## 6. 검증 (오프라인 mock 우선 — lattice와 동일 방식)

1. **mock 시나리오**: `mock_obstacle_pub.cpp` 확장 또는 신규 mock으로 **느린 앞차(움직이는 NPC)** 를 `/Object_topic`에 발행 (기존 mock은 정적장애물 위주).
2. **검증 항목**
   - ① 앞차보다 느린 속도로 수렴 → 안전거리 유지
   - ② 정적장애물 앞에서 정지
   - ③ 앞이 비면 크루즈 속도로 복귀
   - ④ 60kph 캡 준수
3. **관측**: `rostopic echo /target_velocity` + RViz(lead 마커/경로) 확인.
4. 오프라인 통과 후 → MORAI 실차 확인.

---

## 7. 파일 변경 요약

| 파일 | 변경 |
|------|------|
| `path_tracking/src/acc_planner.cpp` | **신규** — ACC planning 노드 |
| `path_tracking/CMakeLists.txt` | acc_planner 빌드 타깃 추가 |
| `path_tracking/launch/sim.launch` | acc_planner 노드 추가 |
| `path_tracking/src/mock_*.cpp` | 움직이는 앞차 시나리오 추가/확장 |
| `path_tracking/scripts/path_tracker.py` | (임시) `/target_velocity` 구독해 종제어에 사용 |
| `docs/22-target_velocity_interface.md` | **신규** — control팀 인계용 인터페이스 명세 |
