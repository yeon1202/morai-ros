# Lattice Planner (횡방향 회피) 설계문서

작성일: 2026-07-16
작성: planning (solo)
상태: **구현·오프라인검증 완료** (정적장애물 회피 end-to-end 확인, 회피 2.65m)

관련 문서: [acc_design.md](acc_design.md), [lattice_code_review.md](lattice_code_review.md)(3차곡선 유도·좌표변환 수식 상세), [perception_interface.md](perception_interface.md), [fot_theory.md](fot_theory.md)

---

## 1. 목적과 범위

ACC가 **종방향(속도)** 을 담당하듯, lattice는 **횡방향(조향 회피)** 을 담당한다. 경로 위 장애물을 좌우로 벌린 후보경로들을 그려 충돌검사하고, 안전하면서 비용 최소인 후보를 골라 회피 지역경로를 만든다.

### 담당 범위
- **정적장애물·NPC·보행자 회피**(횡): 경로 위 장애물을 좌우로 우회
- **차선변경성 기동**: 좌우 offset 후보로 자연스러운 우회
- 못 피하는 경우(전 후보 충돌)는 **경고만** 내고 정지는 ACC/behavior에 위임

### 범위 밖
- **종방향 속도 제어**: ACC 몫 ([acc_design.md](acc_design.md))
- **정지 판단**: 다 막혔을 때 실제 정지는 ACC/behavior FSM
- **시간축 궤적·동적장애물 예측**: FOT(stretch goal, [fot_theory.md](fot_theory.md))

---

## 2. 아키텍처

ACC와 대칭인 **순수 planning 노드**. `/ctrl_cmd`는 건드리지 않는다.

```
                    /Object_topic (perception, 지금은 mock)
                         │
  /local_path ──┐        │
  /ego_status ──┼──▶ [ lattice_planner.cpp ] ──▶ /lattice_path (nav_msgs/Path)  → 제어 추종
                │    (C++, path_tracking 패키지)  └▶ /lattice_candidates (MarkerArray) → RViz
                └────────┘
```

- 노드: `path_tracking/src/lattice_planner.cpp` (30Hz 타이머)
- 출력 `/lattice_path` = 횡방향 의도(회피 지역경로). ACC의 `/target_velocity`와 짝을 이뤄 control이 둘 다 소비.
- 회피 불필요 시 `/local_path`를 그대로 통과발행(오버헤드 최소화).

---

## 3. 데이터 흐름 & 로직 (30Hz `run()`)

```
① gatherObstacles   장애물을 원(중심+반경)으로 수집 (npc+보행자+정적)
② objectOnPath      경로 위 장애물 있나? 없으면 local_path 그대로 발행하고 종료
③ generateCandidates 좌우 offset 6개 후보경로 생성 (좌표변환 + 3차곡선)
④ selectLane        각 후보 충돌검사 → 비용 계산 → 최소비용 argmin 선택
⑤ publish           선택 경로 → /lattice_path, 후보 전체 → /lattice_candidates
```

### 3.1 장애물 모델 (gatherObstacles)
- `npc_list + pedestrian_list + obstacle_list` 전부 수집 (횡회피는 보행자도 대상 — ACC와 다름)
- 각 객체 → **원**: 반경 `r = 0.5 * max(size.x, size.y)`, 최소 0.3m
- 원 근사 → 충돌검사가 `거리 < 반경합`으로 단순

### 3.2 회피 트리거 (objectOnPath)
- 경로 점 × 장애물 원이 `d < r + CAR_HALF_WIDTH + SAFE_MARGIN` 이면 "회피 필요"
- 없으면 lattice 생략(비용 절감)

### 3.3 후보경로 생성 (generateCandidates) — 핵심
- **좌표변환**: local_path 시작점 + 진행방향 `theta`로 world↔local 프레임 정의. 후보를 차 진행방향 기준 local에서 그린 뒤 world로 복귀.
- **횡 offset 6개**(`LANE_OFFSET`)마다 시작 횡위치(현재 차) → 끝 횡위치(offset)로 **부드럽게 잇는 3차곡선**:
  - 경계조건: `y(0)=ps, y'(0)=0, y(xf)=pf, y'(xf)=0` (시작·끝 기울기 0 → 부드러운 진입/복귀)
  - 계수: `a0=ps, a1=0, a2=3(pf-ps)/xf², a3=-2(pf-ps)/xf³`
- 후보 뒤쪽은 기준경로를 따라 연장
- 상세 유도는 [lattice_code_review.md](lattice_code_review.md)

### 3.4 후보 선택 (selectLane)
- 비용 = `BASE_WEIGHT`(중앙 선호 {3,2,1,1,2,3}) + 충돌 시 `COLLISION_PENALTY(100)`
- `argmin(weight)` 선택 → 안 부딪히는 가장 중앙 후보
- **전 후보 충돌** 시 `ROS_WARN_THROTTLE`("다 막힘 - ACC/behavior 정지 필요")

---

## 4. 메시지 인터페이스

| 토픽 | 방향 | 타입 | 내용 |
|------|------|------|------|
| `/local_path` | 구독 | `nav_msgs/Path` | 기준 지역경로(앞 구간) |
| `/ego_status` | 구독 | `morai_msgs/EgoVehicleStatus` | 현재 위치·속도(개발용, 나중 `/odom`) |
| `/Object_topic` | 구독 | `morai_msgs/ObjectStatusList` | 장애물(npc/보행자/정적) |
| `/lattice_path` | 발행 | `nav_msgs/Path` | 선택된 회피경로 → 제어 추종 |
| `/lattice_candidates` | 발행 | `visualization_msgs/MarkerArray` | 후보 전체 시각화(초록=선택 빨강=충돌 회색=여유) |

---

## 5. 엣지/안전 처리

| 상황 | 처리 |
|------|------|
| 경로 위 장애물 없음 | lattice 생략, `/local_path` 그대로 발행 |
| 후보 생성 실패(경로 짧음 등) | `/local_path` 그대로 발행 |
| 전 후보 충돌(다 막힘) | 경고 발행, 최소비용 후보 반환(정지는 ACC/behavior 몫) |
| 경로 끝 인덱스 초과 | `end_idx = min(look*2, n-1)`로 clamp |
| 장애물 size 반영 | 반경에 `0.5*max(size.x,size.y)` 적용 |
| 데이터 미수신 | 발행 보류(`has_path_/has_ego_/has_obj_` 가드) |

---

## 6. 검증

- **오프라인 mock**: `mock_obstacle_pub`(정적장애물) + `object_viz` + RViz(`/lattice_candidates`)
- **결과**: 정적장애물 회피 end-to-end 검증 완료 — 회피 2.65m 벗어남, `path_tracker`가 `/lattice_path` 추종(front_steer 0.223rad)
- 후속: MORAI 실차 회피 확인

---

## 7. Frenet(FOT) 확장 훅 — 나중에 시간축 갈아끼우기 쉽게

지금 구조는 사실상 (s, d) 격자의 단순형이다. 아래를 의식해두면 FOT 이식 비용이 최소화된다. (상세: [frenet-ready-design], docs/fot_theory.md)

| 현재 | Frenet 확장 방향 |
|------|------------------|
| `LANE_OFFSET`(좌우 벌림) | **횡변위 `d` 후보**로 명시 (Frenet d축) |
| `X_INTERVAL`(x 간격) | **종방향 누적거리 `s` 샘플간격**으로 명시 |
| `weight` 한 배열에 기본+충돌 합산 | **비용 항 분리**(횡offset항 / 충돌항 / …) → 나중 **종방향 항(속도·시간) 삽입** 쉽게 |
| 3차곡선(횡만) | 횡 5차 + 종 4/5차 다항식(시간축)으로 확장 |
| 정적 충돌검사 | 동적장애물 예측 충돌검사 |

---

## 8. 파라미터 (튜닝 포인트)

| 파라미터 | 값 | 의미 |
|----------|-----|------|
| `LANE_OFFSET` | {-3.0,-1.75,-1.0,1.0,1.75,3.0} | 후보 횡변위 후보(=d) [m] |
| `BASE_WEIGHT` | {3,2,1,1,2,3} | 중앙 선호 기본비용 |
| `CAR_HALF_WIDTH` | 0.95 | 차폭 1.892/2 [m] |
| `SAFE_MARGIN` | 0.5 | 충돌 안전여유 [m] |
| `COLLISION_PENALTY` | 100.0 | 충돌 후보 벌점 |
| `X_INTERVAL` | 0.5 | 후보경로 점 간격(=s 간격) [m] |
