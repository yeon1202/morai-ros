# perception 연동 설계 — mock 장애물을 실제 인지로 교체

2026-08-26 작성. 짝이 되는 구현 계획은 [25-perception_integration_plan.md](25-perception_integration_plan.md).
인터페이스 계약 자체는 [20-perception_interface.md](20-perception_interface.md) 참고.

## 1. 목적

`mock_obstacle_pub` 이 발행하던 **하드코딩된 정적장애물 1개**를 걷어내고, 팀 perception
스택이 실제로 검출한 물체로 `/Object_topic` 을 채운다. planning 쪽 소비자
(`lattice_planner`, `acc_planner`, `object_viz`)는 **한 줄도 고치지 않는다.**

범위는 인지 전체다 — 라이다 클러스터링 + 카메라 YOLO 분류 + 융합 + 추적. 라이다만
쓰면 분류가 없어 모든 물체가 정적장애물이 되고, 그러면 보행자는 정지 대신 회피
대상이 되고 앞차는 ACC 추종 대신 회피 대상이 된다(설계 의도와 어긋난다).

## 2. 지금 상태

```
mock_obstacle_pub ──20Hz──> /Object_topic ──> lattice_planner / acc_planner / object_viz
  시나리오 좌표 (-60.610, -142.178) 하드코딩, 항상 1개
```

팀 perception 은 이미 돌아가는 코드가 있고(`autonomous_driving/src/perception/`),
종점이 `/perception/tracked_objects` 다. **팀은 `/Object_topic` 변환을 안 한다.**
`object_fusion_node.py:4` 주석에 *"(Planning 쪽 /Object_topic 스펙이 정적장애물도 항상
발행하길 요구하기 때문)"* 이라고 적혀 있다 — 그 변환이 planning 몫이라는 전제다.

## 3. 전체 구조

```
/lidar/points ──> lidar_node ─────────┐
                                      ├─> object_fusion ──> /perception/recognized_objects
/camera1/… ──> camera_detection ──────┘                            │ (lidar 프레임)
                    (YOLO, GPU)                                    ▼
                                                          global_transform_node
                                                       tf: lidar→base_link→odom
                                                                   │
                                              /perception/recognized_objects_global
                                                                   ▼
                                                            tracking_node
                                                     (칼만 추적 → velocity + unique_id)
                                                                   │
                                                    /perception/tracked_objects
                                                                   ▼
                                            ★ object_topic_adapter  (신규, planning)
                                                                   │
                                                            /Object_topic
                                                                   ▼
                                          lattice_planner / acc_planner / object_viz
```

★ 하나만 새로 만든다. 팀 노드 5개는 **무수정**으로 쓴다.

## 4. 설계 결정과 근거

### 4.1 좌표 변환은 팀 `global_transform_node` 를 그대로 쓴다

그 노드는 tf `lidar → base_link → odom` 으로 물체를 전역 좌표로 옮긴다. `odom` 은
EKF 가 만든다. **2026-08-26 실측에서 EKF 횡오차가 RMS 0.79~0.97 m(최대 7.5 m)로 확인**
됐으므로(원자료 `catkin_ws/logs/lat_*_pilot*.csv`, 분석은
`scripts/analyze_latency.py`), 그 오차가 장애물 위치에 그대로 실린다. 회피 여유 1.4 m 의 상당 부분을 갉아먹는다.

이를 피하는 두 대안을 검토하고 **채택하지 않았다**:

| 대안 | 내용 | 안 쓰는 이유 |
|---|---|---|
| 자체 변환 노드 | `/ego_status`(GT)로 직접 변환 | 팀 노드와 같은 일을 하는 노드가 하나 더 생긴다. EKF 를 고치면 걷어내야 할 코드다 |
| `map→odom` 을 GT 보정으로 동적 발행 | REP-105 표준 | 회전 합성이 들어가 이해·디버깅 부담이 커진다. 대회 당일 혼자 봐야 한다 |

**EKF 횡오차는 우회할 문제가 아니라 고칠 문제다.** 다음 실험(`ekf.yaml` 의
`odom1_config` 에서 vy 끄기)이 이미 잡혀 있고, 그게 해결되면 이 결정이 저절로
정답이 된다. 지금 우회로를 만들면 나중에 걷어내야 한다.

**단, 그때까지는 장애물 위치에 1 m 급 횡오차가 실린다는 것을 알고 써야 한다.**
회피 여유 판정을 이 상태의 실측으로 다시 잡으면 안 된다.

### 4.2 어댑터에 물체별 잠금(latch)을 넣지 않는다

`tracking_node` 가 이미 한다 — `MIN_HITS_TO_CONFIRM = 3`(3프레임 연속 잡혀야 발행),
`MAX_MISSES = 5`(5프레임 연속 놓쳐야 삭제). 어댑터에 또 넣으면 이중 지연이 생기고,
두 곳의 파라미터가 어긋나면 원인 추적이 어려워진다.

어댑터가 책임지는 것은 **토픽 자체가 끊기는 경우** 뿐이다(4.4).

### 4.3 `size` 의 x/y 정의 차이는 문제되지 않는다

팀 `RecognizedObject.size` 는 `x=length(주축), y=width(부축)` 이고 `mock_obstacle_pub`
은 `x=width, y=length` 로 반대다. 그런데 `/Object_topic` 의 `size` 를 쓰는 곳은
`lattice_planner` 의 `gatherObstacles()` 하나뿐이고 `0.5 * max(size.x, size.y)` 로
**외접원**을 쓴다 — 순서와 무관하다. `acc_planner` 와 `behavior_fsm` 은 `size` 를
아예 안 본다. 그대로 옮긴다.

### 4.4 인지가 끊기면 빈 목록을 낸다

`lattice_planner::objCb` 는 받은 것을 덮어쓰기만 한다. 인지가 죽으면 **마지막
장애물을 영원히 믿는다** — 이미 지나간 장애물을 계속 피하려 든다.

어댑터는 `/perception/tracked_objects` 가 **0.5초** 안 오면 빈 목록을 발행하고
경고를 한 번 낸다. "장애물 없음"은 `acc_planner.cpp:132` 주석대로 정상 상태이므로
planning 은 이를 안전하게 처리한다.

### 4.5 `/Object_topic` 계약을 유지한다

planning 이 팀 메시지 타입(`RecognizedObjectArray`)을 직접 구독하게 바꾸면
`mock_obstacle_pub` 과 `test_lattice.py` 오프라인 검증(16/16)이 전부 깨진다.
대회 6주 전에 검증 수단을 잃는 건 위험하다. 어댑터 한 겹을 두는 값이 그보다 싸다.

## 5. 새로 만드는 것 — `object_topic_adapter.py`

`catkin_ws/src/path_tracking/scripts/` 에 둔다. planning 쪽 책임이므로.
`path_tracking` 이 `autonomous_driving` 메시지에 의존하게 되므로 `package.xml` /
`CMakeLists.txt` 에 의존을 명시한다(단방향, 순환 없음).

| | |
|---|---|
| 구독 | `/perception/tracked_objects` (`autonomous_driving/RecognizedObjectArray`) |
| 발행 | `/Object_topic` (`morai_msgs/ObjectStatusList`), `frame_id = "map"`, **20 Hz 고정** |

발행 주기를 인지 프레임에 묶지 않고 20 Hz 로 고정한다 — mock 과 같아서 소비자
입장에서 바뀌는 게 없고, 인지가 느려져도 planning 주기가 흔들리지 않는다.

### 변환 규칙

| RecognizedObject | → | ObjectStatus | 비고 |
|---|---|---|---|
| `type` 0 | → | `pedestrian_list` | 보행자 |
| `type` 1 | → | `npc_list` | NPC 차량 |
| `type` 2 | → | `obstacle_list` | 정적장애물(미분류 포함) |
| `type` -1 | → | 버림 | 자차 |
| `unique_id` | → | `unique_id` | 추적 ID 그대로 |
| `class_name` | → | `name` | |
| `center` | → | `position` | 이미 전역 좌표 |
| `size` | → | `size` | 4.3 참고 |
| `yaw` [rad] | → | `heading` [deg] | **단위 변환** |
| `velocity` [m/s] | → | `velocity` [km/h] | **단위 변환**. `acc_planner.cpp:121` 이 `speedKmhToMps` 로 읽는다 |

`num_of_npcs` / `num_of_pedestrian` / `num_of_obstacle` 도 각 목록 길이로 채운다.

## 6. 고치는 것

| 파일 | 변경 |
|---|---|
| `autonomous_driving/launch/localization.launch` | `base_link → lidar` static tf 추가 (`1.4 0 1.23`, 회전 없음) |
| `autonomous_driving/launch/perception.launch` | **신규.** 인지 5개 노드를 한 번에 |
| `path_tracking/launch/sim.launch` | `perception:=true/false` 인자. true 면 인지+어댑터, false 면 mock (**기본 false**) |
| `path_tracking/package.xml`, `CMakeLists.txt` | `autonomous_driving` 의존 추가 |
| `Dockerfile` | torch(cu128) + ultralytics |

**`base_link → lidar` tf 가 지금 아예 없다.** `/tf` 에는 `odom→base_link`(EKF),
`/tf_static` 에는 `base_link→gps` 뿐이다. 그래서 `global_transform_node` 를 지금
켜면 *"Could not obtain transform from lidar to odom"* 으로 아무것도 못 낸다.
값은 MORAI 센서 설정과 일치시킨다: **x=1.4, y=0, z=1.23** (2026-08-26 설정 완료,
지면 반사 z 분포로 교차 확인됨).

기본값을 `perception:=false` 로 두는 이유는 4.5 와 같다. 인지가 충분히 검증될
때까지 mock 경로를 살려 둔다.

## 7. 의존성

| 패키지 | 용도 | 비고 |
|---|---|---|
| `torch` (cu128 빌드) | YOLO | **RTX 5070 은 Blackwell(sm_120)**. 일반 cu121 빌드는 실행 시 `no kernel image is available` 로 죽는다 |
| `ultralytics` | YOLO 래퍼 | `yolo11n.pt` 는 첫 실행 때 자동 다운로드(인터넷 필요) |
| `python3-sklearn`, `python3-scipy` | DBSCAN / 헝가리안 매칭 | 2026-08-26 설치 완료 |

GPU 여유는 확인했다 — MORAI 가 사용률 21~34%, VRAM 4.1/8.1 GB 를 쓰므로
`yolo11n`(VRAM 1 GB 미만)이 들어갈 자리가 있다.

## 8. 알려진 한계

1. **장애물 위치에 EKF 횡오차 ~1 m 가 실린다** (4.1). EKF 를 고치기 전까지는
   이 상태의 실측으로 회피 여유를 재조정하지 말 것.
2. **카메라-라이다 캘리브레이션은 팀 센서 배치를 전제한다.** `calibration.py` 가
   `LIDAR_TO_CAM_T = [0.5, 0, -0.03]` 을 못박고 있다. 센서를 다시 옮기면 이 값도
   같이 바꿔야 하고, 안 그러면 라이다가 잡은 물체에 엉뚱한 카메라 라벨이 붙는다.
3. **GPU 를 시뮬과 나눠 쓴다.** 배속이 떨어지면 시간 측정이 어긋난다
   (04-runbook.md 의 "벽시계로 시간 재지 말 것"). 인지를 켠 뒤 측정할 때는 배속을 반드시 같이 기록한다.
4. **`isMissionObstacle` / `stoplineS` 의 하드코딩 좌표는 그대로 둔다.** 이번 범위
   밖이다. 다만 인지 위치가 프레임마다 떨리므로 `MISSION_MATCH_R` 안에 안정적으로
   들어오는지 실주행에서 확인이 필요하다.

## 9. 검증 계획

순서대로 하나씩 확인한다. 앞 단계가 안 되면 뒤는 볼 필요가 없다.

1. **tf** — `base_link→lidar` 넣고 `global_transform_node` 가 경고 없이 도는가
2. **YOLO** — `/camera1/detections` 에 검출이 뜨는가. GPU 를 실제로 쓰는가(`nvidia-smi`)
3. **추적** — `/perception/tracked_objects` 에 물체가 `unique_id` 를 달고 나오는가
4. **인터페이스 대조** — 어댑터를 켜고 `/Object_topic` 이 mock 과 **같은 모양**인가.
   `rostopic echo` 로 나란히 놓고 필드별로 비교한다. **여기까지가 주행 전 관문이다.**
5. **주행** — mock 을 끄고 한 바퀴. 시나리오 정적장애물을 실제로 피하는가.
   `lap_logger` 로 여유를 재고, `/CollisionData` 로 충돌이 없었는지 확인한다.

## 10. 후속 작업 (이번 범위 밖)

- EKF 횡오차 잡기 (`odom1_config` 의 vy 끄고 재측정) — 8.1 의 근본 해결
- `isMissionObstacle` 을 인지 기반으로 재설계 — 8.4
- 인지 켠 상태에서 회피 여유 재측정 및 `SAFE_MARGIN` 재검토
