# 22 — planning → control 인터페이스 명세

**대상**: control 담당자
**작성**: 2026-09-02 (planning)
**놓일 자리**: 팀 repo `2026_CARSA_AD` 루트의 `docs/22-planning_control_interface.md`
**상태**: 초안. 9장 "합의가 필요한 것" 에 답을 주시면 확정한다.

> 이 문서는 **팀 repo 에서 읽는 것을 전제**로 썼다. 소스 참조는 전부
> `패키지/파일` 형태로 적었고, 값이 필요한 곳은 값을 본문에 함께 적어서
> 파일을 안 열어도 읽히게 했다.

---

## 0. 한 줄 요약

**control 노드가 자기 CSV 대신 `/lattice_path` 를 따라가고, 자기 속도 프로파일에
`/target_velocity` 를 상한으로 씌워주시면 됩니다.**

바꿀 것은 **구독 두 개 추가**와 **`/ctrl_cmd` 발행 주체 정리**뿐이다.
제어 알고리즘(MPC·PID·지연보상·조향 필터)은 그대로 두는 것을 전제로 썼다.

---

## 1. 왜 이 문서가 필요한가 — 지금 두 스택이 따로 돈다

현재 `autonomous_driving/launch/autonomous_driving.launch` 를 띄우면 이렇게 돈다.

```
/odom, /imu ──> vehicle_control ──> /ctrl_cmd
                      ↑
              자기 CSV 직접 로드 (param waypoint_csv)
              자기 속도 프로파일 생성 (곡률 기반)
```

`autonomous_driving/src/control/vehicle_control.cpp` 는 **`/lattice_path` 도
`/target_velocity` 도 구독하지 않는다.** 구독은 `/odom` 과 `/imu` 뿐이다.
즉 planning 출력이 차에 전혀 전달되지 않는다.

| | 경로 | 속도 | 장애물 | 신호등 | 출력 |
|---|---|---|---|---|---|
| control 단독 | 자기 CSV | 자기 곡률 프로파일 | **모름** | **모름** | `/ctrl_cmd` |
| planning | `/lattice_path` | ACC+신호등+회피 합성 | `/Object_topic` | `/speed_limit/traffic_light` | `/ctrl_cmd` |

**둘 다 `/ctrl_cmd` 를 발행한다.** 같이 띄우면 두 노드가 같은 토픽에 동시에 쓴다.
ROS 는 이걸 막지 않고 에러도 안 낸다. 두 명령이 번갈아 나가 차가 이상하게 움직인다.

---

## 2. 배치 전제 — 무엇이 워크스페이스에 있어야 하는가

### ⚠️ 선행 조건: `path_tracking` 패키지가 팀 repo 에 없다

2026-09-02 확인 기준, 팀 repo 5개 브랜치(main / minjjun / woonggook / yeonsoo /
seungyeon) **어디에도 `path_tracking` 이 없다.** planning 노드가 전부 이 패키지에
들어 있으므로, **올리기 전에는 이 명세를 시험할 수 없다.**

→ planning 쪽 할 일. 올린 뒤 이 줄을 지운다.

### 필요한 패키지

| 패키지 | 소유 | 팀 repo 상태 | 역할 |
|---|---|---|---|
| `autonomous_driving` | 공용 | ✅ main | localization 전처리 + **control** |
| `morai_msgs` | 공용 | ✅ main | MORAI 메시지 (`CtrlCmd`, `ObjectStatusList` 등) |
| `udp_bridge` | planning | ✅ seungyeon | MORAI UDP ↔ ROS. 센서와 `/ctrl_cmd` 송신 |
| `sim_rate` | planning | ✅ seungyeon | 시뮬 배속 보정 + localization 스택 launch |
| **`path_tracking`** | planning | ❌ **없음** | **planning 전부 (이 명세의 발행 측)** |

### 실행 순서

```
1) roslaunch udp_bridge  ...              # 센서 토픽 공급
2) roslaunch sim_rate localization.launch # /odom 생성
3) (planning launch)                      # /lattice_path, /target_velocity
4) vehicle_control                        # /ctrl_cmd
```

⚠️ planning 쪽 기존 `path_tracking/launch/sim.launch` 는 **RViz·mock 장애물·
진단 노드까지 같이 띄우므로 통합용으로는 못 쓴다.** planning 노드만 띄우는
가벼운 launch 를 planning 쪽에서 따로 만들어 제공한다.

---

## 3. 발행 노드 일람

명세에 나오는 모든 토픽의 **누가 내고 누가 받는지**다.

### planning 이 발행 — control 이 구독할 것

| 토픽 | 타입 | 발행 노드 | 패키지 / 실행파일 | 주기 |
|---|---|---|---|---|
| `/lattice_path` | `nav_msgs/Path` | `lattice_planner` | `path_tracking` / `lattice_planner` (C++) | **30 Hz** |
| `/target_velocity` | `std_msgs/Float64` | `behavior_fsm` | `path_tracking` / `behavior_fsm` (C++) | **30 Hz** |

### planning 내부 (control 은 안 봐도 되지만, 디버깅에 유용)

| 토픽 | 타입 | 발행 노드 | 패키지 / 실행파일 | 주기 |
|---|---|---|---|---|
| `/local_path` | `nav_msgs/Path` | `path_tracker` | `path_tracking` / `path_tracker.py` | 약 20 Hz |
| `/speed_limit/acc` | `std_msgs/Float64` | `acc_planner` | `path_tracking` / `acc_planner` (C++) | 30 Hz |
| `/speed_limit/avoid` | `std_msgs/Float64` | `lattice_planner` | `path_tracking` / `lattice_planner` | 30 Hz |
| `/speed_limit/active` | `std_msgs/String` | `behavior_fsm` | `path_tracking` / `behavior_fsm` | 30 Hz |
| `/lattice_candidates` | `visualization_msgs/MarkerArray` | `lattice_planner` | `path_tracking` / `lattice_planner` | 30 Hz |
| `/Object_topic` | `morai_msgs/ObjectStatusList` | `object_topic_adapter` | `path_tracking` / `object_topic_adapter.py` | 20 Hz |

`/speed_limit/active` 는 **지금 어느 제약이 속도를 결정하고 있는지**를 문자열로
알려준다. 통합 디버깅에 이게 제일 쓸모 있다.

### 양쪽이 공통으로 구독

| 토픽 | 타입 | 발행 노드 | 패키지 | 주기 |
|---|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | `ekf_localization_node` | `robot_localization` | 8~17 Hz (실측) |
| `/imu` | `sensor_msgs/Imu` | `udp_bridge` | `udp_bridge` | 17 Hz |
| `/ego_status` | `morai_msgs/EgoVehicleStatus` | `udp_bridge` | `udp_bridge` | 20 Hz |

### 출력

| 토픽 | 타입 | 현재 발행자 | **통합 후 발행자** |
|---|---|---|---|
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` | `path_tracker`(planning) **와** `vehicle_control` **둘 다** | **`vehicle_control` 만** |

---

## 4. 목표 구조

```
                    /Object_topic
                          │
   /odom ──> path_tracker ─┴─> lattice_planner ──> /lattice_path ──┐
             (path_tracking)   (path_tracking)                      │
                          └─> acc_planner ─┐                        │
                                           ↓                        │
              /speed_limit/{acc,avoid,traffic_light,pedestrian,intersection}
                                           ↓                        │
                                     behavior_fsm ──> /target_velocity
                                                                    │
                                        vehicle_control <───────────┘
                                    (autonomous_driving)
                                              ↓
                                          /ctrl_cmd ──> udp_bridge ──> MORAI
```

planning 은 **"어디로"(경로)** 와 **"얼마나 빠르게"(속도 상한)** 두 개만 준다.
**조향각·가속·브레이크를 실제로 만드는 일은 전부 control 몫이다.**

---

## 5. 계약 ① `/lattice_path` — 따라갈 경로

| 항목 | 값 |
|---|---|
| 토픽 | `/lattice_path` |
| 타입 | `nav_msgs/Path` |
| 프레임 | `map` (`header.frame_id = "map"`) |
| 발행 노드 | `lattice_planner` (패키지 `path_tracking`, C++ 실행파일 `lattice_planner`) |
| 발행 주기 | **30 Hz** (타이머 기반. 입력이 없어도 주기는 유지) |

### 내용

**항상 발행된다.** 장애물이 없어도 끊기지 않는다.

- **회피 중이 아닐 때** — 차선 중앙 경로를 그대로 중계한다. 앞 **약 70 m**
  (140점 × 0.5 m 간격)
- **회피 중일 때** — 옆으로 비켜나는 3차곡선. 전이 길이는 거리가 아니라
  **시간 기준 2.68초**라 속도에 따라 달라진다 (20 km/h 에서 15 m, 55 km/h 에서 41 m).
  이렇게 하면 횡가속도가 속도와 무관하게 일정해진다.
- 점 간격 **0.5 m**, `pose.position.z` 는 쓰지 않는다.

### control 쪽에서 할 일

**자기 CSV 대신 이걸 추종 대상으로 쓴다.** 곡률·yaw 는 기존처럼 이 경로에서 직접
계산하면 된다 (`vehicle_control.cpp` 의 `loadPathAndGenerateProfile` 이 하는 것과
같은 방식).

⚠️ **경로가 매 프레임 갱신된다는 점이 기존과 다르다.** 회피가 시작되면 경로가
옆으로 이동한다. 인덱스를 프레임 간에 들고 다니는 로직(`current_path_idx_`)이
있으면 확인이 필요하다.

---

## 6. 계약 ② `/target_velocity` — 속도 상한

| 항목 | 값 |
|---|---|
| 토픽 | `/target_velocity` |
| 타입 | `std_msgs/Float64` |
| **단위** | **m/s** ← km/h 아님 |
| 발행 노드 | `behavior_fsm` (패키지 `path_tracking`, C++ 실행파일 `behavior_fsm`) |
| 발행 주기 | **30 Hz** |
| 범위 | `0.0` ~ 순항속도 (기본 55 km/h = 15.28 m/s, rosparam `~cruise_speed_kmh`) |

### 내용

**지금 이 순간 허용되는 최대 속도**다. planning 쪽 제약을 `min` 으로 합성한 결과다.

| 제약 | 입력 토픽 | 내는 노드 |
|---|---|---|
| ACC (앞차 추종) | `/speed_limit/acc` | `acc_planner` |
| 회피 중 감속 | `/speed_limit/avoid` | `lattice_planner` |
| 신호등 정지 | `/speed_limit/traffic_light` | behavior (인계 담당) |
| 교차로 | `/speed_limit/intersection` | behavior (인계 담당) |
| 보행자 | `/speed_limit/pedestrian` | 예정 |

**이미 상승률 제한이 걸려 있다.** 목표가 갑자기 올라가도 발행값은 부드럽게
따라간다 (rosparam `~accel_rate_limit`). control 쪽에서 또 완만하게 만들 필요는 없다.

### control 쪽에서 할 일

```
실제 목표속도 = min( 자기 곡률 프로파일, /target_velocity )
```

**`min` 을 쓰는 이유**는 양쪽 안전장치를 다 살리기 위해서다.

- planning 은 곡률을 세밀하게 모른다 → control 의 횡가속 제한이 잡아준다
- control 은 앞차·신호등을 모른다 → planning 의 `/target_velocity` 가 잡아준다

둘 중 **느린 쪽이 항상 이긴다.** 어느 한쪽이 놓쳐도 차가 빨라지지 않는다.

### ⚠️ 정지 명령

**`/target_velocity = 0.0` 이면 확실히 정지해야 한다.** 신호등 정지선과 보행자가
여기로 온다.

지금 `autonomous_driving.launch` 의 `vehicle_control` 파라미터에
**`v_min_kmh = 15.0`** 이 걸려 있다. 이게 살아 있으면 **0 을 받아도 15 km/h 로
계속 간다.** 신호 위반·사고에 직결되므로 반드시 처리가 필요하다.

제안: `v_min_kmh` 는 "주행 중 하한" 으로만 쓰고, `/target_velocity` 가 문턱
(예: 0.5 m/s) 아래면 **하한을 무시하고 정지**한다.

---

## 7. 계약 ③ `/ctrl_cmd` 는 control 만 발행한다

통합 시점부터 **`/ctrl_cmd` 발행 주체는 `vehicle_control` 하나**로 한다.
planning 쪽 `path_tracker`(`path_tracking/scripts/path_tracker.py`)의 발행은 그때 끈다.

`path_tracker` 는 원래 **"경로를 잘 따라가는지" 확인하려고 만든 임시 노드**다.
조향 정밀 튜닝은 처음부터 control 몫으로 두었다(그 파일 머리주석에 그렇게 적혀 있다).

⚠️ **그 전까지는 둘을 같이 띄우면 안 된다.**

---

## 8. 폴백 — 상대가 죽었을 때

양쪽 다 상대가 없어도 안전하게 서 있어야 한다.

### control 쪽

| 상황 | 동작 |
|---|---|
| `/lattice_path` 가 **0.3초** 이상 안 옴 | 자기 CSV 로 폴백 (회피는 안 되지만 주행은 유지) |
| `/target_velocity` 가 **0.5초** 이상 안 옴 | 자기 곡률 프로파일로 폴백 |

0.3초 / 0.5초는 planning 내부에서 이미 쓰는 값과 같게 맞춘 것이다
(planning 의 lattice 신선도 0.3초, `behavior_fsm` 의 `stale_timeout` 0.5초).

⚠️ **폴백할 때 반드시 로그를 남겨야 한다.** 조용히 폴백하면 planning 이 죽은 것을
아무도 모른 채 주행이 계속된다. 회피와 신호등이 빠진 상태인데 겉보기엔 정상이다.

### planning 쪽 (우리 의무)

- `/lattice_path` 는 장애물 유무와 무관하게 **항상 30 Hz 로 발행**한다
- `/target_velocity` 는 입력 제약이 하나도 없으면 발행을 멈춘다. 이때 control 은
  위 폴백으로 간다
- 노드가 죽으면 토픽이 끊기므로, control 은 **값이 아니라 끊김**으로 판단하면 된다

---

## 9. 합의가 필요한 것 (답을 주세요)

### 9.1 `path_tracking` 을 팀 repo 어디에 올릴까

2장 참고. 루트에 형제 패키지로 두면 된다(팀 repo 는 루트가 곧 catkin `src`).
**어느 브랜치로 올릴지**만 알려주시면 된다.

### 9.2 경로 정본을 무엇으로 할 것인가

| | 파일 | 내용 |
|---|---|---|
| planning | `path_tracking/path/path_smooth.csv` | 4392점, 0.5 m 간격, 2185 m |
| control | `autonomous_driving/path/path_smooth_closed.csv` | 별도 파일 |

`/lattice_path` 를 쓰기 시작하면 control 의 CSV 는 **폴백용으로만** 쓰인다.
그래도 둘이 크게 다르면 폴백 순간에 차가 튄다. **같은 파일로 맞추는 것을 제안한다.**

### 9.3 `min` 합성을 받아들일 수 있는가

6장 참고. "`/target_velocity` 를 그대로 따르고 자기 프로파일은 버린다" 도 가능하지만,
그러면 **곡률 기반 감속을 planning 이 떠안아야 한다.** 지금 planning 에는 곡률 속도
제한이 없다. **당분간은 `min` 이 안전하다고 본다.**

### 9.4 `v_min_kmh = 15.0`

6장 ⚠️ 참고. 정지 명령을 무시하는 문제.

### 9.5 `loop_path = true`

지금 기본값이 무한 순환이다. 대회는 **완주하고 정지**해야 한다.
정지 판단을 control 이 할지(`finish_distance`), planning 이 할지
(`/target_velocity = 0`) 정해야 한다. **planning 쪽에 이미 완주 판정이 있으므로
planning 이 0 을 보내는 쪽을 제안한다.**

### 9.6 `highway_start_idx = 2230`

control 의 자기 CSV 인덱스에 묶인 값이다. `/lattice_path` 를 따르면 **인덱스가
의미를 잃는다.** 구간별 속도 제한은 `behavior_fsm` 이 이미 하는 일이므로
**planning 쪽으로 옮기는 것을 제안한다.** 어느 구간에서 몇 km/h 를 원하는지
알려주시면 반영한다.

### 9.7 전환 시점

10장의 단계를 언제 밟을지.

---

## 10. 검증 절차 — 한 번에 다 바꾸지 않는다

세 단계로 나눈다. **각 단계에서 하나씩만 바뀌므로 문제가 생기면 원인이 명확하다.**

### 단계 1 — 경로만 넘긴다

control 이 `/lattice_path` 를 구독한다. 속도는 아직 자기 것을 쓴다.
planning 의 `path_tracker` 는 `/ctrl_cmd` 발행을 끈다.

**확인**: 장애물 앞에서 차가 옆으로 비켜나는가.

```
rostopic hz   /lattice_path       # 30 Hz 근처
rostopic echo /speed_limit/avoid  # 회피 중이면 값이 뜬다
rostopic info /ctrl_cmd           # Publishers 가 하나인지 반드시 확인
```

### 단계 2 — 속도도 넘긴다

control 이 `/target_velocity` 도 구독해 `min` 을 적용한다.

**확인**: 앞차가 있을 때 감속하는가. `/target_velocity = 0` 에서 서는가.

```
rostopic echo /target_velocity     # m/s
rostopic echo /speed_limit/active  # 지금 어느 제약이 이기고 있는지
```

### 단계 3 — 통합 주행

전체를 켜고 한 바퀴. 벌점(차선·중앙선·정지선)과 완주 시간을 기록한다.

---

## 11. 참고 — control 쪽 현재 설정 중 planning 이 아는 것

명세를 쓰면서 `autonomous_driving.launch` 에서 확인한 값들이다.
**요청이 아니라 확인**이다.

| 파라미터 | 값 | planning 쪽 메모 |
|---|---|---|
| `control_rate_hz` | 20.0 | planning 은 30 Hz 로 준다. 문제 없다 |
| `delay_compensation_sec` | 0.12 | **구동 지연을 이미 보상 중.** planning 은 중복 보상하지 않는다 |
| `max_steering_step_rad` | 0.035 | 조향 변화율 제한이 이미 있다 |
| `steering_filter_alpha` | 0.25 | 조향 저역통과가 이미 있다 |
| `wheelbase` | 3.0 | planning 과 같다 |
| **`max_lateral_accel`** | **1.8** | ⚠️ 아래 참고 |

### ⚠️ 횡가속도 한계가 서로 다르다

planning 의 lattice 는 **3.51 m 를 2.68초에 옮기도록** 회피 후보를 만드는데,
이때 요구 횡가속도가 최대 **2.94 m/s²** 다. control 의 `max_lateral_accel` 이
**1.8** 이면 **회피 기동을 설계대로 못 따라간다.** 차가 덜 비켜나거나 밀려난다.

즉 planning 이 좋은 회피 경로를 줘도 **control 이 그걸 실행하지 못하는 상태**일 수
있다. **통합 전에 어느 값이 맞는지 맞춰야 한다.**

- planning 쪽 2.94 는 "3.51 m 를 안전하게 비켜나려면 이 정도가 필요하다" 는 요구치
- control 쪽 1.8 은 "차가 안정적으로 낼 수 있는 한계" 라는 판단으로 보인다

둘 중 하나가 양보해야 한다. **control 쪽 1.8 의 근거를 알려주시면** planning 이
전이 시간을 늘려(= 더 일찍, 더 완만하게 비켜나도록) 맞출 수 있다. 대신 그만큼
**장애물을 더 멀리서 발견해야** 하므로 인지 지평선과도 엮인다.

---

## 관련 문서 (planning repo `morai-ros/docs/`)

팀 repo 에는 없다. 필요하면 요청해 주세요.

| 문서 | 내용 |
|---|---|
| `30-lattice_design.md` | 회피 후보 생성·충돌검사·비용 선택. 위 2.94 m/s² 의 근거 |
| `40-acc_design.md` | `/target_velocity` 가 종방향 단일 권한이라는 설계 원칙 |
| `50-behavior_fsm_design.md` | 제약 합성(min)과 상승률 제한 |
| `21-localization_interface.md` | `/odom` 명세 (양쪽이 공통으로 소비) |
