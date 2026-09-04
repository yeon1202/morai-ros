# 28 — 제어 노드 검증 결과 (minjjun 브랜치)

**대상**: control 담당자
**작성**: 2026-09-03 (planning)
**대상 코드**: `2026_CARSA_AD` `minjjun` 브랜치, `autonomous_driving/src/control/vehicle_control.cpp`
**놓일 자리**: 팀 repo `2026_CARSA_AD` 루트의 `docs/28-control_review.md`
**상태**: 초안. **아직 안 보냄.**

planning 스택에 붙여 시뮬로 한 바퀴 돌린 결과다. **코드는 안 고쳤다** — 현상과
원인만 정리했으니 고칠 방향은 담당자가 정하시면 된다.

---

## 0. 한 줄 요약

**`/lattice_path` · `/target_velocity` 연동은 명세대로 잘 동작한다.** 4079프레임 중
4065프레임(99.7%)에서 planning 경로를 따랐다.

다만 **속도 상한이 목표에는 걸리는데 실제 속도가 3.7 km/h 넘어선다.** 원인은
**PID 적분 와인드업**이다(3장). 그리고 **`CMakeLists.txt` 의 OsqpEigen 때문에
빌드가 안 된다**(4장).

---

## 1. 어떻게 돌렸나

| | |
|---|---|
| 시뮬 | MORAI, Sunny |
| 위치추정 | `robot_localization` EKF (GPS+IMU+차속) → `/odom` |
| planning | lattice + ACC + behavior_fsm |
| 제어 | `vehicle_control` (이 문서 대상) |
| 기록 | `log_csv_path` 로 남긴 CSV, 4079줄 / 135.9초 |

planning 이 `path_tracker.py` 로도 `/ctrl_cmd` 를 내기 때문에, 그쪽 출력은 remap 으로
돌려 **`vehicle_control` 만 `/ctrl_cmd` 를 발행하도록** 하고 돌렸다.

**우리가 이 파일에 넣은 것이 하나 있다** — `max_speed_kph` 파라미터(기본 40).
검증 중 속도를 묶어두려는 것이고, 아래 3장의 관측은 이 상한이 켜진 상태에서 나왔다.
그 외에는 브랜치 원본 그대로다.

---

## 2. 잘 동작한 것

명세(`22-planning_control_interface.md`)와 대조한 결과다.

| 항목 | 결과 |
|---|---|
| `/lattice_path` 구독 · 추종 | ✅ `path_source=LATTICE` 4065 / `CSV` 14 (99.7%) |
| `/target_velocity` 를 상한으로 사용 | ✅ `min(곡률 프로파일, FSM)` 그대로 |
| `/lattice_path` **0.3초** 폴백 | ✅ 값까지 명세와 일치 |
| `/target_velocity` **0.5초** 폴백 | ✅ |
| 폴백 시 로그 | ✅ `path_source` / `speed_source` 를 ROS 로그와 CSV 양쪽에 |
| 인덱스를 프레임 간에 안 들고 다님 | ✅ 매 주기 전체 탐색 |

**폴백 타임아웃 숫자까지 명세대로 맞춰주셨다.** `/lattice_path` 의 yaw·곡률을 XY 에서
직접 다시 계산하는 것도 planning 이 그 필드를 안 채워도 되게 해줘서 좋았다.

---

## 3. 🔴 속도 상한을 실제 속도가 넘어선다 (적분 와인드업)

### 3-1. 현상

목표속도는 40 km/h 로 잘리고 있다. `speed_source` 열에 `CAP` 이 778프레임 찍혔다.
그런데 **실제 속도가 최대 43.7 km/h** 까지 올라간다.

| 실제 속도 | 값 |
|---|---|
| 중앙 | 5.5 km/h |
| 90% | 38.3 |
| 99% | 42.4 |
| **최대** | **43.7** |
| 40 초과 프레임 | **244개 (6.0%)**, 연속 5구간 |

### 3-2. 결정적 프레임

```
 경과s   속도kph   accel   brake   speed_source
  38.1    28.2     1.000   0.000   CAP
  40.1    35.3     0.970   0.000   CAP
  42.1    40.7     0.143   0.000   CAP    <- 목표를 넘었는데 아직 가속 중
  44.1    32.3     1.000   0.000   CAP
  46.1    40.1     0.230   0.000   CAP
  48.1    43.0     0.000   0.273   CAP    <- 그제야 제동
```

`42.1초` 를 보시면, 속도가 목표(40)를 넘었는데 `accel = 0.143` 으로 **여전히 밟고
있다.** 비례항만 있으면 이 시점에 이미 음수가 나와야 한다.

### 3-3. 원인

```cpp
// vehicle_control.cpp  computePID()
integral_error_ += speed_error * control_period_;
integral_error_ = clampValue(integral_error_, -integral_limit_, integral_limit_);
```

**출력이 포화된 동안에도 적분이 계속 쌓인다.**

로그에서 `20~38초` 구간은 `accel = 1.000` 이 18초간 붙어 있다. 액셀을 더 밟을 수
없는 상태인데 적분은 계속 누적되어 상한 `+5.0` 에 눌러앉는다. 이 주행은 중앙 속도가
5.5 km/h 라(정지·서행 구간이 길었다) 그 상태가 오래 유지됐다.

기여분을 계산하면:

```
ki × I = 0.05 × 5.0 = 0.25        <- 스로틀 0.25 가 "빚" 으로 남는다

이걸 비례항이 이기려면
0.55 × speed_error < −0.25
speed_error < −0.45 m/s = −1.6 km/h
```

**즉 적분항만으로도 목표보다 1.6 km/h 이상 넘어야 비로소 감속이 시작된다.**
여기에 미분항이 더해져 관측된 3.7 km/h 가 된다.

### 3-4. 왜 지금 중요한가

- 규정상 **전 구간 60 kph 제한**이고 초과 즉시 15초, 3초 지속마다 15초다.
  크루즈를 60 으로 두면 같은 이유로 **63.7 km/h** 까지 올라간다.
- 목표속도를 낮춰 흡수할 수는 있지만(우리가 40 으로 묶어 봤다) 그건 원인을
  안 건드리는 방법이고, 넘는 양이 주행 상황에 따라 달라져 얼마를 빼야 할지
  정할 근거가 없다.

### 3-5. 참고 — 표준 해법

조건부 적분(anti-windup)이다. 출력이 포화 중이고 적분이 포화를 더 밀어붙이는
방향이면 적분을 멈춘다.

```
액셀이 이미 최대인데 오차가 여전히 +   ->  적분 멈춤 (이미 최선을 다하는 중)
브레이크가 최대인데 오차가 여전히 −    ->  적분 멈춤
그 외                                ->  평소대로
```

`computePID()` 안에서만 끝난다. **다만 이건 제어 튜닝이라 우리가 손대지 않았다** —
방식은 담당자가 정하시는 게 맞다고 봤다.

---

## 4. 🔴 `CMakeLists.txt` 의 OsqpEigen 때문에 빌드가 안 된다

```cmake
find_package(OsqpEigen REQUIRED)                                    # 18행
target_link_libraries(vehicle_control ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)
```

그런데 **지금 `vehicle_control.cpp` 는 Eigen 도 OSQP 도 쓰지 않는다.** MPC 에서
Pure Pursuit + PID 로 바뀌면서 필요가 없어진 것으로 보인다(파일 전체 grep 0건,
기동 로그도 `Basic PID + Pure Pursuit Controller`).

**우리 도커 이미지에는 OsqpEigen 이 없다**(전체 파일시스템 검색 0건). `REQUIRED` 라
`find_package` 단계에서 **`autonomous_driving` 패키지 전체가 configure 실패**한다.

차선 인지 담당자가 `yeonsoo` 브랜치에서 이 두 줄과 `vehicle_control` 타깃 자체를
주석 처리해 둔 것도 같은 이유로 보인다. **그 상태로 머지되면 제어가 통째로 빌드에서
빠지므로**, 주석이 아니라 의존성을 지우는 쪽이 맞다.

→ **확인했다.** 두 줄을 빼고 `${catkin_LIBRARIES}` 만 링크하니 **경고 없이 빌드되고
정상 동작한다.** `find_package(Eigen3 REQUIRED)` 도 같이 뺄 수 있다.

한 가지 추가: `vehicle_control.cpp` 가 `tf::Quaternion` / `tf::Matrix3x3` 을 쓰는데
(`820행`), `find_package(catkin COMPONENTS ...)` 에 `tf` 가 없다. `package.xml` 에도
없다. 우리 환경에서는 넣어야 빌드됐다.

---

## 5. 🟠 우리 환경에서 걸렸던 것 (경로 기본값)

대회장 PC 나 다른 팀원 환경에서도 같은 일이 날 수 있어 적어둔다.

| 파라미터 | 기본값 | 문제 |
|---|---|---|
| `waypoint_csv` | `/root/catkin_ws/src/autonomous_driving/path/path_smooth_closed.csv` | 그 경로가 없으면 `ROS_FATAL` 후 즉시 종료 |
| `log_csv_path` | `/root/catkin_ws/basic_controller_log.csv` | 컨테이너 사용자가 `root` 가 아니면 못 쓴다. 경고만 뜨고 **로그가 안 남는다** |

`$(find autonomous_driving)/...` 로 기본값을 잡으면 환경과 무관해진다.
`control_only.launch` 는 이미 그렇게 되어 있는데 코드 기본값이 다르다.

**그리고 CSV 형식**: `loadPath()` 가 `x, y, z, yaw` **4개 필드**를 요구한다.
필드가 모자란 줄은 조용히 버려지므로, 3열 CSV 를 주면 파일은 열리는데 0점이 되어
`Not enough waypoint data` 로 죽는다. 에러 메시지만 봐서는 원인을 찾기 어렵다.
헤더 주석에 "4열 필수" 를 적어두면 좋겠다.

---

## 6. 🟡 주석과 코드가 어긋난 곳

기능 문제는 아니지만, 나중에 읽는 사람이 헷갈릴 만한 곳이다.

| 위치 | 주석 | 실제 코드 |
|---|---|---|
| Look-ahead 구간 | `직선 7.0 / 곡선 5.0 / 급곡선 3.0 m` | `16.0 / 8.75 / 3.5` |
| heading 보정 한계 | `최대 ±3 deg` | `5.0 * M_PI / 180.0` |
| 섹션 제목 | `4. 경로 곡률 기반 Feedforward 조향` | `path_curvature` 를 뽑아서 **CSV 로그에만 쓰고 조향식에는 안 더한다** |

마지막 것은 의도한 건지 궁금하다. 최종 조향식이

```cpp
steering = pp_gain * steering_pp + steering_heading + steering_lateral;
```

라 feedforward 항이 없다.

**죽은 코드 둘**

- `control_only.launch` 가 `loop_path` 파라미터를 넘기는데 코드가 안 읽는다
- `current_path_idx_` 가 선언·초기화만 되고 안 쓰인다

**백업 파일 3개** (`vehicle_control_backup_0824_1101.cpp` 등, 합계 154 KB)가 소스
트리에 같이 있다. git 이 이력을 갖고 있으니 지워도 복구된다.

---

## 7. 재현 방법

```bash
# 1) 브릿지
rosrun udp_bridge udp_bridge.py

# 2) localization  (/odom)
roslaunch sim_rate localization.launch

# 3) planning + 제어
roslaunch path_tracking sim.launch perception:=true controller:=external
```

`controller:=external` 이 `path_tracker` 의 `/ctrl_cmd` 를 remap 으로 돌리고
`vehicle_control` 을 띄운다. 파라미터는 launch 가 넘긴다.

로그는 `log_csv_path` 가 가리키는 CSV 한 장이면 된다. 3장의 관측은 전부
`speed_kph` · `accel` · `brake` · `speed_source` 네 열에서 나왔다.

---

## 8. 정리

| # | 항목 | 심각도 |
|---|---|---|
| 1 | `CMakeLists.txt` 의 OsqpEigen 제거 (+ `tf` 추가) | 🔴 이게 없으면 빌드가 안 된다 |
| 2 | PID 적분 와인드업 → 목표속도 3.7 km/h 초과 | 🔴 60 kph 제한에 직접 걸린다 |
| 3 | `waypoint_csv` · `log_csv_path` 기본값 | 🟠 환경 바뀌면 죽거나 로그가 안 남는다 |
| 4 | 주석/코드 불일치 3곳, 죽은 코드 2개, 백업파일 3개 | 🟡 |

2번은 **현상과 원인만 정리했고 코드는 안 고쳤다.** 필요하시면 우리가 남긴 로그
(`vehicle_control.csv`, 4079줄)를 그대로 드릴 수 있다.

---

## 관련 문서

| 문서 | 내용 |
|---|---|
| `22-planning_control_interface.md` | planning ↔ control 인터페이스 명세 (2장에서 대조한 것) |
| `27-lane_review.md` | 차선 인지 코드 리뷰. 5-1 의 OsqpEigen 항목이 이 문서 4장과 같은 건이다 |
