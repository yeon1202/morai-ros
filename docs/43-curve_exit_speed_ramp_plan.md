# 커브 탈출 속도 상승률 제한 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **단, 이 프로젝트는 예외다.** 사용자가 ROS/C++/자율주행을 배우는 중이므로
> 자동 위임 실행을 명시적으로 거부했다. 대화형으로 한 단계씩 함께 진행할 것.

**Goal:** 목표속도가 한 스텝에 크루즈까지 열리는 것을 막아 커브 탈출 시 S자 사행을 줄인다.

**Architecture:** `acc_core.hpp` 에 순수 함수 `rampTarget()` 을 추가하고, `acc_planner` 가 직전 목표속도와 시각을 들고 있다가 `run()` 마지막에 한 번 적용한다. 크루즈·앞차추종·곡률 제한을 모두 합산한 뒤에 걸어야 목표가 어떤 이유로 오르든 동일하게 제한된다.

**Tech Stack:** ROS1 Noetic, C++14, gtest (catkin_add_gtest), Docker 컨테이너 `morai-dev`

설계문서: [42-curve_exit_speed_ramp_design.md](42-curve_exit_speed_ramp_design.md)

## Global Constraints

- 단위는 m/s 로 통일한다. `acc_core.hpp` 는 ROS 비의존 순수 로직이다.
- **감속은 절대 제한하지 않는다.** 급정지·앞차 제동 반응이 느려지면 안전 문제가 된다.
- **안전 불변식: `rampTarget()` 의 반환값은 어떤 경로로도 `desired` 를 넘지 않는다.** 규정 상한(60km/h)과 곡률 제한이 이미 `desired` 에 반영되어 있으므로, 이 불변식이 지켜지는 한 제한을 우회할 수 없다.
- 새 파라미터는 전부 `pnh.param` 으로 노출해 재빌드 없이 튜닝 가능하게 한다.
- 파라미터 기본값: `accel_rate_limit=1.0` [m/s^2], `rate_limit_windup=2.0` [m/s], `rate_dt_max=0.5` [s]
- 빌드·테스트는 컨테이너 안에서 한다. 호스트에서 바로 `catkin_make` 하지 않는다.

**컨테이너 진입 방법** (모든 빌드/테스트 명령의 앞부분):

```bash
docker exec -it morai-dev bash -lc 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && <명령>'
```

노트북을 절전했다 왔다면 먼저 `cd ~/morai-ros && docker compose down && docker compose up -d` 로 컨테이너를 다시 만들어야 GPU 가 붙는다.

## File Structure

| 파일 | 역할 | 변경 |
|---|---|---|
| `catkin_ws/src/path_tracking/include/path_tracking/acc_core.hpp` | ACC 순수 로직 | `AccParams` 에 필드 3개 + `rampTarget()` 추가 |
| `catkin_ws/src/path_tracking/test/acc_core_test.cpp` | gtest | 테스트 6개 추가 (기존 22 → 28) |
| `catkin_ws/src/path_tracking/src/acc_planner.cpp` | ROS 노드 | 상태 2개 + `run()` 말미 적용 + `pnh.param` 3개 |
| `catkin_ws/src/path_tracking/launch/acc.launch` | 실행 구성 | `accel_rate_limit` arg 추가 |

---

### Task 1: rampTarget 순수 함수와 단위 테스트

**Files:**
- Modify: `catkin_ws/src/path_tracking/include/path_tracking/acc_core.hpp` (`AccParams` 끝, 그리고 `computeTargetVelocity` 뒤)
- Test: `catkin_ws/src/path_tracking/test/acc_core_test.cpp` (파일 끝에 추가)

**Interfaces:**
- Consumes: `acc::AccParams` (기존)
- Produces: `acc::rampTarget(double prev, double desired, double ego_vel, double dt, const AccParams& p) -> double`. Task 2 가 이 시그니처로 호출한다.

- [ ] **Step 1: 파라미터 3개를 AccParams 에 추가**

`acc_core.hpp` 의 `struct AccParams` 안, `curve_min_speed` 줄 바로 아래에 넣는다.

```cpp
  // --- 목표속도 상승률 제한 (커브 탈출 사행 억제) ---
  //
  // curvatureSpeedLimit 은 /local_path(앞쪽)만 훑으므로 커브 정점을 지나는 순간
  // 제한이 그 프레임에 즉시 풀린다. 커브 진입에는 감속 프로파일이 있지만
  // 탈출에는 대응하는 가속 프로파일이 없어 목표속도가 한 스텝에 크루즈까지 열린다.
  // 그 급상승이 복귀 조향을 흔들어 S자 사행을 만든다.
  double accel_rate_limit  = 1.0;   // [m/s^2] 목표속도 상승률 한계
  double rate_limit_windup = 2.0;   // [m/s] 실제속도보다 이만큼 이상 앞서지 않게
  double rate_dt_max       = 0.5;   // [s] 한 스텝으로 인정하는 dt 상한
```

- [ ] **Step 2: 실패하는 테스트 6개를 먼저 작성**

`test/acc_core_test.cpp` 맨 끝에 추가한다.

```cpp
// ---------------------------------------------------------------------------
// rampTarget : 목표속도 상승률 제한
// ---------------------------------------------------------------------------

// 목표가 크게 뛰어도 한 스텝에 accel_rate_limit*dt 만큼만 오른다
TEST(RampTarget, RiseIsLimited) {
  AccParams p = defaultParams();
  // prev=4.0 에서 desired=15.3 으로 뛰어도 4.0 + 1.0*0.05 = 4.05
  EXPECT_NEAR(rampTarget(4.0, 15.3, 4.0, 0.05, p), 4.05, 1e-9);
}

// 감속은 제한하지 않는다 (급정지 반응이 느려지면 안 된다)
TEST(RampTarget, FallIsNotLimited) {
  AccParams p = defaultParams();
  EXPECT_NEAR(rampTarget(15.0, 0.0, 15.0, 0.05, p), 0.0, 1e-9);
}

// 목표에 거의 도달했으면 목표를 넘지 않는다 (오버슛 없음)
TEST(RampTarget, DoesNotOvershootDesired) {
  AccParams p = defaultParams();
  // 14.9 + 1.0*0.5 = 15.4 지만 desired 가 15.0 이므로 15.0 에서 멈춘다
  EXPECT_NEAR(rampTarget(14.9, 15.0, 14.9, 0.5, p), 15.0, 1e-9);
}

// dt 가 0 이하이면 제한하지 않는다 (로직이 차를 잠그지 않게)
TEST(RampTarget, ZeroDtPassesThrough) {
  AccParams p = defaultParams();
  EXPECT_NEAR(rampTarget(4.0, 15.3, 4.0, 0.0, p), 15.3, 1e-9);
}

// 프레임이 끊겨 dt 가 커져도 rate_dt_max 로 잘린다
TEST(RampTarget, LargeDtIsClamped) {
  AccParams p = defaultParams();
  // dt=10 이지만 0.5 로 잘려 4.0 + 1.0*0.5 = 4.5
  EXPECT_NEAR(rampTarget(4.0, 15.3, 4.0, 10.0, p), 4.5, 1e-9);
}

// 목표가 실제 속도보다 크게 앞서 있으면 끌어내린 뒤 올린다 (윈드업 억제)
TEST(RampTarget, WindupIsAnchoredToEgoSpeed) {
  AccParams p = defaultParams();
  // prev=14.0 이지만 ego=4.0 이므로 4.0+2.0=6.0 으로 당겨지고, 6.0+0.05 = 6.05
  EXPECT_NEAR(rampTarget(14.0, 15.3, 4.0, 0.05, p), 6.05, 1e-9);
}
```

- [ ] **Step 3: 테스트가 실패하는지 확인**

```bash
docker exec -it morai-dev bash -lc 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && cd /home/dev/catkin_ws && catkin_make acc_core_test'
```

기대: 컴파일 실패. `error: 'rampTarget' was not declared in this scope` 가 6번 나온다.

- [ ] **Step 4: rampTarget 구현**

`acc_core.hpp` 의 `computeTargetVelocity` 함수 바로 뒤, `}  // namespace acc` 앞에 넣는다.

```cpp
// 목표속도의 "상승"만 제한한다. 감속은 그대로 통과시킨다.
//
// 왜 필요한가: curvatureSpeedLimit 은 앞쪽 경로만 보므로 커브를 빠져나오는 순간
// 제한이 즉시 풀린다. 목표속도가 한 스텝에 15 m/s 씩 뛰면 복귀 조향이 수렴하기
// 전에 속도가 붙어 사행이 커진다. 목표를 천천히 올리는 것 자체가 "커브가 아직
// 안 끝났다"는 보수적 판정 역할을 한다.
//
// 안전 불변식: 반환값은 어떤 경로로도 desired 를 넘지 않는다. 이 함수는 목표를
// 늦출 뿐 높이지 않는다. 규정 상한과 곡률 제한은 이미 desired 에 반영되어 있다.
inline double rampTarget(double prev, double desired, double ego_vel,
                         double dt, const AccParams& p) {
  if (dt <= 0.0) return desired;              // 제한 로직이 차를 잠그지 않게
  if (dt > p.rate_dt_max) dt = p.rate_dt_max;  // 프레임 끊김 시 한 번에 뛰지 않게

  // 목표가 실제 속도보다 크게 앞서 있으면 끌어내린다.
  // 차가 목표를 못 따라가는 동안(오르막, 제동 직후) 목표만 혼자 달아나면,
  // 회복되는 순간 그 격차만큼 급가속한다. 제한이 걸린 것처럼 보이지만 무력화된 상태다.
  double anchored = std::min(prev, ego_vel + p.rate_limit_windup);

  if (desired <= anchored) return desired;    // 감속은 제한하지 않는다
  return std::min(desired, anchored + p.accel_rate_limit * dt);
}
```

- [ ] **Step 5: 테스트 28개 전부 통과 확인**

```bash
docker exec -it morai-dev bash -lc 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && cd /home/dev/catkin_ws && catkin_make acc_core_test && ./devel/lib/path_tracking/acc_core_test'
```

기대: `[  PASSED  ] 28 tests.` 기존 22개가 하나도 깨지지 않아야 한다.

- [ ] **Step 6: 커밋**

```bash
cd ~/morai-ros
git add catkin_ws/src/path_tracking/include/path_tracking/acc_core.hpp \
        catkin_ws/src/path_tracking/test/acc_core_test.cpp
git commit -m "feat(acc): 목표속도 상승률 제한 rampTarget 추가

커브 탈출 시 목표속도가 한 스텝에 크루즈까지 열려 S자 사행을 유발하는 문제.
curvatureSpeedLimit 이 앞쪽 경로만 보므로 커브 정점을 지나면 제한이 즉시 풀린다.
진입에는 감속 프로파일이 있으나 탈출에는 가속 프로파일이 없었다.

감속은 제한하지 않는다. 반환값은 어떤 경로로도 desired 를 넘지 않는다.
윈드업 억제로 목표가 실제 속도보다 2m/s 이상 앞서지 못하게 묶는다.

gtest 22 -> 28.

설계: docs/42-curve_exit_speed_ramp_design.md"
```

---

### Task 2: acc_planner 배선

**Files:**
- Modify: `catkin_ws/src/path_tracking/src/acc_planner.cpp` (생성자 파라미터부, 멤버 선언부, `run()` 말미)
- Modify: `catkin_ws/src/path_tracking/launch/acc.launch`

**Interfaces:**
- Consumes: `acc::rampTarget(prev, desired, ego_vel, dt, params)` (Task 1)
- Produces: `/target_velocity` 의 값이 상승률 제한을 거친 값이 된다. 구독자(`path_tracker.py`)는 변경 없다.

- [ ] **Step 1: 타이머 주기를 상수로 빼기**

`acc_planner.cpp` 의 `#include "path_tracking/acc_core.hpp"` 줄 바로 아래, `class AccPlanner` **앞**에 넣는다.

```cpp
// 타이머 주기. run() 에서 첫 틱의 dt 로도 쓰므로 상수로 둔다.
static constexpr double kTimerHz = 30.0;
```

클래스 안이 아니라 파일 스코프에 두는 이유: 이 패키지는 C++ 표준을 명시하지 않아
컴파일러 기본값(gnu++14)을 쓴다. C++14 에서는 클래스 안의 `static constexpr` 멤버가
ODR-use 되면 클래스 밖 정의가 따로 필요해 `undefined reference` 가 날 수 있다.
파일 스코프 상수는 그 문제가 없다.

그리고 생성자의 타이머 생성 줄을 바꾼다.

```cpp
    timer_       = nh.createTimer(ros::Duration(1.0 / kTimerHz), &AccPlanner::run, this);
```

- [ ] **Step 2: 파라미터 3개를 pnh.param 에 추가**

생성자의 `pnh.param("curve_min_speed", ...)` 줄 바로 아래에 넣는다.

```cpp
    pnh.param("accel_rate_limit",   params_.accel_rate_limit,   params_.accel_rate_limit);
    pnh.param("rate_limit_windup",  params_.rate_limit_windup,  params_.rate_limit_windup);
    pnh.param("rate_dt_max",        params_.rate_dt_max,        params_.rate_dt_max);
```

- [ ] **Step 3: 상태 멤버 2개 추가**

`private:` 아래 `morai_msgs::EgoVehicleStatus ego_;` 근처, 멤버 선언들 끝에 넣는다.

```cpp
  // 목표속도 상승률 제한용 상태. prev_time_ 이 zero 면 아직 첫 틱을 안 돈 것이다.
  double    prev_target_ = 0.0;
  ros::Time prev_time_;
```

- [ ] **Step 4: run() 말미에 제한 적용**

`run()` 안에서 곡률 제한을 적용하는 블록(`if (curve_limit < target) { ... }`) **바로 뒤**, `std_msgs::Float64 msg;` **앞**에 넣는다.

```cpp
    // 목표속도 상승률 제한. 크루즈·앞차추종·곡률을 모두 합산한 뒤 마지막에 한 번
    // 적용한다. 목표가 어떤 이유로 오르든 동일하게 제한하기 위해서다.
    ros::Time now = ros::Time::now();
    double dt = prev_time_.isZero() ? (1.0 / kTimerHz) : (now - prev_time_).toSec();
    if (prev_time_.isZero() || dt > params_.rate_dt_max) {
      // 첫 틱이거나 오래 끊겼다. 달리는 차에 낡은 목표(또는 0)를 명령하면
      // 급제동이 걸리므로 현재 속도로 시드한다.
      prev_target_ = ego_vel;
    }
    double ramped = acc::rampTarget(prev_target_, target, ego_vel, dt, params_);
    if (ramped < target) {
      ROS_INFO_THROTTLE(2.0, "[acc] 상승률 제한: %.2f -> %.2f m/s (%.1f km/h)",
                        target, ramped, ramped * 3.6);
    }
    target       = ramped;
    prev_target_ = target;
    prev_time_   = now;
```

- [ ] **Step 5: launch 에 튜닝용 arg 추가**

`acc.launch` 의 `<arg name="lookahead" .../>` 줄 아래에 넣는다.

```xml
  <!-- 목표속도 상승률 한계 [m/s^2]. 커브 탈출 시 속도가 급상승해 S자 사행이
       생기는 것을 막는다. 유턴 탈출 실측 가속도가 2.4 m/s^2 였으므로 그 이상은
       효과가 없다. 낮출수록 사행은 줄지만 완주 시간이 늘어난다. -->
  <arg name="accel_rate_limit" default="1.0"/>
```

그리고 `acc_planner` 노드 블록 안에 param 을 추가한다.

```xml
    <param name="accel_rate_limit" value="$(arg accel_rate_limit)"/>
```

- [ ] **Step 6: 빌드 확인**

```bash
docker exec -it morai-dev bash -lc 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && cd /home/dev/catkin_ws && catkin_make acc_planner acc_core_test && ./devel/lib/path_tracking/acc_core_test'
```

기대: 빌드 성공 + `[  PASSED  ] 28 tests.`

- [ ] **Step 7: 노드 기동 확인 (차를 움직이지 않고)**

브릿지가 떠 있는 상태에서 acc_planner 만 띄우고 `/target_velocity` 를 본다.

```bash
docker exec -it morai-dev bash -lc 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && rosrun path_tracking acc_planner'
```

`/local_path` 가 없으면 발행하지 않는 것이 정상이다(`has_local_` 게이트). `[acc_planner] started` 로그만 확인하고 Ctrl+C 로 끈다.

- [ ] **Step 8: 커밋**

```bash
cd ~/morai-ros
git add catkin_ws/src/path_tracking/src/acc_planner.cpp \
        catkin_ws/src/path_tracking/launch/acc.launch
git commit -m "feat(acc): acc_planner 에 목표속도 상승률 제한 배선

크루즈·앞차추종·곡률 제한을 모두 합산한 뒤 마지막에 한 번 적용한다.
목표가 어떤 이유로 오르든 동일하게 제한하기 위해서다.

첫 틱이거나 dt 가 rate_dt_max 를 넘으면 prev_target_ 을 현재 속도로 시드한다.
노드 재시작 시 낡은 목표를 달리는 차에 명령하면 급제동이 걸린다.

accel_rate_limit 은 acc.launch 의 arg 로도 노출해 튜닝 가능하게 했다."
```

---

### Task 3: 유턴 구간 실차 검증

**Files:**
- 코드 변경 없음. 튜닝 결과 `acc.launch` 의 `accel_rate_limit` 기본값만 바뀔 수 있다.

**Interfaces:**
- Consumes: Task 2 의 배선 결과
- Produces: 없음 (검증 단계)

- [ ] **Step 1: 시뮬에서 차량을 스폰 지점으로 리셋**

MORAI 시뮬에서 차량 위치를 초기화한다. 확인 방법:

```bash
docker exec -i morai-dev bash -lc 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && timeout 5 rostopic echo -n 1 /ego_status | head -12'
```

기대: `position: x 약 -14.19, y 약 -224.21`, `heading 약 62.7`

- [ ] **Step 2: 주행하며 유턴 구간 기록**

`acc_planner` + `path_tracker.py` + `diag_tracker.py` 를 함께 띄우고 idx 330 부근까지 간 뒤 정지한다. RViz 는 띄우지 않는다(절전 후 GPU 이슈 회피).

```bash
docker exec -it morai-dev bash -lc '
source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash
cd /home/dev/catkin_ws/src/path_tracking/scripts
rosrun path_tracking acc_planner > /tmp/acc.log 2>&1 &
python3 path_tracker.py > /tmp/pt.log 2>&1 &
sleep 2
python3 diag_tracker.py 2>&1 | tee /tmp/diag.log
'
```

`-it` 여야 Ctrl+C 가 컨테이너 안 프로세스로 전달된다. `-i` 만 쓰면 끊기지 않는다.

idx 가 330 을 넘으면 Ctrl+C, 이어서 반드시 제동한다.

```bash
docker exec -i morai-dev bash -lc 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && rosnode kill /path_tracker; rosrun path_tracking estop.py'
```

MORAI 는 마지막 `/ctrl_cmd` 를 계속 물고 있으므로 노드만 죽이면 차가 마지막 명령으로 계속 간다. `pkill -f "python3 path_tracker.py"` 는 roslaunch 로 띄운 노드를 못 잡는다(전체 경로로 실행되기 때문).

- [ ] **Step 3: 기준값과 비교**

`/tmp/diag.log` 에서 세 지표를 읽어 아래 표와 비교한다.

| 지표 | 적용 전 (2026-07-29) | 목표 |
|---|---|---|
| 최악 CTE | 2.06m (idx 174) | 감소 |
| idx 181~212 CTE 진동 폭 | 0.55 ~ 1.77m (폭 1.22m) | 감소 |
| idx 144 → 212 통과 시간 | 8.5초 | 12초 이내 |

- [ ] **Step 4: 결과에 따라 판단**

- 사행이 충분히 줄고 통과 시간이 12초 이내 → 그대로 확정
- 사행이 덜 줄었다 → `accel_rate_limit` 을 0.5 로 낮춰 재측정
  ```bash
  roslaunch path_tracking acc.launch accel_rate_limit:=0.5 mock_lead:=false
  ```
- 사행은 줄었는데 여전히 최악 CTE 가 1.5m 를 넘는다 → 이 과제의 범위 밖이다.
  남은 원인은 유턴 정점의 조향 포화다. 별도로 `MIN_LFD` 4.0 → 3.0 을 검토한다
  (설계문서 7절 참고).

- [ ] **Step 5: 결과를 설계문서에 기록하고 커밋**

`42-curve_exit_speed_ramp_design.md` 4절 검증 표에 실측값을 채운다. `accel_rate_limit` 기본값을 바꿨다면 `acc.launch` 도 함께 커밋한다.

```bash
cd ~/morai-ros
git add docs/42-curve_exit_speed_ramp_design.md catkin_ws/src/path_tracking/launch/acc.launch
git commit -m "docs(acc): 상승률 제한 실차 검증 결과 기록"
```

---

## 완료 조건

- [ ] gtest 28개 통과
- [ ] `acc_planner` 빌드 성공, 기동 로그 정상
- [ ] 유턴 구간 재주행에서 CTE 진동 폭이 적용 전보다 감소
- [ ] 통과 시간 12초 이내
- [ ] 설계문서 4절에 실측값 기록
