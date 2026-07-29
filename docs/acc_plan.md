# ACC (Adaptive Cruise Control) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 앞차/정적장애물 기반으로 목표속도를 계산하는 종방향 planning 노드(ACC)를 만들어 `/target_velocity`로 발행한다.

**Architecture:** ROS 없는 순수 C++ 헤더(`acc_core.hpp`)에 제어식과 lead 탐색 로직을 담아 gtest로 단위 테스트하고, `acc_planner.cpp`는 구독/발행만 하는 얇은 ROS 래퍼로 둔다. lattice와 대칭인 순수 planning 노드이며 `/ctrl_cmd`는 절대 건드리지 않는다.

**Tech Stack:** ROS1 Noetic, roscpp, C++11, catkin_make, gtest(catkin_add_gtest), morai_msgs, std_msgs/Float64.

설계문서: [acc_design.md](acc_design.md)

## Global Constraints

- **단위 함정 (반드시 준수):** `EgoVehicleStatus.velocity` = **m/s**, `ObjectStatus.velocity` = **km/h**. ACC 내부 계산은 전부 **m/s**로 통일 — 객체 속도는 반드시 `/3.6` 변환.
- **60kph 하드캡:** 목표속도 상한 = 16.67 m/s (`60 / 3.6`). 하한 = 0.
- **객체 소스:** `npc_list`(움직이는 차) + `obstacle_list`(정적)만 사용. `pedestrian_list`·신호는 이번 범위 밖(behavior FSM 몫).
- **제어식:** 레퍼런스 SSAFY식 그대로 (`MORAI-RoboticsExample/AD/autonomous_driving/planning/adaptive_cruise_control.py`).
- **좌표계:** 모든 위치는 ENU 전역좌표. 헤딩은 deg.
- **`/ctrl_cmd` 금지:** ACC는 `/target_velocity`만 발행.
- **Git:** `~/morai-ros`는 아직 git 저장소가 아님. 시작 전 `cd ~/morai-ros && git init` 권장(그러면 커밋 스텝이 동작). 초기화 안 하면 각 태스크의 **커밋 스텝은 건너뛰고** 대신 그 지점을 리뷰 체크포인트로 삼는다. (주의: `catkin_ws/src/morai_msgs`에 자체 `.git`이 있으니 top-level init 시 `catkin_ws/src/morai_msgs`를 `.gitignore`에 넣거나 그대로 두기.)
- **빌드/소싱:** 모든 빌드는 `cd ~/morai-ros/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make`. 실행 전 `source ~/morai-ros/catkin_ws/devel/setup.bash`.

---

## File Structure

| 파일 | 책임 |
|------|------|
| `catkin_ws/src/path_tracking/include/path_tracking/acc_core.hpp` | **신규**. 순수 로직: 자료구조(`Vec2`,`ObjIn`,`Lead`,`AccParams`), `speedKmhToMps()`, `selectLead()`, `computeTargetVelocity()`. ROS 의존성 없음. |
| `catkin_ws/src/path_tracking/test/acc_core_test.cpp` | **신규**. 위 로직 gtest 단위 테스트. |
| `catkin_ws/src/path_tracking/src/acc_planner.cpp` | **신규**. ROS 래퍼: 구독→헤더 함수 호출→`/target_velocity` 발행. |
| `catkin_ws/src/path_tracking/src/mock_lead_vehicle.cpp` | **신규**. 오프라인 검증용 느린 앞차(NPC) 발행 노드. |
| `catkin_ws/src/path_tracking/CMakeLists.txt` | **수정**. std_msgs 의존, include 디렉토리, acc_planner·mock_lead_vehicle 실행타깃, gtest 타깃. |
| `catkin_ws/src/path_tracking/package.xml` | **수정**. std_msgs 의존 추가. |
| `catkin_ws/src/path_tracking/launch/sim.launch` | **수정**. acc_planner, mock_lead_vehicle 노드 추가. |
| `catkin_ws/src/path_tracking/scripts/path_tracker.py` | **수정(임시)**. `/target_velocity` 구독해 종제어에 사용. |
| `docs/target_velocity_interface.md` | **신규**. control팀 인계용 인터페이스 명세. |

---

## Task 1: ACC 제어식 (computeTargetVelocity) — 순수 로직 + gtest 스캐폴딩

**Files:**
- Create: `catkin_ws/src/path_tracking/include/path_tracking/acc_core.hpp`
- Create: `catkin_ws/src/path_tracking/test/acc_core_test.cpp`
- Modify: `catkin_ws/src/path_tracking/CMakeLists.txt`
- Modify: `catkin_ws/src/path_tracking/package.xml`

**Interfaces:**
- Produces:
  - `struct AccParams { double time_gap; double default_space; double vehicle_length; double distance_threshold; double velocity_gain; double distance_gain; double cruise_speed; double max_speed; };`
  - `double acc::computeTargetVelocity(double ego_vel, const acc::Lead& lead, const acc::AccParams& p);`
  - `struct Lead { bool present=false; double distance=0.0; double velocity=0.0; };` (distance = ego까지 상대거리 − vehicle_length, velocity = m/s)

- [ ] **Step 1: 헤더 스텁 작성** (컴파일은 되되 함수는 미구현으로 실패 유도)

Create `catkin_ws/src/path_tracking/include/path_tracking/acc_core.hpp`:

```cpp
// acc_core.hpp : ACC 순수 로직 (ROS 비의존, 단위=m/s로 통일)
#pragma once
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

namespace acc {

struct Vec2 { double x = 0.0; double y = 0.0; };

// 탐색 입력용 객체 (속도는 이미 m/s로 변환된 값)
struct ObjIn { Vec2 pos; double speed_mps = 0.0; };

// 선택된 앞차/장애물
struct Lead {
  bool   present  = false;
  double distance = 0.0;   // ego 상대거리 − vehicle_length [m]
  double velocity = 0.0;   // [m/s]
};

struct AccParams {
  double time_gap           = 1.0;    // [s]
  double default_space      = 5.0;    // [m] 최소 정지 간격
  double vehicle_length     = 4.635;  // [m] Ioniq5
  double distance_threshold = 2.5;    // [m] 경로 위 판정 횡거리
  double velocity_gain      = 0.5;
  double distance_gain      = 1.0;
  double cruise_speed       = 16.67;  // [m/s] free-flow 목표 (=60kph)
  double max_speed          = 16.67;  // [m/s] 하드캡 (=60kph)
};

// km/h 속도벡터 → m/s 스칼라 (단위 함정 처리)
inline double speedKmhToMps(double vx_kmh, double vy_kmh) {
  return std::hypot(vx_kmh, vy_kmh) / 3.6;
}

// 목표속도 계산 (레퍼런스 SSAFY식). 미구현 스텁.
inline double computeTargetVelocity(double ego_vel, const Lead& lead, const AccParams& p) {
  (void)ego_vel; (void)lead; (void)p;
  return -1.0;  // 스텁: 테스트 실패 유도
}

}  // namespace acc
```

- [ ] **Step 2: 실패 테스트 작성**

Create `catkin_ws/src/path_tracking/test/acc_core_test.cpp`:

```cpp
#include <gtest/gtest.h>
#include "path_tracking/acc_core.hpp"

using namespace acc;

static AccParams defaultParams() {
  AccParams p;  // 헤더 기본값 사용
  return p;
}

// 앞차 없으면 크루즈 속도
TEST(ComputeTargetVelocity, NoLeadReturnsCruise) {
  AccParams p = defaultParams();
  Lead lead;  // present=false
  EXPECT_NEAR(computeTargetVelocity(10.0, lead, p), p.cruise_speed, 1e-6);
}

// 간격이 default_space보다 작으면 정지(0)
TEST(ComputeTargetVelocity, TooCloseStops) {
  AccParams p = defaultParams();
  Lead lead; lead.present = true; lead.distance = 3.0; lead.velocity = 0.0;  // 3 < 5
  EXPECT_NEAR(computeTargetVelocity(10.0, lead, p), 0.0, 1e-6);
}

// 느린 앞차: 크루즈보다 낮고 0보다 큰 값으로 감속
TEST(ComputeTargetVelocity, SlowerLeadDecelerates) {
  AccParams p = defaultParams();
  Lead lead; lead.present = true; lead.distance = 20.0; lead.velocity = 5.0;
  // safe=16.67*1+5=21.67, dist_err=1.67, vel_err=11.67
  // accel=-(0.5*11.67+1.0*1.67)=-7.505, target=min(16.67-7.505,16.67)=9.165
  double v = computeTargetVelocity(16.67, lead, p);
  EXPECT_NEAR(v, 9.165, 0.01);
  EXPECT_LT(v, p.cruise_speed);
  EXPECT_GT(v, 0.0);
}

// 멀고 빠른 앞차: 크루즈로 캡
TEST(ComputeTargetVelocity, FarFastLeadCapsAtCruise) {
  AccParams p = defaultParams();
  Lead lead; lead.present = true; lead.distance = 50.0; lead.velocity = 16.0;
  EXPECT_NEAR(computeTargetVelocity(10.0, lead, p), p.cruise_speed, 1e-6);
}

// 60kph 하드캡: cruise가 더 커도 max_speed로 제한
TEST(ComputeTargetVelocity, HardCapAt60) {
  AccParams p = defaultParams();
  p.cruise_speed = 30.0;  // max_speed(16.67)보다 큼
  Lead lead;              // 앞차 없음 → cruise 반환 시도
  EXPECT_NEAR(computeTargetVelocity(10.0, lead, p), p.max_speed, 1e-6);
}
```

- [ ] **Step 3: CMakeLists / package.xml 스캐폴딩 후 빌드 → 실패 확인**

Modify `catkin_ws/src/path_tracking/CMakeLists.txt` — `find_package` COMPONENTS에 `std_msgs` 추가 (line 10-17 블록):

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  morai_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_msgs
)
```

같은 파일 맨 끝(현재 224줄 `lattice_planner` 블록 아래)에 include 디렉토리와 gtest 타깃 추가:

```cmake
# ================= ACC 추가 =================
include_directories(include ${catkin_INCLUDE_DIRS})

## gtest: ACC 순수 로직 단위 테스트
catkin_add_gtest(acc_core_test test/acc_core_test.cpp)
if(TARGET acc_core_test)
  target_link_libraries(acc_core_test ${catkin_LIBRARIES})
endif()
```

Modify `catkin_ws/src/path_tracking/package.xml` — `<depend>` 블록에 std_msgs 추가 (기존 morai_msgs depend 옆에):

```xml
  <build_depend>std_msgs</build_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>
```

Run:
```bash
cd ~/morai-ros/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make tests && ./devel/lib/path_tracking/acc_core_test
```
Expected: 빌드 성공, 테스트 **FAIL** (스텁이 -1 반환 → 모든 EXPECT 불일치)

- [ ] **Step 4: computeTargetVelocity 구현**

`acc_core.hpp`의 `computeTargetVelocity` 스텁을 교체:

```cpp
inline double computeTargetVelocity(double ego_vel, const Lead& lead, const AccParams& p) {
  double out_vel = p.cruise_speed;

  if (lead.present) {
    double safe_distance  = ego_vel * p.time_gap + p.default_space;
    double velocity_error = ego_vel - lead.velocity;
    double distance_error = safe_distance - lead.distance;
    double acceleration   = -(p.velocity_gain * velocity_error + p.distance_gain * distance_error);
    out_vel = std::min(ego_vel + acceleration, p.cruise_speed);
    if (lead.distance < p.default_space) out_vel = 0.0;
  }

  // 60kph 하드캡 + 하한
  out_vel = std::min(out_vel, p.max_speed);
  if (out_vel < 0.0) out_vel = 0.0;
  return out_vel;
}
```

- [ ] **Step 5: 빌드 → 통과 확인**

Run:
```bash
cd ~/morai-ros/catkin_ws && catkin_make tests && ./devel/lib/path_tracking/acc_core_test
```
Expected: **PASS** (5 tests, ComputeTargetVelocity.*)

- [ ] **Step 6: 커밋**

```bash
cd ~/morai-ros
git add catkin_ws/src/path_tracking/include catkin_ws/src/path_tracking/test \
        catkin_ws/src/path_tracking/CMakeLists.txt catkin_ws/src/path_tracking/package.xml
git commit -m "feat(acc): computeTargetVelocity 제어식 + gtest 스캐폴딩"
```

---

## Task 2: Lead 탐색 (selectLead) + 단위변환 — 순수 로직

**Files:**
- Modify: `catkin_ws/src/path_tracking/include/path_tracking/acc_core.hpp`
- Modify: `catkin_ws/src/path_tracking/test/acc_core_test.cpp`

**Interfaces:**
- Consumes: `AccParams`, `Vec2`, `ObjIn`, `Lead` (Task 1)
- Produces:
  - `acc::Lead acc::selectLead(const std::vector<acc::Vec2>& path, const acc::Vec2& ego, const std::vector<acc::ObjIn>& objs, const acc::AccParams& p);`
  - `double acc::speedKmhToMps(double, double)` (Task 1에 이미 정의됨 — 여기서 테스트만 추가)

- [ ] **Step 1: 실패 테스트 추가**

`acc_core_test.cpp` 끝에 추가:

```cpp
// 경로: x축을 따라 (0,0)~(30,0), 0.5m 간격
static std::vector<Vec2> straightPath() {
  std::vector<Vec2> path;
  for (double x = 0.0; x <= 30.0; x += 0.5) path.push_back({x, 0.0});
  return path;
}

// km/h → m/s: (18,0)km/h = 5 m/s
TEST(SpeedKmhToMps, Converts) {
  EXPECT_NEAR(speedKmhToMps(18.0, 0.0), 5.0, 1e-6);
  EXPECT_NEAR(speedKmhToMps(0.0, 0.0), 0.0, 1e-6);
}

// 경로 위 객체 하나 → 선택, distance = 상대거리 − vehicle_length
TEST(SelectLead, ObjectOnPathSelected) {
  AccParams p = defaultParams();
  std::vector<Vec2> path = straightPath();
  Vec2 ego{0.0, 0.0};
  std::vector<ObjIn> objs = { { {20.0, 0.0}, 5.0 } };  // 정면 20m, 5 m/s
  Lead lead = selectLead(path, ego, objs, p);
  ASSERT_TRUE(lead.present);
  EXPECT_NEAR(lead.distance, 20.0 - p.vehicle_length, 1e-6);
  EXPECT_NEAR(lead.velocity, 5.0, 1e-6);
}

// 경로에서 횡으로 벗어난 객체 → 무시
TEST(SelectLead, ObjectOffPathIgnored) {
  AccParams p = defaultParams();
  std::vector<Vec2> path = straightPath();
  Vec2 ego{0.0, 0.0};
  std::vector<ObjIn> objs = { { {10.0, 5.0}, 0.0 } };  // 횡거리 5 > 2.5
  Lead lead = selectLead(path, ego, objs, p);
  EXPECT_FALSE(lead.present);
}

// 두 객체 → 더 가까운 것 선택
TEST(SelectLead, NearestChosen) {
  AccParams p = defaultParams();
  std::vector<Vec2> path = straightPath();
  Vec2 ego{0.0, 0.0};
  std::vector<ObjIn> objs = { { {20.0, 0.0}, 0.0 }, { {10.0, 0.0}, 0.0 } };
  Lead lead = selectLead(path, ego, objs, p);
  ASSERT_TRUE(lead.present);
  EXPECT_NEAR(lead.distance, 10.0 - p.vehicle_length, 1e-6);
}

// 객체 없음 → present=false
TEST(SelectLead, EmptyNoLead) {
  AccParams p = defaultParams();
  std::vector<Vec2> path = straightPath();
  Vec2 ego{0.0, 0.0};
  std::vector<ObjIn> objs;
  Lead lead = selectLead(path, ego, objs, p);
  EXPECT_FALSE(lead.present);
}
```

- [ ] **Step 2: 빌드 → 실패 확인**

Run:
```bash
cd ~/morai-ros/catkin_ws && catkin_make tests
```
Expected: **컴파일 실패** (`selectLead` 미정의). 이것이 red 상태.

- [ ] **Step 3: selectLead 구현**

`acc_core.hpp`의 `computeTargetVelocity` 위에 추가:

```cpp
// 기준경로 위의 전방 객체 중 ego에 가장 가까운 것을 lead로 선택.
// path 위 판정 = 어떤 path 점과의 횡거리 < distance_threshold.
// distance = ego 상대거리 − vehicle_length.
inline Lead selectLead(const std::vector<Vec2>& path, const Vec2& ego,
                       const std::vector<ObjIn>& objs, const AccParams& p) {
  Lead lead;
  double min_rel = std::numeric_limits<double>::infinity();

  for (const auto& o : objs) {
    // 경로 위인지: 최근접 path 점까지 거리
    double min_to_path = std::numeric_limits<double>::infinity();
    for (const auto& pt : path) {
      double d = std::hypot(pt.x - o.pos.x, pt.y - o.pos.y);
      if (d < min_to_path) min_to_path = d;
    }
    if (min_to_path >= p.distance_threshold) continue;  // 경로 밖

    double rel = std::hypot(o.pos.x - ego.x, o.pos.y - ego.y);
    if (rel < min_rel) {
      min_rel = rel;
      lead.present  = true;
      lead.distance = rel - p.vehicle_length;
      lead.velocity = o.speed_mps;
    }
  }
  return lead;
}
```

- [ ] **Step 4: 빌드 → 통과 확인**

Run:
```bash
cd ~/morai-ros/catkin_ws && catkin_make tests && ./devel/lib/path_tracking/acc_core_test
```
Expected: **PASS** (전체: ComputeTargetVelocity 5 + SpeedKmhToMps 1 + SelectLead 4 = 10 tests)

- [ ] **Step 5: 커밋**

```bash
cd ~/morai-ros
git add catkin_ws/src/path_tracking/include catkin_ws/src/path_tracking/test
git commit -m "feat(acc): selectLead 전방객체 탐색 + km/h→m/s 변환"
```

---

## Task 3: ROS 노드 acc_planner.cpp — 구독/발행 래퍼

**Files:**
- Create: `catkin_ws/src/path_tracking/src/acc_planner.cpp`
- Modify: `catkin_ws/src/path_tracking/CMakeLists.txt`
- Modify: `catkin_ws/src/path_tracking/launch/sim.launch`

**Interfaces:**
- Consumes: `acc::selectLead`, `acc::computeTargetVelocity`, `acc::speedKmhToMps`, `acc::AccParams`, `acc::Vec2`, `acc::ObjIn`, `acc::Lead` (Task 1·2)
- Produces: ROS 노드 `acc_planner`, 토픽 `/target_velocity` (`std_msgs/Float64`, m/s)

- [ ] **Step 1: acc_planner.cpp 작성**

Create `catkin_ws/src/path_tracking/src/acc_planner.cpp`:

```cpp
// acc_planner : 종방향 목표속도 planning 노드 (ACC)
// ------------------------------------------------------------------
// 구독:  /local_path   (nav_msgs/Path)          기준 지역경로
//        /lattice_path (nav_msgs/Path)          lattice 회피경로 (있으면 우선)
//        /ego_status   (EgoVehicleStatus)        현재 위치·속도 [m/s]
//        /Object_topic (ObjectStatusList)        장애물 (속도 [km/h])
// 발행:  /target_velocity (std_msgs/Float64)     목표속도 [m/s]
//
// 동작: 추종경로(lattice 최근값 우선) 위 전방객체 탐색 → 레퍼런스 ACC식 → 60kph 캡.
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <morai_msgs/ObjectStatus.h>
#include "path_tracking/acc_core.hpp"

class AccPlanner
{
public:
  AccPlanner()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // 파라미터 (기본값은 acc::AccParams). cruise/max는 km/h로 받아 내부 m/s 변환.
    double cruise_kmh = 60.0, max_kmh = 60.0;
    pnh.param("time_gap",           params_.time_gap,           params_.time_gap);
    pnh.param("default_space",      params_.default_space,      params_.default_space);
    pnh.param("vehicle_length",     params_.vehicle_length,     params_.vehicle_length);
    pnh.param("distance_threshold", params_.distance_threshold, params_.distance_threshold);
    pnh.param("velocity_gain",      params_.velocity_gain,      params_.velocity_gain);
    pnh.param("distance_gain",      params_.distance_gain,      params_.distance_gain);
    pnh.param("cruise_speed_kmh",   cruise_kmh,                 cruise_kmh);
    pnh.param("max_speed_kmh",      max_kmh,                    max_kmh);
    params_.cruise_speed = cruise_kmh / 3.6;
    params_.max_speed    = max_kmh / 3.6;

    sub_local_   = nh.subscribe("/local_path",   1, &AccPlanner::localCb,   this);
    sub_lattice_ = nh.subscribe("/lattice_path", 1, &AccPlanner::latticeCb, this);
    sub_ego_     = nh.subscribe("/ego_status",   1, &AccPlanner::egoCb,     this);
    sub_obj_     = nh.subscribe("/Object_topic", 1, &AccPlanner::objCb,     this);
    pub_vel_     = nh.advertise<std_msgs::Float64>("/target_velocity", 1);
    timer_       = nh.createTimer(ros::Duration(1.0 / 30.0), &AccPlanner::run, this);
    ROS_INFO("[acc_planner] started (cruise=%.1f km/h, cap=%.1f km/h)", cruise_kmh, max_kmh);
  }

private:
  acc::AccParams params_;
  ros::Subscriber sub_local_, sub_lattice_, sub_ego_, sub_obj_;
  ros::Publisher pub_vel_;
  ros::Timer timer_;

  nav_msgs::Path local_path_, lattice_path_;
  morai_msgs::EgoVehicleStatus ego_;
  morai_msgs::ObjectStatusList objs_;
  ros::Time lattice_stamp_;
  bool has_local_ = false, has_ego_ = false, has_obj_ = false;

  void localCb(const nav_msgs::Path::ConstPtr& m)   { local_path_ = *m; has_local_ = true; }
  void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& m) { ego_ = *m; has_ego_ = true; }
  void objCb(const morai_msgs::ObjectStatusList::ConstPtr& m) { objs_ = *m; has_obj_ = true; }
  void latticeCb(const nav_msgs::Path::ConstPtr& m) { lattice_path_ = *m; lattice_stamp_ = ros::Time::now(); }

  // 추종경로 선택 (path_tracker와 동일 규칙: lattice 최근 0.3s + 점 2개↑면 그것)
  std::vector<acc::Vec2> followPath()
  {
    std::vector<acc::Vec2> out;
    bool lattice_fresh = !lattice_stamp_.isZero() &&
                         (ros::Time::now() - lattice_stamp_).toSec() < 0.3 &&
                         lattice_path_.poses.size() > 1;
    const nav_msgs::Path& src = lattice_fresh ? lattice_path_ : local_path_;
    for (const auto& ps : src.poses)
      out.push_back({ps.pose.position.x, ps.pose.position.y});
    return out;
  }

  // npc_list + obstacle_list → ObjIn (속도 km/h → m/s)
  std::vector<acc::ObjIn> gatherObjects()
  {
    std::vector<acc::ObjIn> v;
    auto add = [&](const std::vector<morai_msgs::ObjectStatus>& list) {
      for (const auto& o : list)
        v.push_back({ {o.position.x, o.position.y},
                      acc::speedKmhToMps(o.velocity.x, o.velocity.y) });
    };
    add(objs_.npc_list);
    add(objs_.obstacle_list);
    return v;
  }

  void run(const ros::TimerEvent&)
  {
    if (!(has_local_ && has_ego_ && has_obj_)) return;  // 미수신 시 발행 보류

    std::vector<acc::Vec2> path = followPath();
    if (path.empty()) return;

    acc::Vec2 ego{ego_.position.x, ego_.position.y};
    double ego_vel = std::hypot(ego_.velocity.x, ego_.velocity.y);  // m/s

    std::vector<acc::ObjIn> objs = gatherObjects();
    acc::Lead lead = acc::selectLead(path, ego, objs, params_);
    double target = acc::computeTargetVelocity(ego_vel, lead, params_);

    std_msgs::Float64 msg;
    msg.data = target;
    pub_vel_.publish(msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "acc_planner");
  AccPlanner ap;
  ros::spin();
  return 0;
}
```

- [ ] **Step 2: CMakeLists에 실행타깃 추가**

`catkin_ws/src/path_tracking/CMakeLists.txt`의 Task 1에서 추가한 `# ==== ACC 추가 ====` 블록 아래(gtest 위 또는 아래)에:

```cmake
add_executable(acc_planner src/acc_planner.cpp)
add_dependencies(acc_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(acc_planner ${catkin_LIBRARIES})
```

- [ ] **Step 3: 빌드 확인**

Run:
```bash
cd ~/morai-ros/catkin_ws && catkin_make
```
Expected: 빌드 성공, `devel/lib/path_tracking/acc_planner` 생성.

- [ ] **Step 4: launch에 노드 추가**

`catkin_ws/src/path_tracking/launch/sim.launch`의 `</launch>` 바로 위에 추가:

```xml
  <node pkg="path_tracking" type="acc_planner" name="acc_planner" output="screen">
    <param name="cruise_speed_kmh" value="50.0"/>
    <param name="max_speed_kmh"    value="60.0"/>
    <param name="time_gap"         value="1.0"/>
    <param name="default_space"    value="5.0"/>
  </node>
```

- [ ] **Step 5: 스모크 테스트 (수동 발행으로 토픽 확인)**

터미널 A:
```bash
cd ~/morai-ros/catkin_ws && source devel/setup.bash && roscore
```
터미널 B (노드 실행):
```bash
source ~/morai-ros/catkin_ws/devel/setup.bash
rosrun path_tracking acc_planner _cruise_speed_kmh:=50 _max_speed_kmh:=60
```
터미널 C (최소 입력 발행 — ego + 빈 객체 + 직선 local_path):
```bash
source ~/morai-ros/catkin_ws/devel/setup.bash
# ego 정지 상태
rostopic pub -r 10 /ego_status morai_msgs/EgoVehicleStatus '{position: {x: 0, y: 0, z: 0}, velocity: {x: 0, y: 0, z: 0}, heading: 0}' &
# 빈 객체 리스트
rostopic pub -r 10 /Object_topic morai_msgs/ObjectStatusList '{num_of_npcs: 0, num_of_pedestrian: 0, num_of_obstacle: 0}' &
# 직선 local_path 2점
rostopic pub -r 10 /local_path nav_msgs/Path '{header: {frame_id: "map"}, poses: [{pose: {position: {x: 0, y: 0, z: 0}}}, {pose: {position: {x: 30, y: 0, z: 0}}}]}' &
```
터미널 D (결과 확인):
```bash
source ~/morai-ros/catkin_ws/devel/setup.bash
rostopic echo /target_velocity
```
Expected: `data: 13.888...` (앞차 없음 → cruise 50km/h = 13.89 m/s). 확인 후 `kill %1 %2 %3`로 pub 종료.

- [ ] **Step 6: 커밋**

```bash
cd ~/morai-ros
git add catkin_ws/src/path_tracking/src/acc_planner.cpp \
        catkin_ws/src/path_tracking/CMakeLists.txt \
        catkin_ws/src/path_tracking/launch/sim.launch
git commit -m "feat(acc): acc_planner ROS 노드 + /target_velocity 발행"
```

---

## Task 4: mock_lead_vehicle.cpp — 오프라인 검증용 느린 앞차 (옵션 B: 경로 추종)

**설계 결정 (2026-07):** 앞차 이동 모델을 **직선(+x)이 아니라 경로 추종(옵션 B)** 으로 확정.
이유: 우리 대회 경로(`path_smooth.csv`)는 굽은 winding 코스라, 직선으로 가는 앞차는 몇 초 뒤 경로에서
2.5m 넘게 벗어나고 `acc_core::selectLead`(경로 위 판정 = 최근접 path 점 횡거리 < `distance_threshold`)가
앞차를 놓쳐 테스트가 무의미해진다. 경로 위를 호길이만큼 전진시키면 앞차가 항상 경로 위에 있어 ACC의
감속-추종을 끝까지 관찰할 수 있고, 실제 대회 NPC(차선 주행)와도 가장 가깝다.

**두 층(layer) 구분 — mock은 버려지지 않음:**
- **Layer 1 (지금, 시뮬 없이):** `mock_lead_vehicle` → `/Object_topic` → ACC. **ACC 로직** 검증.
- **Layer 2 (나중, 시뮬):** MORAI 실제 NPC → perception → `/Object_topic` → ACC. **전체 파이프라인·타이밍·단위** 검증.
- 두 층이 **같은 `/Object_topic` 인터페이스**를 공유 → 시뮬 갈 때 mock만 끄면 됨. (아래 Task 7 참고)

**Files:**
- Create: `catkin_ws/src/path_tracking/src/mock_lead_vehicle.cpp`
- Modify: `catkin_ws/src/path_tracking/CMakeLists.txt` (실행타깃 + `roslib` 컴포넌트 — `ros/package.h`용)
- Modify: `catkin_ws/src/path_tracking/launch/sim.launch` (나중 통합 때)

**Interfaces:**
- Produces: ROS 노드 `mock_lead_vehicle`, 토픽 `/Object_topic`(`morai_msgs/ObjectStatusList`)에 움직이는 NPC 1대 발행.
  (`mock_obstacle_pub`과 **동시 실행 금지** — 둘 다 `/Object_topic` 발행. 택일.)

**동작 흐름:**
1. 시작 시 `path_smooth.csv`(x,y,z 헤더, ~3766점) 1회 로드 → `(x,y)` 벡터 + **누적 호길이** `s[i]` (인접점 거리 누적).
2. 매 틱(20Hz): 앞차의 경로상 위치 `s = start_gap + v·t` (`v = lead_speed_kmh/3.6` [m/s]).
3. `s`가 속한 세그먼트를 **선형보간** → `(x,y)`. 세그먼트 방향 = `heading(yaw)`.
4. `s`가 경로 끝을 넘으면 **마지막 점에 정지(clamp)** (테스트 중엔 무해).
5. `ObjectStatus` NPC로 채워 발행.

**필드 규약:** `type=1`(NPC), `unique_id=10`, `position`=보간 경로점(ENU),
`velocity`=`speed_kmh·(cos yaw, sin yaw)` [km/h, 접선방향 → heading과 일치. acc는 크기만 쓰지만 lattice 예측 재사용 대비],
`size`=1.9×4.6×1.5(Ioniq5), `heading`=yaw[deg]. `frame_id="map"`.

**Params (`~private`):** `start_gap`(기본 30m), `lead_speed_kmh`(기본 18 — 60캡보다 느려 감속 유발),
`path_file`(기본 = `ros::package::getPath("path_tracking")+"/path/path_smooth.csv"`).

- [ ] **Step 1:** `mock_lead_vehicle.cpp` 작성 — csv 로드 + 호길이 누적 → 메인루프(보간·발행). (초보 학습: 한 조각씩 같이 작성)
- [ ] **Step 2:** `CMakeLists.txt` — `find_package` COMPONENTS에 `roslib` 추가 + 실행타깃 3줄:
  ```cmake
  add_executable(mock_lead_vehicle src/mock_lead_vehicle.cpp)
  add_dependencies(mock_lead_vehicle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(mock_lead_vehicle ${catkin_LIBRARIES})
  ```
- [ ] **Step 3:** 빌드 — `cd ~/morai-ros/catkin_ws && catkin_make`, `devel/lib/path_tracking/mock_lead_vehicle` 생성 확인.
- [ ] **Step 4:** 동작 확인 — `mock_lead_vehicle` 실행 후 `rostopic echo /Object_topic`으로 NPC position이 경로 따라 이동하는지, `rviz`(object_viz 마커)로 눈으로 확인.
- [ ] **Step 5:** 커밋 — `git add src/mock_lead_vehicle.cpp CMakeLists.txt` → `feat(acc): mock_lead_vehicle 경로추종 앞차 발행 노드`.

**Layer 2 (시뮬 시나리오) — 나중 확정:** MORAI Scenario 에디터로 ego 앞 차선에 NPC 배치 + 목표속도 ~18km/h
지정(옵션 A 유력). 정확한 에디터 절차는 시뮬 붙일 때 MORAI 공식 시나리오 문서로 확인. (대안: 기록-재생 ghost / 네트워크 NPC 제어)

---

## Task 5: 통합 오프라인 검증 (mock 앞차 → ACC 반응 관찰)

**Files:**
- Modify: `catkin_ws/src/path_tracking/scripts/path_tracker.py` (임시 종제어를 `/target_velocity` 구독으로 교체)

**Interfaces:**
- Consumes: `/target_velocity` (`std_msgs/Float64`, Task 3)

- [ ] **Step 1: path_tracker.py가 /target_velocity를 종제어에 사용 (임시)**

`path_tracker.py` 수정 — 상단 import에 추가:

```python
from std_msgs.msg import Float64
```

`__init__` 안, `rospy.Subscriber('/ego_status', ...)` 아래에 추가:

```python
        self.acc_target_mps = None   # ACC가 준 목표속도 (없으면 자체 target 사용)
        rospy.Subscriber('/target_velocity', Float64, self.acc_callback)
```

`__init__` 안 아무 메서드 정의부에 콜백 추가 (예: `lattice_callback` 아래):

```python
    def acc_callback(self, msg):
        self.acc_target_mps = float(msg.data)
```

`callback` 안 종방향 블록(현재 106-110줄, `if speed < target_velocity:` 부분)을 교체:

```python
        # 종방향: ACC 목표속도가 오면 그걸 우선 사용 (없으면 기존 상수 target)
        desired = self.acc_target_mps if self.acc_target_mps is not None else target_velocity
        if speed < desired:
            accel, brake = 0.3, 0.0
        else:
            accel, brake = 0.0, 0.1
        # 완전 정지 요청(desired≈0) 시 확실히 제동
        if desired < 0.1:
            accel, brake = 0.0, 1.0
```

- [ ] **Step 2: 시나리오 A — 느린 앞차 추종 검증**

터미널 1 (roscore):
```bash
cd ~/morai-ros/catkin_ws && source devel/setup.bash && roscore
```
터미널 2 (acc + mock 앞차 + 가짜 ego/경로):
```bash
source ~/morai-ros/catkin_ws/devel/setup.bash
rosrun path_tracking acc_planner _cruise_speed_kmh:=50 _max_speed_kmh:=60 &
rosrun path_tracking mock_lead_vehicle _start_gap:=30 _lead_speed_kmh:=18 &
# ego: 정면 +x로 10 m/s 주행 중이라고 가정 (정지→접근 관찰하려면 velocity.x 조절)
rostopic pub -r 10 /ego_status morai_msgs/EgoVehicleStatus '{position: {x: 0, y: 0, z: 0}, velocity: {x: 10, y: 0, z: 0}, heading: 0}' &
rostopic pub -r 10 /local_path nav_msgs/Path '{header: {frame_id: "map"}, poses: [{pose: {position: {x: 0, y: 0, z: 0}}}, {pose: {position: {x: 60, y: 0, z: 0}}}]}' &
```
터미널 3 (관찰):
```bash
source ~/morai-ros/catkin_ws/devel/setup.bash
rostopic echo /target_velocity
```
Expected(검증항목 ①③④):
- 앞차(18km/h=5m/s)가 30m 앞 → 초기엔 여유 있어 `target ≈ 13.89`(cruise 50km/h) 근처
- ego(10m/s)가 앞차보다 빨라 접근 → `target`이 **5 m/s(앞차속도) 부근으로 감소**해 추종
- 앞차 제거(터미널2에서 mock kill) → `target`이 다시 **cruise 13.89로 복귀**
- 값이 **16.67(60kph)을 절대 초과하지 않음**

- [ ] **Step 3: 시나리오 B — 정적장애물 정지 검증**

터미널 2의 `mock_lead_vehicle`를 끄고 정적 mock으로 교체:
```bash
# 기존 mock_lead_vehicle 종료 후
rosrun path_tracking mock_obstacle_pub &   # -115.5,-338.5 정적장애물
```
ego/경로를 그 장애물 근처로 발행하도록 좌표만 맞춰서 pub (또는 mock_lead_vehicle을 `_lead_speed_kmh:=0`으로 재실행해 정지 장애물 흉내):
```bash
rosrun path_tracking mock_lead_vehicle _start_gap:=6 _lead_speed_kmh:=0 &
rostopic pub -r 10 /ego_status morai_msgs/EgoVehicleStatus '{position: {x: 0, y: 0, z: 0}, velocity: {x: 5, y: 0, z: 0}, heading: 0}' &
```
Expected(검증항목 ②): 앞 6m에 정지 물체 → `lead.distance = 6 - 4.635 = 1.365 < default_space(5)` → `target = 0` (정지).

- [ ] **Step 4: 커밋**

```bash
cd ~/morai-ros
git add catkin_ws/src/path_tracking/scripts/path_tracker.py
git commit -m "feat(acc): path_tracker가 /target_velocity로 종제어(임시 통합)"
```

---

## Task 6: control팀 인계 인터페이스 문서

**Files:**
- Create: `docs/target_velocity_interface.md`

- [ ] **Step 1: 인터페이스 문서 작성**

Create `docs/target_velocity_interface.md`:

```markdown
# /target_velocity 인터페이스 (planning → control)

작성일: 2026-07-15 / planning

## 개요
ACC(`acc_planner`)가 발행하는 **종방향 목표속도**. control팀 추종기가 이걸 구독해
현재속도와의 오차로 accel/brake(pedal)를 만든다. (개발 중엔 임시 `path_tracker.py`가 소비)

## 토픽
| 항목 | 값 |
|------|-----|
| 토픽명 | `/target_velocity` |
| 타입 | `std_msgs/Float64` |
| 단위 | **m/s** |
| 발행 주기 | 30 Hz |
| 범위 | 0 ~ 16.67 (60kph 하드캡) |

## 계약 (control팀 준수사항)
- `data`는 "지금 내야 할 목표속도[m/s]". 이 속도를 추종하도록 종제어할 것.
- `data == 0`은 **정지 요청**(앞 막힘/근접). 확실히 제동.
- **미수신 또는 stale(>0.3s)** 이면 ACC 미동작으로 간주하고 **안전하게 감속/정지**로 폴백.
  (staleness는 구독측에서 마지막 수신시각으로 판단. 메시지에 stamp 없음 — 필요 시 추후 stamped 타입 논의.)
- 조향은 별개(`/lattice_path` 또는 `/local_path` 추종). ACC는 `/ctrl_cmd`를 건드리지 않음.

## 관련
- 설계: [acc_design.md](acc_design.md)
- lattice(횡) 출력: `/lattice_path`
```

- [ ] **Step 2: 커밋**

```bash
cd ~/morai-ros
git add docs/target_velocity_interface.md
git commit -m "docs(acc): /target_velocity 인터페이스 명세 (control 인계)"
```

---

## 검증 요약 (spec §6 대응)

| 검증항목 | 태스크·스텁 |
|----------|-------------|
| ① 앞차보다 느리게 수렴(간격 유지) | Task 2 `SelectLead.*` + Task 1 `SlowerLeadDecelerates` + Task 5 시나리오 A |
| ② 정적장애물 앞 정지 | Task 1 `TooCloseStops` + Task 5 시나리오 B |
| ③ 앞 비면 크루즈 복귀 | Task 1 `NoLeadReturnsCruise` + Task 5 시나리오 A |
| ④ 60kph 캡 | Task 1 `HardCapAt60` + Task 5 시나리오 A |
| 단위 변환(km/h→m/s) | Task 2 `SpeedKmhToMps` |
| lattice 협력(추종경로 기준) | Task 3 `followPath()` |
