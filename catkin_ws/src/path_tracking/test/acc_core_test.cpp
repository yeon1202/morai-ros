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
