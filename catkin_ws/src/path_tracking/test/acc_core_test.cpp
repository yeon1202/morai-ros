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
