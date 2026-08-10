#include <gtest/gtest.h>
#include "path_tracking/behavior_core.hpp"

using namespace behavior;

// 편의 생성자. now=100.0 을 기준 시각으로 삼는다.
static Limit fresh(double v, double stamp = 100.0) {
  Limit L; L.value = v; L.stamp = stamp; return L;
}
static Limit never() {
  return Limit();  // stamp = -1.0
}

// ---- 기본 합성 -------------------------------------------------------------

TEST(CombineLimits, PicksMinimum) {
  std::vector<Limit> v{fresh(15.0), fresh(8.0), fresh(12.0)};
  Combined c = combineLimits(v, 100.0, 0.5);
  EXPECT_NEAR(c.value, 8.0, 1e-9);
  EXPECT_EQ(c.winner, 1);
  EXPECT_EQ(c.alive, 3);
}

// 0(정지)도 정상적인 제약값이다. 신호등 빨간불이 이렇게 온다.
TEST(CombineLimits, ZeroIsAValidLimit) {
  std::vector<Limit> v{fresh(15.0), fresh(0.0)};
  Combined c = combineLimits(v, 100.0, 0.5);
  EXPECT_NEAR(c.value, 0.0, 1e-9);
  EXPECT_EQ(c.winner, 1);
}

// 아무것도 없으면 alive=0, winner=-1. 노드는 이걸 보고 발행을 멈춰야 한다.
TEST(CombineLimits, NothingReceivedYieldsNoWinner) {
  std::vector<Limit> v{never(), never()};
  Combined c = combineLimits(v, 100.0, 0.5);
  EXPECT_EQ(c.alive, 0);
  EXPECT_EQ(c.winner, -1);
  EXPECT_NEAR(c.value, kNoLimit, 1e-9);
}

TEST(CombineLimits, EmptyListIsSafe) {
  std::vector<Limit> v;
  Combined c = combineLimits(v, 100.0, 0.5);
  EXPECT_EQ(c.alive, 0);
  EXPECT_EQ(c.winner, -1);
}

// ---- "한 번도 안 옴" vs "오다가 끊김" ---------------------------------------

// 미수신은 정책과 무관하게 무시된다. perception 이 없는 개발 단계에서 보행자
// 제약이 걸려 차가 영영 느려지는 것을 막는다.
TEST(CombineLimits, NeverReceivedIsIgnoredEvenIfConservative) {
  Limit ped = never();
  ped.on_stale    = StaleAction::Conservative;
  ped.stale_value = 5.0;

  std::vector<Limit> v{fresh(15.0), ped};
  Combined c = combineLimits(v, 100.0, 0.5);
  EXPECT_NEAR(c.value, 15.0, 1e-9);   // 5.0 이 아니어야 한다
  EXPECT_EQ(c.alive, 1);
}

// 받다가 끊긴 Ignore 제약은 빠진다.
TEST(CombineLimits, StaleIgnoreIsDropped) {
  std::vector<Limit> v{fresh(15.0), fresh(3.0, /*stamp=*/99.0)};  // 1초 전
  Combined c = combineLimits(v, 100.0, 0.5);
  EXPECT_NEAR(c.value, 15.0, 1e-9);
  EXPECT_EQ(c.alive, 1);
  EXPECT_EQ(c.winner, 0);
}

// 받다가 끊긴 Conservative 제약은 대체값으로 살아남는다.
TEST(CombineLimits, StaleConservativeUsesFallback) {
  Limit ped = fresh(15.0, /*stamp=*/99.0);   // 1초 전에 끊김
  ped.on_stale    = StaleAction::Conservative;
  ped.stale_value = 7.5;

  std::vector<Limit> v{fresh(15.0), ped};
  Combined c = combineLimits(v, 100.0, 0.5);
  EXPECT_NEAR(c.value, 7.5, 1e-9);
  EXPECT_EQ(c.winner, 1);
  EXPECT_EQ(c.alive, 2);
}

// 경계: 정확히 timeout 만큼 지난 것은 아직 유효하다(> 로 비교하므로).
TEST(CombineLimits, ExactlyAtTimeoutStillAlive) {
  std::vector<Limit> v{fresh(4.0, /*stamp=*/99.5)};
  Combined c = combineLimits(v, 100.0, 0.5);
  EXPECT_EQ(c.alive, 1);
  EXPECT_NEAR(c.value, 4.0, 1e-9);
}

// ---- 진단용 winner ---------------------------------------------------------

// 같은 값이면 먼저 온 인덱스가 이긴다(< 비교라 교체되지 않음).
// 진단 로그가 매 틱 깜빡이지 않게 하려는 것이다.
TEST(CombineLimits, TieKeepsFirstIndex) {
  std::vector<Limit> v{fresh(9.0), fresh(9.0)};
  Combined c = combineLimits(v, 100.0, 0.5);
  EXPECT_EQ(c.winner, 0);
}

// 제한이 하나도 안 걸린 상태(전부 kNoLimit)에서도 alive 는 세어진다.
// "제약이 살아있지만 아무도 속도를 안 깎는다" 와 "제약이 죽었다" 는 다르다.
TEST(CombineLimits, AllNoLimitStillCountsAlive) {
  std::vector<Limit> v{fresh(kNoLimit), fresh(kNoLimit)};
  Combined c = combineLimits(v, 100.0, 0.5);
  EXPECT_EQ(c.alive, 2);
  EXPECT_NEAR(c.value, kNoLimit, 1e-9);
}
