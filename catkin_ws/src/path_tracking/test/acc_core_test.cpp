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

// 경로: ego 뒤쪽까지 포함한 긴 직선 (-20 ~ 80). ego 는 원점에 둔다.
static std::vector<Vec2> longPath() {
  std::vector<Vec2> path;
  for (double x = -20.0; x <= 80.0; x += 0.5) path.push_back({x, 0.0});
  return path;
}

// 경로: 반지름 20m 사분원. 시작(0,0) → 끝(20,20).
// 호길이 = 2*pi*20/4 = 31.42m 인데, 직선거리는 28.28m 뿐이다.
static std::vector<Vec2> arcPath() {
  std::vector<Vec2> path;
  const double R = 20.0;
  for (int i = 0; i <= 100; ++i) {
    double th = (M_PI / 2.0) * i / 100.0;
    path.push_back({R * std::sin(th), R - R * std::cos(th)});
  }
  return path;
}

// --- 투영 (s, d) ---

// waypoint 사이의 점도 정확히 잡는다 (점 기준이면 0.5m 간격으로 양자화된다)
TEST(ProjectToPath, InterpolatesBetweenWaypoints) {
  std::vector<Vec2> path = longPath();
  Projection pr = projectToPath(path, {10.3, 1.2});
  ASSERT_TRUE(pr.valid);
  EXPECT_NEAR(pr.s, 30.3, 1e-6);      // 경로 시작(-20) 기준
  EXPECT_NEAR(pr.d, 1.2, 1e-6);
}

// 곡선에서 s 는 호길이여야 한다 (직선거리 28.28 이 아니라 31.42)
TEST(ProjectToPath, ArcLengthNotChord) {
  std::vector<Vec2> path = arcPath();
  Projection pr = projectToPath(path, {20.0, 20.0});
  ASSERT_TRUE(pr.valid);
  EXPECT_NEAR(pr.s, 31.416, 0.05);
  EXPECT_NEAR(std::hypot(20.0, 20.0), 28.284, 0.01);   // 직선거리는 이만큼 짧다
}

// --- 앞뒤 구분 / 탐색범위 ---

// 뒤에 있는 차는 lead 가 아니다 (예전 방식은 부호가 없어 앞차로 오인할 수 있었다)
TEST(SelectLead, RearVehicleIgnored) {
  AccParams p = defaultParams();
  std::vector<Vec2> path = longPath();
  Vec2 ego{0.0, 0.0};
  std::vector<ObjIn> objs = { { {-10.0, 0.0}, 5.0 } };   // 10m 뒤
  Lead lead = selectLead(path, ego, objs, p);
  EXPECT_FALSE(lead.present);
}

// 앞뒤에 하나씩 있으면 앞차만 잡는다
TEST(SelectLead, PicksFrontNotNearerRear) {
  AccParams p = defaultParams();
  std::vector<Vec2> path = longPath();
  Vec2 ego{0.0, 0.0};
  std::vector<ObjIn> objs = { { {-5.0, 0.0}, 1.0 },     // 5m 뒤 (더 가까움)
                              { {25.0, 0.0}, 2.0 } };   // 25m 앞
  Lead lead = selectLead(path, ego, objs, p);
  ASSERT_TRUE(lead.present);
  EXPECT_NEAR(lead.distance, 25.0 - p.vehicle_length, 1e-6);
  EXPECT_NEAR(lead.velocity, 2.0, 1e-6);
}

// lookahead 밖의 차는 무시
TEST(SelectLead, BeyondLookaheadIgnored) {
  AccParams p = defaultParams();
  p.lookahead = 30.0;
  std::vector<Vec2> path = longPath();
  Vec2 ego{0.0, 0.0};
  std::vector<ObjIn> objs = { { {45.0, 0.0}, 0.0 } };
  Lead lead = selectLead(path, ego, objs, p);
  EXPECT_FALSE(lead.present);

  p.lookahead = 60.0;                                   // 범위를 넓히면 잡힌다
  lead = selectLead(path, ego, objs, p);
  ASSERT_TRUE(lead.present);
  EXPECT_NEAR(lead.distance, 45.0 - p.vehicle_length, 1e-6);
}

// 곡선에서 gap 은 호길이 기준이어야 한다 (직선거리로 재면 3m 넘게 짧게 나온다)
TEST(SelectLead, CurvedPathUsesArcLength) {
  AccParams p = defaultParams();
  std::vector<Vec2> path = arcPath();
  Vec2 ego{0.0, 0.0};
  std::vector<ObjIn> objs = { { {20.0, 20.0}, 3.0 } };
  Lead lead = selectLead(path, ego, objs, p);
  ASSERT_TRUE(lead.present);
  EXPECT_NEAR(lead.distance, 31.416 - p.vehicle_length, 0.05);
  // 직선거리로 쟀다면 28.284 - vehicle_length 가 나왔을 것 → 3.1m 과소평가
  EXPECT_GT(lead.distance, 28.284 - p.vehicle_length + 2.0);
}

// --- 곡률 ---

// 반지름 R 인 원 위의 세 점 → 외접원 반지름이 정확히 R 이어야 한다.
// (abc/(4K) 대신 abc/|외적| 로 쓰면 2R 이 나온다. 실제로 이 실수를 했었다.)
TEST(CircumRadius, MatchesKnownCircle) {
  for (double R : {5.0, 10.0, 50.0}) {
    for (double th : {0.05, 0.2, 0.5}) {
      Vec2 a{R * std::cos(-th), R * std::sin(-th)};
      Vec2 b{R, 0.0};
      Vec2 c{R * std::cos(th), R * std::sin(th)};
      EXPECT_NEAR(circumRadius(a, b, c), R, R * 1e-6)
          << "R=" << R << " th=" << th;
    }
  }
}

TEST(CircumRadius, StraightLineIsInfinite) {
  EXPECT_TRUE(std::isinf(circumRadius({0, 0}, {1, 0}, {2, 0})));
}

// 반지름 R 인 원호 경로 (0.5m 간격)
static std::vector<Vec2> circlePath(double R, double total_angle) {
  std::vector<Vec2> path;
  int steps = static_cast<int>(R * total_angle / 0.5);
  for (int i = 0; i <= steps; ++i) {
    double th = total_angle * i / steps;
    path.push_back({R * std::sin(th), R - R * std::cos(th)});
  }
  return path;
}

// 직선에서는 제한이 걸리지 않는다
TEST(CurvatureSpeedLimit, StraightPathNoLimit) {
  AccParams p = defaultParams();
  std::vector<Vec2> path;
  for (double x = 0.0; x <= 100.0; x += 0.5) path.push_back({x, 0.0});
  EXPECT_NEAR(curvatureSpeedLimit(path, p), p.max_speed, 1e-6);
}

// 지금 당장 곡선에 들어가 있으면 v = sqrt(a_lat * R)
TEST(CurvatureSpeedLimit, OnCurveUsesLateralAccel) {
  AccParams p = defaultParams();
  p.curve_baseline = 5.0;                       // 짧은 원호라 baseline 을 줄임
  std::vector<Vec2> path = circlePath(30.0, M_PI / 2.0);
  double expected = std::sqrt(p.lat_accel_limit * 30.0);   // = 9.39 m/s
  EXPECT_NEAR(curvatureSpeedLimit(path, p), expected, 0.3);
}

// 곡선이 멀수록 지금 허용속도는 높다 (미리 감속하되 필요 이상으로 느려지지 않음)
TEST(CurvatureSpeedLimit, AnticipatesDistantCurve) {
  AccParams p = defaultParams();
  p.curve_baseline = 5.0;

  std::vector<Vec2> curve = circlePath(15.0, M_PI / 2.0);
  double v_curve = std::sqrt(p.lat_accel_limit * 15.0);     // 6.64 m/s

  // 곡선 앞에 직선을 lead_m 만큼 붙인다
  auto withLeadIn = [&](double lead_m) {
    std::vector<Vec2> path;
    for (double x = -lead_m; x < 0.0; x += 0.5) path.push_back({x, 0.0});
    for (const auto& q : curve) path.push_back(q);
    return path;
  };

  double near_limit = curvatureSpeedLimit(withLeadIn(10.0), p);
  double far_limit  = curvatureSpeedLimit(withLeadIn(40.0), p);

  EXPECT_GT(near_limit, v_curve);          // 아직 곡선 전이므로 곡선속도보다는 빠름
  EXPECT_GT(far_limit, near_limit);        // 멀수록 더 빨라도 됨
  EXPECT_LE(far_limit, p.max_speed);

  // 등감속 공식 v = sqrt(v_curve^2 + 2*a*d) 와 맞아야 한다.
  // 단 d 는 정확히 lead_in(10m) 이 아니다: 직선→곡선 전이 구간에서는 baseline 이
  // 양쪽에 걸쳐 곡률이 완만하게 잡히므로, 진짜 R=15 가 나오는 지점은 곡선 시작보다
  // curve_baseline 만큼 더 안쪽이다. 따라서 실효 d 는 [10, 10+baseline] 사이.
  double v_at_10 = std::sqrt(v_curve * v_curve + 2.0 * p.brake_accel * 10.0);
  double v_at_15 = std::sqrt(v_curve * v_curve +
                             2.0 * p.brake_accel * (10.0 + p.curve_baseline));
  EXPECT_GE(near_limit, v_at_10 - 1e-6);
  EXPECT_LE(near_limit, v_at_15 + 1e-6);
}

// 헤어핀에서도 하한 아래로는 안 내려간다
TEST(CurvatureSpeedLimit, RespectsMinSpeed) {
  AccParams p = defaultParams();
  p.curve_baseline = 2.0;
  p.curve_min_speed = 2.0;
  std::vector<Vec2> path = circlePath(1.0, M_PI);          // 반경 1m 헤어핀
  EXPECT_GE(curvatureSpeedLimit(path, p), p.curve_min_speed - 1e-6);
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
