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
  double distance_threshold = 2.5;    // [m] 경로 위 판정 횡거리 |d|
  double lookahead          = 60.0;   // [m] 전방 탐색 거리 (경로 따라간 종거리 기준)
  double velocity_gain      = 0.5;
  double distance_gain      = 1.0;
  double cruise_speed       = 16.67;  // [m/s] free-flow 목표 (=60kph)
  double max_speed          = 16.67;  // [m/s] 하드캡 (=60kph)

  // --- 곡률 기반 속도 제한 ---
  double lat_accel_limit    = 2.94;   // [m/s^2] 횡가속도 한계 (0.3G)
  double brake_accel        = 2.0;    // [m/s^2] 곡선 진입 전 감속에 쓸 감속도
  double curve_baseline     = 10.0;   // [m] 곡률 계산용 앞뒤 기준거리
  double curve_min_speed    = 2.0;    // [m/s] 곡률 제한의 하한 (헤어핀에서 0 방지)
};

// 경로 위로 투영한 결과 (Frenet 좌표)
struct Projection {
  double s = 0.0;         // 경로 시작부터의 종방향 거리 [m]
  double d = 0.0;         // 경로에서의 횡방향 거리 (부호 없음) [m]
  bool   valid = false;
};

// 점을 선분 [a,b] 에 투영. 반환 = 선분 위 비율 t(0~1) 와 그 지점까지 거리.
inline void projectOnSegment(const Vec2& p, const Vec2& a, const Vec2& b,
                             double& t, double& dist) {
  double vx = b.x - a.x, vy = b.y - a.y;
  double len2 = vx * vx + vy * vy;
  if (len2 < 1e-12) { t = 0.0; dist = std::hypot(p.x - a.x, p.y - a.y); return; }
  t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2;
  t = std::max(0.0, std::min(1.0, t));       // 선분 밖이면 끝점으로 clamp
  double px = a.x + t * vx, py = a.y + t * vy;
  dist = std::hypot(p.x - px, p.y - py);
}

// 점을 경로(폴리라인)에 투영해 (s, d) 를 구한다.
//
// 왜 "가장 가까운 점까지 거리" 가 아니라 투영인가:
//   점 기준이면 (1) 거리가 직선거리라 굽은 길에서 왜곡되고 (2) 앞/뒤 구분이 안 되며
//   (3) waypoint 간격만큼 양자화된다. s 로 바꾸면 앞뒤는 s 부호, 종거리는 s 차이,
//   차선 이탈은 d 로 각각 따로 판정할 수 있다.
inline Projection projectToPath(const std::vector<Vec2>& path, const Vec2& p) {
  Projection best;
  if (path.size() < 2) return best;

  double s_acc = 0.0, best_d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    double seg_len = std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    double t = 0.0, dist = 0.0;
    projectOnSegment(p, path[i], path[i + 1], t, dist);
    if (dist < best_d) {
      best_d = dist;
      best.s = s_acc + t * seg_len;
      best.d = dist;
      best.valid = true;
    }
    s_acc += seg_len;
  }
  return best;
}

// km/h 속도벡터 → m/s 스칼라 (단위 함정 처리)
inline double speedKmhToMps(double vx_kmh, double vy_kmh) {
  return std::hypot(vx_kmh, vy_kmh) / 3.6;
}

// 기준경로 위의 전방 객체 중 ego에 가장 가까운 것을 lead로 선택.
//   - 경로 위 판정 : |d| < distance_threshold   (횡방향)
//   - 전방 판정    : s_obj − s_ego > 0          (부호로 앞뒤 구분)
//   - 탐색 범위    : s_obj − s_ego <= lookahead (종방향, 경로 따라간 거리)
//   - distance     : 종방향 gap − vehicle_length
//
// 예전엔 직선거리 + "최근접 점까지 거리" 로 판정했는데, 그러면 굽은 길에서 거리가
// 왜곡되고(직선거리는 실제 주행거리보다 짧다) 뒤차와 앞차를 구분하지 못했다.
inline Lead selectLead(const std::vector<Vec2>& path, const Vec2& ego,
                       const std::vector<ObjIn>& objs, const AccParams& p) {
  Lead lead;
  Projection ego_proj = projectToPath(path, ego);
  if (!ego_proj.valid) return lead;

  double min_gap = std::numeric_limits<double>::infinity();

  for (const auto& o : objs) {
    Projection op = projectToPath(path, o.pos);
    if (!op.valid) continue;
    if (op.d >= p.distance_threshold) continue;      // 차선 밖

    double gap = op.s - ego_proj.s;                  // 종방향 gap (부호 있음)
    if (gap <= 0.0) continue;                        // 뒤차
    if (gap > p.lookahead) continue;                 // 탐색 범위 밖

    if (gap < min_gap) {
      min_gap = gap;
      lead.present  = true;
      lead.distance = gap - p.vehicle_length;
      lead.velocity = o.speed_mps;
    }
  }
  return lead;
}

// 세 점을 지나는 원의 반지름 (외접원). 직선이면 무한대.
//
//   R = abc / (4K),  K = 삼각형 넓이,  |외적| = 2K   ->   R = abc / (2*|외적|)
//   ※ |외적| 로 한 번만 나누면 2R 이 나온다. 실제로 이 실수를 해서 경로 곡률을
//     2배로 잘못 보고 "차량 최소회전반경 위반 없음" 이라고 판단한 적이 있다.
inline double circumRadius(const Vec2& a, const Vec2& b, const Vec2& c) {
  double cross = std::fabs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
  if (cross < 1e-9) return std::numeric_limits<double>::infinity();
  double la = std::hypot(b.x - a.x, b.y - a.y);
  double lb = std::hypot(c.x - b.x, c.y - b.y);
  double lc = std::hypot(c.x - a.x, c.y - a.y);
  return la * lb * lc / (2.0 * cross);
}

// 경로 곡률로부터 "지금" 낼 수 있는 최대 속도를 구한다.
//
//   반경 R 인 곡선을 횡가속도 한계 a_lat 로 도는 최대 속도 : v_curve = sqrt(a_lat * R)
//   그 곡선이 d[m] 앞에 있다면, 지금은 그보다 빨라도 된다. 등감속 a_brake 로 줄여서
//   진입 시점에 v_curve 가 되면 되므로  v_now <= sqrt(v_curve^2 + 2*a_brake*d).
//   전방 모든 지점에 대해 이 상한의 최솟값이 지금 허용속도다(=미리 감속).
//
// 곡률 baseline 을 길게(기본 10m) 잡는 이유: 기록 경로에는 지터가 있어서 3m 정도로
// 재면 실제로는 없는 급커브가 잡힌다(우리 경로에서 R=4.2m 로 나왔는데, 차량 최소
// 회전반경 5.87m 보다 작아 물리적으로 불가능한 값이었다. 10m 로 재면 5.93m).
inline double curvatureSpeedLimit(const std::vector<Vec2>& path, const AccParams& p) {
  if (path.size() < 3) return p.max_speed;

  std::vector<double> cum(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i)
    cum[i] = cum[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);

  double limit = p.max_speed;
  for (size_t i = 0; i < path.size(); ++i) {
    // i 기준 앞뒤로 curve_baseline[m] 떨어진 점을 찾는다
    size_t a = i, b = i;
    while (a > 0 && cum[i] - cum[a] < p.curve_baseline) --a;
    while (b + 1 < path.size() && cum[b] - cum[i] < p.curve_baseline) ++b;
    if (a == i || b == i) continue;              // 경로 양끝은 baseline 확보 불가

    double v_curve = std::sqrt(p.lat_accel_limit * circumRadius(path[a], path[i], path[b]));
    if (v_curve < p.curve_min_speed) v_curve = p.curve_min_speed;

    double d = cum[i];                            // 경로 시작(=ego)에서 그 지점까지
    double v_now = std::sqrt(v_curve * v_curve + 2.0 * p.brake_accel * d);
    if (v_now < limit) limit = v_now;
  }
  return limit;
}

// 목표속도 계산 (레퍼런스 SSAFY식).
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

}  // namespace acc
