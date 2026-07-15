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
