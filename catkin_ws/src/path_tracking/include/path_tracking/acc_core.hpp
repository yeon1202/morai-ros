// acc_core.hpp : ACC 순수 로직 (ROS 비의존, 단위=m/s로 통일)
#pragma once
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

namespace acc {

struct Vec2 { double x = 0.0; double y = 0.0; };

// 탐색 입력용 객체 (속도는 이미 m/s로 변환된 값)
//
// radius: 물체의 외접원 반경 [m]. lattice_planner 의 gatherObstacles 와 같은
//   방식(0.5*max(size.x,size.y))으로 채운다. 기본 0 이면 점으로 취급한다.
// 인지가 준 물체 하나.
//
// 크기는 두 가지로 줄 수 있다 (호출자가 아는 만큼만 채운다):
//   length/width/yaw : 방향 있는 상자. 이게 있으면 이쪽을 쓴다 (정확).
//   radius           : 원 근사. 크기를 모를 때의 보수적 폴백.
// 왜 상자가 필요한지는 아래 lateralHalfExtent 주석 참고.
struct ObjIn {
  Vec2   pos;
  double speed_mps = 0.0;
  double radius    = 0.0;   // [m] 폴백용 외접원 반경
  double length    = 0.0;   // [m] 주축(긴 쪽) 길이. 0 이면 radius 를 쓴다
  double width     = 0.0;   // [m] 부축 길이
  double yaw       = 0.0;   // [rad] 물체 방향 (경로와 같은 전역 프레임)
};

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
  double distance_threshold = 2.5;    // [m] (미사용 - car_half_width 로 대체됨. 아래 참고)
  double car_half_width     = 0.95;   // [m] 주행 통로 반폭. lattice_planner 의
                                      //     CAR_HALF_WIDTH 와 같은 값이어야 한다.
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

  // --- 목표속도 상승률 제한 (커브 탈출 사행 억제) ---
  //
  // curvatureSpeedLimit 은 /local_path(앞쪽)만 훑으므로 커브 정점을 지나는 순간
  // 제한이 그 프레임에 즉시 풀린다. 커브 진입에는 감속 프로파일이 있지만
  // 탈출에는 대응하는 가속 프로파일이 없어 목표속도가 한 스텝에 크루즈까지 열린다.
  // 그 급상승이 복귀 조향을 흔들어 S자 사행을 만든다.
  double accel_rate_limit   = 1.0;    // [m/s^2] 목표속도 상승률 한계
  double rate_limit_windup  = 2.0;    // [m/s] 실제속도보다 이만큼 이상 앞서지 않게
  double rate_dt_max        = 0.5;    // [s] 한 스텝으로 인정하는 dt 상한
};

// 경로 위로 투영한 결과 (Frenet 좌표)
struct Projection {
  double s = 0.0;         // 경로 시작부터의 종방향 거리 [m]
  double d = 0.0;         // 경로에서의 횡방향 거리 (부호 없음) [m]
  double yaw = 0.0;       // [rad] 그 지점 경로의 진행방향. 물체 방향과의 각도차를
                          //       내는 데 쓴다 (lateralHalfExtent 참고).
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
      best.yaw = std::atan2(path[i + 1].y - path[i].y,
                            path[i + 1].x - path[i].x);
      best.valid = true;
    }
    s_acc += seg_len;
  }
  return best;
}

// 물체가 "경로에 수직인 방향으로" 실제로 차지하는 반폭 [m].
//
// 왜 원 근사로는 안 되는가 (2026-09-03 실측, logs/percobj_percep1.csv)
//   가드레일 조각은 긴 쪽(3.32m)이 도로 진행방향인데, 원은 그 길이를 옆으로도
//   부풀린다. r = 0.5*max(3.32, 0.47) = 1.66 이 되어, 경로에서 2.51m 떨어진
//   물체가 통로 안(2.51 - 1.66 = 0.84 < 0.95)으로 판정됐다. 그 결과 ACC 가
//   앞차로 잡아 목표속도를 0 으로 내렸고, 차가 경로 627.9m 지점에 멈춰
//   끝까지 가지 못했다. 실제 폭은 0.47m 라 여유가 1.33m 있었다.
//
//   원은 "도로와 나란한 길쭉이" 와 "길을 가로막은 상자" 를 구분하지 못해서
//   둘 중 한쪽을 반드시 틀린다. 임계값을 조정해도 안 풀린다 - 두 경우가
//   같은 숫자를 만들기 때문이다. 각도를 쓰면 그 둘이 갈린다:
//
//     theta = 물체 방향 - 그 지점 경로 방향
//     반폭  = |sin theta| * (length/2) + |cos theta| * (width/2)
//
//     theta ~ 0  (도로와 나란)  -> 반폭 = width/2   -> 안 막는다 (가드레일)
//     theta ~ 90 (길을 가로막음) -> 반폭 = length/2  -> 막는다   (회피는 살아있다)
//
// |sin|·|cos| 이라 180도 주기가 저절로 처리된다 (앞뒤 구분이 없는 값이라 맞다).
inline double lateralHalfExtent(double length, double width, double theta_rad) {
  return std::fabs(std::sin(theta_rad)) * 0.5 * length +
         std::fabs(std::cos(theta_rad)) * 0.5 * width;
}

// 물체 하나의 횡 반폭. 크기를 모르면(length/width 가 0) radius 로 폴백한다.
//   ⚠️ lattice_planner.cpp 도 이 함수를 쓴다. 두 모듈이 같은 물체를 다르게
//      판정하면 2026-09-02 같은 일이 생긴다(lattice 는 통과시키는데 ACC 만 정지).
inline double halfExtentOf(const ObjIn& o, double path_yaw) {
  if (o.length <= 0.0 && o.width <= 0.0) return o.radius;
  return lateralHalfExtent(o.length, o.width, o.yaw - path_yaw);
}

// km/h 속도벡터 → m/s 스칼라 (단위 함정 처리)
inline double speedKmhToMps(double vx_kmh, double vy_kmh) {
  return std::hypot(vx_kmh, vy_kmh) / 3.6;
}

// 기준경로 위의 전방 객체 중 ego에 가장 가까운 것을 lead로 선택.
//   - 경로 위 판정 : |d| - radius < car_half_width   (횡방향, 통로와 겹치는가)
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
    // "내 주행 통로와 실제로 겹치는가" 로 판정한다.
    //
    // 예전엔 중심거리만 봤다: |d| < distance_threshold(2.5m).
    // 2026-09-02 실측에서 이게 물렸다 - 경로에서 횡 2.1~2.3m 인 도로변 가로등이
    // 전부 "앞차" 로 잡혀, 9.635m 안에 들어가는 순간 목표속도가 0 이 되고 차가
    // 길 한복판에 섰다. 같은 물체를 lattice 는 통과시켰다(임계 1.75m). 두 모듈이
    // 같은 물체를 다르게 판정한 것이 문제였다.
    //
    // 물체 가장자리까지의 거리(|d| - 반경)를 차 반폭과 비교하면 lattice 와 같은
    // 질문이 된다. 위 가로등은 2.18 - 0.30 = 1.88 > 0.95 라 앞차가 아니다.
    //
    // ⚠️ 호출자가 radius 를 안 채우면 점으로 취급된다. 그러면 차선 안에서 조금
    //    치우친 진짜 앞차를 놓칠 수 있다. acc_planner 는 반드시 크기를 넘길 것.
    if (op.d - halfExtentOf(o, op.yaw) >= p.car_half_width) continue;   // 통로 밖


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

// 목표속도의 "상승"만 제한한다. 감속은 그대로 통과시킨다.
//
// 왜 필요한가: curvatureSpeedLimit 은 앞쪽 경로만 보므로 커브를 빠져나오는 순간
// 제한이 즉시 풀린다. 목표속도가 한 스텝에 11 m/s 씩 뛰면 복귀 조향이 수렴하기
// 전에 속도가 붙어 사행이 커진다. 목표를 천천히 올리는 것 자체가 "커브가 아직
// 안 끝났다"는 보수적 판정 역할을 한다.
//
// 안전 불변식: 반환값은 어떤 경로로도 desired 를 넘지 않는다. 이 함수는 목표를
// 늦출 뿐 높이지 않는다. 규정 상한(60kph)과 곡률 제한은 이미 desired 에 반영되어
// 있으므로, 이 불변식이 지켜지는 한 이 함수가 제한을 우회할 수 없다.
inline double rampTarget(double prev, double desired, double ego_vel,
                         double dt, const AccParams& p) {
  if (dt <= 0.0) return desired;               // 제한 로직이 차를 잠그지 않게
  if (dt > p.rate_dt_max) dt = p.rate_dt_max;  // 프레임 끊김 시 한 번에 뛰지 않게

  // 목표가 실제 속도보다 크게 앞서 있으면 끌어내린다.
  // 차가 목표를 못 따라가는 동안(오르막, 제동 직후) 목표만 혼자 달아나면,
  // 회복되는 순간 그 격차만큼 급가속한다. 제한이 걸린 것처럼 보이지만 무력화된 상태다.
  double anchored = std::min(prev, ego_vel + p.rate_limit_windup);

  if (desired <= anchored) return desired;     // 감속은 제한하지 않는다
  return std::min(desired, anchored + p.accel_rate_limit * dt);
}

}  // namespace acc
