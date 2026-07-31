// lattice_planner : 장애물 회피용 지역경로 생성기 (횡방향)
// ------------------------------------------------------------------
// 구독:  /local_path   (nav_msgs/Path)          기준 지역경로 (path_tracker가 발행)
//        /ego_status   (EgoVehicleStatus)        현재 위치·속도 (개발용, 나중 /odom)
//        /Object_topic (ObjectStatusList)        장애물 (perception, 지금은 mock)
// 발행:  /lattice_path       (nav_msgs/Path)     선택된 회피경로 -> 제어가 추종
//        /lattice_candidates (MarkerArray)       후보 전체 시각화 (초록=선택 빨강=충돌 회색=여유)
//
// 동작: 장애물이 경로 위에 있으면 차선 단위 후보경로 생성 -> 충돌검사 -> 최소비용 선택.
//       없으면 기준경로 그대로 통과.
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <morai_msgs/ObjectStatus.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cmath>
#include <algorithm>

// 차로 폭 [m]. 2026-07-31 시뮬 실측값이다.
//   직선구간에서 차선 두 줄의 좌표 (72.85,-55.36) / (69.34,-55.44) 를 찍어
//   도로 수직 성분을 계산했다. 그 지점 곡률반경이 39928m(사실상 완전 직선)라
//   baseline 을 어떻게 잡아도 3.510m 로 동일했다(편차 0.001m). 종방향으로
//   어긋난 양도 0.060m 뿐이라 거의 순수 횡이동이었다.
//   한국 고속도로 표준 3.5m 와도 맞는다.
//
// 클래스 멤버가 아니라 파일 스코프에 두는 이유는 acc_planner.cpp 의 kTimerHz 와
// 같다. 이 패키지는 C++ 표준을 명시하지 않아 gnu++14 로 빌드되는데, 클래스 안의
// static constexpr 멤버는 ODR-use 시 클래스 밖 정의가 따로 필요할 수 있다.
static constexpr double LANE_WIDTH = 3.51;

class LatticePlanner
{
public:
  LatticePlanner()
  {
    ros::NodeHandle nh;
    sub_path_ = nh.subscribe("/local_path", 1, &LatticePlanner::pathCb, this);
    sub_ego_  = nh.subscribe("/ego_status", 1, &LatticePlanner::egoCb, this);
    sub_obj_  = nh.subscribe("/Object_topic", 1, &LatticePlanner::objCb, this);
    pub_path_ = nh.advertise<nav_msgs::Path>("/lattice_path", 1);
    pub_cand_ = nh.advertise<visualization_msgs::MarkerArray>("/lattice_candidates", 1);
    timer_ = nh.createTimer(ros::Duration(1.0 / 30.0), &LatticePlanner::run, this);
    ROS_INFO("[lattice_planner] started");
  }

private:
  // ---- 파라미터 ----
  // ---- 횡 후보 offset [m] ----
  //
  // 편도 2차선 도로에서 선택지는 "그대로" 아니면 "옆 차선" 둘뿐이다. 그래서
  // 연속 오프셋을 샘플링하지 않고 차선 단위로 이산화한다.
  //
  // 예전에는 {-3.0, -1.75, -1.0, 1.0, 1.75, 3.0} 에 가중치 {3,2,1,1,2,3} 을 썼는데
  // 세 가지가 잘못이었다.
  //
  //   1) 0 후보가 없었다. objectOnPath() 가 참이 되기만 하면 무조건 1m 이상
  //      움직였다. 장애물이 경로 옆에 살짝 걸쳐 그냥 차선을 유지해도 안전한
  //      경우에도 이유 없이 옆으로 나갔다.
  //
  //   2) +-1.0 과 +-1.75 는 어떤 장애물도 피하지 못한다. 충돌 판정 임계가
  //      장애물반경 + CAR_HALF_WIDTH(0.95) + SAFE_MARGIN(0.5) = 반경 + 1.45m 라,
  //      1.75m 를 비켜도 반경 0.3m 짜리 라바콘조차 못 넘긴다. 회피 능력이 0인
  //      후보였다.
  //
  //   3) 그런데 그 쓸모없는 후보들이 가장 쌌다(가중치 1, 2). 차로 3.51m 에
  //      차폭 1.892m 라 차선 내 여유가 편도 (3.51-1.892)/2 = 0.809m 뿐이어서
  //      +-1.0 은 0.19m, +-1.75 는 0.94m 를 침범한다. 대회 규정상 실선 침범은
  //      3초당 5초 페널티다. 아무것도 못 피하면서 페널티만 먹는 셈이었다.
  //
  // 유일하게 의미 있는 회피는 옆 차선으로 통째로 옮기는 것이고, 그 값이
  // LANE_WIDTH 다. 예전의 +-3.0 은 방향은 맞았지만 옆 차선 중심에서 0.51m
  // 어긋나 있었다.
  //
  // 부호: local 프레임이 x=진행방향, y=좌측(+) 이므로 음수 offset 이 우측이다.
  //
  // 실제 회피 미션은 교차로 한복판을 막은 상자다. 교차로 안에는 차선이 없으므로
  // (정지선에서 끊기고 건너편에서 다시 시작한다) 차로 단위로 움직일 이유가 없다.
  // 필요한 것은 상자를 비켜갈 만큼이고, 그 값은
  //   상자 절반폭 + CAR_HALF_WIDTH(0.95) + SAFE_MARGIN(0.5)
  // 이라 상자가 1~2m 폭이면 1.95~2.45m 다. 그래서 중간 오프셋을 촘촘히 두고
  // 비용을 |offset| 에 비례시켜 "충돌을 면하는 가장 작은 회피" 가 뽑히게 한다.
  //
  // 3차선 이상 구간이 확인되면 -2*LANE_WIDTH 를 추가한다.
  const std::vector<double> LANE_OFFSET  = {0.0, -1.0, -2.0, -3.0, -LANE_WIDTH, LANE_WIDTH};

  // 비용.
  //
  // 제자리(0)를 압도적으로 싸게 둔다. 옆 차로가 비어 있어도 굳이 나가지 않고,
  // 원래 차로가 막혔을 때만 움직인다.
  //
  // 좌우가 대칭이 아니다. 정적장애물 구간과 보행자 구간 모두 자차는 1차로
  // (맨 왼쪽 차로)를 달리고 왼쪽은 황색 중앙선이다. 왼쪽으로 피하면 중앙선
  // 침범이고 반대편 차로로 들어가는 것이라 사고 위험도 크다. 오른쪽에는
  // 2차로가 있으므로 회피는 오른쪽으로 간다.
  //
  // 왼쪽을 아예 빼지 않고 큰 비용(20)으로 남겨둔 이유는, 오른쪽까지 막혔을 때
  // 마지막 수단으로라도 쓸 수 있게 하기 위해서다. 충돌 벌점이 100 이므로
  // "오른쪽이 전부 막혔고 왼쪽은 뚫렸다" 일 때만 왼쪽이 선택된다.
  //
  // 우측 비용을 |offset| 에 비례시켜 충돌을 면하는 가장 작은 회피가 뽑히게 한다.
  //                                      0    -1.0  -2.0  -3.0  -3.51  +3.51
  const std::vector<double> BASE_WEIGHT  = {0.0, 1.0, 2.0, 3.0,  4.0,   20.0};
  const double CAR_HALF_WIDTH = 0.95;   // Ioniq5 폭 1.892 / 2
  const double SAFE_MARGIN    = 0.5;    // 안전 여유
  const double COLLISION_PENALTY = 100.0;
  const double X_INTERVAL = 0.5;        // 후보경로 점 간격 [m]

  // 차선 변경 중 허용할 횡가속도 [m/s^2]. 0.3G.
  // ACC 의 곡률 기반 속도 제한(lat_accel_limit)과 같은 값이다. 타이어 한계는
  // 마른 아스팔트에서 약 1G 이지만, 그 3분의 1로 승차감·안전 여유를 둔다.
  const double LAT_ACCEL_LIMIT = 2.94;

  // 정지·극저속에서의 전이 길이 하한 [m].
  // 속도 0 이면 v*T 가 0 이 되어 후보가 생기지 않는다. 회피가 아예 작동하지
  // 않는 것을 막는 최소값이다. 예전 하한(24m)과 달리 주행 영역을 침범하지 않는다
  // (8m 를 넘는 시점이 약 11km/h 이므로 그 위로는 전부 시간 기준이 지배한다).
  const double MIN_TRANSITION = 8.0;

  ros::Subscriber sub_path_, sub_ego_, sub_obj_;
  ros::Publisher pub_path_, pub_cand_;
  ros::Timer timer_;
  nav_msgs::Path local_path_;
  morai_msgs::EgoVehicleStatus ego_;
  morai_msgs::ObjectStatusList objs_;
  bool has_path_ = false, has_ego_ = false, has_obj_ = false;

  void pathCb(const nav_msgs::Path::ConstPtr& m) { local_path_ = *m; has_path_ = true; }
  void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& m) { ego_ = *m; has_ego_ = true; }
  void objCb(const morai_msgs::ObjectStatusList::ConstPtr& m) { objs_ = *m; has_obj_ = true; }

  // 장애물 하나 = 위치 + 반경(size 반영)
  struct Obs { double x, y, r; };

  // 장애물 수집.
  //
  // with_pedestrian 을 나눈 이유:
  //   보행자는 "피하는" 대상이 아니라 "서는" 대상이다. 미션이 횡단보도로
  //   신호를 위반하며 뛰어드는 사람이라, 옆으로 꺾으면 사람이 뛰어든 쪽으로
  //   들어갈 수도 있고 반대 차로로 나갈 수도 있다. 정답은 급정지이고 그건
  //   behavior FSM / ACC 몫이다.
  //   그래서 보행자는 회피를 촉발하지 않는다(objectOnPath 에서 제외).
  //   다만 다른 이유로 이미 회피 중이라면 보행자를 뚫고 가는 후보를 고르면
  //   안 되므로, 후보 충돌 검사에는 포함한다.
  std::vector<Obs> gatherObstacles(bool with_pedestrian)
  {
    std::vector<Obs> v;
    auto add = [&](const std::vector<morai_msgs::ObjectStatus>& list) {
      for (const auto& o : list) {
        double r = 0.5 * std::max(o.size.x, o.size.y);   // size 반영 (리뷰 #4)
        if (r < 0.3) r = 0.3;
        v.push_back({o.position.x, o.position.y, r});
      }
    };
    add(objs_.npc_list);
    add(objs_.obstacle_list);
    if (with_pedestrian) add(objs_.pedestrian_list);
    return v;
  }

  // 기준경로 위에 장애물이 있나? (회피 트리거)
  bool objectOnPath(const std::vector<Obs>& obs)
  {
    for (const auto& p : local_path_.poses)
      for (const auto& o : obs) {
        double d = std::hypot(p.pose.position.x - o.x, p.pose.position.y - o.y);
        if (d < o.r + CAR_HALF_WIDTH + SAFE_MARGIN) return true;
      }
    return false;
  }

  // 후보 경로 생성 (기준경로 시작점 기준 local 프레임 -> 3차곡선 -> map 복귀)
  // offset 0 후보도 의미가 있다. 3차곡선이 차의 현재 횡위치(egoy)에서 기준경로로
  // 부드럽게 복귀시키므로, "그대로 간다" 가 곧 "제 차선으로 돌아온다" 가 된다.
  std::vector<nav_msgs::Path> generateCandidates()
  {
    std::vector<nav_msgs::Path> out;
    const int n = local_path_.poses.size();

    // /ego_status 의 velocity 는 MORAI UDP 원본이라 이미 km/h 다(브릿지가 변환 안 함).
    // 예전엔 이걸 m/s 로 착각하고 3.6을 또 곱해서 전방주시거리가 3.6배로 부풀어 있었다.
    double v_kmh = std::hypot(ego_.velocity.x, ego_.velocity.y);
    double v_mps = v_kmh / 3.6;

    // 전이 길이(옆으로 옮기는 데 앞으로 쓰는 거리)는 "거리" 가 아니라 "시간" 으로 정한다.
    //
    // 예전에는 waypoint 개수로 정했다:
    //     look = v_kmh*0.4;  if (look < 20) look = 20;  end_idx = look*2;
    // 하한 20 이 50km/h 이하를 전부 덮어써서, 주행의 절반이 40점(약 24m)에 고정됐다.
    // 게다가 개수라서 waypoint 간격이 바뀌면 거동이 조용히 달라졌다.
    //
    // 거리를 고정하면 시간이 속도에 따라 널뛴다. 24m 를 20km/h 로 지나면 4.3초,
    // 55km/h 로 지나면 1.6초다. 그만큼 횡가속도도 0.12G ~ 0.72G 로 널뛰었다.
    //   - 저속: 너무 느긋해 15m 앞 상자에 닿았을 때 2.40m 밖에 못 비켜났다(2.45m 필요).
    //   - 고속: 0.72G 를 요구했다. 우리 한계 0.3G 의 2.4배라 차가 못 따라가고 밀려난다.
    //
    // 3차곡선의 최대 곡률은 6*D/L^2 이고 횡가속도는 v^2 를 곱한 값이다. 이를
    // LAT_ACCEL_LIMIT 이하로 두면 L = v * sqrt(6*D/a) 가 되어 속도가 상쇄되고,
    // 남는 것은 "시간" 뿐이다. 그래서 속도와 무관하게 횡가속도가 일정해진다.
    //   D=3.51m, a=2.94 -> T = 2.68초 (20km/h 에서 15m, 55km/h 에서 41m)
    double d_max = 0.0;
    for (double off : LANE_OFFSET) d_max = std::max(d_max, std::fabs(off));
    const double T = std::sqrt(6.0 * d_max / LAT_ACCEL_LIMIT);
    double xf_want = std::max(v_mps * T, MIN_TRANSITION);

    // 누적 거리로 끝점 인덱스를 찾는다. 개수가 아니라 거리로 재므로 waypoint
    // 간격이 달라져도 거동이 바뀌지 않는다.
    int end_idx = 1;
    double acc_len = 0.0;
    for (int i = 1; i < n; ++i) {
      acc_len += std::hypot(
          local_path_.poses[i].pose.position.x - local_path_.poses[i-1].pose.position.x,
          local_path_.poses[i].pose.position.y - local_path_.poses[i-1].pose.position.y);
      end_idx = i;
      if (acc_len >= xf_want) break;
    }
    if (end_idx < 2) return out;                     // 경로가 너무 짧다

    // 좌표변환: 시작점 + 진행방향 theta
    double sx = local_path_.poses[0].pose.position.x;
    double sy = local_path_.poses[0].pose.position.y;
    double nx = local_path_.poses[1].pose.position.x;
    double ny = local_path_.poses[1].pose.position.y;
    double theta = std::atan2(ny - sy, nx - sx);
    double c = std::cos(theta), s = std::sin(theta);

    // world -> local (역변환)
    auto toLocal = [&](double x, double y, double& lx, double& ly) {
      double dx = x - sx, dy = y - sy;
      lx =  c * dx + s * dy;
      ly = -s * dx + c * dy;
    };
    // local -> world
    auto toWorld = [&](double lx, double ly, double& x, double& y) {
      x = sx + c * lx - s * ly;
      y = sy + s * lx + c * ly;
    };

    double ex, ey; toLocal(local_path_.poses[end_idx].pose.position.x,
                           local_path_.poses[end_idx].pose.position.y, ex, ey);
    double egox, egoy; toLocal(ego_.position.x, ego_.position.y, egox, egoy);

    for (double off : LANE_OFFSET) {
      nav_msgs::Path cand;
      cand.header.frame_id = "map";
      double xf = ex;
      double ps = egoy;          // 시작 횡위치 (현재 차)
      double pf = ey + off;      // 끝 횡위치 (offset)
      if (xf < 1.0) { out.push_back(cand); continue; }

      // 3차곡선: y(0)=ps, y'(0)=0, y(xf)=pf, y'(xf)=0
      double a0 = ps, a1 = 0.0;
      double a2 = 3.0 * (pf - ps) / (xf * xf);
      double a3 = -2.0 * (pf - ps) / (xf * xf * xf);

      for (double x = 0.0; x < xf; x += X_INTERVAL) {
        double y = a3 * x * x * x + a2 * x * x + a1 * x + a0;
        double wx, wy; toWorld(x, y, wx, wy);
        geometry_msgs::PoseStamped ps_msg;
        ps_msg.header.frame_id = "map";
        ps_msg.pose.position.x = wx;
        ps_msg.pose.position.y = wy;
        ps_msg.pose.orientation.w = 1.0;
        cand.poses.push_back(ps_msg);
      }
      // 후보 뒤쪽을 기준경로 따라 조금만 연장.
      //
      // TAIL_EXTEND 로 길이를 제한하는 이유:
      //   이 tail 은 모든 후보가 똑같이 공유하는 기준경로다. 즉 후보를 고르는 데는
      //   아무 판별력이 없는데, selectLane 의 충돌검사는 후보 전체를 훑기 때문에
      //   tail 에 장애물이 하나라도 걸리면 6개 후보가 전부 동시에 막힌다.
      //   /local_path 를 ACC 용으로 84m 까지 늘렸으므로, 제한하지 않으면 70m 앞
      //   장애물 때문에 회피 판단이 마비된다. 먼 장애물 대응은 ACC/behavior 몫.
      //   여기 남기는 길이는 pure_pursuit 이 전방주시점(최대 20m)을 찾을 만큼이면 된다.
      //
      // tail 도 offset 을 유지한다(경로 법선 방향으로 밀어서 붙인다).
      //
      //   예전에는 기준경로 원본 점을 그대로 붙여서, S커브 끝(offset)과
      //   tail(offset 0) 사이에 횡방향 불연속이 생겼다. 전이 길이가 24m 로 길던
      //   시절에는 tail 이 장애물보다 뒤쪽에서 시작해 무해했지만, 전이 길이를
      //   속도 비례로 바꾸면서 저속에서 15m 로 짧아졌다. 그러면 tail 이 장애물
      //   바로 위에서 시작해, 애써 비켜난 경로가 그 자리에서 원래 차선으로
      //   되돌아가 버린다. 즉 회피가 무효가 된다.
      const int TAIL_EXTEND = 12;
      for (int i = end_idx; i < n && i < end_idx + TAIL_EXTEND; ++i) {
        // 경로 접선 -> 좌측 법선. 법선 방향으로 off 만큼 민다.
        int a = (i > 0) ? i - 1 : i;
        int b = (i + 1 < n) ? i + 1 : i;
        double tx = local_path_.poses[b].pose.position.x - local_path_.poses[a].pose.position.x;
        double ty = local_path_.poses[b].pose.position.y - local_path_.poses[a].pose.position.y;
        double tl = std::hypot(tx, ty);
        geometry_msgs::PoseStamped tp = local_path_.poses[i];
        if (tl > 1e-9) {
          tp.pose.position.x += (-ty / tl) * off;
          tp.pose.position.y += ( tx / tl) * off;
        }
        cand.poses.push_back(tp);
      }
      out.push_back(cand);
    }
    return out;
  }

  // 충돌검사 -> 각 후보 비용 계산 -> 최소비용 선택. blocked[]도 채움.
  int selectLane(const std::vector<nav_msgs::Path>& cands,
                 const std::vector<Obs>& obs, std::vector<bool>& blocked)
  {
    std::vector<double> weight = BASE_WEIGHT;
    blocked.assign(cands.size(), false);

    for (size_t i = 0; i < cands.size(); ++i) {
      // 벌점은 후보당 한 번만 준다.
      //
      // 예전에는 break 가 안쪽(장애물) 루프만 끊고 바깥(점) 루프는 계속 돌아서,
      // 충돌한 waypoint 개수만큼 +100 이 누적됐다. 장애물 하나인데 가중치가
      // 900, 800, 600 씩 붙었다. 그러면 "충돌하느냐" 가 아니라 "몇 점이나
      // 충돌하느냐" 로 순위가 매겨지고, 중앙선 회피 같은 기본 비용(20)이
      // 수백 점의 벌점 앞에서 무의미해진다.
      bool hit = false;
      for (const auto& p : cands[i].poses) {
        for (const auto& o : obs) {
          double d = std::hypot(p.pose.position.x - o.x, p.pose.position.y - o.y);
          if (d < o.r + CAR_HALF_WIDTH + SAFE_MARGIN) {  // size 반영 충돌
            hit = true;
            break;
          }
        }
        if (hit) break;
      }
      if (hit) {
        weight[i] += COLLISION_PENALTY;
        blocked[i] = true;
      }
    }
    int best = std::min_element(weight.begin(), weight.end()) - weight.begin();

    // 전부 막힘 처리 (리뷰 #8): 최소도 충돌이면 경고 (정지는 behavior/ACC 몫)
    if (blocked[best])
      ROS_WARN_THROTTLE(1.0, "[lattice] all candidates blocked - behavior/ACC must stop");
    return best;
  }

  void publishCandidates(const std::vector<nav_msgs::Path>& cands, int best,
                         const std::vector<bool>& blocked)
  {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del; del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);
    for (size_t i = 0; i < cands.size(); ++i) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "lattice"; m.id = i;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = (static_cast<int>(i) == best) ? 0.3 : 0.1;
      m.color.a = 0.9;
      if (static_cast<int>(i) == best) { m.color.r = 0.1; m.color.g = 1.0; m.color.b = 0.1; }  // 초록=선택
      else if (blocked[i])            { m.color.r = 1.0; m.color.g = 0.1; m.color.b = 0.1; }  // 빨강=충돌
      else                            { m.color.r = 0.6; m.color.g = 0.6; m.color.b = 0.6; }  // 회색=여유
      for (const auto& p : cands[i].poses) {
        geometry_msgs::Point pt;
        pt.x = p.pose.position.x; pt.y = p.pose.position.y; pt.z = 0.2;
        m.points.push_back(pt);
      }
      arr.markers.push_back(m);
    }
    pub_cand_.publish(arr);
  }

  void run(const ros::TimerEvent&)
  {
    if (!(has_path_ && has_ego_ && has_obj_)) return;

    // 회피를 촉발할 대상: NPC + 정적장애물 (보행자 제외 - 보행자는 정지 대상)
    std::vector<Obs> trigger_obs = gatherObstacles(false);

    // 회피할 이유가 없으면 기준경로 그대로 (lattice 안 돌림)
    if (trigger_obs.empty() || !objectOnPath(trigger_obs)) {
      pub_path_.publish(local_path_);
      return;
    }

    std::vector<nav_msgs::Path> cands = generateCandidates();
    if (cands.empty()) { pub_path_.publish(local_path_); return; }

    // 후보 충돌 검사에는 보행자도 포함한다. 이미 회피 중이라면 보행자를 뚫고
    // 가는 후보를 고르면 안 된다.
    std::vector<Obs> obs = gatherObstacles(true);
    std::vector<bool> blocked;
    int best = selectLane(cands, obs, blocked);

    pub_path_.publish(cands[best]);          // 선택 경로 -> 제어
    publishCandidates(cands, best, blocked); // 후보 전체 -> RViz
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lattice_planner");
  LatticePlanner lp;
  ros::spin();
  return 0;
}
