// lattice_planner : 장애물 회피용 지역경로 생성기 (횡방향)
// ------------------------------------------------------------------
// 구독:  /local_path   (nav_msgs/Path)          기준 지역경로 (path_tracker가 발행)
//        /ego_status   (EgoVehicleStatus)        현재 위치·속도 (개발용, 나중 /odom)
//        /Object_topic (ObjectStatusList)        장애물 (perception, 지금은 mock)
// 발행:  /lattice_path       (nav_msgs/Path)     선택된 회피경로 -> 제어가 추종
//        /lattice_candidates (MarkerArray)       후보 전체 시각화 (초록=선택 빨강=충돌 회색=여유)
//
// 동작: 장애물이 경로 위에 있으면 좌우 후보경로 생성 -> 충돌검사 -> 최소비용 선택.
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
  const std::vector<double> LANE_OFFSET  = {-3.0, -1.75, -1.0, 1.0, 1.75, 3.0}; // 횡 후보 offset [m]
  const std::vector<double> BASE_WEIGHT  = {3, 2, 1, 1, 2, 3};                   // 중앙 선호(안쪽 저비용)
  const double CAR_HALF_WIDTH = 0.95;   // Ioniq5 폭 1.892 / 2
  const double SAFE_MARGIN    = 0.5;    // 안전 여유
  const double COLLISION_PENALTY = 100.0;
  const double X_INTERVAL = 0.5;        // 후보경로 점 간격 [m]

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

  // npc + 보행자 + 정적장애물 전부 모음 
  std::vector<Obs> gatherObstacles()
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
    add(objs_.pedestrian_list);
    add(objs_.obstacle_list);
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

  // 후보 경로 6개 생성 (기준경로 시작점 기준 local 프레임 -> 3차곡선 -> map 복귀)
  std::vector<nav_msgs::Path> generateCandidates()
  {
    std::vector<nav_msgs::Path> out;
    const int n = local_path_.poses.size();

    // /ego_status 의 velocity 는 MORAI UDP 원본이라 이미 km/h 다(브릿지가 변환 안 함).
    // 예전엔 이걸 m/s 로 착각하고 3.6을 또 곱해서 전방주시거리가 3.6배로 부풀어 있었다.
    double v_kmh = std::hypot(ego_.velocity.x, ego_.velocity.y);
    int look = static_cast<int>(v_kmh * 0.2 * 2);
    if (look < 20) look = 20;
    int end_idx = std::min(look * 2, n - 1);         // 경로 끝 IndexError 방지 (리뷰 #7)
    if (end_idx < 2) return out;

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
      // ※ 알려진 이슈: 이 tail 은 offset 을 유지하지 않고 기준경로 원본 점을 그대로
      //   붙이므로, S커브 끝(offset)과 tail(offset 0) 사이에 횡방향 불연속이 생긴다.
      //   짧은 회피에선 pure_pursuit 이 뭉개서 무해하다. docs/30-lattice_design.md §3.4 참고.
      const int TAIL_EXTEND = 12;
      for (int i = end_idx; i < n && i < end_idx + TAIL_EXTEND; ++i) {
        cand.poses.push_back(local_path_.poses[i]);
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
      for (const auto& p : cands[i].poses)
        for (const auto& o : obs) {
          double d = std::hypot(p.pose.position.x - o.x, p.pose.position.y - o.y);
          if (d < o.r + CAR_HALF_WIDTH + SAFE_MARGIN) {  // size 반영 충돌
            weight[i] += COLLISION_PENALTY;
            blocked[i] = true;
            break;
          }
        }
    }
    int best = std::min_element(weight.begin(), weight.end()) - weight.begin();

    // 전부 막힘 처리 (리뷰 #8): 최소도 충돌이면 경고 (정지는 behavior/ACC 몫)
    if (blocked[best])
      ROS_WARN_THROTTLE(1.0, "[lattice] 모든 후보 충돌 - behavior/ACC 정지 필요");
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

    std::vector<Obs> obs = gatherObstacles();

    // 장애물 없으면 기준경로 그대로 (lattice 안 돌림)
    if (obs.empty() || !objectOnPath(obs)) {
      pub_path_.publish(local_path_);
      return;
    }

    std::vector<nav_msgs::Path> cands = generateCandidates();
    if (cands.empty()) { pub_path_.publish(local_path_); return; }

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
