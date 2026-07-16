// acc_planner : 종방향 목표속도 planning 노드 (ACC)
// ------------------------------------------------------------------
// 구독:  /local_path   (nav_msgs/Path)          기준 지역경로
//        /lattice_path (nav_msgs/Path)          lattice 회피경로 (있으면 우선)
//        /ego_status   (EgoVehicleStatus)        현재 위치·속도 [m/s]
//        /Object_topic (ObjectStatusList)        장애물 (속도 [km/h])
// 발행:  /target_velocity (std_msgs/Float64)     목표속도 [m/s]
//
// 동작: 추종경로(lattice 최근값 우선) 위 전방객체 탐색 → 레퍼런스 ACC식 → 60kph 캡.
//       계산은 순수 로직 acc_core.hpp가 담당, 이 노드는 구독/발행 배관만.
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

  // npc_list + obstacle_list → ObjIn (속도 km/h → m/s). 보행자는 제외(behavior FSM 몫).
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
