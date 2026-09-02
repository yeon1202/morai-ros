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
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <morai_msgs/ObjectStatus.h>
#include <string>
#include <algorithm>   // std::max (radius 계산)

#include "path_tracking/acc_core.hpp"

// 타이머 주기. run() 에서 첫 틱의 dt 로도 쓰므로 상수로 둔다.
// 클래스 멤버가 아니라 파일 스코프에 두는 이유: 이 패키지는 C++ 표준을 명시하지
// 않아 컴파일러 기본값(gnu++14)을 쓰는데, C++14 에서는 클래스 안의 static
// constexpr 멤버가 ODR-use 되면 클래스 밖 정의가 따로 필요해 undefined reference
// 가 날 수 있다. 파일 스코프 상수는 그 문제가 없다.
static constexpr double kTimerHz = 30.0;

class AccPlanner
{
public:
  AccPlanner()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // 파라미터 (기본값은 acc::AccParams). cruise/max는 km/h로 받아 내부 m/s 변환.
    //
    // 크루즈는 55 로 고정한다. 규정 상한은 60 이지만 target 60 으로 달렸을 때
    // 실측이 60.1~60.2 km/h 로 넘어갔다. 규정상 60 초과는 15초 + 3초당 15초
    // 패널티라 오버슈트 여유를 둔다. max(하드캡)는 규정값 그대로 60 이다.
    //
    // acc.launch 의 cruise_speed_kmh 기본값과 반드시 같아야 한다. 예전에는 여기가
    // 60, launch 가 55 여서 rosrun 으로 띄우면 60, roslaunch 로 띄우면 55 로
    // 조용히 달라졌다. 기준값 비교 주행에서 조건이 어긋나는 원인이 된다.
    double cruise_kmh = 55.0, max_kmh = 60.0;
    pnh.param("time_gap",           params_.time_gap,           params_.time_gap);
    pnh.param("default_space",      params_.default_space,      params_.default_space);
    pnh.param("vehicle_length",     params_.vehicle_length,     params_.vehicle_length);
    pnh.param("distance_threshold", params_.distance_threshold, params_.distance_threshold);
    pnh.param("lookahead",          params_.lookahead,          params_.lookahead);
    pnh.param("lat_accel_limit",    params_.lat_accel_limit,    params_.lat_accel_limit);
    pnh.param("brake_accel",        params_.brake_accel,        params_.brake_accel);
    pnh.param("curve_baseline",     params_.curve_baseline,     params_.curve_baseline);
    pnh.param("curve_min_speed",    params_.curve_min_speed,    params_.curve_min_speed);
    pnh.param("accel_rate_limit",   params_.accel_rate_limit,   params_.accel_rate_limit);
    pnh.param("rate_limit_windup",  params_.rate_limit_windup,  params_.rate_limit_windup);
    pnh.param("rate_dt_max",        params_.rate_dt_max,        params_.rate_dt_max);
    pnh.param("velocity_gain",      params_.velocity_gain,      params_.velocity_gain);
    pnh.param("distance_gain",      params_.distance_gain,      params_.distance_gain);
    pnh.param("cruise_speed_kmh",   cruise_kmh,                 cruise_kmh);
    pnh.param("max_speed_kmh",      max_kmh,                    max_kmh);
    params_.cruise_speed = cruise_kmh / 3.6;
    params_.max_speed    = max_kmh / 3.6;

    sub_local_   = nh.subscribe("/local_path",   1, &AccPlanner::localCb,   this);
    sub_lattice_ = nh.subscribe("/lattice_path", 1, &AccPlanner::latticeCb, this);
    sub_ego_     = nh.subscribe("/ego_status",   1, &AccPlanner::egoCb,     this);
    sub_odom_ = nh.subscribe("/odom", 1, &AccPlanner::odomCb, this);
    sub_obj_     = nh.subscribe("/Object_topic", 1, &AccPlanner::objCb,     this);
    // 발행 토픽. behavior_fsm 이 종방향 단일 권한을 가지므로 기본값은 제약 토픽이다.
    //
    // acc_planner 는 이제 "크루즈·곡률·앞차를 고려하면 이 속도까지" 라는 **제약**만
    // 낸다. 최종 목표속도는 behavior_fsm 이 다른 제약들과 min() 해서 정한다.
    // behavior_fsm 없이 단독으로 돌려보려면 이 값을 /target_velocity 로 바꾼다.
    std::string out_topic = "/speed_limit/acc";
    pnh.param<std::string>("output_topic", out_topic, out_topic);
    pub_vel_     = nh.advertise<std_msgs::Float64>(out_topic, 1);
    timer_       = nh.createTimer(ros::Duration(1.0 / kTimerHz), &AccPlanner::run, this);
    // ROS_INFO 포맷 문자열에는 한글을 쓰지 않는다. 컨테이너 로케일이 UTF-8 이
    // 아니라 로그 출력에서 '?' 로 깨진다(주석의 한글은 컴파일러가 처리하므로 무관).
    ROS_INFO("[acc_planner] started (cruise=%.1f km/h, cap=%.1f km/h, rate_limit=%.2f m/s^2)",
             cruise_kmh, max_kmh, params_.accel_rate_limit);
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
  nav_msgs::Odometry odom_;
  bool has_odom_ = false;
  ros::Subscriber sub_odom_;


  void localCb(const nav_msgs::Path::ConstPtr& m)   { local_path_ = *m; has_local_ = true; }
  // 속도만 쓴다. 위치는 odomCb 에서 받는다 - 대회 규정 채널(9109)은 position 을
  // 0,0,0 으로 주기 때문이다 (2026-08-29 전환).
  void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& m) { ego_ = *m; has_ego_ = true; }

  // 위치는 /odom(GPS+IMU 융합)에서 받는다.
  // ※ twist 는 쓰지 않는다. wheel_speed_scaler 가 시뮬 배속을 곱해 벽시계 단위로
  //   바꿔놓은 값이라 실제 주행속도가 아니다. 속도는 계속 /ego_status 에서 받는다.
  void odomCb(const nav_msgs::Odometry::ConstPtr& m) { odom_ = *m; has_odom_ = true; }
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
  //
  // radius 는 lattice_planner 의 gatherObstacles 와 "같은 식" 이어야 한다.
  // 두 모듈이 같은 물체를 다르게 판정하면 2026-09-02 같은 일이 생긴다 - lattice 는
  // 통과시키는 도로변 가로등을 ACC 만 앞차로 잡아 차가 길 한복판에 섰다.
  //   ⚠️ 여기를 고치면 lattice_planner.cpp 의 gatherObstacles 도 같이 봐야 한다.
  std::vector<acc::ObjIn> gatherObjects()
  {
    std::vector<acc::ObjIn> v;
    auto add = [&](const std::vector<morai_msgs::ObjectStatus>& list) {
      for (const auto& o : list) {
        double r = 0.5 * std::max(o.size.x, o.size.y);
        if (r < 0.3) r = 0.3;                       // 인지가 아주 작게 주는 경우 대비
        v.push_back({ {o.position.x, o.position.y},
                      acc::speedKmhToMps(o.velocity.x, o.velocity.y),
                      r });
      }
    };
    add(objs_.npc_list);
    add(objs_.obstacle_list);
    return v;
  }

  void run(const ros::TimerEvent&)
  {
    // 경로와 자차 상태는 없으면 계산 자체가 불가능하므로 발행 보류.
    //
    // 반면 /Object_topic 은 기다리지 않는다. "장애물이 하나도 없음" 은 정상 상태이지
    // 데이터 없음이 아니다. 예전엔 has_obj_ 까지 요구해서, perception(또는 mock)이
    // 안 떠 있으면 ACC 가 아무것도 발행하지 않았다 - 대회에서 perception 이 늦게
    // 뜨면 그동안 종방향 제어가 통째로 비는 셈이다. 객체가 없으면 크루즈 + 곡률
    // 제한만으로 목표속도를 낸다.
    if (!(has_local_ && has_ego_ && has_odom_)) return;

    std::vector<acc::Vec2> path = followPath();
    if (path.empty()) return;

    acc::Vec2 ego{odom_.pose.pose.position.x, odom_.pose.pose.position.y};
    // /ego_status 의 velocity 는 UDP 원본 그대로라 단위가 km/h 다(브릿지가 변환 안 함).
    // 객체와 똑같이 m/s 로 바꿔야 한다. 안 그러면 실제보다 3.6배 빠른 줄 알고 계산한다.
    double ego_vel = acc::speedKmhToMps(ego_.velocity.x, ego_.velocity.y);  // m/s

    std::vector<acc::ObjIn> objs = gatherObjects();
    acc::Lead lead = acc::selectLead(path, ego, objs, params_);
    double target = acc::computeTargetVelocity(ego_vel, lead, params_);

    // 곡률 제한을 함께 반영한다. /target_velocity 가 종방향의 단일 권한이므로
    // 크루즈·앞차추종·곡률(나중엔 behavior 까지)을 여기서 모두 합쳐 최종값 하나로 낸다.
    double curve_limit = acc::curvatureSpeedLimit(path, params_);
    if (curve_limit < target) {
      ROS_INFO_THROTTLE(2.0, "[acc] curve limit: %.2f -> %.2f m/s (%.1f km/h)",
                        target, curve_limit, curve_limit * 3.6);
      target = curve_limit;
    }

    // 상승률 제한(rampTarget)은 여기서 하지 않는다. behavior_fsm 이 모든 제약을
    // min() 한 **뒤에** 적용한다. 그래야 신호가 녹색으로 바뀌는 순간처럼 다른
    // 제약이 풀릴 때의 급가속도 함께 막을 수 있다. (설계 50-behavior_fsm_design.md 3절)

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
