// behavior_fsm : 종방향 제약을 합쳐 /target_velocity 를 내는 노드
//
// 구독:  /speed_limit/acc            (Float64)  크루즈·곡률·앞차   <- acc_planner
//        /speed_limit/traffic_light  (Float64)  신호등             <- 담당자
//        /speed_limit/pedestrian     (Float64)  보행자             <- perception 대기
//        /speed_limit/intersection   (Float64)  교차로 접근 감속   <- 신호등 담당자
//        /speed_limit/avoid          (Float64)  회피 불가          <- lattice (미구현)
//        /ego_status                 (EgoVehicleStatus)  상승률 제한에 현재 속도가 필요
// 발행:  /target_velocity            (Float64)  종방향 단일 권한 [m/s]
//        /speed_limit/active         (String)   지금 이긴 제약 이름 (진단용)
//
// 설계: docs/50-behavior_fsm_design.md
//
// 이 노드가 생기기 전에는 acc_planner 가 /target_velocity 를 직접 냈다. 종방향을
// 정할 이유가 늘어나면(신호등·보행자·회피불가) 각자 브레이크를 밟아 서로 싸우므로,
// 전부 "상한" 으로만 내놓고 한 곳에서 min() 하도록 바꿨다.
//
// 상승률 제한(rampTarget)도 여기로 옮겼다. 그것은 "제약이 풀리는 순간의 급가속" 을
// 막는 장치인데 min() **뒤에** 있어야 의미가 있다. acc_planner 안에 있으면 곡률
// 제한이 풀릴 때만 눌러주고, 신호가 녹색으로 바뀌는 순간은 못 막는다.
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <morai_msgs/EgoVehicleStatus.h>

#include <string>
#include <vector>

#include "path_tracking/acc_core.hpp"       // rampTarget, speedKmhToMps
#include "path_tracking/behavior_core.hpp"  // combineLimits

namespace {

// 제약 슬롯. 순서가 곧 인덱스이고 winner 가 이 순서를 가리킨다.
//
// avoid 와 highway 는 아직 생산자가 없다. 자리를 잡아두면 나중에 추가할 때
// 인덱스가 밀리지 않는다.
enum LimitIdx {
  LIM_ACC = 0,
  LIM_TRAFFIC_LIGHT,
  LIM_PEDESTRIAN,
  LIM_INTERSECTION,   // 신호등 담당자가 발행 (2026-08-10 인계)
  LIM_AVOID,          // lattice 미구현
  LIM_HIGHWAY,        // 미구현 (v1 에서는 끈다 - 설계 5.1.2)
  LIM_COUNT
};

// 순서가 enum 과 정확히 일치해야 한다. winner 인덱스로 이 배열을 찾기 때문에
// 어긋나면 진단 로그가 조용히 엉뚱한 이름을 낸다.
const char* kNames[LIM_COUNT] = {
  "acc", "traffic_light", "pedestrian", "intersection", "avoid", "highway"
};

constexpr double kTimerHz = 30.0;

}  // namespace

class BehaviorFsm {
public:
  BehaviorFsm(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : limits_(LIM_COUNT)
  {
    double cruise_kmh = 55.0;
    pnh.param("cruise_speed_kmh", cruise_kmh, cruise_kmh);
    pnh.param("stale_timeout",    stale_timeout_,    stale_timeout_);
    pnh.param("accel_rate_limit", params_.accel_rate_limit, params_.accel_rate_limit);
    pnh.param("rate_limit_windup", params_.rate_limit_windup, params_.rate_limit_windup);
    cruise_ = cruise_kmh / 3.6;

    // 보행자만 다른 정책을 쓴다. 안전이 걸린 제약이라 끊겼다고 무시할 수 없지만,
    // 그렇다고 0 으로 세우면 영영 못 간다. 그 사이를 택해 크루즈의 절반으로 간다.
    // (설계 6.1. 완전 무시와 완전 정지 사이의 보수 주행)
    limits_[LIM_PEDESTRIAN].on_stale    = behavior::StaleAction::Conservative;
    limits_[LIM_PEDESTRIAN].stale_value = cruise_ * 0.5;

    sub_acc_ = nh.subscribe<std_msgs::Float64>(
        "/speed_limit/acc", 1, boost::bind(&BehaviorFsm::limitCb, this, _1, LIM_ACC));
    sub_tl_  = nh.subscribe<std_msgs::Float64>(
        "/speed_limit/traffic_light", 1, boost::bind(&BehaviorFsm::limitCb, this, _1, LIM_TRAFFIC_LIGHT));
    sub_ped_ = nh.subscribe<std_msgs::Float64>(
        "/speed_limit/pedestrian", 1, boost::bind(&BehaviorFsm::limitCb, this, _1, LIM_PEDESTRIAN));
    sub_int_ = nh.subscribe<std_msgs::Float64>(
        "/speed_limit/intersection", 1, boost::bind(&BehaviorFsm::limitCb, this, _1, LIM_INTERSECTION));
    sub_avd_ = nh.subscribe<std_msgs::Float64>(
        "/speed_limit/avoid", 1, boost::bind(&BehaviorFsm::limitCb, this, _1, LIM_AVOID));

    sub_ego_ = nh.subscribe("/ego_status", 1, &BehaviorFsm::egoCb, this);

    pub_vel_    = nh.advertise<std_msgs::Float64>("/target_velocity", 1);
    pub_active_ = nh.advertise<std_msgs::String>("/speed_limit/active", 1);

    timer_ = nh.createTimer(ros::Duration(1.0 / kTimerHz), &BehaviorFsm::run, this);

    // ROS_INFO 포맷 문자열에는 한글을 쓰지 않는다. 컨테이너 로케일 때문에 깨진다.
    ROS_INFO("[behavior_fsm] started (cruise=%.1f km/h, stale_timeout=%.2fs, rate_limit=%.2f m/s^2)",
             cruise_kmh, stale_timeout_, params_.accel_rate_limit);
  }

private:
  void limitCb(const std_msgs::Float64::ConstPtr& msg, int idx) {
    limits_[idx].value = msg->data;
    limits_[idx].stamp = ros::Time::now().toSec();
  }

  void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    // /ego_status 의 velocity 는 UDP 원본 그대로라 km/h 다(브릿지가 변환 안 함).
    ego_vel_ = acc::speedKmhToMps(msg->velocity.x, msg->velocity.y);
    has_ego_ = true;
  }

  void run(const ros::TimerEvent&) {
    if (!has_ego_) return;   // 상승률 제한에 현재 속도가 필요하다

    const double now = ros::Time::now().toSec();
    behavior::Combined c = behavior::combineLimits(limits_, now, stale_timeout_);

    // 유효한 제약이 하나도 없으면 발행하지 않는다.
    //
    // 이때 kNoLimit(1e6)을 그대로 내보내면 "무제한" 이 되어 위험하다. 발행을 멈추면
    // path_tracker 가 0.5초 뒤 자체 폴백 속도로 넘어간다 - 이미 있는 안전망이다.
    if (c.alive == 0) {
      ROS_WARN_THROTTLE(2.0, "[behavior_fsm] no active speed limit - not publishing");
      prev_time_ = ros::Time();   // 다음 발행 때 dt 를 새로 시드하도록 리셋
      return;
    }

    // 상승률 제한. min() 뒤에 한 번만 적용한다. 목표가 어떤 이유로 오르든
    // 동일하게 눌러야 하기 때문이다.
    ros::Time t = ros::Time::now();
    double dt = prev_time_.isZero() ? (1.0 / kTimerHz) : (t - prev_time_).toSec();
    if (prev_time_.isZero() || dt > params_.rate_dt_max) {
      // 첫 틱이거나 오래 끊겼다. 달리는 차에 낡은 목표(또는 0)를 명령하면 급제동이
      // 걸리므로 현재 속도로 시드한다.
      prev_target_ = ego_vel_;
    }
    double target = acc::rampTarget(prev_target_, c.value, ego_vel_, dt, params_);
    prev_target_ = target;
    prev_time_   = t;

    std_msgs::Float64 out;
    out.data = target;
    pub_vel_.publish(out);

    // 진단: 지금 누가 이기고 있나. 제약이 여섯 개가 되면 "차가 왜 느린가" 를
    // 눈으로 알 수 없다. rostopic echo 로 즉답이 되고 lap_logger 에 같이 기록하면
    // 구간별로 어느 제약이 지배했는지 프로파일이 나온다.
    std_msgs::String act;
    act.data = (c.winner >= 0) ? kNames[c.winner] : "none";
    if (target < c.value - 1e-6) act.data += "+ramp";   // 상승률 제한이 더 눌렀다
    pub_active_.publish(act);
  }

  std::vector<behavior::Limit> limits_;
  acc::AccParams params_;          // rampTarget 이 쓰는 항목만 의미가 있다
  double cruise_        = 55.0 / 3.6;
  double stale_timeout_ = 0.5;     // [s]

  double ego_vel_ = 0.0;           // [m/s]
  bool   has_ego_ = false;
  double prev_target_ = 0.0;       // [m/s]
  ros::Time prev_time_;

  ros::Subscriber sub_acc_, sub_tl_, sub_ped_, sub_int_, sub_avd_, sub_ego_;
  ros::Publisher  pub_vel_, pub_active_;
  ros::Timer      timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "behavior_fsm");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  BehaviorFsm node(nh, pnh);
  ros::spin();
  return 0;
}
