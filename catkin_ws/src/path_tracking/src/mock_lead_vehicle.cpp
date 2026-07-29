// mock_lead_vehicle : ACC 검증용, 대회 경로 위를 느리게 달리는 앞차(NPC) 1대를 발행하는 개발 노드.
// 발행: /Object_topic (morai_msgs/ObjectStatusList)  - 실제 perception과 동일 인터페이스
//
// 왜 경로 추종인가(옵션 B): 우리 경로는 굽은 winding 코스라, 앞차를 직선(+x)으로 보내면 금방
// 경로에서 벗어나 acc_core::selectLead(경로 위 판정)가 앞차를 놓친다. 그래서 path_smooth.csv를
// 읽어 그 위를 호길이(s)만큼 전진시켜 앞차가 항상 경로 위에 있게 한다.
//   s = s_ego + start_gap + v * t   (s_ego = 노드 시작 시점의 ego 경로상 위치)
//
// 기준점이 왜 ego인가: 예전엔 s = start_gap + v*t 로 "경로 시작"을 기준으로 삼았는데,
// 그러면 노드를 켠 순간부터 앞차가 혼자 달려나간다. launch를 먼저 켜두고 나중에
// path_tracker를 띄우면 그 시간차만큼(예: 4분 = 1.2km) 앞차가 저 멀리 가버려서
// ACC가 앞차를 못 잡는다. ego 위치를 앵커로 쓰면 언제 켜도 항상 start_gap 앞에 선다.
//
// mock_obstacle_pub과 동시 실행 금지(둘 다 /Object_topic 발행). 택일.
#include <ros/ros.h>
#include <ros/package.h>
#include <morai_msgs/ObjectStatusList.h>
#include <morai_msgs/ObjectStatus.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <limits>

struct P { double x = 0.0, y = 0.0; };

// path_smooth.csv (헤더 "x,y,z" + 각 줄 x,y,z) 를 읽어 (x,y) 벡터로 반환.
static std::vector<P> loadPath(const std::string& file)
{
  std::vector<P> pts;
  std::ifstream in(file.c_str());
  if (!in.is_open()) return pts;               // 열기 실패 → 빈 벡터

  std::string line;
  std::getline(in, line);                       // 첫 줄(헤더 "x,y,z") 버림
  while (std::getline(in, line))
  {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string sx, sy;
    std::getline(ss, sx, ',');                  // x
    std::getline(ss, sy, ',');                  // y  (z는 안 씀)
    P p; p.x = std::stod(sx); p.y = std::stod(sy);
    pts.push_back(p);
  }
  return pts;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mock_lead_vehicle");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double start_gap = 30.0, lead_speed_kmh = 18.0;
  std::string path_file = ros::package::getPath("path_tracking") + "/path/path_smooth.csv";
  pnh.param("start_gap",      start_gap,      start_gap);       // 경로 시작에서 앞차까지 초기 거리 [m]
  pnh.param("lead_speed_kmh", lead_speed_kmh, lead_speed_kmh);  // 앞차 속도 [km/h] (60캡보다 느리게)
  pnh.param("path_file",      path_file,      path_file);       // 추종할 경로 csv

  // --- 경로 로드 + 누적 호길이 계산 ---
  std::vector<P> path = loadPath(path_file);
  if (path.size() < 2) {
    ROS_FATAL("[mock_lead_vehicle] 경로 로드 실패(점 %zu개): %s", path.size(), path_file.c_str());
    return 1;
  }
  std::vector<double> cum(path.size(), 0.0);    // cum[i] = 시작~i번째 점까지 주행거리
  for (size_t i = 1; i < path.size(); ++i)
    cum[i] = cum[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  const double path_len = cum.back();

  // --- ego 현재 위치를 경로상 호길이(s_ego)로 환산해 앞차의 출발 기준으로 삼는다 ---
  double s_ego = 0.0;
  auto ego = ros::topic::waitForMessage<morai_msgs::EgoVehicleStatus>(
      "/ego_status", nh, ros::Duration(5.0));
  if (ego)
  {
    size_t best = 0;
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i)
    {
      double dx = path[i].x - ego->position.x;
      double dy = path[i].y - ego->position.y;
      double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    s_ego = cum[best];
    ROS_INFO("[mock_lead_vehicle] ego 경로상 위치 s=%.1fm (경로에서 %.2fm 떨어짐)",
             s_ego, std::sqrt(best_d2));
  }
  else
  {
    ROS_WARN("[mock_lead_vehicle] /ego_status 를 못 받음 - 경로 시작(s=0)을 기준으로 발행");
  }

  ros::Publisher pub = nh.advertise<morai_msgs::ObjectStatusList>("/Object_topic", 1);
  ros::Rate rate(20);   // 20Hz
  const double v_mps = lead_speed_kmh / 3.6;
  const ros::Time t0 = ros::Time::now();
  size_t seg = 0;       // 현재 앞차가 올라탄 구간 인덱스(단조 증가 → 커서로 전진)

  ROS_INFO("[mock_lead_vehicle] 경로추종 앞차 발행 (gap=%.1fm, %.1f km/h, 경로길이=%.1fm)",
           start_gap, lead_speed_kmh, path_len);

  while (ros::ok())
  {
    double dt = (ros::Time::now() - t0).toSec();
    double s = s_ego + start_gap + v_mps * dt;  // 앞차의 경로상 위치(주행거리)
    if (s > path_len) s = path_len;             // 경로 끝을 넘으면 마지막 점에 정지(clamp)

    // s가 속한 구간 [seg, seg+1] 찾기 (s는 단조증가 → 커서만 앞으로)
    while (seg + 1 < path.size() - 1 && cum[seg + 1] < s) ++seg;

    // 구간 안에서 선형보간: ratio = (s - cum[seg]) / 구간길이
    double seg_len = cum[seg + 1] - cum[seg];
    double ratio = (seg_len > 1e-9) ? (s - cum[seg]) / seg_len : 0.0;
    double px = path[seg].x + ratio * (path[seg + 1].x - path[seg].x);
    double py = path[seg].y + ratio * (path[seg + 1].y - path[seg].y);
    double yaw = std::atan2(path[seg + 1].y - path[seg].y, path[seg + 1].x - path[seg].x);

    morai_msgs::ObjectStatusList msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    // --- 경로 위를 달리는 앞차 NPC 1대 ---
    morai_msgs::ObjectStatus npc;
    npc.unique_id = 10;
    npc.type = 1;                               // 0:보행자 1:NPC 2:정적장애물
    npc.name = "mock_lead";
    npc.position.x = px;                        // 보간된 경로점 (ENU 전역좌표)
    npc.position.y = py;
    npc.position.z = 0.0;
    npc.velocity.x = lead_speed_kmh * std::cos(yaw);   // [km/h] 접선방향 → heading과 일치
    npc.velocity.y = lead_speed_kmh * std::sin(yaw);
    npc.velocity.z = 0.0;
    npc.size.x = 1.9; npc.size.y = 4.6; npc.size.z = 1.5;   // Ioniq5 승용차 [m]
    npc.heading = yaw * 180.0 / M_PI;           // [deg]

    msg.npc_list.push_back(npc);
    msg.num_of_npcs = 1;
    msg.num_of_pedestrian = 0;
    msg.num_of_obstacle = 0;

    pub.publish(msg);
    rate.sleep();
  }
  return 0;
}
