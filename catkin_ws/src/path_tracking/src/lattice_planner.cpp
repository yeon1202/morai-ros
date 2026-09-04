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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <morai_msgs/ObjectStatus.h>
#include <visualization_msgs/MarkerArray.h>

// "제한 없음" 약속값(1e6)을 behavior 쪽과 공유한다. 두 곳에 따로 적으면 어긋난다.
#include "path_tracking/behavior_core.hpp"
#include "path_tracking/acc_core.hpp"   // lateralHalfExtent - ACC 와 같은 판정식을 쓰기 위해
#include "path_tracking/road_core.hpp"  // 옆 차로 유무 - 도로 밖 후보를 거른다
#include <ros/package.h>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

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
    ros::NodeHandle pnh("~");

    // 도로 경계표를 읽는다. build_lane_table.py 가 미리 만들어 둔 것이다.
    //
    // ⚠️ 못 읽으면 "옆 차로가 없다" 로 답하게 되어 회피가 통째로 꺼진다.
    //    조용히 넘어가면 안 되는 상태라 ERROR 로 남긴다. 이 선택의 근거는
    //    road_core.hpp 의 at() 주석 참고 - 모를 때 나가면 도로 밖이고,
    //    모를 때 안 나가면 ACC 가 속도를 줄인다. 두 실패의 무게가 다르다.
    std::string lane_csv;
    pnh.param<std::string>("lane_table",
                           lane_csv,
                           ros::package::getPath("path_tracking") + "/map/lane_table.csv");
    if (lane_table_.load(lane_csv)) {
      ROS_INFO("[lattice] lane_table %zu points (%s)", lane_table_.size(), lane_csv.c_str());
    } else {
      ROS_ERROR("[lattice] cannot read lane_table: %s", lane_csv.c_str());
      ROS_ERROR("[lattice] every side offset will be rejected - avoidance is OFF");
      ROS_ERROR("[lattice] build it with: rosrun path_tracking build_lane_table.py");
    }

    sub_path_ = nh.subscribe("/local_path", 1, &LatticePlanner::pathCb, this);
    sub_ego_  = nh.subscribe("/ego_status", 1, &LatticePlanner::egoCb, this);
    sub_odom_ = nh.subscribe("/odom", 1, &LatticePlanner::odomCb, this);
    sub_obj_  = nh.subscribe("/Object_topic", 1, &LatticePlanner::objCb, this);
    pub_path_ = nh.advertise<nav_msgs::Path>("/lattice_path", 1);
    pub_cand_ = nh.advertise<visualization_msgs::MarkerArray>("/lattice_candidates", 1);
    // 종방향 제약. behavior_fsm 이 min() 으로 합친다(설계 5.1.1 의 미구현 항목).
    // 목표속도가 아니라 상한이다. 제한할 이유가 없으면 behavior::kNoLimit 를 계속 낸다.
    pub_avoid_ = nh.advertise<std_msgs::Float64>("/speed_limit/avoid", 1);
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
  //   1) 0 후보가 없었다. 회피 트리거가 걸리기만 하면 무조건 1m 이상
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

  // ---- 정적장애물 미션 전용 예외 (2026-08-19) ----
  //
  // 이 미션에서만 "감속 + 횡가속 완화" 를 쓴다. 다른 구간, 특히 고주로에서
  // 같은 처리를 하면 80km/h 로 달리다 24km/h 로 기어가게 되어 훨씬 위험하다.
  // 그래서 좌표로 못을 박는다. 일반화는 MGeo 로 차선 유무를 읽게 된 뒤에 한다.
  //
  // 왜 필요한가:
  //   장애물(경로 328.9m)의 바로 앞 정지선이 315.9m 다. 그 사이 13.0m 가
  //   교차로 안이라 차선이 없다. 회피를 그 안에서 끝내면 실선 접촉이 없다.
  //   그런데 13m 안에 3.0m 를 옮기려면 속도를 낮추거나 횡가속을 키워야 한다.
  //     0.3G -> 18.9 km/h,  0.5G -> 24.4 km/h,  0.7G -> 28.9 km/h
  //   0.5G 를 택했다. 그때 최대 곡률 R=9.4m, 조향 17.7° 로 차량 한계
  //   (최소회전반경 5.87m, 조향 40°) 안이다. 2026-07-29 에 "0.72G 는 차가 못
  //   따라가고 밀려난다" 고 확인한 것은 고속 구간이라 여기와는 다른 영역이다.
  //
  //   속도 대가는 사실상 없다. ACC 가 이 장애물을 앞차로 보고 이미 17km/h 까지
  //   떨어뜨리고 있다(2026-08-19 실측). 지금은 그 대가를 치르면서 회피는 30m
  //   전에 시작해 이득을 하나도 못 받는 상태다.
  const double MISSION_OBS_X = -60.610;   // 시나리오 objectList[0].pos
  const double MISSION_OBS_Y = -142.178;
  const double MISSION_MATCH_R = 5.0;     // 이 반경 안의 장애물이면 그 미션으로 본다
  const double MISSION_AVOID_SPAN = 13.0; // 정지선(315.9m) ~ 장애물(328.9m)

  // 회피 개시 게이트 - 이 정지선을 넘기 전에는 옆으로 나가지 않는다.
  //
  // 52-stopline_table.txt 의 C1256W000091 (경로 315.9m). 여기부터 교차로 안이라
  // 차선이 없다. 감속만으로는 부족했다(2026-08-19 실측): 차가 310m 지점에서
  // 아직 35km/h 일 때 계산된 시작점이 310.3m 라 거기서 이미 출발해버리고,
  // 그 뒤 감속으로 시작점이 뒤로 밀려도 이미 나간 뒤였다. 그래서 위치로 막는다.
  //
  // 정지선에 닿았을 때 24.3km/h(실측)이므로 남은 13m 로 3.0m 를 옮기면 0.496G 다.
  // 완화 예산 0.5G 안이라 물리적으로 가능하다 - 감속이 그 조건을 만들어준다.
  const double MISSION_STOP_X = -59.833;
  const double MISSION_STOP_Y = -155.393;
  const double MISSION_STOP_MATCH_R = 3.0;  // local_path 에서 정지선을 찾는 허용 오차

  // 게이트를 물고 있어도 되는 속도 초과 허용률.
  //
  // 우리가 요청한 상한(/speed_limit/avoid)을 차가 이 배수 안에서 따라오고 있으면
  // 정지선 도착 속도가 보장되므로 게이트를 유지한다. 벗어나 있으면 감속이 안 걸린
  // 것이므로 게이트를 놓고 일찍 나간다 - 따라갈 수 없는 경로를 명령해 회피가
  // 실패하는 것보다 낫다. 차선 접촉(3초당 5초)보다 충돌(15초)이 훨씬 비싸다.
  const double GATE_SPEED_TOLERANCE = 1.15;
  const double AVOID_ACCEL_LIMIT = 4.90;  // 0.5G. 회피 기동 전용(크루즈 곡률제한은 0.3G 그대로)
  const double AVOID_BRAKE_ACCEL = 2.0;   // 감속 상한을 거리로 풀 때 쓰는 감속도. ACC 와 같은 값

  // 회피 기동을 장애물 이만큼 앞에서 끝낸다 [m].
  // 차 길이가 4.635m 라 대략 한 대분이다. 0 으로 두면 장애물과 나란해지는 순간에야
  // 목표 offset 에 도달해 여유가 없다. 시뮬에서 조정할 값이다.
  const double COMPLETE_MARGIN = 5.0;

  // ---- 복귀 곡선 (2026-08-21) ----
  //
  // 회피가 끝나면 차는 아직 옆으로 나가 있는데, 예전에는 기준경로 원본을 그대로
  // 냈다. 그러면 명령 경로가 한 틱에 횡으로 점프하고 pure_pursuit 이 과보정한다.
  // 실측(logs/lap_gate3.csv, 38km/h): 도착각 27도, 반대쪽으로 1.21m 오버슈트,
  // 차로 여유 0.809m 를 7.5m / 1.29초 초과. 넘어가는 방향이 하필 황색 중앙선이다.
  //
  // 2026-08-19 에 한 번 넣었다가 되돌렸다. 그때는 복귀 곡선의 끝점도 "차에서
  // L 앞" 이라 차를 따라 도망갔다. 매 틱 차 위치에서 다시 그려지니 pure_pursuit
  // 이 보는 오차가 늘 0 에 가깝고, 그래서 되돌아오질 않았다(31m -> 103m 악화).
  //
  // 이번에는 회피 후보와 같은 방식으로 **목표점을 도로 위에 못박는다**. 한 번
  // 걸어두면 차가 다가갈수록 남은 거리가 줄어 곡선이 가팔라지고, 그래서 실제로
  // 수렴한다. 8/19 방식과 정확히 반대 방향이다.
  const double RETURN_DONE     = 0.25;  // 이 안이면 복귀 완료. 기준경로 원본을 낸다 [m]
  const double RETURN_MIN_SPAN = 3.0;   // 목표점이 이보다 가까우면 놓고 다시 건다 [m]

  // 복귀 전이 길이를 정하는 횡가속 예산 [m/s^2]. 0.3G.
  //
  // 여기가 이 기능의 유일한 튜닝 손잡이다. 크게 잡으면 복귀가 짧고 가팔라져
  // 차선 밖에 있는 시간이 줄지만 오버슈트가 남고, 작게 잡으면 부드러운 대신
  // 차선 밖에 오래 있는다. 규정이 접촉 3초당 5초라 "부드러움" 이 아니라
  // **여유 초과 총 거리**로 정해야 한다. 우선 회피와 같은 0.3G 로 두고
  // 실측(lap CSV 의 0.809m 초과 거리)으로 조정한다.
  const double RETURN_ACCEL_LIMIT = 2.94;

  ros::Subscriber sub_path_, sub_ego_, sub_obj_;
  ros::Publisher pub_path_, pub_cand_, pub_avoid_;
  ros::Timer timer_;
  nav_msgs::Path local_path_;
  morai_msgs::EgoVehicleStatus ego_;
  morai_msgs::ObjectStatusList objs_;
  bool has_path_ = false, has_ego_ = false, has_obj_ = false;
  nav_msgs::Odometry odom_;
  bool has_odom_ = false;
  ros::Subscriber sub_odom_;
  bool cand_shown_ = false;   // RViz 에 후보 마커가 떠 있는 상태인가
  // 후보별 요구 횡가속도 [m/s^2]. generateCandidates 가 채우고 run 이 고른 것만 본다.
  std::vector<double> cand_accel_;
  double cand_accel_limit_ = 2.94;   // 그때 적용한 예산(미션이면 완화값)

  // 복귀 목표점 (world). ret_active_ 인 동안 도로 위에 고정된다.
  // 차가 아니라 도로를 기준으로 잡는 것이 이 기능의 핵심이다.
  bool   ret_active_ = false;
  double ret_x_ = 0.0, ret_y_ = 0.0;
  // 직전에 회피하던 것이 미션 장애물이었나. 복귀 예산을 정하는 데 쓴다.
  bool   last_mission_ = false;   // 매 틱 갱신
  bool   ret_mission_  = false;   // latch 시점에 걸어두고 복귀가 끝날 때까지 유지

  void pathCb(const nav_msgs::Path::ConstPtr& m) { local_path_ = *m; has_path_ = true; }
  // 속도만 쓴다. 위치는 odomCb 에서 받는다 - 대회 규정 채널(9109)은 position 을
  // 0,0,0 으로 주기 때문이다 (2026-08-29 전환).
  void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& m) { ego_ = *m; has_ego_ = true; }

  // 위치는 /odom(GPS+IMU 융합)에서 받는다.
  // ※ twist 는 쓰지 않는다. wheel_speed_scaler 가 시뮬 배속을 곱해 벽시계 단위로
  //   바꿔놓은 값이라 실제 주행속도가 아니다. 속도는 계속 /ego_status 에서 받는다.
  void odomCb(const nav_msgs::Odometry::ConstPtr& m) { odom_ = *m; has_odom_ = true; }
  void objCb(const morai_msgs::ObjectStatusList::ConstPtr& m) { objs_ = *m; has_obj_ = true; }

  // 장애물 하나 = 위치 + 반경(size 반영)
  // 물체 하나. r 은 예전 원 근사(크기를 모를 때 폴백), len/wid/yaw 가 방향 있는 상자.
  // 2026-09-03: 원 근사가 가드레일의 길이를 옆으로 부풀려 경로 밖 물체가 전역경로를
  // 막던 문제 때문에 상자를 추가했다. 판정식은 acc_core.hpp 의 lateralHalfExtent
  // 하나만 쓴다 - ACC 와 다른 기준을 쓰면 "lattice 는 통과시키는데 ACC 는 세우는"
  // 일이 또 생긴다.
  // 도로 경계표. 후보가 지나갈 구간에 옆 차로가 있는지 본다.
  // 2026-09-03 실패: 오탐으로 우측 -3.51 을 골랐는데 12m 앞에서 우측 차로가
  // 끝나 인도로 올라갔다. 전역경로의 75% 가 우측 차로 없는 구간이다.
  road::LaneTable lane_table_;
  // 후보별 "도로 안인가". generateCandidates 가 채우고 selectLane 이 쓴다.
  std::vector<char> cand_allowed_;

  struct Obs { double x, y, r, len, wid, yaw; };

  // 경로점 i 에서의 경로 진행방향 [rad]. 물체 방향과의 각도차를 내는 데 쓴다.
  static double pathYawAt(const nav_msgs::Path& path, std::size_t i)
  {
    if (path.poses.size() < 2) return 0.0;
    std::size_t a = (i == 0) ? 0 : i - 1;
    std::size_t b = std::min(i + 1, path.poses.size() - 1);
    return std::atan2(path.poses[b].pose.position.y - path.poses[a].pose.position.y,
                      path.poses[b].pose.position.x - path.poses[a].pose.position.x);
  }

  // 물체가 이 경로점 방향 기준으로 차지하는 횡 반폭.
  static double halfExtent(const Obs& o, double path_yaw)
  {
    if (o.len <= 0.0 && o.wid <= 0.0) return o.r;
    return acc::lateralHalfExtent(o.len, o.wid, o.yaw - path_yaw);
  }

  // 장애물 수집.
  //
  // with_pedestrian 을 나눈 이유:
  //   보행자는 "피하는" 대상이 아니라 "서는" 대상이다. 미션이 횡단보도로
  //   신호를 위반하며 뛰어드는 사람이라, 옆으로 꺾으면 사람이 뛰어든 쪽으로
  //   들어갈 수도 있고 반대 차로로 나갈 수도 있다. 정답은 급정지이고 그건
  //   behavior FSM / ACC 몫이다.
  //   그래서 보행자는 회피를 촉발하지 않는다(blockedSpan 에서 제외).
  //   다만 다른 이유로 이미 회피 중이라면 보행자를 뚫고 가는 후보를 고르면
  //   안 되므로, 후보 충돌 검사에는 포함한다.
  std::vector<Obs> gatherObstacles(bool with_pedestrian)
  {
    std::vector<Obs> v;
    auto add = [&](const std::vector<morai_msgs::ObjectStatus>& list) {
      for (const auto& o : list) {
        double r = 0.5 * std::max(o.size.x, o.size.y);   // size 반영 (리뷰 #4)
        if (r < 0.3) r = 0.3;
        // size 규약: x=length(주축), y=width(부축). heading 은 도 단위.
        v.push_back({o.position.x, o.position.y, r,
                     o.size.x, o.size.y, o.heading * M_PI / 180.0});
      }
    };
    add(objs_.npc_list);
    add(objs_.obstacle_list);
    if (with_pedestrian) add(objs_.pedestrian_list);
    return v;
  }

  // 트리거 장애물이 "그 정적장애물 미션" 인가? 좌표로 판정한다.
  //
  // 자차 위치가 아니라 장애물 위치로 보는 이유: 장애물이 곧 그 미션의 정체다.
  // 자차 기준으로 구간을 잡으면 그 구간에 들어온 다른 물체(NPC 등)에까지
  // 예외가 적용된다.
  bool isMissionObstacle(const std::vector<Obs>& obs) const
  {
    for (const auto& o : obs)
      if (std::hypot(o.x - MISSION_OBS_X, o.y - MISSION_OBS_Y) < MISSION_MATCH_R)
        return true;
    return false;
  }

  // 정지선이 지금 local_path 위 어디인가 [m]. 시야 밖이면 -1.
  //
  // local_path 가 월드 좌표라 좌표만 알면 그 위에서 바로 찾을 수 있다.
  // path_tracker 가 전역 인덱스를 따로 발행할 필요가 없다.
  double stoplineS(const std::vector<double>& cum) const
  {
    int best = -1;
    double bd = MISSION_STOP_MATCH_R;
    for (std::size_t i = 0; i < local_path_.poses.size(); ++i) {
      double d = std::hypot(local_path_.poses[i].pose.position.x - MISSION_STOP_X,
                            local_path_.poses[i].pose.position.y - MISSION_STOP_Y);
      if (d < bd) { bd = d; best = static_cast<int>(i); }
    }
    return (best < 0) ? -1.0 : cum[best];
  }

  // 기준경로의 누적거리 [m].
  //
  // 후보별 전이 길이와 기동 시작점을 "몇 번째 점" 이 아니라 "몇 m" 로 재기 위해
  // 한 번 만들어 돌려 쓴다. waypoint 간격이 바뀌어도 거동이 안 바뀌는 이유다.
  std::vector<double> cumulativeS() const
  {
    const int n = local_path_.poses.size();
    std::vector<double> cum(n, 0.0);
    for (int i = 1; i < n; ++i)
      cum[i] = cum[i-1] + std::hypot(
          local_path_.poses[i].pose.position.x - local_path_.poses[i-1].pose.position.x,
          local_path_.poses[i].pose.position.y - local_path_.poses[i-1].pose.position.y);
    return cum;
  }

  // 기준경로가 장애물에 막히는 구간 [m]. 막힌 곳이 없으면 valid=false.
  struct Span {
    bool   valid = false;
    double s_first = 0.0;   // 판정원에 들어가는 첫 점
    double s_last  = 0.0;   // 빠져나오는 마지막 점
    double mid() const { return 0.5 * (s_first + s_last); }   // 장애물의 경로상 위치
  };

  // 처음 막히는 "구간" 을 찾는다. 두 번째 장애물까지 삼키지 않도록 첫 연속 구간만.
  //
  // 예전 objectOnPath() 는 "있다/없다" 만 돌려줬다. 회피 기동을 장애물 기준으로
  // 배치하려면 위치가 필요해서 값으로 바꿨고, 그 다음엔 끝(s_last)도 필요해졌다.
  // offset 을 유지할 구간이 "장애물을 지날 때까지" 여야 하기 때문이다. 판정식 자체는
  // 처음 그대로다.
  Span blockedSpan(const std::vector<Obs>& obs, const std::vector<double>& cum)
  {
    Span sp;
    auto blocked = [&](std::size_t i) {
      const auto& p = local_path_.poses[i];
      const double pyaw = pathYawAt(local_path_, i);
      for (const auto& o : obs) {
        double d = std::hypot(p.pose.position.x - o.x, p.pose.position.y - o.y);
        if (d < halfExtent(o, pyaw) + CAR_HALF_WIDTH + SAFE_MARGIN) return true;
      }
      return false;
    };
    for (std::size_t i = 0; i < local_path_.poses.size(); ++i) {
      if (!blocked(i)) continue;
      sp.valid = true;
      sp.s_first = cum[i];
      sp.s_last  = cum[i];
      for (std::size_t j = i + 1; j < local_path_.poses.size(); ++j) {
        if (!blocked(j)) break;
        sp.s_last = cum[j];
      }
      break;
    }
    return sp;
  }

  // 후보 경로 생성 (기준경로 시작점 기준 local 프레임 -> 3차곡선 -> map 복귀)
  // offset 0 후보도 의미가 있다. 3차곡선이 차의 현재 횡위치(egoy)에서 기준경로로
  // 부드럽게 복귀시키므로, "그대로 간다" 가 곧 "제 차선으로 돌아온다" 가 된다.
  std::vector<nav_msgs::Path> generateCandidates(const std::vector<double>& cum, const Span& sp,
                                                 bool mission)
  {
    std::vector<nav_msgs::Path> out;
    const int n = local_path_.poses.size();
    if (n < 3) return out;

    // 기동에 쓸 횡가속도 예산. 정적장애물 미션에서만 0.5G 로 완화한다(상수 주석 참고).
    const double a_lat = mission ? AVOID_ACCEL_LIMIT : LAT_ACCEL_LIMIT;
    // 회피 개시 게이트(정지선) 위치. 미션이 아니거나 시야 밖이면 -1.
    const double s_gate = mission ? stoplineS(cum) : -1.0;

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
    //
    // 2026-08-19: 이 길이를 여섯 후보가 공유하던 것을 후보별로 나눴다. 아래 ① 참고.

    // 후보 0(제자리)의 도달거리 = 탐지 지평.
    //
    // 후보 0 이 막히는 순간이 곧 "회피해야 한다" 는 신호다. 여기를 짧게 잡으면
    // 가장 큰 회피(3.51m)를 시작하기에 이미 늦은 시점에야 알아차린다. 그래서
    // 이 후보만은 최대 offset 기준(d_max) 길이를 그대로 쓴다.
    double d_max = 0.0;
    for (double off : LANE_OFFSET) d_max = std::max(d_max, std::fabs(off));
    const double reach0 = std::max(v_mps * std::sqrt(6.0 * d_max / LAT_ACCEL_LIMIT),
                                   MIN_TRANSITION);

    cand_accel_.clear();
    cand_allowed_.clear();
    cand_accel_limit_ = a_lat;

    for (double off : LANE_OFFSET) {
      nav_msgs::Path cand;
      cand.header.frame_id = "map";
      const double a_off = std::fabs(off);
      // 후보마다 정확히 하나. 중간에 continue 로 빠져도 인덱스가 어긋나지 않도록
      // 먼저 자리를 잡아두고 나중에 채운다.
      cand_accel_.push_back(0.0);
      cand_allowed_.push_back(1);

      // ① 이 후보가 실제로 필요한 전이 길이.
      //
      // 예전에는 여섯 후보가 전부 d_max(3.51m) 기준 길이를 같이 썼다. 2.0m 만
      // 옮기면 되는 후보도 3.51m 용 길이를 쓰니 횡가속도가 예산(0.30G)의 절반
      // 밖에 안 나왔고(55km/h 에서 0.17G) 그만큼 기동이 길게 늘어져 있었다.
      // offset 마다 따로 구하면 모든 후보가 균일하게 0.3G 를 쓴다.
      //   55km/h:  1.0m -> 21.8m,  2.0m -> 30.9m,  3.51m -> 40.9m (예전엔 전부 40.9m)
      const double L = (a_off < 1e-9)
          ? reach0
          : std::max(v_mps * std::sqrt(6.0 * a_off / a_lat), MIN_TRANSITION);

      // ② 기동을 최대한 늦게 시작한다.
      //
      // 예전에는 후보 곡선이 항상 차 바로 앞(x=0)에서 시작했다. 후보 0 이 막히는
      // 순간(55km/h 에서 장애물 약 47m 앞) 곧바로 옆으로 나가기 시작하는데 실제로
      // 필요한 길이는 30.9m 뿐이라, 남는 16m 를 차선 밟은 채로 흘려보내고 있었다.
      //
      // 차로 안 여유가 편도 0.809m 뿐이라(차폭 1.892 / 차로 3.51) 일찍 나갈수록
      // 실선 접촉 시간이 그대로 늘어난다. 규정은 접촉 3초당 5초다.
      //
      // 그래서 "장애물 COMPLETE_MARGIN 앞에서 기동이 끝나도록" 역산해 시작점을
      // 뒤로 민다. 그 전까지는 기준경로를 그대로 따라간다.
      //
      // ①만 하고 이걸 안 하면 오히려 나빠진다. 시작이 그대로인 채 전이만
      // 짧아져서 목표 offset 에 더 빨리 도달하기 때문이다. 둘은 한 세트다.
      //
      // ⚠️ 2026-08-19 정정. 처음엔 x_start 만 뒤로 밀고 끝점을 x_start + L 로 뒀는데,
      // 여유가 없어 x_start 가 0 으로 잘리면 **끝점이 "차에서 L 앞" 이 되어 차를
      // 따라다녔다.** 차가 1m 가면 끝점도 1m 도망가서 영영 도달하지 못한다.
      // 실측에서 장애물 32.5m 전부터 끝점이 장애물 뒤로 넘어갔고, 계속 밀려서
      // 옆을 지날 때는 18.9m 뒤에 있었다. 그래서 차는 곡선의 완만한 앞부분만
      // 밟았고 -3.00m 명령에 -2.36m 밖에 도달하지 못했다(0.64m 미달).
      //
      // 그런데 selectLane 은 자기가 그린 경로(-3.00m 도달)로 충돌을 판정하므로
      // "0.80m 여유로 통과" 라고 봤다. 실제 여유는 0.18m 였다. **planner 가 조용히
      // 낙관적으로 틀린다.** 그래서 끝점을 도로 위 한 지점에 못박는다.
      //
      // 우선순위: ① 장애물 옆에 닿기 전에 끝낸다  ② 0.3G 를 지킨다  ③ 여유를 둔다
      // 셋을 다 못 지키면 ③부터 버린다. ②까지 버려야 하면 그건 "제때 못 피한다" 는
      // 뜻이고, 조용히 넘어가지 않도록 경고를 낸다.
      double x_end, hold_end;
      if (a_off < 1e-9) {
        // 후보 0 은 기동이 아니라 탐지 지평이다. 차 기준이 맞다.
        x_end    = reach0;
        hold_end = reach0;   // 아래에서 TAIL_EXTEND 로 조금 연장한다
      } else {
        const double s_mid = sp.mid();                       // 장애물의 경로상 위치
        x_end = std::min(s_mid, std::max(s_mid - COMPLETE_MARGIN, L));
        x_end = std::max(x_end, 2.0);                        // 퇴화 방지
        // 유지 구간: 장애물을 완전히 지날 때까지 offset 을 물고 간다.
        // 이게 없으면 후보 경로가 장애물 최근접점 앞에서 끊겨, 충돌검사가
        // "안 부딪힌다" 고 잘못 판정한다(경로가 거기까지 안 가니까).
        hold_end = std::max(x_end, sp.s_last + COMPLETE_MARGIN);
      }
      double x_start = (a_off < 1e-9) ? 0.0 : std::max(0.0, x_end - L);

      // 미션 게이트: 정지선을 넘기 전에는 옆으로 나가지 않는다.
      //
      // 감속만으로는 부족하다. 시작점은 매 틱 현재 속도로 계산되는데, 차가 아직
      // 빠를 때 계산된 이른 시작점을 한 번 지나가 버리면 그걸로 끝이다. 위치로
      // 막아야 "정지선 이후" 가 보장된다.
      //
      // 남은 거리로 감당이 안 되면(감속이 안 걸린 경우) 게이트를 놓는다.
      // 따라갈 수 없는 경로를 명령해 회피가 실패하는 것보다는 일찍 나가는 편이 낫다.
      // 해제 판정은 **지금 속도가 아니라 감속 계획을 지키고 있는지**로 한다.
      //
      // 지금 속도로 재면 항상 풀린다. 정지선 20m 전에서 38km/h 면 남은 13m 로는
      // 1.24G 가 필요하지만, 정작 정지선에 도착할 땐 24km/h 라 0.5G 로 된다.
      // 그래서 "우리가 요청한 상한을 차가 따라오고 있는가" 를 본다.
      // 따라오고 있으면 도착 시점 속도가 보장되므로 게이트를 물고,
      // 벗어나 있으면(감속이 안 걸림) 게이트를 놓고 일찍 나간다.
      if (mission && a_off > 1e-9 && s_gate >= 0.0 && s_gate > x_start) {
        const double v_allow = missionSpeedLimit(sp, off);
        const double span = x_end - s_gate;
        if (span > 1.0 && v_mps <= v_allow * GATE_SPEED_TOLERANCE) {
          x_start = s_gate;
        } else {
          ROS_WARN_THROTTLE(1.0,
              "[lattice] stopline gate released: v=%.1f > allow %.1f km/h (span %.1fm)",
              v_mps * 3.6, v_allow * 3.6, span);
        }
      }

      // 거리 -> 인덱스
      int i_start = 0;
      while (i_start + 1 < n && cum[i_start] < x_start) ++i_start;
      int i_end = i_start;
      while (i_end + 1 < n && cum[i_end] < x_end) ++i_end;
      if (i_end < i_start + 2) { out.push_back(cand); continue; }  // 남은 경로가 짧다
      int i_hold = i_end;
      while (i_hold + 1 < n && cum[i_hold] < hold_end) ++i_hold;

      // ③ 도로 경계 게이트 (2026-09-03)
      //
      // 이 후보가 옆으로 나가 있는 구간 [i_start, i_hold] 전체에서 그쪽 차로가
      // 실제로 존재하는지 본다. 하나라도 없으면 후보를 버린다.
      //
      // ⚠️ 자차 위치 하나만 보면 안 된다. 실패 당시 회피 시작점(경로 417.8m)의
      //    링크는 can_move_right_lane=True 였는데, 12m 앞 429.9m 에서 링크가
      //    바뀌며 False 가 됐다. 차는 그 사이에 인도로 올라갔다.
      //
      // 좌표로 조회하는 이유: local_path_ 는 전역경로에서 잘린 조각이라 자기가
      // 전역경로의 몇 번 점인지 모른다. 표가 좌표를 들고 있어서 그걸로 찾는다.
      //
      // 실패 세 가지를 구분한다. 셋을 뭉뚱그리면 원인을 못 찾는다.
      //   (a) 표를 못 읽었다        -> 옆으로 못 나간다 (보수적). 생성자가 ERROR 를 냈다.
      //   (b) 표는 있는데 코스 밖이다 -> 게이트가 답할 말이 없다. 통과시키되 WARN.
      //   (c) 정상 조회             -> 표대로 판정.
      // 차로 안에서 비켜가는 수준(|offset| <= kNudgeMax)은 게이트 대상이 아니다.
      // 옆 차로가 없어도 해야 한다 - 안 그러면 편도 1차선 구간의 정적장애물 앞에서
      // 그냥 선다(2026-09-03 실측). 자세한 근거는 road_core.hpp kNudgeMax 주석.
      if (a_off > 1e-9 && !road::isNudge(off)) {
        if (!lane_table_.loaded()) {
          cand_allowed_.back() = 0;
          out.push_back(cand);
          continue;
        }
        const auto& p0 = local_path_.poses[i_start].pose.position;
        const auto& p1 = local_path_.poses[std::min(i_hold, n - 1)].pose.position;
        const std::size_t g0 = lane_table_.nearestIndex(p0.x, p0.y);
        const std::size_t g1 = lane_table_.nearestIndex(p1.x, p1.y);

        if (g0 == road::kNoIndex || g1 == road::kNoIndex) {
          // 전역경로에서 3m 넘게 떨어져 있다. 오프라인 테스트의 가짜 경로이거나,
          // 실주행이라면 위치추정이 크게 틀어진 것이다. 후자라면 lattice 의 전제
          // (local_path 가 전역경로의 조각이다) 자체가 깨진 상태라 게이트만의
          // 문제가 아니다. 조용히 회피를 끄면 원인을 못 찾으므로 경고를 낸다.
          ROS_WARN_THROTTLE(5.0,
              "[lattice] local_path is >3m off the global path "
              "- road boundary gate not applied");
        } else if (!road::offsetAllowed(lane_table_, off, g0, g1)) {
          cand_allowed_.back() = 0;
          ROS_INFO_THROTTLE(2.0,
              "[lattice] offset %+.2f m is off-road - candidate rejected (wp %zu~%zu)",
              off, g0, g1);
          out.push_back(cand);   // 자리는 지킨다 (인덱스 정렬)
          continue;
        }
      }

      // 기동 전 구간은 기준경로를 그대로 붙인다. 회피할 이유가 없을 때 raw
      // local_path 를 내보내는 것과 같은 상태이므로 새로 만들 것이 없다.
      for (int i = 0; i < i_start; ++i) cand.poses.push_back(local_path_.poses[i]);

      // 좌표변환 기준을 기동 시작점으로 잡는다. 직선 프레임을 47m 씩 끌고 가면
      // 곡선 구간에서 오차가 커지므로, 시작점을 뒤로 옮기는 편이 오히려 정확하다.
      double sx = local_path_.poses[i_start].pose.position.x;
      double sy = local_path_.poses[i_start].pose.position.y;
      double nx = local_path_.poses[i_start + 1].pose.position.x;
      double ny = local_path_.poses[i_start + 1].pose.position.y;
      double theta = std::atan2(ny - sy, nx - sx);
      double c = std::cos(theta), sn = std::sin(theta);

      // world -> local (역변환)
      auto toLocal = [&](double x, double y, double& lx, double& ly) {
        double dx = x - sx, dy = y - sy;
        lx =  c * dx + sn * dy;
        ly = -sn * dx + c * dy;
      };
      // local -> world
      auto toWorld = [&](double lx, double ly, double& x, double& y) {
        x = sx + c * lx - sn * ly;
        y = sy + sn * lx + c * ly;
      };

      double ex, ey; toLocal(local_path_.poses[i_end].pose.position.x,
                             local_path_.poses[i_end].pose.position.y, ex, ey);
      double xf = ex;
      if (xf < 1.0) { out.push_back(cand); continue; }

      // 시작 횡위치.
      //   지연 없이 지금 시작하는 경우(i_start==0)에만 차의 실제 횡위치에서
      //   출발한다. 그래야 offset 0 후보가 "제 차선으로 부드럽게 복귀" 를
      //   표현하고, 이미 회피 중일 때도 지금 있는 자리에서 이어진다.
      //   지연된 경우엔 그 지점까지 기준경로를 따라가 있으므로 0 에서 시작한다.
      double ps = 0.0;
      if (i_start == 0) {
        double egox, egoy;
        toLocal(odom_.pose.pose.position.x, odom_.pose.pose.position.y, egox, egoy);
        ps = egoy;
      }
      double pf = ey + off;      // 끝 횡위치 (offset)

      // 3차곡선: y(0)=ps, y'(0)=0, y(xf)=pf, y'(xf)=0
      double a0 = ps, a1 = 0.0;
      double a2 = 3.0 * (pf - ps) / (xf * xf);
      double a3 = -2.0 * (pf - ps) / (xf * xf * xf);

      // 이 곡선이 실제로 요구하는 횡가속도. 최대 곡률이 6*D/L^2 이므로 v^2 를 곱한다.
      //
      // 예산(0.3G)을 넘으면 "제때 못 피한다" 는 뜻이다. 예전에는 끝점이 도망가느라
      // 이 상황이 아예 드러나지 않았다 - 항상 여유로운 곡선을 그려놓고 그 앞부분만
      // 밟았기 때문이다. 이제는 숫자로 나온다.
      //
      // 여기서 바로 경고하지 않고 후보별로 담아두는 이유: 못 피하는 작은 offset
      // (-1.0 등)은 늘 압축돼 한계를 넘는데, 어차피 선택되지 않는다. 그것까지
      // 경고하면 진짜 위험한 경우가 묻힌다. run() 에서 **고른 후보만** 본다.
      cand_accel_.back() = (xf > 1e-6)
          ? 6.0 * std::fabs(pf - ps) / (xf * xf) * v_mps * v_mps
          : 0.0;

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
      //
      // 2026-08-19: 회피 후보는 i_hold(장애물 통과 지점)까지, 후보 0 은 예전처럼
      // TAIL_EXTEND 만큼만 연장한다. 후보 0 을 길게 늘이면 탐지 지평이 함께 늘어나
      // 먼 장애물에 회피가 조기 발동한다.
      const int TAIL_EXTEND = 12;
      const int tail_last = (a_off < 1e-9) ? std::min(n - 1, i_end + TAIL_EXTEND - 1)
                                           : i_hold;
      for (int i = i_end; i <= tail_last && i < n; ++i) {
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
      // 도로 밖 후보는 충돌과 같은 무게로 막는다.
      //
      // 벌점만 주고 넘어가면 안 된다. 도로 밖 후보는 generateCandidates 가
      // 점을 하나도 안 넣고 반환하는데, 그러면 아래 충돌 검사가 "부딪히는 점이
      // 없다" 로 통과시켜 버린다. 기준경로(비용 0)가 막혔을 때 비용 1~4 짜리
      // 빈 후보가 최소값이 되어 뽑히고, lattice 가 빈 경로를 발행하게 된다.
      if (i < cand_allowed_.size() && !cand_allowed_[i]) {
        weight[i] += COLLISION_PENALTY;
        blocked[i] = true;
        continue;
      }
      // 벌점은 후보당 한 번만 준다.
      //
      // 예전에는 break 가 안쪽(장애물) 루프만 끊고 바깥(점) 루프는 계속 돌아서,
      // 충돌한 waypoint 개수만큼 +100 이 누적됐다. 장애물 하나인데 가중치가
      // 900, 800, 600 씩 붙었다. 그러면 "충돌하느냐" 가 아니라 "몇 점이나
      // 충돌하느냐" 로 순위가 매겨지고, 중앙선 회피 같은 기본 비용(20)이
      // 수백 점의 벌점 앞에서 무의미해진다.
      bool hit = false;
      for (std::size_t k = 0; k < cands[i].poses.size(); ++k) {
        const auto& p = cands[i].poses[k];
        // ⚠️ 각도는 "기준경로" 방향으로 잰다. 후보 자신의 방향이 아니다.
        //
        // 후보의 방향을 쓰면 회피 전이 구간에서 후보가 기울어 있는 만큼 물체의
        // 횡 반폭이 커진다. 정사각형에 가까운 물체는 45도에서 대각선(=반폭 1.41배)
        // 이 걸려, 회피를 시작하는 순간 갑자기 더 넓은 회피가 필요해지는 되먹임이
        // 생긴다(test_lattice 의 '미션 장애물 13m' 가 -3.0 -> -3.51 로 밀렸다).
        //
        // 더 중요한 이유: blockedSpan 과 acc_core 도 기준경로로 잰다. 세 곳이
        // 같은 물체를 같은 기준으로 봐야 "lattice 는 통과시키는데 ACC 는 세우는"
        // 2026-09-02 같은 불일치가 안 생긴다.
        const double pyaw = pathYawAt(local_path_,
                                      std::min(k, local_path_.poses.size() - 1));
        for (const auto& o : obs) {
          double d = std::hypot(p.pose.position.x - o.x, p.pose.position.y - o.y);
          if (d < halfExtent(o, pyaw) + CAR_HALF_WIDTH + SAFE_MARGIN) {  // 상자 판정
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

  // 후보 마커 지우기.
  //
  // RViz 마커는 "지워라" 고 말해주지 않으면 계속 남는다. 회피가 끝나 run() 이
  // 일찍 반환하면 publishCandidates() 가 안 불리고, 마지막 화면이 그대로 얼어붙는다.
  // 그러면 이미 지나친 장애물 때문에 아직도 전부 막힌 것처럼 보인다.
  // (2026-08-04 주행 영상에서 실제로 이렇게 보였다.)
  //
  // 이미 지운 상태면 매 틱 쏘지 않는다. 상태가 바뀔 때만 한 번 보낸다.
  void clearCandidates()
  {
    if (!cand_shown_) return;
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);
    pub_cand_.publish(arr);
    cand_shown_ = false;
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
    cand_shown_ = true;
  }

  void publishAvoidLimit(double v)
  {
    std_msgs::Float64 m; m.data = v;
    pub_avoid_.publish(m);
  }

  // 정적장애물 미션에서 "회피를 MISSION_AVOID_SPAN 안에서 끝내려면" 허용되는 속도.
  //
  // 두 단계다.
  //   ① 기동 자체가 요구하는 속도
  //        span 안에 D 를 옮길 때 횡가속도 = 6*D/span^2 * v^2 <= a
  //        -> v_avoid = sqrt(a * span^2 / (6*D))
  //        D=3.0m, span=13.0m, a=0.5G -> 6.78 m/s = 24.4 km/h
  //   ② 지금 당장 그 속도일 필요는 없다. 기동 시작점까지 d 남았으면 그동안 줄이면 된다.
  //        v_now <= sqrt(v_avoid^2 + 2*a_brake*d)
  //        50m 남으면 56km/h(사실상 무제한), 20m 40km/h, 10m 33km/h, 0m 24.4km/h
  //
  // ②가 없으면 장애물이 local_path 에 들어오는 70m 전부터 24km/h 로 기어간다.
  // acc_core 의 curvatureSpeedLimit() 과 같은 꼴이라 "이제 감속 시작" 판단이 필요 없다.
  double missionSpeedLimit(const Span& sp, double off) const
  {
    const double D = std::fabs(off);
    if (D < 1e-9) return behavior::kNoLimit;              // 회피 안 함 -> 제한할 이유 없음

    const double v_avoid = std::sqrt(AVOID_ACCEL_LIMIT * MISSION_AVOID_SPAN * MISSION_AVOID_SPAN
                                     / (6.0 * D));
    const double d = std::max(0.0, sp.mid() - MISSION_AVOID_SPAN);   // 기동 시작점까지
    return std::sqrt(v_avoid * v_avoid + 2.0 * AVOID_BRAKE_ACCEL * d);
  }

  // 복귀 곡선을 만든다. 만들 필요가 없으면(이미 차로 안) false 를 돌려주고,
  // 그때는 호출자가 기준경로 원본을 낸다.
  //
  // 좌표 프레임은 generateCandidates 와 같은 방식으로 기준경로 시작점에 잡는다.
  bool buildReturnPath(nav_msgs::Path& out)
  {
    const int n = static_cast<int>(local_path_.poses.size());
    if (n < 3) return false;

    const double sx = local_path_.poses[0].pose.position.x;
    const double sy = local_path_.poses[0].pose.position.y;
    const double nx = local_path_.poses[1].pose.position.x;
    const double ny = local_path_.poses[1].pose.position.y;
    const double theta = std::atan2(ny - sy, nx - sx);
    const double c = std::cos(theta), sn = std::sin(theta);

    auto toLocal = [&](double x, double y, double& lx, double& ly) {
      const double dx = x - sx, dy = y - sy;
      lx =  c * dx + sn * dy;
      ly = -sn * dx + c * dy;
    };
    auto toWorld = [&](double lx, double ly, double& x, double& y) {
      x = sx + c * lx - sn * ly;
      y = sy + sn * lx + c * ly;
    };

    double egox, egoy;
    toLocal(odom_.pose.pose.position.x, odom_.pose.pose.position.y, egox, egoy);

    // 차로 안으로 들어왔으면 복귀 끝. 걸어둔 목표점을 놓는다.
    if (std::fabs(egoy) < RETURN_DONE) { ret_active_ = false; return false; }

    const double v_mps = std::hypot(ego_.velocity.x, ego_.velocity.y) / 3.6;
    const std::vector<double> cum = cumulativeS();

    // 복귀 예산. 미션 회피였으면 나갈 때와 같은 0.5G 를 쓴다.
    //
    // 나갈 때 0.5G 를 허용해놓고 돌아올 때만 0.3G 로 묶으면 차선 밖에 있는 시간만
    // 길어진다. 복귀가 차선 밖에 머무는 시간은 0.62*sqrt(6D/a) 로 **속도와 무관**
    // 하고 예산에만 달렸다 - 0.3G 에서 1.53초, 0.5G 에서 1.19초다.
    // 실측(lap_return1.csv)에서 여유 초과가 3.54초였고 벌점 경계가 3.0초다.
    //
    // latch 시점에 걸어두고 복귀가 끝날 때까지 바꾸지 않는다. 매 틱 다시 판정하면
    // 복귀 도중에 예산이 바뀌어 곡선이 튄다.
    if (!ret_active_) ret_mission_ = last_mission_;
    const double a_ret = ret_mission_ ? AVOID_ACCEL_LIMIT : RETURN_ACCEL_LIMIT;

    // 목표점을 건다(latch). 한 번 걸면 도로 위에 고정이라 차를 따라오지 않는다.
    if (!ret_active_) {
      const double L = std::max(MIN_TRANSITION,
          v_mps * std::sqrt(6.0 * std::fabs(egoy) / a_ret));
      const double s_target = egox + L;
      int j = -1;
      for (int i = 0; i < n; ++i) if (cum[i] >= s_target) { j = i; break; }
      if (j < 0) return false;          // 남은 경로가 짧다. 이번 틱은 원본을 낸다
      ret_x_ = local_path_.poses[j].pose.position.x;
      ret_y_ = local_path_.poses[j].pose.position.y;
      ret_active_ = true;
      ROS_INFO("[lattice] return latched: offset %.2f m, L %.1f m, budget %.2f m/s^2 (%s)",
               egoy, L, a_ret, ret_mission_ ? "mission" : "normal");
    }

    // 걸어둔 목표점까지 남은 거리. 차가 갈수록 줄어 곡선이 가팔라진다.
    double rx, ry;
    toLocal(ret_x_, ret_y_, rx, ry);
    const double xf = rx - egox;
    if (xf < RETURN_MIN_SPAN) {         // 지나쳤거나 너무 가깝다. 놓고 다시 건다
      ret_active_ = false;
      return false;
    }

    // 3차곡선: y(0)=ps, y'(0)=0, y(xf)=pf, y'(xf)=0
    //
    // y'(0) 은 아직 0 이다. 차의 실제 헤딩을 넣는 것은 다음 단계로 분리했다.
    // 끝점 고정만으로 얼마나 좋아지는지를 먼저 재기 위해서다.
    const double ps = egoy, pf = ry;
    const double D  = pf - ps;
    const double a2 =  3.0 * D / (xf * xf);
    const double a3 = -2.0 * D / (xf * xf * xf);

    // 요구 횡가속도(최대 곡률 6|D|/xf^2). 예산을 넘으면 차가 못 따라가 오버슈트가
    // 남는다. 조용히 넘어가지 않도록 경고한다 - 회피 쪽 cand_accel_ 과 같은 취지다.
    const double need = 6.0 * std::fabs(D) / (xf * xf) * v_mps * v_mps;
    if (need > a_ret)
      ROS_WARN_THROTTLE(1.0,
          "[lattice] return needs %.2f m/s^2 (limit %.2f, span %.1fm) - overshoot likely",
          need, a_ret, xf);

    out = nav_msgs::Path();
    out.header = local_path_.header;
    for (double x = 0.0; x < xf; x += X_INTERVAL) {
      const double y = ps + a2 * x * x + a3 * x * x * x;
      double wx, wy;
      toWorld(egox + x, y, wx, wy);
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = wx;
      p.pose.position.y = wy;
      p.pose.orientation.w = 1.0;
      out.poses.push_back(p);
    }

    // 목표점 이후는 기준경로를 그대로 붙인다. pure_pursuit 이 전방주시점을
    // 찾을 만큼은 남아 있어야 한다. 목표점을 world 로 저장해두었으므로 매 틱
    // 최근접 인덱스를 다시 찾는다 - local_path 창이 앞으로 밀리기 때문이다.
    int j = 0;
    double bd = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i) {
      const double d = std::hypot(local_path_.poses[i].pose.position.x - ret_x_,
                                  local_path_.poses[i].pose.position.y - ret_y_);
      if (d < bd) { bd = d; j = i; }
    }
    for (int i = j + 1; i < n; ++i) out.poses.push_back(local_path_.poses[i]);

    return out.poses.size() > 1;
  }

  void run(const ros::TimerEvent&)
  {
    if (!(has_path_ && has_ego_ && has_obj_ && has_odom_)) return;

    // 회피를 촉발할 대상: NPC + 정적장애물 (보행자 제외 - 보행자는 정지 대상)
    std::vector<Obs> trigger_obs = gatherObstacles(false);

    // 경로 누적거리와 "처음 막히는 구간". 기동을 장애물 기준으로 배치하는 데 쓴다.
    std::vector<double> cum = cumulativeS();
    Span sp = blockedSpan(trigger_obs, cum);

    // 회피할 이유가 없으면 기준경로 그대로 (lattice 안 돌림).
    //
    // ⚠️ 2026-08-19 실패 기록. 여기서 "복귀 곡선"(차의 현재 횡위치에서 시작해
    // 기준경로로 수렴하는 경로)을 내보내 봤다. 회피가 끝나는 순간 경로가 한 틱에
    // 횡으로 점프하는 것을 없애려는 의도였다.
    //
    // 실차에서 크게 나빠졌다. 차선 이탈이 31m/8.6초 -> **103m/12.9초** 가 됐다.
    // 이유는 이 파일에서 방금 고친 것과 같은 버그였다 - 복귀 곡선의 끝점도
    // "차에서 L 앞" 이라 차를 따라 도망갔다. 매 틱 차 위치에서 다시 그려지니
    // pure_pursuit 이 보는 오차가 늘 0 에 가깝고, 그래서 되돌아오질 않았다.
    // (50km/h, 횡오차 3m 면 L=34m. 331m 에서 3.16m 였던 CTE 가 413m 에서야
    //  0.809m 아래로 내려왔다.)
    //
    // 다시 만들려면 **끝점을 도로 위 한 지점에 고정**해야 한다. 회피가 끝나는
    // 순간의 위치를 걸어두고(latch), 거기서 L 떨어진 지점을 목표로 삼아 남은
    // 거리를 매 틱 줄여가는 방식이어야 한다. 차가 다가갈수록 곡선이 가팔라져
    // 실제로 수렴한다. 지금 방식은 그 반대였다.
    //
    // 그때까지는 기준경로를 그대로 낸다. 회피 직후 한 틱 점프하고 pure_pursuit 이
    // 과보정해 CTE 가 0.2~0.5m 진동하지만(속도 비례), 차로 여유 0.809m 안이다.
    if (trigger_obs.empty() || !sp.valid) {
      nav_msgs::Path ret;
      if (buildReturnPath(ret)) pub_path_.publish(ret);
      else                      pub_path_.publish(local_path_);
      publishAvoidLimit(behavior::kNoLimit);   // 제한 없음. 지나고 나면 여기로 돌아온다
      clearCandidates();
      return;
    }

    // 회피가 다시 걸렸다. 걸어둔 복귀 목표점은 의미가 없어졌으므로 놓는다.
    ret_active_ = false;

    // 정적장애물 미션에서만 예외 처리(감속 + 횡가속 완화). 좌표로 판정한다.
    const bool mission = isMissionObstacle(trigger_obs);
    last_mission_ = mission;   // 복귀 예산을 정할 때 쓴다

    std::vector<nav_msgs::Path> cands = generateCandidates(cum, sp, mission);
    if (cands.empty()) {
      pub_path_.publish(local_path_);
      publishAvoidLimit(behavior::kNoLimit);
      clearCandidates();
      return;
    }

    // 후보 충돌 검사에는 보행자도 포함한다. 이미 회피 중이라면 보행자를 뚫고
    // 가는 후보를 고르면 안 된다.
    std::vector<Obs> obs = gatherObstacles(true);
    std::vector<bool> blocked;
    int best = selectLane(cands, obs, blocked);

    // 고른 경로가 예산을 넘게 꺾어야 하면 "제때 못 피한다" 는 뜻이다. 차가 명령을
    // 못 따라가 실제 여유가 계산보다 줄어든다.
    if (best < static_cast<int>(cand_accel_.size()) && cand_accel_[best] > cand_accel_limit_)
      ROS_WARN_THROTTLE(1.0,
          "[lattice] chosen offset needs %.2f m/s^2 (limit %.2f) - too late to avoid smoothly",
          cand_accel_[best], cand_accel_limit_);

    // 종방향 요청: 이 회피를 정해진 구간 안에서 끝내려면 얼마나 느려야 하는가.
    //
    // 미션 구간에서만 낸다. 고주로 등에서 같은 처리를 하면 고속 주행 중에 24km/h
    // 상한이 걸려 훨씬 위험하다.
    publishAvoidLimit(mission ? missionSpeedLimit(sp, LANE_OFFSET[best]) : behavior::kNoLimit);

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
