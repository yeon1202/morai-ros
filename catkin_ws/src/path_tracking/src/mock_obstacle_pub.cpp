// mock_obstacle_pub : perception 팀 데이터 오기 전, 가짜 장애물을 발행하는 개발용 노드
// 발행: /Object_topic (morai_msgs/ObjectStatusList)  - 실제 perception과 동일 인터페이스
// 대회 시나리오의 정적 장애물을 그대로 발행한다.
//
// 값 출처: SaveFile/Scenario/R_KR_PR_K-city_2025/2026_molit_comp_sample_scene.json
//   objectList[0]  pos (-60.610, -142.178, 28.374)  rot.yaw -83.219  scale (2, 3, 2)
// 배포 전역경로 기준 328.9m 지점, 경로에서 0.79m 이격.
//
// 예전에는 (-115.5, -338.5) 에 2x2x1.5 로 임의의 값을 넣었다. 시뮬에는 대응하는
// 실물이 없어서 "계산은 맞는데 실제로 통과하나" 를 확인할 수 없었다. 이제 시나리오를
// 로드하면 같은 자리에 실물이 있으므로, 회피 폭이 부족하면 실제로 부딪힌다.
//
// perception 이 준비되면 이 노드를 끄면 된다(sim.launch 에서 한 줄). lattice 는
// /Object_topic 만 보므로 아무것도 바뀌지 않는다.
#include <ros/ros.h>
#include <morai_msgs/ObjectStatusList.h>
#include <morai_msgs/ObjectStatus.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mock_obstacle_pub");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<morai_msgs::ObjectStatusList>("/Object_topic", 1);
  ros::Rate rate(20);   // 20Hz

  ROS_INFO("[mock_obstacle_pub] 가짜 장애물 발행 시작 -> /Object_topic");

  while (ros::ok())
  {
    morai_msgs::ObjectStatusList msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    // --- 정적 장애물 1개 (경로 위) ---
    morai_msgs::ObjectStatus obs;
    obs.unique_id = 1;
    obs.type = 2;                 // 0:보행자 1:NPC 2:정적장애물
    obs.name = "mock_static";
    obs.position.x = -60.610;     // 시나리오 objectList[0].pos (ENU, 맵 좌표)
    obs.position.y = -142.178;
    obs.position.z = 28.374;
    // size 규약: x=length(주축), y=width(부축), z=height.
    // 2026-08-27 에 팀 perception 의 RecognizedObject 규약으로 통일했다
    // (예전에는 x=width, y=length 로 반대였다). 시나리오 scale (2, 3, 2) 에서
    // 길이가 3, 폭이 2 다.
    // ※ lattice 는 0.5*max(x,y) 외접원이라 이 교체로 충돌 판정은 안 바뀐다.
    //   바뀌는 것은 object_viz 가 그리는 모양뿐이고, 이제 실제 인지와 같게 보인다.
    obs.size.x = 3.0;             // length [m]
    obs.size.y = 2.0;             // width  [m]
    obs.size.z = 2.0;             // height [m]
    // 충돌 검사는 0.5*max(size.x, size.y) 로 외접원을 쓰므로 heading 과 무관하게
    // 보수적으로 판정된다. 그래도 시각화/디버깅을 위해 실제 값을 넣어둔다.
    obs.heading = -83.219;        // [deg] 시나리오 rot.yaw

    msg.obstacle_list.push_back(obs);
    msg.num_of_obstacle = 1;
    msg.num_of_npcs = 0;
    msg.num_of_pedestrian = 0;

    pub.publish(msg);
    rate.sleep();
  }
  return 0;
}
