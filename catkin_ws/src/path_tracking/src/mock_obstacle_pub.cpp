// mock_obstacle_pub : perception 팀 데이터 오기 전, 가짜 장애물을 발행하는 개발용 노드
// 발행: /Object_topic (morai_msgs/ObjectStatusList)  - 실제 perception과 동일 인터페이스
// 경로 100m 지점(-115.5, -338.5)에 정적 장애물 1개를 계속 발행한다.
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
    obs.position.x = -115.5;      // 경로 100m 지점 (ENU, 전역좌표)
    obs.position.y = -338.5;
    obs.position.z = 28.7;
    obs.size.x = 2.0;             // width  [m]
    obs.size.y = 2.0;             // length [m]
    obs.size.z = 1.5;             // height [m]
    obs.heading = 0.0;

    msg.obstacle_list.push_back(obs);
    msg.num_of_obstacle = 1;
    msg.num_of_npcs = 0;
    msg.num_of_pedestrian = 0;

    pub.publish(msg);
    rate.sleep();
  }
  return 0;
}
