// object_viz : /Object_topic 을 받아 RViz 마커로 표시 (perception 실제 데이터 와도 계속 사용)
// 구독: /Object_topic (morai_msgs/ObjectStatusList)
// 발행: /object_markers (visualization_msgs/MarkerArray)  - 타입별 색: 보행자=빨강 NPC=주황 장애물=노랑
//
// size 규약 (2026-08-27 통일): x=length(주축), y=width(부축), z=height.
//   팀 perception 의 RecognizedObject 가 이 규약이라 거기에 맞췄다. 예전에는
//   mock_obstacle_pub 이 x=width, y=length 로 반대였는데 그쪽을 같이 고쳤다.
//   ※ lattice_planner 는 0.5*max(x,y) 외접원을 쓰므로 순서와 무관하다. 이 규약이
//     실제로 영향을 주는 곳은 "그리기" 뿐이다.
#include <cmath>

#include <ros/ros.h>
#include <morai_msgs/ObjectStatusList.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;

static visualization_msgs::Marker makeMarker(const morai_msgs::ObjectStatus& obj,
                                             int id, float r, float g, float b)
{
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = "objects";
  m.id = id;
  m.type = visualization_msgs::Marker::CUBE;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = obj.position.x;
  m.pose.position.y = obj.position.y;
  m.pose.position.z = obj.position.z;
  // heading 을 z 축 회전으로 준다. 예전에는 orientation.w=1.0 (회전 없음) 이라
  // 비스듬히 놓인 물체가 축정렬 박스로 그려져서, 라이다 클러스터 마커
  // (lidar_node 는 yaw 를 준다) 와 모양이 달라 보였다. 주행에는 영향이 없지만
  // 눈으로 검증할 때 헷갈린다.
  const double heading_rad = obj.heading * M_PI / 180.0;   // ObjectStatus.heading 은 [deg]
  m.pose.orientation.z = std::sin(heading_rad / 2.0);
  m.pose.orientation.w = std::cos(heading_rad / 2.0);

  m.scale.x = obj.size.x > 0.1 ? obj.size.x : 1.0;   // length (주축)
  m.scale.y = obj.size.y > 0.1 ? obj.size.y : 1.0;   // width  (부축)
  m.scale.z = obj.size.z > 0.1 ? obj.size.z : 1.0;   // height
  m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.8;
  m.lifetime = ros::Duration(0.5);
  return m;
}

void callback(const morai_msgs::ObjectStatusList::ConstPtr& msg)
{
  visualization_msgs::MarkerArray arr;
  // 지난 프레임 마커 싹 지우고 다시 그림
  visualization_msgs::Marker del;
  del.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(del);

  int id = 0;
  for (const auto& o : msg->npc_list)         arr.markers.push_back(makeMarker(o, id++, 1.0, 0.6, 0.0)); // 주황
  for (const auto& o : msg->pedestrian_list)  arr.markers.push_back(makeMarker(o, id++, 1.0, 0.0, 0.0)); // 빨강
  for (const auto& o : msg->obstacle_list)    arr.markers.push_back(makeMarker(o, id++, 1.0, 1.0, 0.0)); // 노랑

  marker_pub.publish(arr);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_viz");
  ros::NodeHandle nh;
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/object_markers", 1);
  ros::Subscriber sub = nh.subscribe("/Object_topic", 1, callback);
  ROS_INFO("[object_viz] /Object_topic -> /object_markers");
  ros::spin();
  return 0;
}
