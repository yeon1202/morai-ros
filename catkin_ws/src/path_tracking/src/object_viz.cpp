// object_viz : /Object_topic 을 받아 RViz 마커로 표시 (perception 실제 데이터 와도 계속 사용)
// 구독: /Object_topic (morai_msgs/ObjectStatusList)
// 발행: /object_markers (visualization_msgs/MarkerArray)  - 타입별 색: 보행자=빨강 NPC=주황 장애물=노랑
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
  m.pose.orientation.w = 1.0;
  m.scale.x = obj.size.x > 0.1 ? obj.size.x : 1.0;
  m.scale.y = obj.size.y > 0.1 ? obj.size.y : 1.0;
  m.scale.z = obj.size.z > 0.1 ? obj.size.z : 1.0;
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
