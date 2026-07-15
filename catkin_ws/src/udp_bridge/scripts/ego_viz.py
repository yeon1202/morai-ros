#!/usr/bin/env python3
"""
/ego_status (morai_msgs/EgoVehicleStatus) -> RViz 시각화용 변환 노드
------------------------------------------------------------------
발행:
  TF  map -> base_link   (차량 위치/방향)
  /ego_marker  (visualization_msgs/Marker) 차량 박스
  /ego_path    (nav_msgs/Path) 지나온 궤적
RViz: Fixed Frame=map, Marker(/ego_marker) / Path(/ego_path) / TF 추가
"""
import math
import rospy
import tf
from morai_msgs.msg import EgoVehicleStatus
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class EgoViz:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.marker_pub = rospy.Publisher('/ego_marker', Marker, queue_size=1)
        self.path_pub = rospy.Publisher('/ego_path', Path, queue_size=1)
        self.path = Path()
        self.path.header.frame_id = 'map'
        rospy.Subscriber('/ego_status', EgoVehicleStatus, self.cb)
        rospy.loginfo('[ego_viz] started')

    def cb(self, msg):
        now = rospy.Time.now()
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        yaw = math.radians(msg.heading)
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

        # TF: map -> base_link
        self.br.sendTransform((x, y, z), q, now, 'base_link', 'map')

        # 차량 박스 마커 (base_link 기준, Ioniq5 대략 크기)
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = now
        m.ns = 'ego'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.z = 0.75
        m.scale.x, m.scale.y, m.scale.z = 4.6, 1.9, 1.5
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.6, 1.0, 0.9
        self.marker_pub.publish(m)

        # 궤적
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = now
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = q
        self.path.header.stamp = now
        self.path.poses.append(ps)
        if len(self.path.poses) > 3000:
            self.path.poses.pop(0)
        self.path_pub.publish(self.path)


if __name__ == '__main__':
    rospy.init_node('ego_viz')
    EgoViz()
    rospy.spin()
