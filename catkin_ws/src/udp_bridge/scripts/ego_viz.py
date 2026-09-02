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

        # TF: map -> base_link_gt   (GT 기준 차량 위치)
        #
        # !! base_link 로 내면 안 된다 (2026-08-27 수정) !!
        #   ekf_localization_node 가 odom -> base_link 를 내고 있어서, 여기서
        #   map -> base_link 를 또 내면 base_link 의 부모가 둘이 된다. tf 트리는
        #   프레임마다 부모가 하나여야 하므로, 두 곳이 각자 갱신하면 RViz 의 차가
        #   GT 위치와 EKF 위치 사이를 왔다갔다 뛴다(실측 진행방향 차이 -1.4~+1.5m).
        #
        #   base_link 는 EKF 가 소유한다 - 팀 perception 의 global_transform_node 가
        #   lidar -> base_link -> odom 사슬로 좌표를 바꾸기 때문에, 그 사슬 안의
        #   base_link 는 odom(=EKF 세계)과 같은 기준이어야 앞뒤가 맞는다.
        #
        #   덤: base_link_gt 와 base_link 를 RViz 에 같이 띄우면 그 간격이 곧
        #   EKF 오차다. 눈으로 바로 보인다.
        self.br.sendTransform((x, y, z), q, now, 'base_link_gt', 'map')

        # 차량 박스 마커 (base_link 기준 = EKF 추정 위치, Ioniq5 대략 크기).
        # 마커가 프레임 이름을 직접 들고 다니므로 RViz 설정은 안 고쳐도 된다.
        #
        # 위 tf 는 base_link_gt 로 내면서 마커만 base_link 인 게 모순처럼 보이지만
        # 아니다. tf 는 "프레임을 소유하고 갱신하는" 행위라 부모가 하나여야 하지만,
        # 마커의 frame_id 는 "그 프레임을 읽어서 거기 그려달라"는 부탁일 뿐이다.
        # 읽는 건 몇 명이 해도 충돌하지 않는다. base_link 는 EKF 가 계속 소유한다.
        #
        # stamp 를 now 가 아니라 Time(0) 으로 두는 이유:
        #   base_link_gt 는 바로 위에서 우리가 같은 now 로 냈으니 항상 맞아떨어졌다.
        #   base_link 는 EKF 소유라 갱신 시각이 우리와 다르다(실측 EKF 11~20Hz,
        #   /ego_status 20~27Hz). now 로 두면 아직 안 나온 미래 tf 를 요구하게 돼
        #   RViz 가 마커를 버리고 상자가 깜빡인다. Time(0) 은 "가장 최근 tf 를 써라"
        #   라는 뜻이라 이 문제가 안 생긴다.
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = rospy.Time(0)
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
