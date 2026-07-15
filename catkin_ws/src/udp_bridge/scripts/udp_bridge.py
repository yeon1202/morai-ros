#!/usr/bin/env python3
"""
UDP <-> ROS 브릿지
------------------------------------------------
MORAI와는 UDP로, 우리 컨트롤러 노드들과는 ROS 토픽으로 통신합니다.

발행: /ego_status  (morai_msgs/EgoVehicleStatus 재사용, 필요한 필드만 채움)
구독: /ctrl_cmd     (morai_msgs/CtrlCmd) -> UDP로 변환해서 MORAI로 전송
"""

import math
import socket
import struct
import threading

import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from geometry_msgs.msg import Vector3

DEST_IP = "127.0.0.1"
CTRL_CMD_PORT = 9093          # MORAI Network Settings의 Host PORT랑 일치해야 함
EGO_INFO_RECV_PORT = 9111     # MORAI가 우리한테 보내는 목적지 포트

STEER_RATIO_CORRECTION = 0.70
STEER_SIGN = 1.0


class CtrlCmdUDP:
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (ip, port)
        message_name = '#MoraiCtrlCmd$'.encode()
        data_length = struct.pack('i', 23)
        aux_data = struct.pack('iii', 0, 0, 0)
        self.header = message_name + data_length + aux_data
        self.tail = '\r\n'.encode()

    def send(self, accel, brake, front_steer_rad, gear=4, mode=2, cmd_type=1,
             velocity=0.0, acceleration=0.0):
        body = (struct.pack('b', mode) + struct.pack('b', gear) + struct.pack('b', cmd_type) +
                struct.pack('f', velocity) + struct.pack('f', acceleration) +
                struct.pack('f', accel) + struct.pack('f', brake) + struct.pack('f', front_steer_rad))
        self.sock.sendto(self.header + body + self.tail, self.address)


class EgoInfoReceiverUDP:
    HEADER = '#MoraiInfo$'

    def __init__(self, ip, port, callback):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self._callback = callback
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not rospy.is_shutdown():
            raw_data, _ = self.sock.recvfrom(65535)
            parsed = self._parse(raw_data)
            if parsed is not None:
                self._callback(parsed)

    def _parse(self, raw_data):
        try:
            if self.HEADER != raw_data[0:11].decode(errors="ignore"):
                return None
            gear = struct.unpack('b', raw_data[36:37])[0]
            pos_x, pos_y, pos_z = struct.unpack('fff', raw_data[77:89])
            roll, pitch, yaw = struct.unpack('fff', raw_data[89:101])
            vel_x, vel_y, vel_z = struct.unpack('fff', raw_data[101:113])
            ang_vel_x, ang_vel_y, ang_vel_z = struct.unpack('fff', raw_data[113:125])
            front_steer = struct.unpack('f', raw_data[137:141])[0]
            return {
                "gear": gear, "pos": (pos_x, pos_y, pos_z), "yaw_deg": yaw,
                "vel": (vel_x, vel_y, vel_z), "ang_vel_z_deg_s": ang_vel_z,
                "front_steer_deg": front_steer,
            }
        except (struct.error, IndexError):
            return None


class UdpBridge:
    def __init__(self):
        self.ctrl = CtrlCmdUDP(DEST_IP, CTRL_CMD_PORT)
        self.ego_pub = rospy.Publisher('/ego_status', EgoVehicleStatus, queue_size=1)
        rospy.Subscriber('/ctrl_cmd', CtrlCmd, self._on_ctrl_cmd)
        self.info_receiver = EgoInfoReceiverUDP("0.0.0.0", EGO_INFO_RECV_PORT, self._on_ego_info)
        rospy.loginfo("[udp_bridge] started")

    def _on_ctrl_cmd(self, msg):
        # 컨트롤러가 이미 라디안으로 계산해서 보낸다고 가정, 여기서 실측 보정만 적용
        deg = math.degrees(msg.front_steer)
        corrected_deg = STEER_SIGN * deg / STEER_RATIO_CORRECTION
        steer_rad = math.radians(corrected_deg)
        self.ctrl.send(
            accel=msg.accel, brake=msg.brake, front_steer_rad=steer_rad,
            gear=4, mode=2, cmd_type=int(msg.longlCmdType),
        )

    def _on_ego_info(self, data):
        out = EgoVehicleStatus()
        out.header.stamp = rospy.Time.now()
        out.position = Vector3(*data["pos"])
        out.velocity = Vector3(*data["vel"])
        out.heading = data["yaw_deg"]
        out.front_steer_angle = data["front_steer_deg"]
        self.ego_pub.publish(out)


def main():
    rospy.init_node('udp_bridge')
    UdpBridge()
    rospy.spin()


if __name__ == '__main__':
    main()
