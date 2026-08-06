#!/usr/bin/env python3
"""
UDP <-> ROS ë¸Œë¦¿ì§€ (GPS/IMU/ì¹´ë©”ë¼x3/LiDAR í†µí•©íŒ)
------------------------------------------------
MORAIì™€ëŠ” UDPë¡œ, ìš°ë¦¬ ì»¨íŠ¸ë¡¤ëŸ¬/ì¸ì§€ ë…¸ë“œë“¤ê³¼ëŠ” ROS í† í”½ìœ¼ë¡œ í†µì‹ í•©ë‹ˆë‹¤.

ë°œí–‰:
  /ego_status              (morai_msgs/EgoVehicleStatus, í•„ìš”í•œ í•„ë“œë§Œ)
  /gps                     (morai_msgs/GPSMessage) - NMEA 0183(GPRMC/GPGGA)
  /imu                     (sensor_msgs/Imu) - ì‹¤ì°¨ í…ŒìŠ¤íŠ¸ë¡œ ì¶•/ë¶€í˜¸ ê²€ì¦ ì™„ë£Œ
  /camera1/image_jpeg/compressed  (sensor_msgs/CompressedImage)
  /camera2/image_jpeg/compressed  (sensor_msgs/CompressedImage)
  /camera3/image_jpeg/compressed  (sensor_msgs/CompressedImage)
  /lidar/points             (sensor_msgs/PointCloud2) - í‘œì¤€ Velodyne VLP-16 íŒ¨í‚· ë””ì½”ë”©
êµ¬ë…:
  /ctrl_cmd                (morai_msgs/CtrlCmd) -> UDPë¡œ ë³€í™˜í•´ì„œ MORAIë¡œ ì „ì†¡

ê° íŒ¨í‚· í¬ë§·ì€ udp_packet_inspector.py / camera_packet_probe.py / camera_frame_save.pyë¡œ
ì‹¤ì œ ìº¡ì²˜í•´ì„œ ê²€ì¦í•œ ê²ƒìž…ë‹ˆë‹¤ (GPS: í‘œì¤€ NMEA, IMU: ì‹¤ì°¨ í…ŒìŠ¤íŠ¸ë¡œ ì¶• ê²€ì¦ ì™„ë£Œ,
ì¹´ë©”ë¼: SOI~EOI ìž˜ë¼ë‚´ë©´ ì™„ì „í•œ JPEG í•œ ìž¥, LiDAR: í‘œì¤€ Velodyne VLP-16 1206ë°”ì´íŠ¸ í¬ë§·).
"""
import math
import socket
import struct
import threading
import numpy as np
import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage, CollisionData, ObjectStatus
from sensor_msgs.msg import Imu, CompressedImage, PointCloud2, PointField
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header, String

DEST_IP = "127.0.0.1"
CTRL_CMD_PORT = 9093          # MORAI Network Settingsì˜ Host PORTëž‘ ì¼ì¹˜í•´ì•¼ í•¨
EGO_INFO_RECV_PORT = 9111     # [개발검증용 임시] 규정제출시 9109(Competition)로 복구!
GPS_RECV_PORT = 2503
IMU_RECV_PORT = 2505
CAMERA_PORTS = {1: 2507, 2: 2509, 3: 2511}
LIDAR_RECV_PORT = 2501
# CollisionData 수신 포트. 규정 허용 채널이라 써도 된다.
# MORAI Network Settings 의 CollisionData 항목 Destination PORT 와 맞출 것.
# 0 으로 두면 이 기능을 끈다(항목을 안 켰을 때 포트만 점유하지 않도록).
COLLISION_RECV_PORT = 9092

STEER_RATIO_CORRECTION = 0.70
STEER_SIGN = 1.0

# ---- VLP-16 í‘œì¤€ ìŠ¤íŽ™ (Velodyne ê³µì‹ ì±„ë„ë³„ ìˆ˜ì§ê°ë„, ê³µê°œëœ ê°’) ----
VLP16_VERTICAL_ANGLES_DEG = [
    -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15
]
VLP16_VERTICAL_ANGLES_RAD = [math.radians(a) for a in VLP16_VERTICAL_ANGLES_DEG]


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
            # MGeo 링크 ID (12바이트 ASCII, 예: "A2256W000748").
            #
            # 규정 허용 채널인 9109(181B)에도 실려 온다 - 2026-08-06 실측 확인.
            # 시뮬이 "지금 이 링크 위에 있다" 를 직접 알려주므로 최근접 탐색이 필요 없다.
            #
            # 지금은 위치가 정확해서(개발 중 9111 = ground truth) 최근접 탐색으로도
            # 충분하다. 경로 위 40지점 표본에서 1순위/2순위 링크 거리차 중앙값이
            # 3.50m(차로폭)라 위치오차 0.171m 로는 안전하다.
            #
            # 값어치가 나오는 곳은 두 군데다.
            #   - 교차로·분기점: 40지점 중 5곳은 거리차가 1m 미만이다. 위치 오차 0.5m 만으로
            #     다른 링크를 고르는데, 하필 신호등·합류가 있는 곳이라 related_signal 이나
            #     can_move_*_lane 을 잘못 읽으면 판단이 틀린다.
            #   - 이상 감지: path_manager 의 최근접 idx 가 가리키는 링크와 이 값이 다르면
            #     "위치 추정이 틀렸다" 를 직접 알 수 있다.
            #
            # 주의: GPS 음영구간에는 도움이 안 된다. 118m 를 지나는 동안 링크 전이가
            # 2회뿐이고 그중 하나가 119m 짜리라 사실상 내내 같은 링크다. 추측항법 오차를
            # 잡는 용도로 쓸 수 없다.
            link_id = raw_data[141:153].decode(errors="ignore").rstrip('\x00').strip()
            return {
                "gear": gear, "pos": (pos_x, pos_y, pos_z), "yaw_deg": yaw,
                "vel": (vel_x, vel_y, vel_z), "ang_vel_z_deg_s": ang_vel_z,
                "front_steer_deg": front_steer, "link_id": link_id,
            }
        except (struct.error, IndexError):
            return None


class GpsReceiverUDP:
    """NMEA 0183 í…ìŠ¤íŠ¸(GPRMC/GPGGA)."""

    def __init__(self, ip, port, callback):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self._callback = callback
        self._last_alt = 0.0
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not rospy.is_shutdown():
            raw_data, _ = self.sock.recvfrom(65535)
            for line in raw_data.decode(errors="ignore").strip().split('\r\n'):
                self._parse_line(line.strip())

    @staticmethod
    def _nmea_to_deg(dm_str, direction):
        if not dm_str or '.' not in dm_str:
            return None
        dot = dm_str.index('.')
        deg_len = dot - 2
        deg = float(dm_str[:deg_len])
        minutes = float(dm_str[deg_len:])
        val = deg + minutes / 60.0
        if direction in ('S', 'W'):
            val = -val
        return val

    def _parse_line(self, line):
        if not line.startswith('$'):
            return
        fields = line.split(',')
        sentence = fields[0][1:]
        try:
            if sentence == 'GPRMC' and len(fields) >= 7 and fields[2] == 'A':
                lat = self._nmea_to_deg(fields[3], fields[4])
                lon = self._nmea_to_deg(fields[5], fields[6])
                if lat is not None and lon is not None:
                    self._callback(lat, lon, self._last_alt)
            elif sentence == 'GPGGA' and len(fields) >= 10:
                lat = self._nmea_to_deg(fields[2], fields[3])
                lon = self._nmea_to_deg(fields[4], fields[5])
                if fields[9]:
                    self._last_alt = float(fields[9])
                if lat is not None and lon is not None:
                    self._callback(lat, lon, self._last_alt)
        except (ValueError, IndexError):
            pass


class ImuReceiverUDP:
    """MORAI ì»¤ìŠ¤í…€ ë°”ì´ë„ˆë¦¬ IMU íŒ¨í‚·. ì‹¤ì°¨ í…ŒìŠ¤íŠ¸ë¡œ ì¶•/ë¶€í˜¸ ê²€ì¦ ì™„ë£Œ
    (ì •ì§€ì‹œ linear_acceleration.z=9.81, ì¢ŒíšŒì „ì‹œ angular_velocity.z ì–‘ìˆ˜ = ENU ì •í•©)."""
    HEADER = '#IMUData$'

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
            if raw_data[0:9].decode(errors="ignore") != self.HEADER:
                return None
            secs, nsecs = struct.unpack('<II', raw_data[25:33])
            qw, qx, qy, qz = struct.unpack('<dddd', raw_data[33:65])
            avx, avy, avz = struct.unpack('<ddd', raw_data[65:89])
            lax, lay, laz = struct.unpack('<ddd', raw_data[89:113])
            return {
                "secs": secs, "nsecs": nsecs,
                "quat_wxyz": (qw, qx, qy, qz),
                "angular_velocity": (avx, avy, avz),
                "linear_acceleration": (lax, lay, laz),
            }
        except (struct.error, IndexError):
            return None


class CameraReceiverUDP:
    """모라이 공식 카메라 UDP 프로토콜 (26.R1 기준, Timestamp 필드 추가된 버전).
    프레임 하나가 여러 UDP 패킷(조각)으로 나뉘어 올 수 있어서 Index/Size/Tail로 재조립한다.

    패킷 구조 (총 65000 byte 고정):
      Header(3B)="MOR" + Timestamp(8B) + Index(4B) + Size(4B)
      + Partial JPEG Data(64979B, 뒤는 0 패딩, 앞 Size바이트만 유효) + Tail(2B)="AI"/"EI"(마지막 조각)
    """

    HEADER = b'MOR'
    DATA_OFFSET = 19       # 3(header) + 8(timestamp) + 4(index) + 4(size)
    DATA_FIELD_LEN = 64979
    TAIL_LAST = b'EI'

    def __init__(self, ip, port, callback):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind((ip, port))
        self._callback = callback
        self._chunks = {}
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not rospy.is_shutdown():
            raw_data, _ = self.sock.recvfrom(70000)
            self._on_packet(raw_data)

    def _on_packet(self, data):
        if len(data) < self.DATA_OFFSET or data[0:3] != self.HEADER:
            return
        try:
            index, size = struct.unpack('<ii', data[11:19])
        except struct.error:
            return
        if size < 0 or self.DATA_OFFSET + size > len(data):
            return
        tail = data[self.DATA_OFFSET + self.DATA_FIELD_LEN:self.DATA_OFFSET + self.DATA_FIELD_LEN + 2]

        if index == 0:
            self._chunks = {}
        self._chunks[index] = data[self.DATA_OFFSET:self.DATA_OFFSET + size]

        if tail == self.TAIL_LAST:
            jpeg = b''.join(self._chunks[i] for i in sorted(self._chunks))
            self._chunks = {}
            self._callback(jpeg)


class Vlp16ReceiverUDP:
    """í‘œì¤€ Velodyne VLP-16 UDP ë°ì´í„° íŒ¨í‚· (ê³µê°œ ìŠ¤íŽ™, 1206ë°”ì´íŠ¸) ë””ì½”ë”©.
    í•œ ë°”í€´(360ë„) ë‹¤ ëŒë©´(ë°©ìœ„ê°ì´ í™• ìž‘ì•„ì§€ëŠ” ì§€ì  = wrap-around) ê·¸ë•Œê¹Œì§€ ëª¨ì€
    ì ë“¤ì„ PointCloud2 í•˜ë‚˜ë¡œ ë¬¶ì–´ì„œ ë°œí–‰.

    !! CONFIRM: ì¢Œí‘œì¶•(x=ì˜¤ë¥¸ìª½,y=ì•ž,z=ìœ„) ê´€ë¡€ê°€ ìš°ë¦¬ ENU ì¢Œí‘œê³„ëž‘ ë§žëŠ”ì§€, ê·¸ë¦¬ê³ 
       ë°©ìœ„ê° ê¸°ì¤€(0ë„ê°€ ì •ë©´ì¸ì§€)ë„ ì‹¤ì œë¡œ ë¬¼ì²´ ë†“ê³  ê²€ì¦ í•„ìš”. ì§€ê¸ˆì€ Velodyne
       ê³µì‹ ë¬¸ì„œ ê¸°ì¤€ ê´€ë¡€ë¥¼ ê·¸ëŒ€ë¡œ ì‚¬ìš©."""

    PACKET_LEN = 1206
    BLOCK_LEN = 100
    NUM_BLOCKS = 12
    CHANNELS_PER_BLOCK = 32  # 16ì±„ë„ x 2 firing sequence
    FLAG = 0xEEFF

    def __init__(self, ip, port, callback):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self._callback = callback
        self._points = []
        self._prev_azimuth = None
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not rospy.is_shutdown():
            raw_data, _ = self.sock.recvfrom(65535)
            if len(raw_data) != self.PACKET_LEN:
                continue
            self._parse_packet(raw_data)

    def _parse_packet(self, data):
        for b in range(self.NUM_BLOCKS):
            off = b * self.BLOCK_LEN
            flag = struct.unpack('<H', data[off:off + 2])[0]
            if flag != self.FLAG:
                continue
            azimuth_raw = struct.unpack('<H', data[off + 2:off + 4])[0]
            azimuth_deg = azimuth_raw / 100.0

            # íšŒì „ í•œ ë°”í€´(360ë„) ì™„ë£Œ ê°ì§€: ë°©ìœ„ê°ì´ í¬ë‹¤ê°€ ê°‘ìžê¸° í™• ìž‘ì•„ì§€ë©´ wrap
            if self._prev_azimuth is not None and azimuth_deg < self._prev_azimuth - 300:
                if self._points:
                    self._callback(self._points)
                self._points = []
            self._prev_azimuth = azimuth_deg

            az_rad = math.radians(azimuth_deg)
            # 32ê°œ ì±„ë„(16ì±„ë„ x 2 firing sequence) - ë‘ ì‹œí€€ìŠ¤ ëª¨ë‘ ì´ ë¸”ë¡ì˜ azimuthë¥¼
            # ê·¸ëŒ€ë¡œ ì”€ (ì •ë°€ ë³´ê°„ ìƒëžµ, í•„ìš”ì‹œ ë‚˜ì¤‘ì— ì •ë°€í™” ê°€ëŠ¥)
            ch_off = off + 4
            for seq in range(2):
                for ch in range(16):
                    coff = ch_off + (seq * 16 + ch) * 3
                    distance_raw = struct.unpack('<H', data[coff:coff + 2])[0]
                    reflectivity = data[coff + 2]
                    if distance_raw == 0:
                        continue  # ë¬´ë°˜ì‚¬(ì¸¡ì • ì‹¤íŒ¨)
                    distance_m = distance_raw * 0.002  # 2mm ë‹¨ìœ„
                    vert = VLP16_VERTICAL_ANGLES_RAD[ch]
                    x = distance_m * math.cos(vert) * math.sin(az_rad)
                    y = distance_m * math.cos(vert) * math.cos(az_rad)
                    z = distance_m * math.sin(vert)
                    self._points.append((x, y, z, float(reflectivity)))


class CollisionReceiverUDP:
    """MORAI CollisionData 패킷. 회피 검증 자동화용 (규정 허용 채널).

    구조 (MORAI-NetworkModule 24.R2.0 의 lib/define/CollisionData.py 기준):
        header      char*15    "#CollisionData$"
        data_lenght int
        aux_data    int*3
        sec, nsec   int, int
        _data       Data*5     객체 5개분
        tail        char*2
      Data 하나:
        objType short, obj_id short, pose_x/y/z float, globalOffset_x/y/z float

    2026-08-06 실측으로 확정됨 (포트 9092, host 9091):
        길이 181B,  헤더 "#CollisionData$",  data_length 필드 = 148
        148 = sec(4) + nsec(4) + Data(28) * 5
        offset 31 의 sec 가 유효한 유닉스 시각으로 읽히는 것까지 확인
      문서 요약의 201B/32B 는 틀렸고 패킹(정렬 padding 없음) 계산이 맞았다.

      길이가 다르면 발행하지 않고 경고만 낸다. 오프셋을 잘못 잡아 조용히
      쓰레기값을 내보내는 것이 제일 나쁘다(km/h 를 m/s 로 착각해 12.96배가 됐던
      전례가 있다). 다시 재려면:
        python3 scripts/diag_udp_dump.py --port 9092
    """
    HEADER = b'#CollisionData$'
    DATA_OFFSET = 39          # header15 + len4 + aux12 + sec4 + nsec4 (패킹 가정)
    DATA_SIZE   = 28          # short2 + short2 + float4*6
    NUM_DATA    = 5
    EXPECT_LEN  = DATA_OFFSET + DATA_SIZE * NUM_DATA + 2   # = 181

    def __init__(self, ip, port, callback):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self._callback = callback
        self._warned = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not rospy.is_shutdown():
            raw, _ = self.sock.recvfrom(65535)
            parsed = self._parse(raw)
            if parsed is not None:
                self._callback(parsed)

    def _parse(self, raw):
        if not raw.startswith(self.HEADER):
            return None
        if len(raw) != self.EXPECT_LEN:
            if not self._warned:
                self._warned = True
                rospy.logwarn(
                    "[udp_bridge] CollisionData length %d != expected %d. "
                    "Not publishing. Fix DATA_SIZE/DATA_OFFSET in CollisionReceiverUDP.",
                    len(raw), self.EXPECT_LEN)
            return None
        try:
            sec, nsec = struct.unpack('<ii', raw[31:39])
            objs = []
            for i in range(self.NUM_DATA):
                off = self.DATA_OFFSET + i * self.DATA_SIZE
                t, oid = struct.unpack('<hh', raw[off:off + 4])
                px, py, pz, gx, gy, gz = struct.unpack('<ffffff', raw[off + 4:off + 28])
                # objType 0 이고 id 0 이면 빈 슬롯으로 본다
                if t == 0 and oid == 0 and px == 0.0 and py == 0.0:
                    continue
                objs.append({'type': t, 'id': oid, 'pos': (px, py, pz),
                             'offset': (gx, gy, gz)})
            return {'sec': sec, 'nsec': nsec, 'objects': objs}
        except (struct.error, IndexError):
            return None


class UdpBridge:
    def __init__(self):
        self.ctrl = CtrlCmdUDP(DEST_IP, CTRL_CMD_PORT)

        self.ego_pub = rospy.Publisher('/ego_status', EgoVehicleStatus, queue_size=1)
        self.gps_pub = rospy.Publisher('/gps', GPSMessage, queue_size=1)
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)
        self.cam_pubs = {
            cam_id: rospy.Publisher(f'/camera{cam_id}/image_jpeg/compressed',
                                     CompressedImage, queue_size=1)
            for cam_id in CAMERA_PORTS
        }
        self.lidar_pub = rospy.Publisher('/lidar/points', PointCloud2, queue_size=1)
        self.collision_pub = rospy.Publisher('/CollisionData', CollisionData, queue_size=10)
        self.link_pub = rospy.Publisher('/ego_link_id', String, queue_size=1)

        rospy.Subscriber('/ctrl_cmd', CtrlCmd, self._on_ctrl_cmd)

        self.info_receiver = EgoInfoReceiverUDP("0.0.0.0", EGO_INFO_RECV_PORT, self._on_ego_info)
        self.gps_receiver = GpsReceiverUDP("0.0.0.0", GPS_RECV_PORT, self._on_gps)
        self.imu_receiver = ImuReceiverUDP("0.0.0.0", IMU_RECV_PORT, self._on_imu)
        self.cam_receivers = {
            cam_id: CameraReceiverUDP("0.0.0.0", port, self._make_camera_callback(cam_id))
            for cam_id, port in CAMERA_PORTS.items()
        }
        self.lidar_receiver = Vlp16ReceiverUDP("0.0.0.0", LIDAR_RECV_PORT, self._on_lidar)
        self.collision_receiver = None
        if COLLISION_RECV_PORT:
            self.collision_receiver = CollisionReceiverUDP(
                "0.0.0.0", COLLISION_RECV_PORT, self._on_collision)

        rospy.loginfo("[udp_bridge] started (ego_info:%d gps:%d imu:%d cameras:%s lidar:%d)",
                      EGO_INFO_RECV_PORT, GPS_RECV_PORT, IMU_RECV_PORT,
                      CAMERA_PORTS, LIDAR_RECV_PORT)

    def _on_collision(self, data):
        """충돌을 /CollisionData 로 발행하고 한 번 로그를 남긴다.

        회피 검증 자동화용이다. 예전에는 RViz 를 보면서 "부딪혔나?" 를 눈으로
        판단해야 했다. 규정상 충돌은 1회 15초라 완주 시간에 직접 들어간다.
        """
        out = CollisionData()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = 'map'
        for o in data['objects']:
            os_ = ObjectStatus()
            os_.unique_id = o['id']
            os_.type = o['type']
            os_.position = Vector3(*o['pos'])
            out.collision_object.append(os_)
        if data['objects']:
            g = data['objects'][0]['offset']
            out.global_offset_x, out.global_offset_y, out.global_offset_z = g
            # ROS_INFO 포맷에 한글을 쓰면 컨테이너 로케일 때문에 깨진다
            rospy.logwarn('[udp_bridge] COLLISION x%d at (%.2f, %.2f)',
                          len(data['objects']),
                          data['objects'][0]['pos'][0], data['objects'][0]['pos'][1])
        self.collision_pub.publish(out)

    def _on_ctrl_cmd(self, msg):
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

        # MGeo 링크 ID. EgoVehicleStatus 에 담을 자리가 없어 별도 토픽으로 낸다.
        # 소비자는 link_set.json 에서 max_speed / can_move_*_lane / related_signal 등을
        # 바로 조회할 수 있다.
        if data.get("link_id"):
            self.link_pub.publish(String(data=data["link_id"]))

    def _on_gps(self, lat, lon, alt):
        out = GPSMessage()
        out.header.stamp = rospy.Time.now()
        out.latitude = lat
        out.longitude = lon
        out.altitude = alt
        out.eastOffset = 0.0
        out.northOffset = 0.0
        out.status = 1
        self.gps_pub.publish(out)

    def _on_imu(self, data):
        out = Imu()
        out.header.stamp = rospy.Time(secs=data["secs"], nsecs=data["nsecs"])
        qw, qx, qy, qz = data["quat_wxyz"]
        out.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        out.angular_velocity = Vector3(*data["angular_velocity"])
        out.linear_acceleration = Vector3(*data["linear_acceleration"])
        self.imu_pub.publish(out)

    def _make_camera_callback(self, cam_id):
        def _cb(jpeg_bytes):
            out = CompressedImage()
            out.header.stamp = rospy.Time.now()
            out.format = "jpeg"
            out.data = jpeg_bytes
            self.cam_pubs[cam_id].publish(out)
        return _cb

    def _on_lidar(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "lidar"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        arr = np.array(points, dtype=np.float32)
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 16  # 4 floats x 4 bytes
        cloud.row_step = cloud.point_step * len(points)
        cloud.is_dense = True
        cloud.data = arr.tobytes()
        self.lidar_pub.publish(cloud)


def main():
    rospy.init_node('udp_bridge')
    UdpBridge()
    rospy.spin()


if __name__ == '__main__':
    main()
