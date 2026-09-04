#!/usr/bin/env python3
"""
UDP <-> ROS 브릿지 (GPS/IMU/카메라x3/LiDAR 통합판)
------------------------------------------------
MORAI와는 UDP로, 우리 컨트롤러/인지 노드들과는 ROS 토픽으로 통신합니다.

발행:
  /ego_status              (morai_msgs/EgoVehicleStatus, 필요한 필드만)
  /gps                     (morai_msgs/GPSMessage) - NMEA 0183(GPRMC/GPGGA)
  /imu                     (sensor_msgs/Imu) - 실차 테스트로 축/부호 검증 완료
  /camera1/image_jpeg/compressed  (sensor_msgs/CompressedImage)
  /camera2/image_jpeg/compressed  (sensor_msgs/CompressedImage)
  /camera3/image_jpeg/compressed  (sensor_msgs/CompressedImage)
  /camera4/image_jpeg/compressed  (sensor_msgs/CompressedImage) - 차선 인지 전용
  /lidar/points             (sensor_msgs/PointCloud2) - 표준 Velodyne VLP-16 패킷 디코딩,
                            포인트별 정밀시각 기반 모션 왜곡(motion distortion) 보정 포함
  ※ 아래 둘은 팀 원본에 없고 이 작업본에만 있다:
  /CollisionData           (morai_msgs/CollisionData) - 충돌 감지, 회피 검증 자동화용
  /ego_link_id             (std_msgs/String) - MGeo 링크 ID
구독:
  /ctrl_cmd                (morai_msgs/CtrlCmd) -> UDP로 변환해서 MORAI로 전송

각 패킷 포맷은 udp_packet_inspector.py / camera_packet_probe.py / camera_frame_save.py로
실제 캡처해서 검증한 것입니다 (GPS: 표준 NMEA, IMU: 실차 테스트로 축 검증 완료,
카메라: Index/Size/Tail로 조각 재조립(SD는 조각 1개, HD는 여러 개),
LiDAR: 표준 Velodyne VLP-16 1206바이트 포맷).
"""
import math
import queue
import socket
import struct
import threading
from collections import deque
import numpy as np
import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage, CollisionData, ObjectStatus
from sensor_msgs.msg import Imu, CompressedImage, PointCloud2, PointField
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header, String

# ⚠️ 이 값은 PC 마다 다르다. MORAI 시뮬이 도는 머신의 주소를 넣는다.
#   같은 머신(도커 host 네트워크)  -> 127.0.0.1
#   시뮬이 별도 VM/PC             -> 그 머신의 주소 (예: 192.168.56.101)
#   woonggook 브랜치는 192.168.56.101 이고 여기는 127.0.0.1 이라 머지 때마다
#   충돌한다. 코드가 틀린 게 아니라 환경 차이다.
DEST_IP = "127.0.0.1"
CTRL_CMD_PORT = 9093          # MORAI Network Settings의 Host PORT랑 일치해야 함
# 9109 = Competition Vehicle Status (대회 규정 허용 채널). position 이 0,0,0 으로 온다.
# 9111 = Ego Vehicle Status (ground truth). 개발 검증용이고 제출본에 쓰면 실격이다.
#
# !!! 2026-08-29 9111 -> 9109 로 전환 !!!
# 이제 planning 이 위치를 /ego_status 가 아니라 /odom(GPS+IMU 융합)에서 받는다.
# 9109 는 velocity 는 그대로 주므로(localization_node 의 EgoSpeedPreprocessor 가
# 대회채널에서도 쓴다고 확인됨) 속도계 경로는 영향 없다.
#
# ⚠️ 되돌릴 때: 진단 도구(diag_latency/analyze_latency)는 GT 위치가 있어야 오차를
#   잴 수 있다. 측정할 때만 9111 로 바꾸고, 제출본은 반드시 9109 여야 한다.
#
# 2026-09-03: 상수를 rosparam 으로 뺐다.
#   손으로 9111 <-> 9109 를 오가면 언젠가 9111 인 채로 제출하게 된다.
#   기본값을 규정 채널(9109)로 두고, 측정할 때만 넘긴다:
#       rosrun udp_bridge udp_bridge.py _ego_info_port:=9111
EGO_INFO_RECV_PORT_DEFAULT = 9109
GPS_RECV_PORT = 2503
IMU_RECV_PORT = 2505
CAMERA_PORTS = {1: 2507, 2: 2509, 3: 2511, 4: 2513}
#   4번은 차선 인지가 쓰는 카메라다. 팀 repo 의 yeonsoo 브랜치가
#   /camera4/image_jpeg/compressed 를 구독하므로 여기서 안 열면 인지가 아무것도 못 받는다.
#   포트는 MORAI Network Settings 의 카메라4 Host PORT 와 일치해야 한다.
LIDAR_RECV_PORT = 2501
# 충돌 감지 (회피 검증 자동화용, 규정 허용 채널). 0 이면 수신 안 함.
COLLISION_RECV_PORT = 9092

STEER_RATIO_CORRECTION = 0.70
STEER_SIGN = 1.0

# ---- VLP-16 표준 스펙 (Velodyne 공식 채널별 수직각도, 공개된 값) ----
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
            link_id = raw_data[141:153].decode(errors="ignore").rstrip('\x00').strip()
            return {
                "gear": gear, "pos": (pos_x, pos_y, pos_z), "yaw_deg": yaw,
                "vel": (vel_x, vel_y, vel_z), "ang_vel_z_deg_s": ang_vel_z,
                "front_steer_deg": front_steer, "link_id": link_id,
            }
        except (struct.error, IndexError):
            return None


class GpsReceiverUDP:
    """NMEA 0183 텍스트(GPRMC/GPGGA)."""

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
    """MORAI 커스텀 바이너리 IMU 패킷. 실차 테스트로 축/부호 검증 완료
    (정지시 linear_acceleration.z=9.81, 좌회전시 angular_velocity.z 양수 = ENU 정합)."""
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
    """MORAI 공식 카메라 UDP 프로토콜 (Timestamp 필드 포함된 버전).
    프레임 하나가 여러 UDP 패킷(조각)으로 나뉘어 올 수 있어서 Index/Size/Tail로 재조립함.
    (SD 카메라처럼 조각이 1개뿐이면 바로 합쳐지고, HD처럼 여러 개면 순서대로 모아서 합침)

    패킷 구조 (총 65000바이트 고정):
      Header(3B)="MOR" + Timestamp(8B) + Index(4B) + Size(4B)
      + Partial JPEG Data(64979B, 앞에서 Size바이트만 유효, 나머지는 0 패딩)
      + Tail(2B) - 마지막 조각이면 "EI"
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
            self._chunks = {}  # 새 프레임 시작 -> 이전에 덜 모인 조각은 버림
        self._chunks[index] = data[self.DATA_OFFSET:self.DATA_OFFSET + size]

        if tail == self.TAIL_LAST:
            jpeg = b''.join(self._chunks[i] for i in sorted(self._chunks))
            self._chunks = {}
            self._callback(jpeg)


class Vlp16ReceiverUDP:
    """표준 Velodyne VLP-16 UDP 데이터 패킷 (공개 스펙, 1206바이트) 디코딩.
    한 바퀴(360도) 다 돌면(방위각이 확 작아지는 지점 = wrap-around) 그때까지 모은
    점들을 PointCloud2 하나로 묶어서 발행.

    좌표축: x=전방, y=좌측, z=위 (REP-103 오른손좌표계 표준: x×y=z).
    Velodyne 공식 문서의 raw 공식은 원래 x=sin(az)/y=cos(az)(x=오른쪽,y=앞)이지만,
    이 프로젝트 전체 관례에 맞춰 x,y 스왑 + y 부호 반전해서 사용
    (2026-08-05 x,y 스왑 확정 -> 같은 날 REP-103 정합 위해 y 부호 반전 추가 확정)."""

    PACKET_LEN = 1206
    BLOCK_LEN = 100
    NUM_BLOCKS = 12
    CHANNELS_PER_BLOCK = 32  # 16채널 x 2 firing sequence
    FLAG = 0xEEFF
    # VLP-16 싱글 리턴 모드 firing 타이밍 (공식 스펙, 모션왜곡 보정용 포인트별 정밀시각 계산에 사용).
    # 패킷당 firing sequence 24개(블록 12 x 시퀀스 2), 시퀀스 간 55.296us, 시퀀스 내 채널 간 2.304us.
    FIRING_SEQ_US = 55.296
    CHANNEL_US = 2.304
    # 2026-08-19 (팀 woonggook 브랜치): 커널 소켓 drops 카운터(/proc/net/udp)가 0으로
    # 확인됨 - 패킷이 유실되는 게 아니라 순서가 뒤바뀌어 도착해서 wrap 판정이 헷갈리는
    # 것으로 확인. 패킷 자체 타임스탬프가 이 값(500ms)보다 작게 과거로 튀면 뒤늦게 도착한
    # 뒤섞인 패킷으로 보고 통째로 버림 (한 스캔 길이가 ~100ms라 500ms면 순서 뒤바뀜이라기엔
    # 넉넉하게 큰 값 - 진짜 정시(1시간) 롤오버는 이거보다 훨씬 큰 폭으로 떨어지므로 오탐 안 함).
    REORDER_DISCARD_THRESHOLD_US = 500000

    def __init__(self, ip, port, callback):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # rmem_max(208KB)에 막혀 4MB 요청해도 실제로는 416KB로만 잡히지만, 카메라 소켓과
        # 동일하게 여유는 조금이라도 늘려둠(핵심 수정은 아래 큐 분리).
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind((ip, port))
        self._callback = callback
        self._points = []
        self._prev_azimuth = None
        self._max_timestamp_us = None
        self._blocks_since_wrap = 0
        self._recent_block_counts = deque(maxlen=8)
        # 모션왜곡 보정(callback = numpy deskew + PointCloud2 publish)이 수신 스레드 안에서
        # 동기 실행되면, 한 바퀴 끝날 때마다 그 스레드가 recvfrom을 못 부르고 멈춰있게 됨 ->
        # 그 사이 도착한 패킷이 커널 버퍼 넘쳐서 드롭(실측: 포인트 7253개 -> 최저 2624개까지
        # 튐, 정지 물체인데도 발생). 별도 스레드+큐로 분리해서 수신 스레드는 파싱만 하고
        # 항상 즉시 recvfrom으로 복귀하게 함 - 처리가 얼마나 걸리든 수신 루프엔 영향 없음.
        # 큐가 꽉 차면(처리가 못 따라가는 중) 가장 오래된 스캔을 버리고 최신 걸 우선함.
        self._scan_queue = queue.Queue(maxsize=3)
        threading.Thread(target=self._loop, daemon=True).start()
        threading.Thread(target=self._process_loop, daemon=True).start()

    def _process_loop(self):
        while not rospy.is_shutdown():
            try:
                points = self._scan_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            self._callback(points)

    def _loop(self):
        while not rospy.is_shutdown():
            raw_data, _ = self.sock.recvfrom(65535)
            if len(raw_data) != self.PACKET_LEN:
                continue
            self._parse_packet(raw_data)

    def _parse_packet(self, data):
        # 패킷 끝 6바이트(offset 1200~1205) = Timestamp(4B, 정시 기준 마이크로초) + Factory(2B,
        # return mode/product id). 값 정상 수신 확인 완료(2026-08-06, product_id=0x22=VLP-16) -
        # 이제 모션 왜곡 보정용 포인트별 정밀 시각 계산에 사용.
        timestamp_us = struct.unpack('<I', data[1200:1204])[0]

        # 이 패킷이 최근에 처리한 것보다 시간상 과거인데 그 폭이 작으면(진짜 정시 롤오버가
        # 아니면) 네트워크에서 순서가 뒤바뀌어 늦게 도착한 패킷 - 어느 스캔에 속하는지
        # 애매해서 섞이면 스캔이 깨지니 통째로 버림.
        if self._max_timestamp_us is not None:
            behind_us = self._max_timestamp_us - timestamp_us
            if 0 < behind_us < self.REORDER_DISCARD_THRESHOLD_US:
                rospy.logwarn_throttle(
                    5.0, "[Vlp16Receiver] 패킷 순서 뒤바뀜 감지(%.1fms 과거) - 버림",
                    behind_us / 1000.0)
                return
        self._max_timestamp_us = timestamp_us

        for b in range(self.NUM_BLOCKS):
            off = b * self.BLOCK_LEN
            flag = struct.unpack('<H', data[off:off + 2])[0]
            if flag != self.FLAG:
                continue
            azimuth_raw = struct.unpack('<H', data[off + 2:off + 4])[0]
            azimuth_deg = azimuth_raw / 100.0

            # 회전 한 바퀴(360도) 완료 감지: 방위각이 크다가 갑자기 확 작아지면 wrap
            if self._prev_azimuth is not None and azimuth_deg < self._prev_azimuth - 300:
                if self._points:
                    # 커널 소켓 drops=0인데도 스캔 크기가 흔들리는 게 확인됨(2026-08-19) ->
                    # 호스트에 도착하기 전에(가상 네트워크 어딘가) 패킷이 실제로 유실되는
                    # 것으로 추정 - wrap 판정용 패킷이 유실되면 다음 wrap까지 두 바퀴가
                    # 합쳐짐. 절대 개수 대신 "최근 스캔들 대비"로 판정해야 장면마다 포인트
                    # 밀도가 달라도(예: 7264개대 장면 vs 9000개대 장면) 정상 스캔까지
                    # 걸러지지 않음(고정 임계값을 썼다가 정상 스캔까지 다 버려져서 아무것도
                    # 발행 안 되는 문제를 실측으로 확인했음).
                    typical = (sorted(self._recent_block_counts)[len(self._recent_block_counts) // 2]
                               if len(self._recent_block_counts) >= 4 else None)
                    if typical is not None and self._blocks_since_wrap > typical * 1.6:
                        rospy.logwarn_throttle(
                            2.0, "[Vlp16Receiver] 스캔 병합 의심(블록 %d개, 최근 평소 %d개) - 버림",
                            self._blocks_since_wrap, typical)
                    else:
                        self._recent_block_counts.append(self._blocks_since_wrap)
                        try:
                            self._scan_queue.put_nowait(self._points)
                        except queue.Full:
                            try:
                                self._scan_queue.get_nowait()  # 가장 오래된 스캔 버리고
                            except queue.Empty:
                                pass
                            self._scan_queue.put_nowait(self._points)  # 최신 스캔 넣기
                self._points = []
                self._blocks_since_wrap = 0
            self._prev_azimuth = azimuth_deg
            self._blocks_since_wrap += 1

            az_rad = math.radians(azimuth_deg)
            # 32개 채널(16채널 x 2 firing sequence) - 두 시퀀스 모두 이 블록의 azimuth를
            # 기하 계산(각도)엔 그대로 쓰지만, 시각(t_us)은 시퀀스/채널별로 정밀 계산.
            ch_off = off + 4
            for seq in range(2):
                firing_seq_idx = b * 2 + seq  # 패킷 내 firing sequence 번호 (0~23)
                for ch in range(16):
                    coff = ch_off + (seq * 16 + ch) * 3
                    distance_raw = struct.unpack('<H', data[coff:coff + 2])[0]
                    reflectivity = data[coff + 2]
                    if distance_raw == 0:
                        continue  # 무반사(측정 실패)
                    distance_m = distance_raw * 0.002  # 2mm 단위
                    vert = VLP16_VERTICAL_ANGLES_RAD[ch]
                    # x=전방,y=좌측 관례로 스왑+부호반전 (Velodyne raw 공식은 원래 x=오른쪽,y=앞)
                    x = distance_m * math.cos(vert) * math.cos(az_rad)
                    y = -distance_m * math.cos(vert) * math.sin(az_rad)
                    z = distance_m * math.sin(vert)
                    t_us = (timestamp_us + self.FIRING_SEQ_US * firing_seq_idx
                            + self.CHANNEL_US * ch)
                    self._points.append((x, y, z, float(reflectivity), t_us))


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
        # 라이다 모션 왜곡 보정용 최신 차량 상태 (아직 /ego_status, /imu 못 받았으면 0 = 무보정).
        self._forward_speed_mps = 0.0
        self._yaw_rate_rad_s = 0.0

        self.ctrl = CtrlCmdUDP(DEST_IP, CTRL_CMD_PORT)

        # GPS 전송 지연 보정값 [초]. 아래 _on_gps 주석 참고.
        # 대회장 머신에서는 재측정해서 이 파라미터만 바꾸면 된다.
        self.gps_lag = rospy.Duration(rospy.get_param('~gps_lag_sec', 0.30))

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

        ego_port = int(rospy.get_param('~ego_info_port', EGO_INFO_RECV_PORT_DEFAULT))
        if ego_port != EGO_INFO_RECV_PORT_DEFAULT:
            # 규정 채널이 아닌 값으로 돌고 있다는 것을 크게 남긴다. 이 로그가 보이면
            # 제출본이 아니다.
            rospy.logwarn("[udp_bridge] ego_info_port=%d (DIAGNOSTIC). "
                          "Competition build must use %d.",
                          ego_port, EGO_INFO_RECV_PORT_DEFAULT)
        self.info_receiver = EgoInfoReceiverUDP("0.0.0.0", ego_port, self._on_ego_info)
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
                      ego_port, GPS_RECV_PORT, IMU_RECV_PORT,
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

        # vel_x/vel_y의 정확한 축 관례는 검증 전이라(라이다처럼 스왑/부호반전이 필요할 수도
        # 있음) 안 믿고, 크기(속력)만 써서 "거의 항상 전방으로 움직인다"는 근사로 라이다
        # 모션왜곡 보정에 사용 - 회전축 부호는 이미 검증된 IMU 쪽(_on_imu)에서 가져옴.
        # !! 팀 원본 버그 수정 (2026-08-26) !!
        # 팀 코드는 `math.hypot(vx, vy)` 를 그대로 _forward_speed_mps 에 넣는데,
        # ego UDP 패킷의 vel 은 m/s 가 아니라 **km/h** 다. 그대로 쓰면 속도를
        # 3.6배로 보고 모션왜곡 보정이 그만큼 과하게 걸린다(43km/h 주행 시
        # 점을 최대 4.3m 밀어버린다 - 정답은 1.2m). 보정을 안 하느니만 못하다.
        #
        # 근거: pilot2 로그에서 velocity.x 원값 43.00 일 때 실제 지면속도가
        # 11.50 m/s = 43/3.6 로 측정됨. 팀 localization_node.cpp 도
        # `msg->velocity.x / 3.6  // km/h -> m/s` 로 같은 변환을 한다.
        vx, vy, _ = data["vel"]
        self._forward_speed_mps = math.hypot(vx, vy) / 3.6

        # MGeo 링크 ID. EgoVehicleStatus 에 담을 자리가 없어 별도 토픽으로 낸다.
        # 소비자는 link_set.json 에서 max_speed / can_move_*_lane / related_signal 등을
        # 바로 조회할 수 있다.
        if data.get("link_id"):
            self.link_pub.publish(String(data=data["link_id"]))

    def _on_gps(self, lat, lon, alt):
        """MORAI 의 GPS 전송 지연만큼 스탬프를 과거로 찍는다.

        패킷이 우리 소켓에 도착한 시각은 그 위경도가 "측정된" 시각이 아니다.
        MORAI 가 센서를 시뮬레이션하고 직렬화해 UDP 로 보내는 데 시간이 걸린다.
        rospy.Time.now() 를 그대로 쓰면 0.3초 묵은 관측에 "방금" 도장을 찍는
        셈이고, EKF 는 스탬프를 물리적 사실로 믿으므로 현재 위치를 0.3초 전
        지점으로 끌어당긴다. 11m/s 에서 3.4m 다.

        실측 0.30초 (2026-08-27). lat_navsat_* 를 GT 대비 tau 만큼 밀어보며
        잔차가 최소가 되는 tau 를 찾았다. 잔차 중앙값이 2.1~2.9m -> 0.47~0.67m.
        배속이 1.57배 차이나는 두 런(0.615 / 0.964)에서 tau 가 같게 나왔으므로,
        시뮬 프레임 수가 아니라 벽시계 기준 상수(전송 지연)로 본다.

        ※ ekf.yaml 의 smooth_lagged_data 가 켜져 있어야 한다. 꺼져 있으면
          robot_localization 이 자기 상태보다 오래된 관측을 그냥 버려서
          ("history interval is 0, so ignoring") GPS 를 통째로 잃는다.
        """
        out = GPSMessage()
        out.header.stamp = rospy.Time.now() - self.gps_lag
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

        # angular_velocity.z는 이미 축/부호 검증됨(좌회전=양수, ENU/REP-103 정합) - 라이다
        # 모션왜곡 보정의 요레이트로 그대로 사용.
        self._yaw_rate_rad_s = data["angular_velocity"][2]

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
        arr = np.array(points, dtype=np.float64)  # x,y,z,intensity,t_us
        xy = self._deskew_xy(arr[:, 0], arr[:, 1], arr[:, 4])
        out_arr = np.column_stack([xy, arr[:, 2], arr[:, 3]]).astype(np.float32)

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 16  # 4 floats x 4 bytes
        cloud.row_step = cloud.point_step * len(points)
        cloud.is_dense = True
        cloud.data = out_arr.tobytes()
        self.lidar_pub.publish(cloud)

    def _deskew_xy(self, x, y, t_us):
        """모션 왜곡(motion distortion) 보정 - 한 바퀴(100ms) 도는 동안 차가 움직인 만큼
        점마다 다른 시각에 찍힌 걸, 스캔 마지막 시점(t_ref) 기준 좌표로 되돌림.
        2D 등속+등각속도 근사(bicycle model) - dt가 최대 100ms라 이 근사로 충분.
        전방속도는 크기만(축관례 미검증), 요레이트는 IMU(검증됨)에서 가져옴."""
        t_ref = t_us.max()
        dt = (t_ref - t_us) * 1e-6  # 스캔 끝보다 얼마나 먼저 찍혔는지 (초, >=0)
        dtheta = self._yaw_rate_rad_s * dt
        dx = self._forward_speed_mps * dt
        cos_t, sin_t = np.cos(dtheta), np.sin(dtheta)
        x_shift = x - dx
        new_x = cos_t * x_shift + sin_t * y
        new_y = -sin_t * x_shift + cos_t * y
        return np.column_stack([new_x, new_y])


def main():
    rospy.init_node('udp_bridge')
    UdpBridge()
    rospy.spin()


if __name__ == '__main__':
    main()
