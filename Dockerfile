FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000

# ------------------------------------------------------------------
# 개발 도구 + MORAI(Unity) 실행에 필요한 GPU/GUI 런타임 라이브러리
# (베이스 이미지는 Ubuntu 20.04 focal 이므로 focal 패키지명 사용)
# ------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      # ROS / 빌드 도구
      python3-catkin-tools python3-osrf-pycommon python3-pip \
      # localization 스택 (autonomous_driving/launch/localization.launch 가 쓴다)
      #   navsat_transform_node : 위경도 -> UTM52N -> 맵 로컬좌표
      #   ekf_localization_node : IMU + GPS + 차속 융합 -> /odom
      # 컨테이너에 직접 apt 로 넣으면 docker compose down/up 때 날아간다(2026-08-21 겪음).
      ros-noetic-robot-localization \
      # perception 스택 (autonomous_driving/src/perception 의 파이썬 노드들)
      #   sklearn / scipy : 이 둘만 실제로 빠져 있었다 (2026-08-26 확인)
      #     sklearn = lidar_node 의 DBSCAN 클러스터링
      #     scipy   = tracking_node 의 헝가리안 매칭 + 카이제곱 게이팅
      #   tf2-* / message-filters 는 지금은 ROS 설치에 딸려 오지만, 직접 쓰는
      #   의존이므로 전이 의존에 기대지 않고 명시해 둔다.
      ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-message-filters \
      python3-sklearn python3-scipy \
      build-essential git wget curl vim nano tmux sudo \
      net-tools iputils-ping iproute2 \
      # OpenGL / GLVND (NVIDIA 드라이버 라이브러리는 컨테이너 툴킷이 주입)
      libgl1-mesa-glx libgl1-mesa-dri mesa-utils \
      libglvnd0 libgl1 libglx0 libegl1 libgles2 \
      # Vulkan (Unity 렌더링)
      libvulkan1 vulkan-tools \
      # Unity / GUI 의존 X 라이브러리
      libxrandr2 libxinerama1 libxcursor1 libxi6 libxxf86vm1 \
      libxkbcommon0 libxss1 libgtk-3-0 libnss3 libasound2 libpulse0 \
      x11-apps \
  && rm -rf /var/lib/apt/lists/*

# MORAI 예제/브리지가 자주 쓰는 파이썬 패키지
RUN pip3 install --no-cache-dir pygame numpy

# ------------------------------------------------------------------
# 호스트 uid/gid(1000)와 맞춘 비루트 사용자 → 마운트 볼륨 권한 문제 방지
# ------------------------------------------------------------------
RUN groupadd -g ${GID} ${USERNAME} 2>/dev/null || true \
 && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && usermod -aG dialout,video ${USERNAME}

# NVIDIA 컨테이너 툴킷이 읽는 환경변수 (graphics/display 포함해야 Unity 렌더 가능)
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# 노트북 Optimus 환경: GLVND 가 소프트웨어(llvmpipe)로 폴백하지 않고
# NVIDIA GPU 로 하드웨어 렌더링하도록 강제 (이거 없으면 llvmpipe 로 떨어짐)
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
ENV __VK_LAYER_NV_optimus=NVIDIA_only

# Vulkan ICD: 컨테이너 툴킷이 nvidia_icd.json 을 자동 주입하지 않으므로 직접 생성
#   (library_path 는 이미 주입되는 libGLX_nvidia.so.0 을 가리킴)
ENV VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
RUN mkdir -p /usr/share/vulkan/icd.d && \
    printf '%s\n' \
      '{' \
      '    "file_format_version" : "1.0.1",' \
      '    "ICD": {' \
      '        "library_path": "libGLX_nvidia.so.0",' \
      '        "api_version" : "1.4.329"' \
      '    }' \
      '}' > /usr/share/vulkan/icd.d/nvidia_icd.json

# ------------------------------------------------------------------
# YOLO 전용 Python 3.9 환경 (/opt/yolo39)
# ------------------------------------------------------------------
# 왜 별도 파이썬인가:
#   ROS Noetic 은 Ubuntu 20.04 에 묶여 있어 시스템 파이썬이 3.8 이다. 그런데
#   PyTorch 의 CUDA 12.8(cu128) 빌드는 cp39 부터만 존재하고(공식 인덱스 확인,
#   2026-08-26), RTX 5070 은 Blackwell(sm_120)이라 cu128 이 아니면 추론 시점에
#   "no kernel image is available for execution on the device" 로 죽는다.
#   그래서 camera_detection_node 만 이 venv 로 돌린다.
#
# 왜 이게 안전한가:
#   camera_detection_node.py 가 쓰는 ROS 모듈(rospy, sensor_msgs.msg,
#   autonomous_driving.msg)은 전부 순수 파이썬이라 3.9 에서도 그대로 import 된다.
#   cv_bridge / tf2_py 같은 C++ 확장 모듈은 안 쓴다(이미지도 cv2.imdecode 로
#   직접 푼다). 2026-08-26 에 3.9 에서 import 되는 것까지 확인했다.
#   ROS 경로는 실행할 때 PYTHONPATH 로 들어오므로 venv 안에 넣지 않는다.
#
# 여기 없는 rospy 의존(PyYAML, rospkg, numpy, netifaces)은 3.8 쪽 dist-packages
# 에만 있어서 venv 에 따로 깔아줘야 한다.
#   netifaces 는 rosgraph 가 "발행자 주소가 로컬인가"를 판단할 때 쓴다. 구독이
#   실제로 연결되는 순간에만 불려서, import 테스트나 발행만 하는 테스트로는
#   빠진 게 안 드러난다(2026-08-27 에 그렇게 놓쳤다). 구독 경로까지 돌려봐야 한다.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3.9 python3.9-venv \
 && rm -rf /var/lib/apt/lists/* \
 && python3.9 -m venv /opt/yolo39 \
 && /opt/yolo39/bin/pip install --no-cache-dir --upgrade pip \
 && /opt/yolo39/bin/pip install --no-cache-dir PyYAML rospkg numpy \
 && /opt/yolo39/bin/pip install --no-cache-dir \
      --index-url https://download.pytorch.org/whl/cu128 torch torchvision \
 && /opt/yolo39/bin/pip install --no-cache-dir ultralytics \
 && chmod -R a+rX /opt/yolo39

# 모델 가중치를 빌드 시점에 받아 둔다. 안 그러면 대회장에서 인터넷이 없을 때
# 첫 실행이 그대로 막힌다. 노드에는 _model_path 로 이 경로를 준다.
RUN cd /opt/yolo39 \
 && /opt/yolo39/bin/python -c "from ultralytics import YOLO; YOLO('yolo11n.pt')" \
 && chmod a+r /opt/yolo39/yolo11n.pt

# netifaces 는 rospy 의존인데 rosgraph 가 "발행자 주소가 로컬인가" 를 판단할 때만
# 불린다. 구독이 실제로 연결되는 순간에만 부르므로 import 테스트나 발행 테스트로는
# 안 드러나서 뒤늦게 발견했다 (2026-08-27).
#
# 왜 위 RUN 에 안 끼워 넣는가: 도커 캐시는 RUN 문자열이 한 글자만 달라도 그 레이어
# 부터 전부 무효화한다. 위 RUN 은 torch cu128 이 들어 있어 7.4GB 다. 단어 하나
# 때문에 그걸 다시 받게 된다. 앞으로 rospy 의존이 또 나와도 여기에만 추가할 것.
#
# ※ chmod 를 /opt/yolo39 전체에 -R 로 걸면 안 된다. 메타데이터가 바뀐 파일이 전부
#   새 레이어에 복사돼 이미지가 7GB 불어난다. 새로 깔린 것만 건드린다.
RUN /opt/yolo39/bin/pip install --no-cache-dir netifaces \
 && chmod -R a+rX /opt/yolo39/lib/python3.9/site-packages/netifaces*

# 셸 진입 시 ROS + 워크스페이스 자동 source
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo "[ -f /home/${USERNAME}/catkin_ws/devel/setup.bash ] && source /home/${USERNAME}/catkin_ws/devel/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo "export ROS_MASTER_URI=\${ROS_MASTER_URI:-http://localhost:11311}" >> /home/${USERNAME}/.bashrc \
 && echo "alias udp='roscore & sleep 2 && rosrun udp_bridge udp_bridge.py'" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/catkin_ws

CMD ["bash"]
