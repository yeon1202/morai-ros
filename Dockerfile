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

# 셸 진입 시 ROS + 워크스페이스 자동 source
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo "[ -f /home/${USERNAME}/catkin_ws/devel/setup.bash ] && source /home/${USERNAME}/catkin_ws/devel/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo "export ROS_MASTER_URI=\${ROS_MASTER_URI:-http://localhost:11311}" >> /home/${USERNAME}/.bashrc \
 && echo "alias udp='roscore & sleep 2 && rosrun udp_bridge udp_bridge.py'" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/catkin_ws

CMD ["bash"]
