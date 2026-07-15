# MORAI + ROS1 Noetic 2-컨테이너 개발 환경

Ubuntu 24.04 호스트에서 ROS1 Noetic(=Ubuntu 20.04)을 도커로 돌리는 구성.
**RTX 5070 Laptop GPU 하드웨어 렌더링(OpenGL 4.6 / Vulkan 1.4) 검증 완료.**

- **sim** 컨테이너 : MORAI 시뮬레이터(Linux 빌드) + MORAI ROS 브리지 실행 (GPU/GUI)
- **dev** 컨테이너 : 본인 알고리즘 개발 (RViz 등)
- 두 컨테이너 모두 **host 네트워크** → `ROS_MASTER_URI=http://localhost:11311` 로 서로 통신
- 공용 **roscore** 는 아무 컨테이너에서나 한 번만 실행

```
morai-ros/
├── Dockerfile            # noetic-desktop-full + GPU/GUI 런타임 + Optimus 렌더 강제 + Vulkan ICD
├── docker-compose.yml    # sim / dev 두 서비스
├── .env                  # DISPLAY 값
├── catkin_ws/            # 공유 워크스페이스 (호스트↔컨테이너 마운트)
│   └── src/              #   ← 여기에 morai_msgs, 본인 패키지 clone
└── morai_sim/            # MORAI Linux 빌드
    └── MoraiLauncher_Lin/
        └── MoraiLauncher_Lin.x86_64   ← 실제 실행 런처
```

---

## 이 환경의 핵심 (왜 이렇게 세팅했나)

- **베이스**: `osrf/ros:noetic-desktop-full` (RViz·rqt 포함, 내부는 Ubuntu 20.04라 24.04 호스트와 무관)
- **컨테이너 사용자 uid/gid = 1000** : 호스트 `yeon` 과 동일 → 마운트한 파일이 root 소유로 안 생김
- **Optimus(노트북 하이브리드 그래픽) 렌더 강제** : 그냥 두면 OpenGL 이 `llvmpipe`(CPU 소프트웨어)로 폴백함.
  Dockerfile 에 아래 env 를 박아서 NVIDIA GPU 로 하드웨어 렌더링하게 함:
  ```
  __NV_PRIME_RENDER_OFFLOAD=1
  __GLX_VENDOR_LIBRARY_NAME=nvidia
  __VK_LAYER_NV_optimus=NVIDIA_only
  ```
- **Vulkan ICD 수동 생성** : nvidia-container-toolkit 이 `nvidia_icd.json` 을 자동 주입하지 않아
  Dockerfile 에서 직접 만들어 넣음 (`library_path: libGLX_nvidia.so.0`).

---

## 0. 최초 1회 준비 (호스트)

```bash
# (1) docker compose 플러그인 설치
#     ※ Ubuntu 저장소 기반 docker 라서 패키지명이 docker-compose-v2 (docker-compose-plugin 아님!)
sudo apt-get update && sudo apt-get install -y docker-compose-v2
docker compose version        # v2.x 나오면 OK

# (2) 컨테이너가 X 서버(GUI)에 접근하도록 허용 — 재부팅/재로그인마다 한 번
xhost +local:
```

## 1. 이미지 빌드

```bash
cd ~/morai-ros
docker compose build          # 처음엔 몇 분 (베이스 이미지 ~4GB + apt 패키지)
```

## 2. 컨테이너 실행

```bash
docker compose up -d          # morai-sim, morai-dev 두 컨테이너 실행
docker compose ps             # 상태 확인
```

## 3. 각 컨테이너에 접속해서 작업

```bash
# 개발용 컨테이너 (여기서 roscore 를 띄운다고 가정)
docker compose exec dev bash
#   $ roscore

# 시뮬용 컨테이너 (새 터미널)
docker compose exec sim bash
#   $ cd ~/morai_sim/MoraiLauncher_Lin
#   $ ./MoraiLauncher_Lin.x86_64        # MORAI 런처 실행 → 로그인/라이센스 활성화
stage_26molit_026 // morai5061@
```

> roscore 는 sim / dev 어느 쪽에서 띄워도 됨 (host 네트워크라 localhost 로 공유).
> MORAI 런처는 **반드시 `MoraiLauncher_Lin/` 폴더 안에서** 실행해야 `_Data` 를 찾음.

## 4. 종료

```bash
docker compose down           # 두 컨테이너 정지 & 제거 (워크스페이스·MORAI 빌드는 호스트에 남음)
```

---

## MORAI / 워크스페이스 세팅

- **MORAI 빌드**: `morai_sim/MoraiLauncher_Lin/` 에 이미 배치됨. 실행권한도 부여 완료.
  - 라이센스는 **URL/계정 방식** → 런처 실행 후 로그인으로 활성화.
  - `keylok*` 파일들은 **USB 동글(하드웨어 라이센스)** 용이라 이 환경에선 **무시**. (동글 없어도 런처 정상 구동)
- **morai_msgs / 본인 패키지** 클론:
  ```bash
  cd ~/morai-ros/catkin_ws/src
  git clone https://github.com/MORAI-Autonomous/MORAI-DriveExample_ROS.git   # 예시
  ```
- **빌드** (아무 컨테이너 안에서 한 번만 — 두 컨테이너가 같은 catkin_ws 공유):
  ```bash
  docker compose exec dev bash
  cd ~/catkin_ws && catkin_make        # 또는 catkin build
  source devel/setup.bash              # (.bashrc 에서 자동 source 됨)
  ```

---

## UDP ↔ ROS 브리지 (제어 & 차량상태 · RViz)

대회 커스텀 빌드는 **제어/차량상태를 UDP**로 주고받음 (센서는 MORAI 자체 ROS 브리지 사용).
`udp_bridge` 패키지가 그 UDP ↔ ROS 를 통역함. 내 알고리즘은 ROS 토픽으로만 개발하면 됨.

```
   [MORAI 시뮬레이터]              [ udp_bridge.py ]              [ 내 ROS 세계 ]
   제어 수신 :9093  ◄──UDP #MoraiCtrlCmd$──  구독 /ctrl_cmd ◄── teleop_keyboard.py
   상태 송신 :9110  ──UDP #MoraiInfo$─►:9111  발행 /ego_status ──► ego_viz.py ─► RViz
                                                                   (TF+Marker+Path)
```

### 패키지 구성

```
catkin_ws/src/udp_bridge/
├── scripts/
│   ├── udp_bridge.py       # UDP ↔ ROS 통역 (제어 송신 / 차량상태 수신)
│   ├── ego_viz.py          # /ego_status → TF + Marker(차 박스) + Path(궤적) : RViz 표시용
│   └── teleop_keyboard.py  # 키보드 입력 → /ctrl_cmd 발행 (제어 테스트)
└── rviz/ego.rviz           # RViz 프리셋 (Fixed Frame=map, 차량 추종 시점)
```

> `.bashrc` 에 alias 등록됨: `udp` = `roscore & sleep 2 && rosrun udp_bridge udp_bridge.py`

### 토픽 / 포트

| 방향 | ROS 토픽 | UDP 헤더 | 포트 (MORAI 항목) |
|---|---|---|---|
| 제어 (내 → MORAI) | `/ctrl_cmd` (구독) | `#MoraiCtrlCmd$` | → **9093** (MoraiCmdController) |
| 차량상태 (MORAI → 내) | `/ego_status` (발행) | `#MoraiInfo$` | **9111** ← 9110 (MoraiInfoPublisher) |

- 제어 메시지(`morai_msgs/CtrlCmd`): `longlCmdType=1`(Throttle), `accel`/`brake` 0~1, `front_steer` 라디안.
  브리지가 조향 실측 보정(`STEER_RATIO_CORRECTION=0.70`) 적용 후 송신.
- 두 컨테이너 host 네트워크라 IP 는 전부 `127.0.0.1`.

### MORAI 쪽 설정 (Network Settings → **Ego Network** 탭)

차량 스폰 + 시나리오 재생 상태에서:

1. **MoraiCmdController** → UDP, IP `127.0.0.1`, **Host Port 9093**
2. **MoraiInfoPublisher** → UDP, IP `127.0.0.1`, **Destination Port 9111**
3. 각 항목의 점을 **초록(연결)** 으로 켜고, 상단 **Status: Connected** 확인

> ⚠️ **가장 흔한 함정**: MoraiInfoPublisher 를 포트만 바꾸고 **안 켜면**(빨간 점) MORAI 가
> 송신 소켓(9110)을 안 잡아서 `/ego_status` 에 아무것도 안 옴. 반드시 **초록**으로.

### 실행 (각 터미널 = `docker compose exec dev bash`)

```bash
# 1) 브리지 (roscore + udp_bridge)
udp

# 2) 시각화 노드
rosrun udp_bridge ego_viz.py

# 3) RViz  (파란 차 박스 + 초록 궤적, 시점은 차량 추종)
rosrun rviz rviz -d ~/catkin_ws/src/udp_bridge/rviz/ego.rviz

# 4) 제어 — 키보드로 운전
rosrun udp_bridge teleop_keyboard.py
#   W/S 가속·브레이크, A/D 조향, Space 정지, X 조향중립, Q 종료
```

제어 빠른 확인(한 줄, 전진):
```bash
rostopic pub -r 20 /ctrl_cmd morai_msgs/CtrlCmd \
  "{longlCmdType: 1, accel: 0.3, brake: 0.0, front_steer: 0.0}"
rostopic echo /ego_status/velocity      # 가속 시 값 증가 확인
```

### 최초 세팅(이미 완료된 상태 기록)

```bash
cd ~/catkin_ws/src
git clone https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git
mv MORAI-ROS_morai_msgs morai_msgs          # ⚠️ 하이픈 있으면 catkin 인식 못 함
catkin_create_pkg udp_bridge rospy morai_msgs
# scripts/*.py 배치 후
cd ~/catkin_ws && catkin_make
```

### 브리지 트러블슈팅

- **`/ego_status` 값 안 옴** →
  1. MORAI Ego Network 에서 **MoraiInfoPublisher 초록**인지 (원인 대부분 이것)
  2. Destination Port **9111** / IP **127.0.0.1** 인지
  3. 실제 수신 스니핑: 브리지 잠깐 끄고
     ```bash
     python3 -c "import socket; s=socket.socket(2,2); s.bind(('0.0.0.0',9111)); print(s.recvfrom(65535)[0][:12])"
     # b'#MoraiInfo$...' 나오면 MORAI 가 보내는 중
     ```
- **제어 안 먹음** → MoraiCmdController **초록** + Host Port 9093, 메시지 `longlCmdType=1` 인지.
- **좌표/속도가 엉뚱함** → 이 빌드 EgoInfo 패킷은 **229바이트**(문서상 181과 다름). 추가 48B 는
  뒤쪽(link_id 이후)에 붙어 **앞쪽 오프셋(pos/heading/vel)은 그대로 유효**. 뒤쪽 필드를 쓸 때만
  `udp_bridge.py` 의 `struct.unpack` 오프셋을 재확인.
- **어느 포트가 활성인지 확인** → `ss -unap | grep -E ':9093|:9110|:9111'`
  (MORAI 는 **켜진 항목만** 소켓을 잡으므로 활성 여부 판단 가능). 10326/10508 은 시뮬레이터
  내부 통신(`#SimStatus$`)이라 ego 데이터와 **무관**.
- **RViz 검은 화면 / 안 뜸** → `export LIBGL_ALWAYS_SOFTWARE=1` 후 재실행 (X11/GL 폴백).

---

## 동작 확인 (GPU / GUI) — 정상 출력 예시

```bash
docker compose exec sim bash

nvidia-smi                                  # RTX 5070 보이면 OK
glxinfo -B | grep "OpenGL renderer"         # → NVIDIA GeForce RTX 5070 ...  (llvmpipe 아님!)
vulkaninfo | grep deviceName                # → NVIDIA GeForce RTX 5070 ...
```

MORAI 런처 실행 후 Unity 로그로 GPU 렌더링 확인:
```bash
grep -E "Renderer:|graphics device" ~/.config/unity3d/Morai/Launcher/Player.log
# Renderer: NVIDIA GeForce RTX 5070 Laptop GPU
# OPENGL LOG: Creating OpenGL 4.5 graphics device
```

---

## 트러블슈팅

- **`glxinfo` 가 `llvmpipe` 로 나옴 (GPU 가속 안 됨)** → Optimus 렌더 강제 env 가 안 먹은 것.
  `docker compose exec sim bash -lc 'echo $__GLX_VENDOR_LIBRARY_NAME'` 가 `nvidia` 인지 확인.
  아니면 이미지 재빌드(`docker compose build`) 후 `docker compose up -d --force-recreate`.
- **`vulkaninfo` 에 디바이스 없음** → `/usr/share/vulkan/icd.d/nvidia_icd.json` 존재 확인. 없으면 재빌드.
- **GUI 창이 안 뜸 / `cannot open display`** → 호스트에서 `xhost +local:` 실행했는지,
  `.env` 의 `DISPLAY` 가 호스트 `echo $DISPLAY` (`:1`) 와 같은지 확인.
- **`nvidia-smi` 안 나옴** → `docker info | grep -i runtime` 에 `nvidia` 있는지 확인.
- **노드끼리 통신 안 됨** → 두 컨테이너 다 `network_mode: host`, `ROS_MASTER_URI=http://localhost:11311`,
  roscore 가 떠 있는지 확인.
- **런처 로그인/다운로드 유지** → 이미 적용됨. named volume 으로 `~/.config`, `~/.local` 을 유지해서
  `docker compose down` 해도 재로그인이 필요 없음 (아래 참고).

### 로그인/다운로드 유지 (적용 완료)

`docker-compose.yml` 에 named volume 이 이미 설정돼 있음:
- `morai_sim_config` / `morai_sim_local` → sim 컨테이너의 `~/.config`, `~/.local`
- `dev_config` / `dev_local` → dev 컨테이너 (sim 과 별도 볼륨 → 충돌 방지)

⚠️ **주의**: named volume 은 처음 생성 시 **root 소유**라 컨테이너 사용자(dev)가 못 씀.
최초 1회(또는 볼륨을 `docker compose down -v` 로 지우고 다시 만든 경우) 소유권을 dev 로 바꿔야 함:
```bash
docker compose exec -u 0 sim chown -R dev:dev /home/dev/.config /home/dev/.local
docker compose exec -u 0 dev chown -R dev:dev /home/dev/.config /home/dev/.local
```
(이 소유권은 볼륨에 저장되므로 이후 `up/down` 에는 다시 안 해도 됨. `down -v` 로 볼륨까지 지웠을 때만 재실행.)
