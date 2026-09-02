# perception 연동 구현 계획

> **에이전트용:** 이 계획을 task 단위로 실행할 때는 superpowers:subagent-driven-development
> (권장) 또는 superpowers:executing-plans 를 쓴다. 단계는 체크박스(`- [ ]`)로 추적한다.

**목표:** `mock_obstacle_pub` 의 하드코딩 장애물을 팀 perception 의 실제 검출 결과로
교체한다. planning 소비자(`lattice_planner`, `acc_planner`, `object_viz`)는 무수정.

**구조:** 팀 perception 5개 노드는 그대로 쓰고, 종점 `/perception/tracked_objects` 를
`/Object_topic` 으로 옮기는 어댑터 하나만 새로 만든다. 좌표 변환은 팀
`global_transform_node` 가 tf 로 처리하므로 `base_link → lidar` static tf 를 채워준다.

**기술 스택:** ROS1 noetic, Python 3.8(컨테이너), rospy, PyTorch(cu128) + ultralytics,
morai_msgs, autonomous_driving 커스텀 메시지

**스펙:** [24-perception_integration_design.md](24-perception_integration_design.md)

## 전역 제약

- 편집은 **호스트**, 빌드·실행·테스트는 **`docker exec morai-dev`**, 커밋은 호스트에서 한다.
- 커밋 메시지에 `Co-Authored-By` 트레일러를 **넣지 않는다**.
- 라이다 마운트: **x=1.4, y=0, z=1.23** (MORAI 센서 설정과 반드시 일치).
- `/Object_topic` 발행 주기는 **20 Hz 고정** (mock 과 동일).
- 인지 타임아웃은 **0.5초**.
- `sim.launch` 의 `perception` 인자 기본값은 **`false`** — mock 경로를 살려 둔다.
- ROS 명령을 쓰기 전에 반드시 `source /home/dev/catkin_ws/devel/setup.bash` 한다.
  `/opt/ros/noetic/setup.bash` 만 하면 커스텀 메시지를 못 찾아 "토픽이 안 나온다"로
  오진하게 된다(2026-08-26 실제로 겪음).

## 파일 구조

| 파일 | 책임 |
|---|---|
| `catkin_ws/src/path_tracking/scripts/lib/object_convert.py` | **신규.** ROS 노드와 무관한 **순수 변환 함수**. 단위 테스트 대상 |
| `catkin_ws/src/path_tracking/scripts/object_topic_adapter.py` | **신규.** 위 함수를 구독/발행/타임아웃으로 감싸는 얇은 노드 |
| `catkin_ws/src/path_tracking/scripts/test_object_convert.py` | **신규.** 변환 단위 테스트 (ROS 마스터 불필요) |
| `catkin_ws/src/autonomous_driving/launch/perception.launch` | **신규.** 인지 5개 노드 |
| `catkin_ws/src/autonomous_driving/launch/localization.launch` | `base_link → lidar` static tf 추가 |
| `catkin_ws/src/path_tracking/launch/sim.launch` | `perception` 인자로 mock ↔ 인지 전환 |
| `catkin_ws/src/path_tracking/package.xml` | `autonomous_driving` 의존 추가 |
| `Dockerfile` | torch(cu128) + ultralytics |

변환 로직을 노드에서 분리하는 이유: rospy 없이 테스트할 수 있어야 대회 당일
시뮬 없이도 회귀를 잡을 수 있다. `acc_core.hpp` / `behavior_core.hpp` 를 gtest 로
분리해 둔 것과 같은 이유다.

---

### Task 1: `base_link → lidar` static tf

**파일:**
- 수정: `catkin_ws/src/autonomous_driving/launch/localization.launch`

**인터페이스:**
- 생산: tf `base_link → lidar`. Task 3 의 `global_transform_node` 가 이걸 쓴다.

- [ ] **Step 1: 지금 tf 가 없다는 것부터 확인**

```bash
docker exec morai-dev bash -lc 'cd /home/dev/catkin_ws && source devel/setup.bash && \
  rosrun tf tf_echo base_link lidar'
```

기대: `Frame lidar does not exist` 또는 무한 대기. **있다고 나오면 이미 누가 발행
중이라는 뜻이므로 중복 발행하지 말고 그 출처를 먼저 찾을 것.**

- [ ] **Step 2: static tf 추가**

`localization.launch` 의 `base_link_to_gps` 노드 **바로 뒤**에 넣는다:

```xml
  <!-- 6) base_link -> lidar 정적 tf (라이다 장착 위치).
       팀 perception 의 global_transform_node 가 물체를 라이다 좌표에서 전역
       좌표로 옮길 때 이 tf 를 찾는다. 없으면
       "Could not obtain transform from lidar to odom" 으로 아무것도 못 낸다.

       값은 MORAI 센서 설정과 반드시 일치해야 한다 (2026-08-26 기준 x=1.4,
       y=0, z=1.23). 지면 반사의 z 분포로 교차 확인했다 - 라이다가 지면 위
       1.53m 이므로 차량좌표 지면 z=-0.3 을 더하면 마운트 z=1.23 이다.
       ※ MORAI 에서 센서를 다시 옮기면 이 값도 같이 바꿔야 한다.
       ※ args 순서: x y z yaw pitch roll parent child -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_lidar"
        args="1.4 0 1.23 0 0 0 base_link lidar"/>
```

- [ ] **Step 3: 재실행하고 tf 확인**

`localization.launch` 를 껐다 켠 뒤:

```bash
docker exec morai-dev bash -lc 'cd /home/dev/catkin_ws && source devel/setup.bash && \
  timeout 5 rosrun tf tf_echo base_link lidar'
```

기대: `- Translation: [1.400, 0.000, 1.230]` 이 반복 출력.

- [ ] **Step 4: 전역 변환 사슬이 이어지는지 확인**

```bash
docker exec morai-dev bash -lc 'cd /home/dev/catkin_ws && source devel/setup.bash && \
  timeout 5 rosrun tf tf_echo odom lidar'
```

기대: 값이 나오고, 차를 움직이면 값이 따라 변한다(EKF 가 `odom→base_link` 를
갱신하므로). 여기서 실패하면 EKF 가 안 돌고 있는 것이다.

- [ ] **Step 5: 커밋**

```bash
cd /home/yeon/morai-ros
git add catkin_ws/src/autonomous_driving/launch/localization.launch
git commit -m "feat(localization): base_link -> lidar static tf 추가

팀 perception 의 global_transform_node 가 물체를 전역좌표로 옮길 때 이 tf 를
찾는데 지금까지 아무도 발행하지 않았다. MORAI 센서 설정과 같은 값(1.4/0/1.23)."
```

---

### Task 2: PyTorch(cu128) + ultralytics 설치

**파일:**
- 수정: `Dockerfile`

**인터페이스:**
- 생산: 컨테이너에서 `import torch; torch.cuda.is_available() == True`. Task 3 의
  `camera_detection_node` 가 이걸 쓴다.

- [ ] **Step 1: 지금 없다는 것 확인**

```bash
docker exec morai-dev bash -lc 'python3 -c "import torch" 2>&1 | tail -1'
```

기대: `ModuleNotFoundError: No module named 'torch'`

- [ ] **Step 2: cu128 빌드 설치**

**일반 `pip install torch` 를 쓰면 안 된다.** RTX 5070 은 Blackwell(sm_120)이라
기본 배포(cu121)에는 커널이 없고, 실행 시점에 `no kernel image is available for
execution on the device` 로 죽는다. 설치는 되므로 이 단계에서는 안 드러난다.

```bash
docker exec -u root morai-dev bash -lc \
  'pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cu128'
docker exec -u root morai-dev bash -lc 'pip3 install --no-cache-dir ultralytics'
```

- [ ] **Step 3: GPU 가 실제로 쓰이는지 확인 (설치 확인이 아니라 연산 확인)**

```bash
docker exec morai-dev bash -lc 'python3 - <<PY
import torch
print("torch", torch.__version__, "cuda", torch.version.cuda)
print("available:", torch.cuda.is_available())
print("device:", torch.cuda.get_device_name(0))
print("capability:", torch.cuda.get_device_capability(0))
x = torch.randn(1000, 1000, device="cuda")
print("matmul ok:", float((x @ x).sum()) == float((x @ x).sum()))
PY'
```

기대: `available: True`, `capability: (12, 0)`, `matmul ok: True`.
`matmul` 에서 `no kernel image` 가 나오면 cu128 이 아닌 빌드가 깔린 것이다.

- [ ] **Step 4: 모델 다운로드 확인**

```bash
docker exec morai-dev bash -lc 'cd /tmp && python3 -c "
from ultralytics import YOLO
m = YOLO(\"yolo11n.pt\")
print(\"loaded, classes:\", len(m.names))
"'
```

기대: 다운로드 진행 후 `loaded, classes: 80`. 인터넷이 없으면 여기서 막힌다.

- [ ] **Step 5: Dockerfile 에 반영**

`RUN pip3 install --no-cache-dir pygame numpy` 줄 **뒤에** 추가:

```dockerfile
# perception 의 camera_detection_node(YOLO) 용.
# !! cu128 인덱스를 반드시 지정할 것 !! RTX 5070 은 Blackwell(sm_120)이라
# 기본 배포(cu121)에는 커널이 없다. 설치는 조용히 되고 추론 시점에
# "no kernel image is available for execution on the device" 로 죽는다.
RUN pip3 install --no-cache-dir --index-url https://download.pytorch.org/whl/cu128 \
      torch torchvision \
 && pip3 install --no-cache-dir ultralytics
```

- [ ] **Step 6: 커밋**

```bash
cd /home/yeon/morai-ros
git add Dockerfile
git commit -m "build: YOLO 용 torch(cu128) + ultralytics 추가

RTX 5070(Blackwell, sm_120)이라 cu128 인덱스를 명시해야 한다. 기본 배포는
설치는 되지만 추론에서 no kernel image 로 죽는다."
```

---

### Task 3: `perception.launch`

**파일:**
- 생성: `catkin_ws/src/autonomous_driving/launch/perception.launch`

**인터페이스:**
- 소비: Task 1 의 `base_link → lidar` tf, Task 2 의 torch
- 생산: `/perception/tracked_objects` (`autonomous_driving/RecognizedObjectArray`).
  Task 5 의 어댑터가 구독한다.

- [ ] **Step 1: launch 파일 작성**

```xml
<?xml version="1.0"?>
<!--
  팀 perception 스택 실행 (2026-08-26 planning 이 작성).
  전제: udp_bridge(/lidar/points, /camera1/…) 와 localization.launch(tf) 가 떠 있어야 한다.

  ┌─ 흐름 ─────────────────────────────────────────────────────────────┐
  │ /lidar/points ─> lidar_node ────────┐                              │
  │                                     ├─> object_fusion ─> /perception/recognized_objects
  │ /camera1/…    ─> camera_detection ──┘                              │
  │                     (YOLO, GPU)                                    │
  │ recognized_objects ─> global_transform ─> …_global ─> tracking ─> /perception/tracked_objects
  └────────────────────────────────────────────────────────────────────┘

  ※ global_transform_node 는 tf(lidar->base_link->odom)를 쓴다. base_link->lidar 는
    localization.launch 가, odom->base_link 는 ekf_localization_node 가 낸다.
-->
<launch>
  <arg name="camera" default="true"/>   <!-- false 면 YOLO 없이 라이다만 (융합은 멈춘다) -->

  <!-- 1) 라이다 지면제거 + DBSCAN 클러스터링 -->
  <node pkg="autonomous_driving" type="lidar_node.py" name="lidar_node" output="screen"/>

  <!-- 2) 카메라 YOLO 2D 검출.
       img_size 기본값 416 은 팀이 CPU 환경 기준으로 잡은 값이다. 우리는 GPU 가
       있으므로 640 으로 올려 정확도를 확보한다. torch_num_threads 도 GPU 추론에는
       병목이 아니므로 기본값(2)을 그대로 둔다. -->
  <node if="$(arg camera)" pkg="autonomous_driving" type="camera_detection_node.py"
        name="camera_detection_node" output="screen">
    <param name="img_size" value="640"/>
    <param name="conf_threshold" value="0.4"/>
  </node>

  <!-- 3) 라이다 클러스터 + 카메라 검출 융합 (시각 동기화) -->
  <node if="$(arg camera)" pkg="autonomous_driving" type="object_fusion_node.py"
        name="object_fusion_node" output="screen"/>

  <!-- 4) 라이다 좌표 -> 전역(odom) 좌표 -->
  <node if="$(arg camera)" pkg="autonomous_driving" type="global_transform_node.py"
        name="global_transform_node" output="screen"/>

  <!-- 5) 칼만 추적 (unique_id + velocity 부여) -->
  <node if="$(arg camera)" pkg="autonomous_driving" type="tracking_node.py"
        name="tracking_node" output="screen"/>
</launch>
```

- [ ] **Step 2: 실행권한 확인**

`rosrun`/`roslaunch` 는 실행 가능 파일만 찾는다.

```bash
ls -l catkin_ws/src/autonomous_driving/src/perception/*.py | awk '{print $1, $NF}'
```

기대: `lidar_node.py`, `camera_detection_node.py`, `object_fusion_node.py`,
`global_transform_node.py`, `tracking_node.py` 가 전부 `-rwx`. 아니면 `chmod +x`.

- [ ] **Step 3: 실행**

브릿지와 `localization.launch` 가 떠 있는 상태에서:

```bash
docker exec -it morai-dev bash -lc \
  'source /home/dev/catkin_ws/devel/setup.bash && roslaunch autonomous_driving perception.launch'
```

- [ ] **Step 4: 단계별로 토픽이 살아나는지 확인**

다른 터미널에서 **`devel/setup.bash` 를 source 한 뒤**:

```bash
for t in /lidar/clusters /camera1/detections /perception/recognized_objects \
         /perception/recognized_objects_global /perception/tracked_objects; do
  printf "%-42s " "$t"
  timeout 6 rostopic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "안 나옴"
done
```

기대: 다섯 개 전부 rate 가 나온다. **앞에서부터 순서대로 본다** — `/lidar/clusters`
가 안 나오면 뒤는 볼 필요가 없다.

- [ ] **Step 5: GPU 를 실제로 쓰는지 확인**

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader
```

기대: MORAI `Simulator.x86_64` 외에 python 프로세스가 하나 더 잡힌다.
안 잡히면 YOLO 가 CPU 로 돌고 있는 것이다(Task 2 Step 3 을 다시 볼 것).

- [ ] **Step 6: 추적 결과에 ID 가 붙는지 확인**

```bash
timeout 10 rostopic echo -n1 /perception/tracked_objects
```

기대: `unique_id` 가 0 이 아닌 값. 0 만 나오면 `tracking_node` 가 아직 확정
트랙을 못 만든 것이다(`MIN_HITS_TO_CONFIRM=3` 이므로 물체가 3프레임 이상
연속으로 잡혀야 한다).

- [ ] **Step 7: 커밋**

```bash
cd /home/yeon/morai-ros
git add catkin_ws/src/autonomous_driving/launch/perception.launch
git commit -m "feat(perception): 인지 스택 launch 추가

라이다 클러스터링 + 카메라 YOLO + 융합 + 전역변환 + 추적 5개 노드.
GPU 가 있으므로 img_size 를 팀 기본값 416 -> 640 으로 올렸다."
```

---

### Task 4: 변환 로직과 단위 테스트

**파일:**
- 생성: `catkin_ws/src/path_tracking/scripts/lib/object_convert.py`
- 테스트: `catkin_ws/src/path_tracking/scripts/test_object_convert.py`

**인터페이스:**
- 생산: `to_object_status_list(tracked, stamp) -> morai_msgs/ObjectStatusList`.
  Task 5 의 노드가 이 함수만 호출한다.

**주의 — 타입이 다르다:** `RecognizedObject.center` 는 `geometry_msgs/Point` 이고
`ObjectStatus.position` 은 `geometry_msgs/Vector3` 다. 필드 이름이 같아도 **다른
타입이라 통째로 대입하면 직렬화가 깨진다.** 반드시 x/y/z 를 하나씩 옮긴다.

- [ ] **Step 1: 실패하는 테스트 작성**

`catkin_ws/src/path_tracking/scripts/test_object_convert.py`:

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""object_convert 단위 테스트. ROS 마스터 없이 돈다 (메시지 타입만 필요).

실행:
  docker exec morai-dev bash -lc \
    'cd /home/dev/catkin_ws && source devel/setup.bash && \
     python3 src/path_tracking/scripts/test_object_convert.py'
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from autonomous_driving.msg import RecognizedObject, RecognizedObjectArray
from lib.object_convert import empty_object_status_list, to_object_status_list


def make_obj(type_, x=1.0, y=2.0, yaw=0.0, vx=0.0, vy=0.0, uid=7, name='thing'):
    o = RecognizedObject()
    o.type = type_
    o.unique_id = uid
    o.class_name = name
    o.center.x, o.center.y, o.center.z = x, y, 3.0
    o.size.x, o.size.y, o.size.z = 4.0, 1.8, 1.5
    o.yaw = yaw
    o.velocity.x, o.velocity.y, o.velocity.z = vx, vy, 0.0
    return o


def wrap(objs):
    arr = RecognizedObjectArray()
    arr.objects = list(objs)
    return arr


def check(name, cond):
    print(('  PASS  ' if cond else '  FAIL  ') + name)
    return cond


def main():
    ok = True

    # 1) 종류별로 올바른 목록에 들어가고 개수가 맞는다
    out = to_object_status_list(wrap([make_obj(0), make_obj(1), make_obj(2), make_obj(2)]), None)
    ok &= check('보행자 1개', len(out.pedestrian_list) == 1 and out.num_of_pedestrian == 1)
    ok &= check('NPC 1개', len(out.npc_list) == 1 and out.num_of_npcs == 1)
    ok &= check('정적장애물 2개', len(out.obstacle_list) == 2 and out.num_of_obstacle == 2)

    # 2) 자차(-1)는 버린다
    out = to_object_status_list(wrap([make_obj(-1), make_obj(2)]), None)
    ok &= check('type -1 은 버려진다',
                len(out.obstacle_list) == 1 and not out.npc_list and not out.pedestrian_list)

    # 3) 속도 m/s -> km/h
    out = to_object_status_list(wrap([make_obj(1, vx=10.0, vy=-2.0)]), None)
    v = out.npc_list[0].velocity
    ok &= check('vx 10 m/s -> 36 km/h', abs(v.x - 36.0) < 1e-6)
    ok &= check('vy -2 m/s -> -7.2 km/h', abs(v.y - (-7.2)) < 1e-6)

    # 4) yaw 라디안 -> heading 도
    out = to_object_status_list(wrap([make_obj(2, yaw=math.pi / 2)]), None)
    ok &= check('yaw pi/2 -> heading 90도', abs(out.obstacle_list[0].heading - 90.0) < 1e-6)

    # 5) center(Point) -> position(Vector3) 값이 보존된다
    out = to_object_status_list(wrap([make_obj(2, x=-60.61, y=-142.178)]), None)
    p = out.obstacle_list[0].position
    ok &= check('position 값 보존', abs(p.x + 60.61) < 1e-6 and abs(p.y + 142.178) < 1e-6)
    ok &= check('position 타입이 Vector3', type(p).__name__ == 'Vector3')

    # 6) size 와 id, name 이 그대로 넘어간다
    o = out.obstacle_list[0]
    ok &= check('size 보존', abs(o.size.x - 4.0) < 1e-6 and abs(o.size.y - 1.8) < 1e-6)
    ok &= check('unique_id 보존', o.unique_id == 7)
    ok &= check('name 보존', o.name == 'thing')

    # 7) 빈 입력 -> 빈 목록, 개수 0
    out = to_object_status_list(wrap([]), None)
    ok &= check('빈 입력이면 개수 전부 0',
                out.num_of_obstacle == 0 and out.num_of_npcs == 0 and out.num_of_pedestrian == 0)

    # 8) frame_id 는 map (planning 의 전역경로와 같은 프레임)
    ok &= check("frame_id 는 'map'", out.header.frame_id == 'map')

    # 9) 인지가 끊겼을 때 낼 빈 목록도 같은 모양이어야 한다
    #    (어댑터가 이 함수를 쓰므로 여기서 같이 지킨다)
    e = empty_object_status_list(None)
    ok &= check('빈 목록도 frame_id 가 map', e.header.frame_id == 'map')
    ok &= check('빈 목록은 세 목록이 다 비어 있다',
                not e.obstacle_list and not e.npc_list and not e.pedestrian_list)
    ok &= check('빈 목록은 개수도 0',
                e.num_of_obstacle == 0 and e.num_of_npcs == 0 and e.num_of_pedestrian == 0)

    print('')
    print('결과: ' + ('전부 통과' if ok else '실패 있음'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 2: 실패하는지 확인**

```bash
docker exec morai-dev bash -lc 'cd /home/dev/catkin_ws && source devel/setup.bash && \
  python3 src/path_tracking/scripts/test_object_convert.py'
```

기대: `ModuleNotFoundError: No module named 'lib.object_convert'`

- [ ] **Step 3: 변환 함수 구현**

`catkin_ws/src/path_tracking/scripts/lib/object_convert.py`:

```python
# -*- coding: utf-8 -*-
"""팀 perception 의 RecognizedObjectArray 를 planning 의 ObjectStatusList 로 옮긴다.

rospy 를 쓰지 않는 순수 함수다. 시뮬레이터도 ROS 마스터도 없이 테스트할 수 있어야
대회 당일 회귀를 빨리 잡을 수 있다 (acc_core.hpp 를 gtest 로 분리해 둔 것과 같은 이유).

단위가 두 군데 다르다 - 여기서 맞춰주지 않으면 조용히 틀린다:
  velocity  팀은 m/s, /Object_topic 소비자는 km/h (acc_planner.cpp 의 speedKmhToMps)
  방향      팀 yaw 는 라디안, ObjectStatus.heading 은 도

타입도 하나 다르다:
  RecognizedObject.center 는 geometry_msgs/Point,
  ObjectStatus.position   은 geometry_msgs/Vector3.
  필드 이름이 같아서 통째로 대입하고 싶어지지만 다른 타입이라 직렬화가 깨진다.
  x/y/z 를 하나씩 옮긴다.
"""
import math

from morai_msgs.msg import ObjectStatus, ObjectStatusList

MPS_TO_KMH = 3.6

# Planning 스펙 (object_fusion_node.py 의 CLASS_NAME_TO_TYPE 과 같은 정의)
TYPE_PEDESTRIAN = 0
TYPE_NPC = 1
TYPE_STATIC_OBSTACLE = 2


def to_object_status_list(tracked, stamp):
    """RecognizedObjectArray -> ObjectStatusList.

    stamp 가 None 이면 header.stamp 를 건드리지 않는다(테스트용).
    """
    out = ObjectStatusList()
    if stamp is not None:
        out.header.stamp = stamp
    # planning 의 전역경로와 같은 프레임. 팀 노드는 'odom' 으로 채워 보내지만
    # map 과 odom 은 항등 변환으로 이어져 있어 값은 같다(localization.launch 참고).
    out.header.frame_id = 'map'

    for obj in tracked.objects:
        if obj.type not in (TYPE_PEDESTRIAN, TYPE_NPC, TYPE_STATIC_OBSTACLE):
            continue                      # -1 = 자차. 버린다.

        st = ObjectStatus()
        st.unique_id = obj.unique_id
        st.type = obj.type
        st.name = obj.class_name

        st.position.x = obj.center.x      # Point -> Vector3, 통째 대입 금지
        st.position.y = obj.center.y
        st.position.z = obj.center.z

        st.size.x = obj.size.x
        st.size.y = obj.size.y
        st.size.z = obj.size.z

        st.heading = math.degrees(obj.yaw)

        st.velocity.x = obj.velocity.x * MPS_TO_KMH
        st.velocity.y = obj.velocity.y * MPS_TO_KMH
        st.velocity.z = obj.velocity.z * MPS_TO_KMH

        if obj.type == TYPE_PEDESTRIAN:
            out.pedestrian_list.append(st)
        elif obj.type == TYPE_NPC:
            out.npc_list.append(st)
        else:
            out.obstacle_list.append(st)

    out.num_of_pedestrian = len(out.pedestrian_list)
    out.num_of_npcs = len(out.npc_list)
    out.num_of_obstacle = len(out.obstacle_list)
    return out


def empty_object_status_list(stamp):
    """인지가 끊겼을 때 낼 빈 목록. '장애물 없음' 은 정상 상태다."""
    out = ObjectStatusList()
    if stamp is not None:
        out.header.stamp = stamp
    out.header.frame_id = 'map'
    return out
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
docker exec morai-dev bash -lc 'cd /home/dev/catkin_ws && source devel/setup.bash && \
  python3 src/path_tracking/scripts/test_object_convert.py'
```

기대: 모든 줄 `PASS`, 마지막 줄 `결과: 전부 통과`

- [ ] **Step 5: 커밋**

```bash
cd /home/yeon/morai-ros
git add catkin_ws/src/path_tracking/scripts/lib/object_convert.py \
        catkin_ws/src/path_tracking/scripts/test_object_convert.py
git commit -m "feat(planning): perception -> /Object_topic 변환 함수와 단위테스트

속도 m/s->km/h, yaw 라디안->도, center(Point)->position(Vector3) 를 맞춘다.
rospy 를 안 써서 ROS 마스터 없이 테스트된다."
```

---

### Task 5: 어댑터 노드

**파일:**
- 생성: `catkin_ws/src/path_tracking/scripts/object_topic_adapter.py`
- 수정: `catkin_ws/src/path_tracking/package.xml`
- 수정: `catkin_ws/src/path_tracking/CMakeLists.txt`

**인터페이스:**
- 소비: Task 4 의 `to_object_status_list()`, `empty_object_status_list()`
- 생산: `/Object_topic` (`morai_msgs/ObjectStatusList`) 20 Hz

- [ ] **Step 1: 노드 작성**

`catkin_ws/src/path_tracking/scripts/object_topic_adapter.py`:

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""object_topic_adapter : 팀 perception 의 결과를 planning 의 /Object_topic 으로 옮긴다.

  /perception/tracked_objects  ──>  [이 노드]  ──>  /Object_topic
  (autonomous_driving)                              (morai_msgs)

팀은 /Object_topic 을 만들지 않는다. object_fusion_node.py 머리말에
"(Planning 쪽 /Object_topic 스펙이 ...)" 이라고 적혀 있듯, 그 변환은 planning 몫이다.

이 노드는 얇게 유지한다 - 실제 변환은 lib/object_convert.py 가 하고 여기서는
구독/발행/타임아웃만 맡는다.

물체별 깜빡임 보정(latch)은 여기서 하지 않는다. tracking_node 가 이미 한다
(MIN_HITS_TO_CONFIRM=3, MAX_MISSES=5). 두 군데서 하면 지연이 겹치고 파라미터가
어긋났을 때 원인을 못 찾는다.

대신 "인지 노드가 통째로 죽는 경우"는 여기서 막는다. lattice_planner::objCb 는
받은 것을 덮어쓰기만 해서, 아무도 안 보내면 마지막 장애물을 영원히 믿는다.
이미 지나간 장애물을 계속 피하려 들게 된다.

사용법
  rosrun path_tracking object_topic_adapter.py
  rosrun path_tracking object_topic_adapter.py _timeout:=1.0

  _timeout : 이 시간[초] 동안 인지가 없으면 빈 목록을 낸다 (기본 0.5)
  _rate    : 발행 주기 [Hz] (기본 20 - mock_obstacle_pub 과 동일)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from autonomous_driving.msg import RecognizedObjectArray
from morai_msgs.msg import ObjectStatusList

from lib.object_convert import empty_object_status_list, to_object_status_list


class ObjectTopicAdapter:
    def __init__(self):
        self.timeout = rospy.get_param('~timeout', 0.5)
        rate_hz = rospy.get_param('~rate', 20.0)

        self.last_msg = None
        self.last_time = None
        self.stale_warned = False

        self.pub = rospy.Publisher('/Object_topic', ObjectStatusList, queue_size=1)
        rospy.Subscriber('/perception/tracked_objects', RecognizedObjectArray,
                         self.cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / rate_hz), self.tick)

        rospy.loginfo('[object_topic_adapter] start - %.1fHz, timeout %.2fs',
                      rate_hz, self.timeout)

    def cb(self, msg):
        if self.stale_warned:
            rospy.loginfo('[object_topic_adapter] perception 복구됨')
            self.stale_warned = False
        self.last_msg = msg
        self.last_time = rospy.Time.now()

    def tick(self, _event):
        now = rospy.Time.now()

        if self.last_msg is None:
            # 아직 한 번도 못 받았다. 시작 직후의 정상 상태다 - 경고하지 않는다.
            self.pub.publish(empty_object_status_list(now))
            return

        age = (now - self.last_time).to_sec()
        if age > self.timeout:
            if not self.stale_warned:
                self.stale_warned = True
                # ROS_INFO 포맷에 한글을 쓰면 컨테이너 로케일 때문에 깨진다
                rospy.logwarn('[object_topic_adapter] perception stale %.2fs '
                              '- publishing empty list', age)
            self.pub.publish(empty_object_status_list(now))
            return

        self.pub.publish(to_object_status_list(self.last_msg, now))


if __name__ == '__main__':
    rospy.init_node('object_topic_adapter')
    ObjectTopicAdapter()
    rospy.spin()
```

- [ ] **Step 2: 실행권한**

```bash
chmod +x catkin_ws/src/path_tracking/scripts/object_topic_adapter.py
```

- [ ] **Step 3: `package.xml` 에 의존 추가**

`<build_depend>morai_msgs</build_depend>` 줄 **뒤에** 넣는다:

```xml
  <!-- perception 결과(RecognizedObjectArray)를 /Object_topic 으로 변환하는
       object_topic_adapter.py 가 쓴다. planning -> perception 단방향이라
       순환 의존은 없다. -->
  <build_depend>autonomous_driving</build_depend>
  <exec_depend>autonomous_driving</exec_depend>
```

- [ ] **Step 3b: `CMakeLists.txt` 에도 의존 추가**

`find_package(catkin REQUIRED COMPONENTS ...)` 의 `std_msgs` 뒤에 넣는다:

```cmake
  autonomous_driving   # object_topic_adapter.py 가 쓰는 RecognizedObjectArray
```

파이썬 노드라 컴파일 의존은 없지만, 명시해 두면 catkin 이 `autonomous_driving`
메시지를 먼저 생성하도록 빌드 순서를 보장한다. (`autonomous_driving` 은
`path_tracking` 을 참조하지 않으므로 순환은 생기지 않는다.)

- [ ] **Step 4: 빌드해서 의존이 깨지지 않는지 확인**

```bash
docker exec morai-dev bash -lc 'cd /home/dev/catkin_ws && source /opt/ros/noetic/setup.bash && \
  catkin_make 2>&1 | tail -5'
```

기대: 에러 없이 완료. `autonomous_driving` 이 `path_tracking` 보다 먼저 빌드된다.

- [ ] **Step 5: 인지 없이 띄워서 타임아웃 동작 확인**

`perception.launch` 를 **끈 상태**로:

```bash
docker exec -it morai-dev bash -lc 'source /home/dev/catkin_ws/devel/setup.bash && \
  rosrun path_tracking object_topic_adapter.py'
```

다른 터미널에서:

```bash
timeout 6 rostopic hz /Object_topic 2>/dev/null | grep -m1 "average rate"
timeout 6 rostopic echo -n1 /Object_topic
```

기대: 약 20 Hz 로 발행되고, 내용은 `num_of_obstacle: 0` 인 빈 목록.
**인지가 없어도 토픽 자체는 계속 나와야 한다** — planning 이 "장애물 없음"과
"인지 죽음"을 구분할 필요 없이 안전하게 도는 설계다.

- [ ] **Step 6: 인지를 켜고 실제 물체가 실리는지 확인**

`perception.launch` 를 켠 뒤:

```bash
timeout 10 rostopic echo -n1 /Object_topic
```

기대: `obstacle_list` 또는 `npc_list` 에 항목이 있고 `position` 이 맵 좌표
스케일(수십~수백 m)이다. 값이 한 자리 수면 라이다 좌표가 그대로 새 나온 것이므로
`global_transform_node` 를 다시 볼 것.

- [ ] **Step 7: 커밋**

```bash
cd /home/yeon/morai-ros
git add catkin_ws/src/path_tracking/scripts/object_topic_adapter.py \
        catkin_ws/src/path_tracking/package.xml \
        catkin_ws/src/path_tracking/CMakeLists.txt
git commit -m "feat(planning): perception -> /Object_topic 어댑터 노드

20Hz 고정 발행. 인지가 0.5초 끊기면 빈 목록을 낸다 - lattice 가 지나간
장애물을 영원히 믿는 것을 막는다. 물체별 latch 는 tracking_node 몫이라 안 한다."
```

---

### Task 6: `sim.launch` 전환 스위치와 통합 검증

**파일:**
- 수정: `catkin_ws/src/path_tracking/launch/sim.launch`

**인터페이스:**
- 소비: Task 3 의 `perception.launch`, Task 5 의 어댑터

- [ ] **Step 1: `sim.launch` 수정**

`mock_obstacle_pub` 노드 줄을 통째로 아래로 바꾼다:

```xml
  <!-- 장애물 공급원 전환.
       perception:=false (기본) - mock_obstacle_pub 이 시나리오 좌표를 발행한다.
         오프라인 검증(test_lattice.py)과 회귀 비교의 기준이라 기본값으로 남겨둔다.
       perception:=true         - 팀 인지 스택 + 어댑터가 실제 검출을 발행한다.
         전제: udp_bridge 와 localization.launch 가 떠 있어야 한다(tf 필요). -->
  <arg name="perception" default="false"/>

  <node unless="$(arg perception)" pkg="path_tracking" type="mock_obstacle_pub"
        name="mock_obstacle_pub" output="screen"/>

  <include if="$(arg perception)" file="$(find autonomous_driving)/launch/perception.launch"/>
  <node if="$(arg perception)" pkg="path_tracking" type="object_topic_adapter.py"
        name="object_topic_adapter" output="screen"/>
```

- [ ] **Step 2: 기본값(mock)이 그대로인지 확인**

```bash
docker exec -it morai-dev bash -lc 'source /home/dev/catkin_ws/devel/setup.bash && \
  roslaunch path_tracking sim.launch'
```

다른 터미널:

```bash
rosnode list | grep -E "mock_obstacle_pub|object_topic_adapter"
```

기대: `mock_obstacle_pub` 만 나온다. **여기서 어댑터가 같이 뜨면 인자 조건이
반대로 걸린 것이다.**

- [ ] **Step 3: 오프라인 회귀가 안 깨졌는지 확인**

```bash
docker exec morai-dev bash -lc 'cd /home/dev/catkin_ws && source devel/setup.bash && \
  python3 src/path_tracking/scripts/test_lattice.py 2>&1 | tail -5'
docker exec morai-dev bash -lc 'cd /home/dev/catkin_ws && source devel/setup.bash && \
  catkin_make run_tests 2>&1 | grep -E "PASSED|FAILED" | tail -3'
```

기대: `test_lattice.py` 16/16, gtest 28개 통과. **여기가 깨지면 되돌린다.**

⚠️ `test_lattice.py` 를 돌리기 전에 `pgrep -x lattice_planner` 로 0개인지 확인할 것.
`rosrun` 을 죽여도 바이너리가 살아남아 같은 토픽에 이중 발행하면 결과가 뒤섞인다
(04-runbook.md 7절).

- [ ] **Step 4: 인지 모드로 띄우기**

```bash
docker exec -it morai-dev bash -lc 'source /home/dev/catkin_ws/devel/setup.bash && \
  roslaunch path_tracking sim.launch perception:=true'
```

```bash
rosnode list | grep -E "mock_obstacle_pub|object_topic_adapter|lidar_node|tracking_node"
```

기대: `mock_obstacle_pub` 은 **없고**, 어댑터와 인지 노드들이 있다.

- [ ] **Step 5: mock 과 실제 인지의 `/Object_topic` 을 나란히 비교 — 주행 전 관문**

두 모드에서 각각 한 건씩 떠서 필드별로 대조한다.

```bash
# mock 모드로 띄운 상태에서
timeout 6 rostopic echo -n1 /Object_topic > /tmp/obj_mock.txt
# perception 모드로 띄운 상태에서
timeout 10 rostopic echo -n1 /Object_topic > /tmp/obj_real.txt
diff <(sed 's/[-0-9.]\+/N/g' /tmp/obj_mock.txt) <(sed 's/[-0-9.]\+/N/g' /tmp/obj_real.txt)
```

숫자를 `N` 으로 바꿔서 **구조만** 비교한다. 기대: 목록 이름과 필드 구성이 같다.
차이가 나면 어느 필드가 빠졌는지 여기서 잡는다. **이 단계를 통과하기 전에는
주행하지 않는다.**

- [ ] **Step 6: 실주행**

`perception:=true` 로 한 바퀴 돌린다. 시나리오 정적장애물 구간을 지나며 확인:

```bash
# 별도 터미널에서 미리
rosrun path_tracking lap_logger.py _out:=/home/dev/catkin_ws/logs/lap_perception1.csv
rostopic echo /CollisionData
```

기대:
- RViz 에서 `object_viz` 마커가 **실제 장애물 자리**에 뜬다
- 회피가 일어나고 `/CollisionData` 에 아무것도 안 뜬다
- `lap_perception1.csv` 로 잰 여유가 mock 기준(`lap_return2`)과 크게 다르지 않다

⚠️ **여유가 나빠졌더라도 이 수치로 `SAFE_MARGIN` 을 재조정하지 말 것.**
지금 장애물 위치에는 EKF 횡오차 ~1 m 가 실려 있다(설계 8.1). EKF 를 고친 뒤
다시 재야 의미가 있다.

⚠️ **시간은 거리÷속도로 잴 것.** lap CSV 의 `t` 열은 벽시계라 배속에 따라
같은 주행이 다르게 나온다. 인지를 켜면 GPU/CPU 부하가 늘어 배속이 더 떨어진다.

- [ ] **Step 7: 커밋**

```bash
cd /home/yeon/morai-ros
git add catkin_ws/src/path_tracking/launch/sim.launch
git commit -m "feat(planning): sim.launch 에 perception 전환 인자 추가

perception:=true 면 인지 스택 + 어댑터, false(기본)면 mock. 기본값을 mock 으로
두는 이유는 오프라인 검증(test_lattice 16/16)의 기준을 잃지 않기 위해서다."
```

---

## 완료 조건

- [ ] `test_object_convert.py` 전부 통과
- [ ] `test_lattice.py` 16/16, gtest 28개 통과 (회귀 없음)
- [ ] `perception:=false` 로 기존과 동일하게 동작
- [ ] `perception:=true` 로 `/Object_topic` 에 실제 검출이 실림
- [ ] 인지를 죽여도 `/Object_topic` 이 20 Hz 로 계속 나오고 빈 목록이 됨
- [ ] 시나리오 정적장애물을 실제로 회피하고 `/CollisionData` 무발생

## 이 계획에서 하지 않는 것

- EKF 횡오차 수정 (`odom1_config` 의 vy) — 별도 작업. 설계 8.1/10 참고
- `isMissionObstacle` / `stoplineS` 하드코딩 좌표 제거 — 설계 8.4
- 인지 켠 상태에서의 회피 여유 재측정 및 `SAFE_MARGIN` 재검토 — EKF 수정 이후
