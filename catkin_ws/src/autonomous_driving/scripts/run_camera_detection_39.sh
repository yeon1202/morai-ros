#!/usr/bin/env bash
# camera_detection_node.py 를 YOLO 전용 Python 3.9 (/opt/yolo39) 로 실행한다.
#
# 왜 래퍼가 필요한가
#   ROS Noetic 은 Ubuntu 20.04 라 시스템 파이썬이 3.8 인데, PyTorch 의 cu128
#   빌드는 cp39 부터만 있다. RTX 5070 은 Blackwell(sm_120)이라 cu128 이 아니면
#   추론 시점에 "no kernel image is available" 로 죽는다. 그래서 이 노드만
#   3.9 venv 로 돌린다 (Dockerfile 의 /opt/yolo39 참고).
#
# 왜 셔뱅을 안 고치고 래퍼를 쓰는가
#   camera_detection_node.py 는 팀 코드다. 셔뱅을 고치면 팀이 그 파일을
#   업데이트할 때마다 매번 다시 고쳐야 한다. 래퍼는 팀 파일을 안 건드린다.
#
# 이 노드가 3.9 에서 도는 이유 (다른 인지 노드에는 그대로 적용 안 된다)
#   이 노드가 쓰는 ROS 모듈(rospy, sensor_msgs.msg, autonomous_driving.msg)은
#   전부 순수 파이썬이라 3.9 에서도 import 된다. cv_bridge / tf2_py 같은 C++
#   확장(.so)은 3.8 용으로 빌드돼 있어 3.9 에서 안 열린다 - 그걸 쓰는
#   global_transform_node 등은 반드시 3.8 로 돌려야 한다.
#   ROS 경로는 roslaunch 가 물려주는 PYTHONPATH 로 들어오므로 venv 에 안 넣는다.
set -e

# /opt/yolo39 는 우리 도커 이미지에만 있는 venv 다. 없으면 시스템 파이썬으로 돈다.
#
# 왜 없어도 되는가
#   3.9 가 필요한 이유는 순전히 GPU 다. RTX 5070 은 Blackwell(sm_120)이라 cu128
#   빌드가 있어야 하는데 그게 cp39 부터만 나온다. 다른 GPU 를 쓰는 PC 는 시스템
#   파이썬(3.8)용 torch 로 그냥 돌아간다.
#   ROS 쪽 import 는 어느 쪽이든 roslaunch 가 준 PYTHONPATH 로 들어온다.
#
# ⚠️ 폴백으로 갈 때는 그 파이썬에 torch / ultralytics / opencv 가 깔려 있어야 한다.
#    없으면 여기서가 아니라 노드 안에서 ImportError 로 죽는다.
PY39=/opt/yolo39/bin/python

if [ -x "$PY39" ]; then
  PY="$PY39"
else
  PY="$(command -v python3 || true)"
  if [ -z "$PY" ]; then
    echo "[run_camera_detection_39] python3 를 못 찾았다." >&2
    exit 1
  fi
  echo "[run_camera_detection_39] $PY39 가 없어 시스템 파이썬으로 돈다: $PY" >&2
  echo "  torch / ultralytics 가 이 파이썬에 깔려 있어야 한다." >&2
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODE="$HERE/../src/perception/camera_detection_node.py"

if [ ! -f "$NODE" ]; then
  echo "[run_camera_detection_39] 노드 파일이 없다: $NODE" >&2
  exit 1
fi

# "$@" 로 roslaunch 가 주는 __name:= / __log:= 인자를 그대로 넘긴다.
exec "$PY" "$NODE" "$@"
