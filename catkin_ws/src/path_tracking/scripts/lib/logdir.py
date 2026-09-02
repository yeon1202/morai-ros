# -*- coding: utf-8 -*-
"""진단 스크립트들의 CSV 저장 폴더를 환경에 맞게 고른다.

왜 필요한가
  예전엔 '/home/dev/catkin_ws/logs' 가 각 스크립트에 박혀 있었다. 이건 도커
  개발환경에서만 존재하는 경로다(호스트의 catkin_ws 가 그 자리에 마운트돼
  있어서, 컨테이너 안에서 쓴 CSV 를 호스트에서 바로 열 수 있다).
  도커를 안 쓰는 PC 에서는 그 폴더가 없어서 진단 노드가 시작하자마자
  FileNotFoundError 로 죽는다.

고르는 순서
  1. _out_dir 파라미터 (호출자가 명시하면 무조건 그것)
  2. 마운트된 워크스페이스 logs/ 가 실제로 있으면 그것 (도커 개발환경 유지)
  3. 없으면 ~/morai_logs (없으면 만든다)
"""
import os

# 도커 개발환경에서 호스트와 공유되는 자리. 있으면 여기에 쌓아야 호스트에서
# 바로 분석할 수 있다.
_MOUNTED = '/home/dev/catkin_ws/logs'
_FALLBACK = '~/morai_logs'


def default_log_dir():
    if os.path.isdir(_MOUNTED):
        return _MOUNTED
    d = os.path.expanduser(_FALLBACK)
    os.makedirs(d, exist_ok=True)
    return d
