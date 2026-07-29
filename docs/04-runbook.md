# 실행 · 정지 런북

매번 다시 찾게 되는 명령어를 모아둔다. 위에서 아래 순서대로 하면 된다.

모든 명령은 컨테이너 `morai-dev` 안에서 돈다. 앞부분이 길어 아래처럼 줄여 쓴다.

```bash
# 이 문서에서 "DEV" 라고 쓰면 아래를 뜻한다
alias DEV='docker exec -it morai-dev bash -lc'
```

`-it` 여야 Ctrl+C 가 컨테이너 안 프로세스로 전달된다. `-i` 만 쓰면 안 끊긴다.

---

## 0. 컨테이너 준비

노트북을 절전(suspend)했다 왔으면 **반드시 먼저 한다.**

```bash
cd ~/morai-ros && docker compose down && docker compose up -d
```

서스펜드/리줌하면 그 전부터 돌던 컨테이너가 GPU 접근을 잃는다. 증상은
호스트 `nvidia-smi` 는 정상인데 컨테이너에서 `Failed to initialize NVML`,
`X_GLXCreateContext BadValue` 가 뜨고 MoraiLauncher_Lin 과 RViz 가 안 뜬다.
`restart` 보다 `down`/`up` 이 확실하다. NVIDIA 런타임 훅이 컨테이너를 만들 때
디바이스를 다시 물리기 때문이다.

잃는 것은 없다. MORAI 로그인은 named volume, `catkin_ws` 는 호스트 마운트다.
다만 컨테이너 안 `/tmp` 의 임시 스크립트는 날아간다(그래서 비상정지는
`/tmp` 가 아니라 `scripts/estop.py` 에 있다).

## 1. 브릿지 실행

```bash
DEV 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && roscore & sleep 2 && rosrun udp_bridge udp_bridge.py'
```

컨테이너 `.bashrc` 에 별칭이 있다.

```bash
DEV 'udp'
```

확인 — `/ego_status` 에 값이 흐르는지 본다.

```bash
DEV 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && rostopic echo -n 1 /ego_status'
```

`position` 이 0,0,0 으로 나오면 브릿지가 Competition Vehicle Status(9109)를
받고 있는 것이다. 규정상 최종본은 그게 맞지만 개발 검증에는 위치가 필요하다.
`udp_bridge.py` 의 `EGO_INFO_RECV_PORT` 를 9111 로 바꾸고 시뮬에서
Ego Vehicle Status 를 켜면 ground-truth 위치가 들어온다.

## 2. 주행 실행

기본 (경로추종 + ACC + lattice, RViz 포함):

```bash
DEV 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && roslaunch path_tracking acc.launch'
```

자주 쓰는 조합:

```bash
# 한 바퀴 완주 확인 — 가짜 앞차를 끈다
roslaunch path_tracking acc.launch mock_lead:=false

# 상승률 제한 튜닝
roslaunch path_tracking acc.launch accel_rate_limit:=0.5

# 앞차 조건 바꾸기
roslaunch path_tracking acc.launch start_gap:=50 lead_speed_kmh:=10

# 차를 안 움직이고 RViz 배치만 보기 (단 /local_path 가 안 나가 ACC 는 앞차를 못 잡는다)
roslaunch path_tracking acc.launch drive:=false
```

절전 후 GPU 가 의심스러우면 RViz 없이 헤드리스로 돈다.

```bash
DEV 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && cd /home/dev/catkin_ws/src/path_tracking/scripts &&
  rosrun path_tracking acc_planner > /tmp/acc.log 2>&1 &
  python3 path_tracker.py > /tmp/pt.log 2>&1 &
  sleep 2 && python3 diag_tracker.py'
```

**주의: `path_tracker` 를 두 번 띄우지 말 것.** ROS 는 같은 이름의 노드가 새로
등록되면 먼저 뜬 쪽을 죽인다. 그러면 `/local_path` 가 끊기고 ACC 가 앞차를
못 찾아 크루즈만 내보낸다.

## 3. 정지

**가장 중요한 절이다. MORAI 는 마지막으로 받은 `/ctrl_cmd` 를 계속 물고 있는다.**
노드만 죽이면 차는 마지막 accel 명령으로 계속 가속한다(실측 5.4 -> 17.4 m/s).

정상 정지:

```bash
DEV 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && rosnode kill /path_tracker && rosrun path_tracking estop.py'
```

`path_tracker.py` 에는 SIGINT 를 가로채 제동하는 장치가 들어 있어 Ctrl+C 로도
선다. 다만 crash 나 `kill -9` 로 그 경로를 못 타는 경우가 있어 `estop.py` 를
따로 둔다.

비상정지 (뭐가 됐든 일단 세우기):

```bash
DEV 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && rosrun path_tracking estop.py'
DEV '... rosrun path_tracking estop.py _sec:=10'   # 더 오래 밟기
```

**`pkill -f "python3 path_tracker.py"` 는 듣지 않는다.** roslaunch 로 띄운 노드는
전체 경로로 실행되어 그 패턴에 안 걸린다. 반드시 `rosnode kill` 을 쓴다.

## 4. 진단 도구

전부 읽기 전용이거나 차를 세워둔 채 도는 것이다.

```bash
# 경로 추종 상태 — 최근접 idx / CTE / 전방점 / 목표점 / 조향 / 시작점까지 나침반
DEV '... && cd /home/dev/catkin_ws/src/path_tracking/scripts && python3 diag_tracker.py'

# 조향 포화 지점과 단위 판정 (브레이크 밟은 채 조향만 단계별로 올린다)
DEV '... && rosrun path_tracking diag_steer.py'

# 목표속도를 직접 관측 (노드 로그와 독립적인 증거)
DEV '... && rostopic echo -p /target_velocity'
```

경로 관련 오프라인 도구 (ROS 불필요, 호스트에서 바로 실행 가능):

```bash
python3 scripts/path_join.py          # approach.csv + course.csv -> path.csv
python3 scripts/path_smoother.py      # path.csv -> path_smooth.csv
python3 scripts/test_path_manager.py  # 겹침 구간 최근접 탐색 회귀 테스트
```

경로를 새로 기록할 때는 **파일명을 반드시 준다.** 안 주면 기존 `path.csv` 를
그 자리에서 덮어쓴다.

```bash
DEV '... && rosrun path_tracking path_recorder.py _file:=approach.csv'
```

## 5. 빌드와 테스트

```bash
# 전체 빌드
DEV 'source /opt/ros/noetic/setup.bash && source /home/dev/catkin_ws/devel/setup.bash && cd /home/dev/catkin_ws && catkin_make'

# 특정 타깃만
DEV '... && cd /home/dev/catkin_ws && catkin_make acc_planner acc_core_test'

# 단위 테스트 (ACC 순수 로직)
DEV '... && /home/dev/catkin_ws/devel/lib/path_tracking/acc_core_test'
```

Python 노드는 재빌드가 필요 없다. `catkin_ws` 가 호스트 마운트라 호스트에서
파일을 고치면 컨테이너에 바로 반영된다.

## 6. 경로 재기록 (전체)

global path 는 lattice·ACC·behavior 가 전부 그 위에서 도는 토대다. 다시 딸 때는
아래 순서를 그대로 따른다.

### 6.0 준비

```bash
# 백업 (git 에도 있지만 손에 두는 편이 편하다)
cd ~/morai-ros/catkin_ws/src/path_tracking/path
for f in approach.csv course.csv path.csv path_smooth.csv; do cp -v $f $f.bak; done
```

주행 노드를 반드시 내린다. 켜져 있으면 차가 저 혼자 달린다.

```bash
DEV '... && rosnode kill /path_tracker /acc_planner /lattice_planner 2>/dev/null; rosnode cleanup'
```

`/ego_status` 의 position 이 0,0,0 이면 브릿지가 Competition(9109)을 받는 중이라
기록이 불가능하다. `udp_bridge.py` 의 `EGO_INFO_RECV_PORT` 를 9111 로 바꾸고
시뮬에서 Ego Vehicle Status 를 켠다.

MORAI 를 **수동 조작 모드**로 바꾼다. 마지막 `/ctrl_cmd`(보통 estop 의 brake=1)를
계속 물고 있어 그냥은 안 움직인다.

### 6.1 접근구간 기록 (스폰 -> 대회 라인 진입)

차량을 스폰 지점 `(-14.19, -224.21)` 으로 리셋하고,

```bash
DEV '... && rosrun path_tracking path_recorder.py _file:=approach.csv'
```

대회 라인 진입점까지 몰고 **Ctrl+C**. 진입점을 조금 지나쳐도 된다.
`path_join.py` 가 지나친 만큼 잘라낸다.

**`_file:=` 를 빼먹으면 기존 `path.csv` 를 그 자리에서 덮어쓴다.**

### 6.2 코스 기록 (한 바퀴)

진입점에서 이어서,

```bash
DEV '... && rosrun path_tracking path_recorder.py _file:=course.csv'
```

한 바퀴 돌고 **출발점을 조금 지나서** Ctrl+C. 겹친 만큼은 `path_join.py` 가
잘라낸다. 반대로 못 미치면 경로가 끊긴다.

기록 중에는 **차로 중앙을 따라 매끄럽게** 몰 것. 여기서 대충 몰면 그 오차가
그대로 global path 가 되고, 추종을 아무리 잘해도 복구되지 않는다.
차선 변경은 코스상 필요한 곳에서만 한다.

### 6.3 합치고 다듬기

```bash
cd ~/morai-ros/catkin_ws/src/path_tracking/scripts
python3 path_join.py        # approach + course -> path.csv
python3 path_smoother.py    # path.csv -> path_smooth.csv (이동평균)
# 코너가 너무 깎이면 창 크기를 줄인다 (기본 9)
python3 path_smoother.py 5
```

### 6.4 검증 (주행 전에)

```bash
python3 test_path_manager.py   # 겹침 구간 최근접 탐색 + 완주 latch
```

둘 다 PASS 여야 한다. 그 다음 실제 주행으로 확인한다.

```bash
DEV '... && rosrun path_tracking lap_logger.py _out:=/tmp/lap.csv'
```

기준값(2026-07-29 초안 경로): 대회구간 CTE 평균 0.149m, 최대 0.429m,
차로여유 0.654m 초과 0건. 새 경로가 이보다 나빠지면 안 된다.

## 7. 자주 걸리는 함정

| 증상 | 원인과 대처 |
|---|---|
| 컨테이너에서 GPU 가 안 잡힌다 | 절전 후 증상. `docker compose down && up -d` (0절) |
| 로그가 하나도 안 찍힌다 | 노드 stdout 은 **블록 버퍼링**이다. `kill` 하면 버퍼가 통째로 날아간다. **로그가 비었다고 기능이 안 돈 것으로 읽지 말 것.** `stdbuf -oL -eL` 를 앞에 붙이거나 토픽을 직접 관측한다 |
| 파라미터를 안 줬는데 이상한 값으로 돈다 | **rosparam 은 노드가 죽어도 마스터에 남는다.** `_param:=값` 을 한 번 주면 계속 그 값이다. `rosparam list` 로 확인, `rosparam delete /acc_planner` 로 정리 |
| 로그의 한글이 `???` 로 깨진다 | 컨테이너 로케일이 UTF-8 이 아니다. **`ROS_INFO` 포맷 문자열에는 한글을 쓰지 않는다.** 주석의 한글은 무관하다 |
| 차가 명령을 무시하고 계속 간다 | MORAI 가 마지막 `/ctrl_cmd` 를 물고 있다. `estop.py` (3절) |
| `pkill` 로 노드가 안 죽는다 | roslaunch 노드는 전체 경로로 실행된다. `rosnode kill` 을 쓴다 |
| 차가 경로에서 멀리 떨어져 정지한다 | `path_tracker` 의 `MAX_CTE`(6.0m) 가드다. 시뮬에서 차를 경로 위로 되돌린다 |
| 속도가 이상하다 | `/ego_status` 의 velocity 는 m/s 가 아니라 **km/h** 다. 소비 지점마다 변환해야 한다 |

## 8. 고정값

바꾸기 전에 이유를 먼저 확인할 값들이다.

| 값 | 위치 | 이유 |
|---|---|---|
| 크루즈 **55 km/h** | `acc.launch` arg, `acc_planner.cpp` 기본값 (**둘이 같아야 한다**) | 규정 상한은 60 이지만 target 60 이면 실측이 60.1~60.2 로 넘어간다. 60 초과는 15초 + 3초당 15초 패널티 |
| 하드캡 60 km/h | `acc_planner.cpp` `max_kmh` | 규정값 그대로 |
| `MAX_STEER` 0.65 rad | `path_tracker.py` | 차량 한계는 0.698(40도). 시뮬이 잘라내는 지점에 명령을 걸치지 않도록 여유를 둔다 |
| `accel_rate_limit` 1.0 | `acc.launch` arg | 유턴 탈출 실측 가속도가 2.4 m/s^2 라 그 이상은 효과가 없다. 진동 폭 1.22 -> 0.21m 로 확인됨 |
| `LFD_GAIN` 0.5 | `path_tracker.py` | 경로에 기록된 차선변경(횡 3.8m 를 44m 에 걸쳐 이동)에서 0.7 이면 S자를 가로지른다 |
