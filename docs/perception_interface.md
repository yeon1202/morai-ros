# Perception → Planning 인터페이스 명세

> Planning 팀이 장애물 정보를 받기 위한 규약. **Perception 팀은 아래 토픽/메시지로 발행**해 주세요.

## 1. 인터페이스 (한 줄 요약)

| 항목 | 값 |
|---|---|
| **토픽 이름** | `/Object_topic` |
| **메시지 타입** | `morai_msgs/ObjectStatusList` |
| **발행 주기** | 최소 10Hz, 권장 20~30Hz (ego 상태와 비슷하게) |

이 메시지는 MORAI 표준(`morai_msgs`)이라 별도 커스텀 메시지 만들 필요 없습니다.

## 2. 메시지 구조

### `ObjectStatusList` (한 번에 보내는 전체 묶음)
```
Header header

int32 num_of_npcs          # NPC(움직이는 차) 개수
int32 num_of_pedestrian    # 보행자 개수
int32 num_of_obstacle      # 정적 장애물 개수

ObjectStatus[] npc_list          # NPC 차량 목록
ObjectStatus[] pedestrian_list   # 보행자 목록
ObjectStatus[] obstacle_list     # 정적 장애물 목록
```

### `ObjectStatus` (객체 하나)
```
int32 unique_id                    # 객체 고유 id (추적용 - 프레임 간 동일 객체는 같은 id 유지 권장)
int32 type                         # 0:보행자  1:NPC차량  2:정적장애물  -1:ego
string name
float64 heading                    # 진행 방향 [deg]
geometry_msgs/Vector3 velocity     # 속도 [km/h]
geometry_msgs/Vector3 acceleration # 가속도 [m/s^2]
geometry_msgs/Vector3 size         # 바운딩박스 (width, length, height) [m]
geometry_msgs/Vector3 position     # 위치 [m]  ← ★★ 아래 3번 필수
```

## 3. ★ 반드시 지켜야 할 것 (통합 실패 1순위)

### (a) 좌표계 — `position`은 **global ENU (map 프레임)**
- Planning의 경로·차량 위치(`/ego_status.position`)와 **동일한 ENU 전역 좌표계**여야 함.
- 즉 "센서/차량 기준 상대좌표"가 아니라 **맵 전역 좌표**로 주세요.
- (센서 검출이 차량 기준으로 나오면, ego pose로 전역좌표 변환 후 발행)
- 이게 어긋나면 회피 로직이 장애물을 엉뚱한 데 놓습니다.

### (b) 단위 (메시지 정의 그대로)
- `position`, `size` = **m**, `velocity` = **km/h**, `heading` = **deg**

### (c) 빈 경우 처리
- 감지된 객체가 없으면 **빈 배열 + `num_* = 0`** 으로라도 계속 발행 (토픽 끊기지 않게)

### (d) `position` 기준점
- 객체의 **중심점**인지 협의 필요 (Planning은 중심 + `size`로 바운딩박스 충돌검사 예정)

## 4. Perception 팀에 확인할 질문

1. **Ground truth vs 실제 센서 검출?**
   - MORAI 정답값(Object Info 그대로) 인지, 카메라/라이다 검출 결과인지.
   - 검출이면 **미검출·지연·노이즈**가 생기니 Planning이 그걸 감안해야 함(중요).
2. **좌표계** 다시 확인 — global ENU 맞는지.
3. **`type` 값 규약** — 위 표(0/1/2)대로인지.
4. **`unique_id`** 가 프레임 간 유지되는지 (동적 객체 추적/예측에 필요).

## 5. 통합 전 협의 (권장)

- Planning은 **가짜 장애물(mock) publisher**로 먼저 개발 중.
- Perception 준비되면 **토픽/메시지/좌표계만 위 규약과 일치**하면 그대로 붙습니다.
- 붙이기 전에 `rostopic echo /Object_topic` 로 한 번 같이 확인하면 좋음.

---

**Planning이 실제로 쓰는 핵심 필드**: `position`(회피 경로 충돌검사), `size`(장애물 크기), `type`(보행자면 더 크게 회피/급정지), `velocity`(움직이는 차 예측). 나머지는 있으면 좋고.
