# Perception → Planning 인터페이스 명세

Planning이 장애물 정보를 소비하기 위한 규약이다. Perception 팀은 아래 토픽/메시지 규약에 맞춰 발행한다.

## 1. 인터페이스 요약

| 항목 | 값 |
|---|---|
| 토픽 | `/Object_topic` |
| 메시지 | `morai_msgs/ObjectStatusList` |
| 발행 주기 | 최소 10Hz, 20~30Hz 권장 (ego 상태 갱신과 유사하게) |

MORAI 표준 메시지(`morai_msgs`)를 사용하므로 별도 커스텀 메시지는 불필요하다.

## 2. 메시지 구조

### `ObjectStatusList` (전체 묶음)
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
int32 unique_id                    # 객체 고유 id (프레임 간 동일 객체는 동일 id 유지)
int32 type                         # 0:보행자  1:NPC차량  2:정적장애물  -1:ego
string name
float64 heading                    # 진행 방향 [deg]
geometry_msgs/Vector3 velocity     # 속도 [km/h]
geometry_msgs/Vector3 acceleration # 가속도 [m/s^2]
geometry_msgs/Vector3 size         # 바운딩박스 (width, length, height) [m]
geometry_msgs/Vector3 position     # 위치 [m]
```

Planning이 실제로 소비하는 핵심 필드: `position`(충돌검사), `size`(장애물 크기), `type`(보행자 여부에 따른 회피/급정지 강도), `velocity`(움직이는 차 예측).

## 3. 준수 요건 (통합 실패 1순위)

### (a) 좌표계 — `position`은 global ENU (map 프레임)
- Planning의 경로 및 차량 위치와 동일한 map 프레임 ENU 전역 좌표를 사용한다 (localization의 `/odom`과 동일 프레임, 개발 단계에서는 `/ego_status` 스탠드인과 동일).
- 센서/차량 기준 상대좌표가 아니라 맵 전역 좌표로 발행한다.
- 센서 검출이 차량 기준으로 나오면 ego pose로 전역 변환 후 발행한다.
- 이 기준이 어긋나면 회피 로직이 장애물을 잘못된 위치에 놓는다.

### (b) 단위 (메시지 정의 그대로)
- `position`, `size` = m
- `velocity` = km/h
- `heading` = deg

### (c) 빈 경우 처리
- 감지된 객체가 없어도 빈 배열 + `num_* = 0`으로 계속 발행한다 (토픽 단절 방지).

### (d) `position` 기준점
- 객체의 중심점 기준으로 발행한다. Planning은 중심점 + `size`로 바운딩박스 충돌검사를 수행한다.

## 4. 협의 필요 사항

1. Ground truth vs 실제 센서 검출 여부.
   - 검출 결과라면 미검출·지연·노이즈가 발생하므로 Planning이 이를 감안한다.
2. 좌표계가 global ENU가 맞는지 재확인.
3. `type` 값 규약이 위 표(0/1/2/-1)대로인지.
4. `unique_id`가 프레임 간 유지되는지 (동적 객체 추적/예측에 필요).

## 5. 통합 단계 협의

- Planning은 mock publisher(`/Object_topic` 동일 인터페이스)로 선행 개발한다.
- Perception 준비 시 토픽·메시지·좌표계가 본 규약과 일치하면 그대로 연결된다.
- 연결 전 `rostopic echo /Object_topic`으로 공동 확인을 권장한다.
