# Localization → Planning 인터페이스 명세

Planning이 차량 위치를 소비하기 위한 규약이다. Localization 팀(GPS+IMU 융합)은 아래 토픽/메시지 규약에 맞춰 발행한다.

대회 규정상 ground-truth(MoraiInfoPublisher) 사용은 금지되므로, 위치는 반드시 GPS+IMU 기반으로 산출해야 한다.

## 1. 인터페이스 요약

| 항목 | 값 |
|---|---|
| 토픽 | `/odom` |
| 메시지 | `nav_msgs/Odometry` |
| 발행 주기 | 30~50Hz 권장 (lattice 재계획 및 종방향 제어용) |

## 2. 메시지 내용 (`nav_msgs/Odometry`)

```
Header header              # frame_id = "map" (전역 ENU)
string child_frame_id      # "base_link"
geometry_msgs/PoseWithCovariance pose
  Pose pose
    Point position         # 전역 ENU 위치 [m]
    Quaternion orientation # 자세 (heading은 yaw로 추출)
  float64[36] covariance   # 위치/자세 불확실성 (음영구역에서 커짐)
geometry_msgs/TwistWithCovariance twist
  Twist twist
    Vector3 linear         # 속도 [m/s] (종방향 제어에 필요)
    Vector3 angular        # 각속도 (yaw rate)
  float64[36] covariance
```

Planning이 실제로 소비하는 핵심 필드: `pose.position`(현재 위치), `pose.orientation`(heading), `twist.linear`(속도). `covariance`는 음영구간 보수 주행 판단의 참고값이다.

## 3. 준수 요건

### (a) 좌표계 — global ENU (map 프레임)
- Planning의 경로 및 장애물(`/Object_topic`)과 동일한 ENU 전역 좌표계를 사용한다.
- perception과도 동일 프레임이어야 회피 계산이 성립한다.

### (b) velocity 포함
- `twist.linear`로 속도를 제공한다. 종방향 제어(ACC)의 상태 입력으로 사용된다.

### (c) GPS 음영구역 연속성
- 음영구역(GPS blackout)에서도 pose를 끊김 없이 발행한다 (IMU 추측항법 기반).
- 불확실성은 `covariance`로 표현한다.

## 4. 협의 필요 사항

1. 좌표계가 global ENU가 맞는지 (perception·경로와 일치).
2. velocity(`twist.linear`)를 채워 발행하는지.
3. 음영구역에서도 pose가 연속 발행되는지 (추측항법).
4. heading 규약 — quaternion yaw가 ENU 기준인지.
5. 발행 주기 확정.

## 5. 개발 단계 협의

- Planning은 개발 중 `/ego_status`(ground-truth)를 임시 스탠드인으로 사용한다.
- Localization 준비 시 입력 토픽을 `/odom`으로 교체한다 (코드 변경 최소).
- 대회 최종본은 반드시 `/odom`(GPS+IMU)을 사용한다. ground-truth 사용 시 실격.
