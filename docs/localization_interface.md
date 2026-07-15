# Localization → Planning 인터페이스 명세

> Planning이 차량 위치를 받기 위한 규약. **Localization 팀(GPS+IMU 융합)은 아래 토픽/메시지로 발행**해 주세요.
> ⚠️ 대회 규정상 ground-truth(MoraiInfoPublisher) 사용 금지 → 위치는 반드시 GPS+IMU 기반.

## 1. 인터페이스

| 항목 | 값 |
|---|---|
| **토픽** | `/odom` (또는 `/localization/pose`) |
| **메시지** | **`nav_msgs/Odometry`** |
| **발행 주기** | 권장 30~50Hz (FOT 재계획·제어용) |

## 2. 메시지 내용 (`nav_msgs/Odometry`)

```
Header header              # frame_id = "map" (전역 ENU)
string child_frame_id      # "base_link"
geometry_msgs/PoseWithCovariance pose
  Pose pose
    Point position         # 전역 ENU 위치 [m]  ← ★
    Quaternion orientation # 자세(heading은 여기서 yaw로) ← ★
  float64[36] covariance   # 위치/자세 불확실성 (음영구역서 커짐)
geometry_msgs/TwistWithCovariance twist
  Twist twist
    Vector3 linear         # 속도 [m/s]  ← ★ FOT 종방향에 필요
    Vector3 angular        # 각속도(yaw rate)
  float64[36] covariance
```

## 3. ★ 반드시 지켜야 할 것

### (a) 좌표계 — **global ENU (map 프레임)**
- Planning의 경로·장애물(`/Object_topic`)과 **동일 ENU 전역좌표**.
- perception과도 같은 프레임이어야 회피 계산이 맞음.

### (b) velocity 포함
- `twist.linear` 로 속도 제공 (FOT 종방향 상태 `s'` 에 필요).

### (c) GPS 음영구역 연속성
- 음영구역(GPS blackout)에서도 **pose를 끊김 없이 발행** (IMU 추측항법으로).
- 불확실성은 `covariance` 로 표현 (Planning이 음영구간 보수적 주행에 참고 가능).

## 4. Localization 팀에 확인할 질문

1. **좌표계 global ENU 맞는지** (perception·경로와 일치).
2. **velocity(twist) 채워주는지**.
3. **음영구역서도 pose 연속 발행되는지** (추측항법).
4. **heading 규약** — quaternion yaw = ENU 기준인지.
5. 발행 주기.

## 5. 개발 협의

- Planning은 개발 중 **`/ego_status`(ground-truth)를 스탠드인**으로 사용.
- Localization 준비되면 **FOT 입력 토픽만 `/odom`으로 교체** (한 줄 수준).
- 대회 최종본은 **반드시 `/odom`(GPS+IMU)** — ground-truth 금지(실격).

---

**Planning이 쓰는 핵심**: `pose.position`(현재위치), `orientation`(heading), `twist.linear`(속도). covariance는 음영구간 보수주행 참고용.
