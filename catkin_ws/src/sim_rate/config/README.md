# localization 설정 (sim_rate 패키지 소유 사본)

`autonomous_driving/config/` 의 사본이고, **`sim_rate/launch/localization.launch`
는 이쪽을 읽는다.**

## 왜 따로 두나

배속 보정을 켜려면 `ekf.yaml` 을 고쳐야 하는데(아래), 그건 localization 팀
설정이다. 같은 파일을 두 팀이 각자의 이유로 고치면 서로 덮어쓴다. 그래서
`sim_rate` 는 자기 사본을 들고 다니고, 팀 원본은 건드리지 않는다.

`localization.launch` 의 `config` 인자로 경로를 넘긴다:

    <arg name="config" default="$(find sim_rate)/config"/>

## 팀 원본과 다른 곳 (2026-08-29 기준)

| 항목 | 팀 원본 | 여기 | 왜 |
|---|---|---|---|
| `smooth_lagged_data` | 없음(false) | `true` | 브릿지가 GPS 스탬프를 전송지연(0.30초)만큼 되감는데, 이게 없으면 EKF 가 "너무 오래된 관측" 이라며 GPS 를 통째로 버린다 |
| `history_length` | 없음 | `0.5` | 되감기 폭(0.30)보다 커야 한다 |
| `odom1` | `/odom/wheel_speed` | `/odom/wheel_speed_scaled` | `wheel_speed_scaler` 가 배속만큼 줄인 속도 |
| `imu0_config` 가속도 | `true, true` | `false, false` | 시뮬 단위 가속도를 벽시계 dt 로 적분하는 경로 제거 |
| `odom1_config` vy | `true` | `false` | vy 가 적분 추정치라 편향이 있다. 실측에서 횡 RMS 0.99 → 0.10 m |

각 항목의 근거는 `ekf.yaml` 안 주석에 실측치와 함께 적혀 있다.

## 주의

**팀이 `autonomous_driving/config/` 를 고쳐도 여기엔 반영되지 않는다.**
팀 설정이 바뀌면 위 표의 항목만 남기고 나머지는 여기로 가져와야 한다.
