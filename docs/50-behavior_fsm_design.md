# Behavior FSM 설계 — 우선순위 중재와 횡방향 상태기계

작성일: 2026-08-03 / planning
관련: [40-acc_design.md](40-acc_design.md), [30-lattice_design.md](30-lattice_design.md),
[51-traffic_light_brief.md](51-traffic_light_brief.md) 5절, [10-planning_wbs.md](10-planning_wbs.md) 5장

---

## 0. 한 줄 요약

**종방향은 상태가 없다. 제약을 모아 `min()` 하면 된다.
상태기계가 필요한 곳은 횡방향이다 — 차선변경은 한순간이 아니라 과정이기 때문이다.**

## 1. 왜 만드나

종방향 속도를 정할 이유가 여럿이고 계속 늘어난다. 크루즈, 곡률, 앞차, 신호등,
보행자, 회피불가. 각자 브레이크를 밟으면 서로 싸운다.

횡방향도 마찬가지다. `lattice` 는 "장애물이 있으니 피한다" 는 할 수 있지만
"내가 추월하기로 결정했다" 는 표현할 수 없다. 결정할 주체가 없다.

`behavior_fsm` 이 그 두 가지를 맡는다.

## 2. 구조

```
acc_planner      → /speed_limit/acc           크루즈·곡률·앞차
[신호등]         → /speed_limit/traffic_light
[보행자]         → /speed_limit/pedestrian    (perception 대기)
lattice_planner  → /speed_limit/avoid         전 후보 막힘
(FSM 내부)       → /speed_limit/intersection  정지선까지 남은 거리
(FSM 내부)       → /speed_limit/highway       고주로 (v1 은 끔)
                          │
                    ┌─────▼──────────────────────────┐
                    │        behavior_fsm            │
                    │                                │
                    │  종방향:  min() → 상승률 제한   │
                    │  횡방향:  상태기계              │
                    └─────┬──────────────────┬───────┘
                          │                  │
                   /target_velocity    /lateral_intent
                          │                  │
                    path_tracker      lattice_planner
                    (control 팀 노드로          │
                     대체 예정)          /lattice_status
                                              │
                                        (FSM 으로 되먹임)
```

### 왜 이렇게 나누나

각 노드가 **자기가 아는 것만** 말한다.

- `acc_planner` 는 "앞차와 곡률을 고려하면 이 속도까지" 라고만 한다. 신호등을 모른다
- `lattice_planner` 는 "이 경로로 가면 안 부딪힌다" 고만 한다. 왜 그 차선인지 모른다
- `behavior_fsm` 만 전체를 보고 결정한다

신호등 담당자가 붙어도, 보행자가 추가돼도 **FSM 이 구독을 하나 늘릴 뿐**이다.
다른 노드는 바뀌지 않는다.

## 3. 기존 노드에서 바뀌는 것

### `acc_planner`

| | 지금 | 바뀜 |
|---|---|---|
| 발행 | `/target_velocity` | **`/speed_limit/acc`** |
| 크루즈·곡률·앞차 | 있음 | 그대로 |
| `rampTarget` (상승률 제한) | 있음 | **FSM 으로 이동** |

**상승률 제한을 옮기는 것이 핵심이다.** 그것은 "제약이 풀리는 순간의 급가속" 을
막는 장치인데, `min()` **뒤에** 있어야 의미가 있다. `acc_planner` 안에 두면
신호등이 녹색으로 바뀌는 순간의 급가속은 막지 못한다.

`rampTarget` 자체는 순수 함수(`acc_core.hpp`)이므로 호출 위치만 옮기면 된다.
2026-07-29 실차 검증된 로직을 그대로 재사용한다.

### ⚠️ ACC 와 lattice 가 서로를 모른다 (2026-08-06 실측)

회피 구간(328.9m)에서 속도가 **51.9 -> 15.5 km/h** 까지 떨어졌다. `acc_core` 의
`selectLead` 가 정적 장애물도 앞차 후보로 보기 때문이다.

안전하지만 손해다. lattice 가 이미 피하고 있는데 ACC 는 그걸 모르고 브레이크를 밟는다.
**"피할 수 있으니 덜 줄여도 된다" 를 조율할 층이 없다** — FSM 이 풀어야 할 문제다.

v1 에서 다루는 방법: `lattice_status.intent_feasible` 이 참이면(= 회피 경로가 살아 있으면)
FSM 이 `/speed_limit/acc` 를 완화해서 쓴다. 완화량은 구현·튜닝 단계에서 정한다.

### `lattice_planner`

- `/lateral_intent` 구독 → **의도 방향 후보에 가점**
- `/lattice_status` 발행 → 의도 달성 여부 보고
- `/speed_limit/avoid` 발행 → 전 후보 막힘

**후보 생성·충돌검사·비용선택 로직은 건드리지 않는다.** 비용 함수에 항이 하나
붙을 뿐이다. 2026-07-31 재설계분이 그대로 살아있다.

### `path_tracker`

변경 없음. `/target_velocity` 를 계속 구독한다.

## 4. 횡방향 상태기계

### 상태

```
intent :  KEEP  │  LEFT  │  RIGHT
phase  :  (KEEP 일 땐 없음)   WAIT  │  EXEC
```

3×3 이 아니라 **KEEP + (LEFT|RIGHT)×(WAIT|EXEC) = 5가지**다.

```
                    ┌──────────┐
          ┌────────▶│   KEEP   │◀────────┐
          │         └────┬─────┘         │
     ⑤ 포기              │ ① 트리거      │ ③ 완료
          │              ▼               │
          │      ┌───────────────┐       │
          └──────┤  L/R  WAIT    │       │
                 └───────┬───────┘       │
                         │ ② 진입가능     │
                         ▼               │
                 ┌───────────────┐       │
                 │  L/R  EXEC    ├───────┘
                 └───────┬───────┘
                         │ ④ 다시 막힘
                         └──────▶ WAIT
```

### 전이 조건

| | 전이 | 조건 |
|---|---|---|
| ① | KEEP → WAIT | 차선변경 트리거 발생 (4.3절) |
| ② | WAIT → EXEC | `lattice_status.intent_feasible == true` |
| ③ | EXEC → KEEP | `selected_offset` 이 목표 차선에 도달 + `DONE_HOLD_TICKS` 유지 |
| ④ | EXEC → WAIT | `intent_feasible == false` 가 `BLOCK_TICKS` 지속 |
| ⑤ | WAIT → KEEP | `WAIT_TIMEOUT` 초과 또는 트리거 소멸 |

### 4.2 ② 를 lattice 보고로 대체한 이유

간격이 있는지를 FSM 이 직접 판단하지 않는다. `lattice` 는 이미 장애물을 보고
충돌검사를 하고 있으므로, **"의도 방향 후보가 살아남았다" 가 곧 간격 신호**다.

덕분에 **FSM 은 `/Object_topic` 을 구독하지 않는다.** 객체 처리 로직이 두 곳에
생기는 것을 막는다. `lattice` 의 충돌검사는 전이 길이(2.68초) 구간을 훑으므로
어느 정도 앞을 본다 — v1 에는 충분하다.

### 4.3 트리거 — v1 은 새 토픽 없이

```
/speed_limit/acc 가 크루즈 속도보다 OVERTAKE_MARGIN 이상 낮은 상태가
OVERTAKE_HOLD 초 지속  →  "앞차 때문에 못 달리고 있다"  →  추월 트리거
```

`acc_planner` 가 앞차를 잡으면 제약값이 떨어진다. **그 값 자체가 신호**이므로
앞차 정보를 따로 발행할 필요가 없다.

방향은 v1 에서 **좌측 우선**(추월의 기본). MGeo 도입 후 `left_link_id` /
`right_link_id` 로 실제 차로 유무를 확인하도록 교체한다.

### 4.4 ⚠️ 알려진 한계 — 곡선 구간

`lattice` 의 후보는 **`/local_path` 기준 offset** 이다. 차선을 바꿔도 기준경로는
원래 차선에 남아 있으므로, 차선변경은 "영구적으로 3.5m offset 을 유지하는 것" 으로
표현된다. (2026-07-31 tail offset 유지 방식을 그대로 쓴다.)

**곡선 구간에서 어긋난다.** 경로를 평행이동하면 곡률이 달라진다 — 안쪽으로 3.5m
밀면 반경이 3.5m 줄어든다.

**v1 은 직선 구간 추월에 한정한다.** 곡률이 `LANE_CHANGE_KAPPA_MAX` 이상인
구간에서는 트리거를 걸지 않는다.

**MGeo 도입 시 자연히 해소된다.** `left_link_id` 로 옆 차선의 실제 중심선을 받아
기준경로를 교체하면 된다. → 6절 이행 경로.

## 5. 인터페이스

### 5.1 속도 제약 — `/speed_limit/*`

`51-traffic_light_brief.md` 5절 규약을 그대로 따른다.

| 항목 | 값 |
|---|---|
| 타입 | `std_msgs/Float64` |
| 단위 | **m/s** |
| 의미 | **지금 이 순간 허용되는 최대 속도** (목표속도가 아니라 상한) |
| 제한 없음 | `1e6` |
| 정지 | `0.0` |
| 주기 | 10Hz 이상 |

### 5.1.1 제약 생산자 목록 (2026-08-10 확정)

| 토픽 | 생산자 | 상태 |
|---|---|---|
| `/speed_limit/acc` | `acc_planner` | 있음(발행 토픽만 바꾸면 됨) |
| `/speed_limit/traffic_light` | 신호등 담당자 | 대기 |
| `/speed_limit/pedestrian` | perception | **대기. mock 추가 안 함** |
| `/speed_limit/avoid` | `lattice_planner` | 미구현 |
| `/speed_limit/intersection` | `behavior_fsm` 내부 | **신규.** 정지선 5곳(`52-stopline_table.txt`)까지 남은 거리로 미리 감속 |
| `/speed_limit/highway` | `behavior_fsm` 내부 | **신규. v1 에서는 끈다** — 아래 |

**구간 제한속도(`section`)는 넣지 않는다.** 대회 규정이 전구간 60kph 이고 크루즈 55 가
이미 그 아래다. MGeo `max_speed` 를 상한으로 쓸 이유가 없다.

### 5.1.2 ⚠️ 고주로 상한 — 설계만 하고 v1 에서는 끈다

고주로 구간(경로 1123~1597m, 474m, MGeo `max_speed` 120)은 규정상 60 상한의 예외다.
계산상 80km/h 로 달리면 **8.8초**를 아낄 수 있다.

**그런데 2026-08-10 실측에서 위험이 확인됐다.**

| 크루즈 | 고주로 CTE 최대 | NPC 충돌 |
|---|---|---|
| 55 | 0.294 m | 없음 |
| 60 (하드캡에 걸린 값) | **1.069 m** | **있음 (s≈1397m)** |

충돌 지점은 시나리오 NPC2(s=1399.2m, 목표속도 40)·NPC1(s=1406.7m, 50)과 일치하고,
그때 우리 속도가 38~39km/h 로 NPC 목표속도에 묶여 있었다.

**원인은 속도 자체가 아니라 인지 부재다.**
```
NPC 목표속도 40~60 km/h
우리 55  ->  NPC 를 못 따라잡음  ->  안 만남   (충돌 없음은 운이었다)
우리 60  ->  NPC 를 따라잡음    ->  못 보고 추돌
```
`/Object_topic` 에 NPC 가 없으므로 ACC 가 앞차를 잡지 못한다. **빨리 갈수록 NPC 와
만날 확률만 높아진다.**

→ `acc_core.hpp` 의 `max_speed = 16.67`(60kph) 하드캡이 결과적으로 안전장치였다.
→ **perception 이 NPC 를 주기 전까지 이 제약은 켜지 않는다.** 켤 때는 `max_speed`
   하드캡도 같이 올려야 한다(`cruise_speed_kmh` 만 바꾸면 60 에서 잘린다 — 실측 확인).
→ 탈출 감속은 걱정할 필요 없었다. 55/60 둘 다 곡률 제한이 구간 끝 커브를 미리 보고
   30km/h 로 줄였고 CTE 는 0.28m 로 안정적이었다.

### 5.2 `/lateral_intent` (신규, `path_tracking/LateralIntent`)

```
Header header
int8    intent          # 0=KEEP  1=LEFT  2=RIGHT
int8    phase           # 0=NONE  1=WAIT  2=EXEC
float64 target_offset   # 목표 횡 offset [m], /local_path 기준. 좌측 +
```

### 5.3 `/lattice_status` (신규, `path_tracking/LatticeStatus`)

```
Header header
bool    intent_feasible   # 의도 방향 후보가 충돌검사를 통과했나
float64 selected_offset   # 실제 선택된 후보의 횡 offset [m]
bool    all_blocked       # 모든 후보가 막혔나
```

메시지는 `path_tracking` 패키지 안에 `msg/` 를 만들어 정의한다.

### 5.4 파라미터 — 값은 구현·튜닝 단계에서 정한다

아래는 **이름만 정한 것**이다. 확정값이 아니다. 괄호는 출발점으로 삼을 감각.

| 이름 | 뜻 | 출발값(가안) |
|---|---|---|
| `OVERTAKE_MARGIN` | 크루즈 대비 이만큼 낮으면 "막혔다" [m/s] | 3.0 |
| `OVERTAKE_HOLD` | 그 상태가 이만큼 지속되어야 트리거 [s] | 3.0 |
| `WAIT_TIMEOUT` | WAIT 에서 이만큼 못 들어가면 포기 [s] | 8.0 |
| `BLOCK_TICKS` | EXEC 중 이만큼 연속으로 막히면 WAIT 복귀 | 5 |
| `DONE_HOLD_TICKS` | 목표 offset 도달 후 이만큼 유지되면 완료 | 10 |
| `RETRIGGER_COOLDOWN` | 포기 후 재트리거 금지 시간 [s] | 5.0 |
| `LANE_CHANGE_KAPPA_MAX` | 이보다 굽은 곳에서는 트리거 금지 [1/m] | 0.02 |
| `CRUISE_SPEED` | 크루즈 상한. 트리거 판정과 6.1 폴백의 기준 [m/s] | 55/3.6 |

`CRUISE_SPEED` 는 `acc.launch` 의 `cruise_speed_kmh` 와 **같은 값이어야 한다.**
어긋나면 트리거가 상시 발동하거나 영영 안 걸린다. launch 에서 한 곳으로 묶는다.

## 6. 실패 처리

### 6.1 제약 토픽이 끊길 때

| 제약 | 정책 | 근거 |
|---|---|---|
| `acc`, `traffic_light`, `avoid` | 0.5초 후 **무시** + 경고 | 노드가 죽었다고 코스 한복판에 서면 곤란하다 |
| `pedestrian` | 0.5초 후 **크루즈의 절반으로 제한** + 경고 | 안전 제약이라 무시할 수 없다. 그렇다고 정지시키면 영영 못 간다 |

`51-...md` 5절에 *"보행자 급정지처럼 안전이 걸린 제약은 반대 정책을 쓸 예정"* 이라고만
적어두었던 부분을 여기서 확정한다. **완전 무시와 완전 정지 사이의 보수 주행**을 택한다.

### 6.2 그 밖

| 상황 | 처리 |
|---|---|
| `lattice_status` 끊김 | 횡방향을 **KEEP 으로 강제 복귀**. 의도를 유지한 채 눈이 머는 상태를 막는다 |
| `/local_path` 끊김 | 종방향 제약을 유지하되 경고. `path_tracker` 에 자체 폴백이 이미 있다 |
| 차선변경 타임아웃 | ⑤ 전이로 포기하고 KEEP. 재트리거는 `RETRIGGER_COOLDOWN` 이후 |
| 전 후보 막힘 | `lattice` 가 `/speed_limit/avoid` 로 감속을 요청. FSM 은 min 으로 반영만 한다 |

## 7. 검증

### 7.1 순수 로직 — gtest

`acc_core.hpp` 방식을 따라 상태 없는 함수로 분리한다.

- `combineLimits()` — min 합성, 끊김 판정, 단위
- `rampTarget()` — 이미 있음. 호출 위치만 바뀌므로 기존 테스트 유지
- `nextState()` — 전이표. 5개 상태 × 트리거 조합을 표로 검증

### 7.2 오프라인 하니스

`test_lattice.py` 구조를 그대로 재사용한다. **시뮬 없이 roscore 만으로 돈다.**

- 가짜 `/speed_limit/*` 를 쏘고 `/target_velocity` 를 검증
- 가짜 `/lattice_status` 를 쏘고 `/lateral_intent` 의 상태 전이를 검증
- 끊김 상황(발행 중단)을 재현해 6.1 정책을 검증

`test_lattice.py` 가 눈으로는 안 보이던 충돌 벌점 버그를 잡아낸 전례가 있다.
같은 방식으로 전이 버그를 잡는다.

### 7.3 실차

곡률 제한·상승률 제한 때와 같이 유턴 구간과 직선 추월 구간에서 확인한다.
`lap_logger.py` 로 한 바퀴 프로파일을 남긴다.

## 8. 이행 경로 (MGeo 도입 시)

| 항목 | v1 | MGeo 후 |
|---|---|---|
| 차선 유무 판단 | 좌측 우선 고정 | `left_link_id` / `right_link_id` |
| 차선변경 표현 | `/local_path` 기준 영구 offset | 기준경로를 옆 차선 중심선으로 교체 |
| 곡선 구간 | 트리거 차단 | 제한 해제 |
| 구간 속도 상한 | 크루즈 고정값 | 링크 `speed` 필드 |

**FSM 의 상태·전이·인터페이스는 바뀌지 않는다.** 트리거 판단과 목표 offset 산출의
입력만 교체된다.

## 9. 범위 밖 (v1 에 넣지 않는다)

- 합류·끼어들기 (NPC 사이 진입). 간격 예측이 필요해 반응형으로는 부족하다
- 신호등 판단 자체 (별도 담당자, `/speed_limit/traffic_light` 로 붙는다)
- 보행자 검출 자체 (perception. FSM 은 제약만 받는다)
- 경로 탐색 (어느 링크를 이어 갈지). MGeo 도입 시 별도 과제
