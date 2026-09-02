# Planning 문서 색인

개발한 순서대로 번호를 매겼다. 위에서부터 읽으면 이 프로젝트가 어떤 순서로
만들어졌는지 그대로 따라간다.

번호는 단계별로 10 단위씩 끊었다. 나중에 문서가 중간에 끼어들어도 전체를 다시
번호 매기지 않기 위해서다.

## 0x — 배경·제약·운영

| 문서 | 내용 |
|---|---|
| [00-reference.md](00-reference.md) | 참고 자료 링크 모음 |
| [01-competition_rules.md](01-competition_rules.md) | 대회 규정 요약(planning 관점). 60km/h 상한, 허용 네트워크, 미션 목록, 차량 제원 |
| [02-planning_algorithms.md](02-planning_algorithms.md) | 알고리즘 서베이. lattice / FOT / MPCC 비교와 선택 근거 |
| [03-fot_theory.md](03-fot_theory.md) | Frenet Optimal Trajectory 이론. 현재는 stretch goal이지만 (s,d) 사고방식의 출처 |
| **[04-runbook.md](04-runbook.md)** | **실행·정지 명령어와 자주 걸리는 함정.** 매일 여는 문서 |

## 1x — 로드맵

| 문서 | 내용 |
|---|---|
| [10-planning_wbs.md](10-planning_wbs.md) | Planning 모듈 WBS. 전체 작업 분해와 진행 상태 |

## 2x — 팀 인터페이스 (다른 팀과의 계약)

| 문서 | 내용 |
|---|---|
| [20-perception_interface.md](20-perception_interface.md) | perception → planning. `/Object_topic`, `morai_msgs/ObjectStatusList` |
| [21-localization_interface.md](21-localization_interface.md) | localization → planning. `/odom`, `nav_msgs/Odometry` |
| [22-planning_control_interface.md](22-planning_control_interface.md) | **planning → control 명세.** `/lattice_path`·`/target_velocity`·`/ctrl_cmd` 소유권. 초안, 합의 대기 |
| [23-localization_node_review.md](23-localization_node_review.md) | localization 팀 `localization_node.cpp` 리뷰. `/odom` 명세 대조와 좌표 프레임 정합 리스크 |
| [24-perception_integration_design.md](24-perception_integration_design.md) | perception 연동 설계. mock 장애물을 실제 인지로 교체하는 구조와 근거 |
| [25-perception_integration_plan.md](25-perception_integration_plan.md) | 위 설계의 구현 계획. Task 6개, 단계별 검증 명령 포함 |
| [26-lane_interface.md](26-lane_interface.md) | **perception(차선) → planning 명세.** 차량좌표 3차 다항식·촬영시각·실선/점선. 초안, 협의 대기 |

## 3x — 횡방향 (lattice)

| 문서 | 내용 |
|---|---|
| [30-lattice_design.md](30-lattice_design.md) | lattice planner 설계. 후보 생성, 충돌 검사, 비용 선택 |
| [31-lattice_code_review.md](31-lattice_code_review.md) | 코드 완전 해설. 3차곡선 유도와 좌표변환 수식 포함 |

## 4x — 종방향 (ACC)

| 문서 | 내용 |
|---|---|
| [40-acc_design.md](40-acc_design.md) | ACC 설계. `/target_velocity`가 종방향 단일 권한이라는 원칙 |
| [41-acc_plan.md](41-acc_plan.md) | ACC 구현 계획(Task 단위). 진행 상태 포함 |
| [42-curve_exit_speed_ramp_design.md](42-curve_exit_speed_ramp_design.md) | 커브 탈출 시 목표속도 상승률 제한 설계 |
| [43-curve_exit_speed_ramp_plan.md](43-curve_exit_speed_ramp_plan.md) | 위 설계의 구현 계획(Task 단위) |

## 5x — behavior FSM

| 문서 | 내용 |
|---|---|
| [50-behavior_fsm_design.md](50-behavior_fsm_design.md) | **FSM 설계.** 종방향 제약 합성(min)과 횡방향 상태기계, 우선순위 중재 |
| [51-traffic_light_brief.md](51-traffic_light_brief.md) | **planning 인수인계 브리프.** 경로·local path·lattice·ACC 를 어떻게 구성했는지, 세팅과 함정 |

## 다음 번호대 (예정)

| 번호 | 예정 문서 |
|---|---|
| 5x | 끼어들기, 보행자 급정지 |
| 6x | 통합·완주 리허설 |
