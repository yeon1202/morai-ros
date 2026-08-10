# Planning 모듈 WBS (Work Breakdown Structure)

작성일: 2026-07-16 / planning (solo)
범위: **planning 모듈 중심** + 타팀(localization·perception·control) 인터페이스
관련: [morai 대회/로드맵], docs/40-acc_design.md, docs/30-lattice_design.md, docs/41-acc_plan.md

**상태 범례:** ✅ 완료 · 🔄 진행중 · ⬜ 예정 · ⏸ 보류(stretch)

---

## 1. 개발환경·인프라
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 1.1 | catkin 워크스페이스 구성 (`catkin_ws`) | ✅ |
| 1.2 | Docker 개발환경 (`morai-noetic`, sim/dev 2컨테이너) | ✅ |
| 1.3 | `morai_msgs` 연동 (MORAI 메시지/서비스) | ✅ |
| 1.4 | git 버전관리 (로컬, `feature/acc` 브랜치) | ✅ |
| 1.5 | MORAI UDP↔ROS 브릿지 (`udp_bridge`, 제어9093/상태9111) | 🔄 | 규정 제출 시 9109 전환 필요(미완). GPS 중복 발행 버그 발견 → `23-...md` 7.7 |
| 1.6 | 센서(카메라·라이다) ROS 브릿지 설정 | ⬜ |

## 2. 경로 (Global / Local Path)
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 2.1 | 전역경로 기록 (`path_recorder`) | ✅ | 07-29 전체 재기록. approach 980 + course 3571 → `path_smooth.csv` 4544점 2830m |
| 2.2 | 경로 스무딩 (`path_smoother`) | ✅ | |
| 2.3 | 지역경로 추출 (`path_manager` → `/local_path`) | ✅ | 앞 **140점 ≈84m** (`LOCAL_PATH_SIZE`) |
| 2.4 | 경로추종 (`pure_pursuit`+PID, `path_tracker` 임시) | ✅ | control팀 정식 노드로 대체 예정 |

## 3. 횡방향 회피 (Lattice) — `/lattice_path`
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 3.1 | 장애물 mock (`mock_obstacle_pub`, `object_viz`) | ✅ | 정적장애물 |
| 3.2 | 후보경로 생성 (좌표변환 + 3차곡선) | ✅ | |
| 3.3 | 충돌검사 + 비용선택 (argmin) | ✅ | |
| 3.4 | 오프라인 검증 (정적장애물 회피 2.65m) | ✅ | |
| 3.5 | MORAI 실차 회피 확인 | ⬜ | 후보·전이길이 변경 후 재확인 필요 |
| 3.6 | [개선] tail 불연속 수정 (법선방향 offset 유지) | ✅ | 07-31. 전이길이 단축으로 필수가 됨 |
| 3.7 | 충돌 벌점 중복 적용 버그 수정 | ✅ | 07-31. 후보당 1회로 |
| 3.8 | 차로폭 실측(3.51m) + 후보 집합 재설계 | ✅ | 07-31. 0 후보 추가·우측 우선 |
| 3.9 | 전이 길이를 시간 기준(2.68초)으로 | ✅ | 07-31. 저속 회피불가·고속 0.72G 해소 |
| 3.10 | 보행자를 회피 트리거에서 제외 | ✅ | 07-31. 보행자는 정지 대상 |
| 3.11 | 오프라인 하니스 `test_lattice.py` (9케이스) | ✅ | 07-31 |
| — | 문서: `30-lattice_design.md`, `31-lattice_code_review.md` | ✅ | |

## 4. 종방향 제어 (ACC) — `/target_velocity`
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 4.1 | 순수로직 `acc_core.hpp` (제어식·lead탐색·단위변환) + gtest 10개 | ✅ | Task1·2 |
| 4.2 | ROS 노드 `acc_planner.cpp` (`/target_velocity` 발행) | ✅ | Task3, 스모크테스트 통과 |
| 4.3 | `mock_lead_vehicle.cpp` (움직이는 앞차 검증노드) | ✅ | Task4. `acc.launch` 에 배선 완료 |
| 4.4 | `path_tracker` 임시 통합 (`/target_velocity` 구독) | ✅ | Task5. 0.5s 끊김 시 자체속도 폴백까지 (`path_tracker.py:129`) |
| 4.5 | `/target_velocity` 인터페이스 문서 (control 인계) | ⬜ | Task6 |
| 4.6 | 실차 확인 + gain 튜닝 (velocity_gain/distance_gain) | ⬜ | 시뮬에서 |
| — | 문서: `40-acc_design.md`, `41-acc_plan.md` | ✅ | |

## 5. Behavior FSM (미션 로직) — 점수원
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 5.1 | 상태 설계 (FSM 구조·전이) | ✅ | 08-03. `50-behavior_fsm_design.md`. 종방향=제약합성, 횡방향=상태기계 |
| 5.2 | 신호등 정지 + 교차로 통과 | ➡️ | **08-10 인계.** 신호판정·접근감속·정지선정지·딜레마존. `/speed_limit/{traffic_light,intersection}` 발행 |
| 5.3 | 보행자 급정지 (즉발동, 트랙 갓잡혀도) | ⬜ | ACC 범위 밖이었음 |
| 5.4 | 끼어들기/차선변경 판단 | ⬜ | 차선변경은 5.1 에 설계됨. 끼어들기(합류)는 v1 범위 밖 |
| 5.5 | lattice·ACC와 우선순위 조율 | ⬜ | behavior가 상위 오버라이드 |

## 6. 통합·검증
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 6.1 | 오프라인 end-to-end (mock→planning→차 반응) | ⬜ | 선행조건(4.3·4.4) 완료됨 → 착수 가능 |
| 6.2 | MORAI 실차 통합 | ⬜ | |
| 6.3 | 속도프로파일 최적화 (곡률기반) | ⬜ | Phase6 |
| 6.4 | 완주 리허설 (제한 15분, 채점=시간+패널티) | ⬜ | |

## 7. 타팀 인터페이스 (내가 소비/생산하는 접점)
| ID | 인터페이스 | 방향 | 타입 | 상태 |
|----|-----------|------|------|------|
| 7.1 | localization `/odom` (ENU pose+twist) | 소비 | `nav_msgs/Odometry` | 🔄 08-03 좌표 프레임 실측 → **최대 11.3m 불일치**. 원인 3개 특정, 전달 대기 (`23-...md`) |
| 7.2 | perception `/Object_topic` (Kalman 적용) | 소비 | `morai_msgs/ObjectStatusList` | 🔄 합의됨, mock으로 개발중 |
| 7.3 | control ← `/lattice_path` (횡) | 생산 | `nav_msgs/Path` | ✅ 발행중 |
| 7.4 | control ← `/target_velocity` (종) | 생산 | `std_msgs/Float64` | 🔄 발행중, 인터페이스 문서화(4.5) 예정 |

## 8. [Stretch] Frenet / FOT
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 8.1 | (s,d) 구조화·비용항 분리 준비 | 🔄 | 설계에 훅 반영중 |
| 8.2 | FOT 이식 (횡5차+종4/5차, 시간축, 동적장애물 예측) | ⏸ | 완주 후 여유시 |

---

## 현재 위치 / 다음 3스텝

*갱신: 2026-08-03*

- **지금**: 종방향(4장)은 4.5 문서만 남기고 사실상 완료. 횡방향(3장)은 재설계 후
  **실차 확인(3.5)이 미완**. behavior 는 설계(5.1)까지 완료.
- **다음**: 3.5 lattice 실차 회피 확인 → 5.1 구현(`behavior_fsm` 노드) → 4.5 인터페이스 문서.
- **별건 진행중**: localization 실측 결과 전달, 브릿지 GPS 중복 발행 수정(담당자), 9109 전환.
- **우선순위 원칙**: 네트워크규정 > localization > planning(충분히) > **behavior(빡세게, 점수원)**.
