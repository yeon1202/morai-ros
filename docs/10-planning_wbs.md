# Planning 모듈 WBS (Work Breakdown Structure)

작성일: 2026-07-16 / planning (solo)
범위: **planning 모듈 중심** + 타팀(localization·perception·control) 인터페이스
관련: [morai 대회/로드맵], docs/40-acc_design.md, docs/30-lattice_design.md, docs/41-acc_plan.md

**상태 범례:** ✅ 완료 · 🔄 진행중 · ⬜ 예정 · ⏸ 보류(stretch)

---

## 1. 개발환경·인프라
| ID | 작업 | 상태 |
|----|------|------|
| 1.1 | catkin 워크스페이스 구성 (`catkin_ws`) | ✅ |
| 1.2 | Docker 개발환경 (`morai-noetic`, sim/dev 2컨테이너) | ✅ |
| 1.3 | `morai_msgs` 연동 (MORAI 메시지/서비스) | ✅ |
| 1.4 | git 버전관리 (로컬, `feature/acc` 브랜치) | ✅ |
| 1.5 | MORAI UDP↔ROS 브릿지 (`udp_bridge`, 제어9093/상태9111) | 🔄 |
| 1.6 | 센서(카메라·라이다) ROS 브릿지 설정 | ⬜ |

## 2. 경로 (Global / Local Path)
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 2.1 | 전역경로 기록 (`path_recorder`) | 🔄 | 자동주행 경로≠대회코스 → teleop 손기록 예정 |
| 2.2 | 경로 스무딩 (`path_smoother`) | ✅ | |
| 2.3 | 지역경로 추출 (`path_manager` → `/local_path`) | ✅ | 앞 50점 ≈25m |
| 2.4 | 경로추종 (`pure_pursuit`+PID, `path_tracker` 임시) | ✅ | control팀 정식 노드로 대체 예정 |

## 3. 횡방향 회피 (Lattice) — `/lattice_path`
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 3.1 | 장애물 mock (`mock_obstacle_pub`, `object_viz`) | ✅ | 정적장애물 |
| 3.2 | 후보경로 생성 (좌표변환 + 3차곡선 6후보) | ✅ | |
| 3.3 | 충돌검사 + 비용선택 (argmin) | ✅ | |
| 3.4 | 오프라인 검증 (정적장애물 회피 2.65m) | ✅ | |
| 3.5 | MORAI 실차 회피 확인 | ⬜ | |
| 3.6 | [개선] tail 불연속 수정 (법선방향 offset 유지) | ⏸ | Frenet이 근본해결 |
| — | 문서: `30-lattice_design.md`, `31-lattice_code_review.md` | ✅ | |

## 4. 종방향 제어 (ACC) — `/target_velocity`
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 4.1 | 순수로직 `acc_core.hpp` (제어식·lead탐색·단위변환) + gtest 10개 | ✅ | Task1·2 |
| 4.2 | ROS 노드 `acc_planner.cpp` (`/target_velocity` 발행) | ✅ | Task3, 스모크테스트 통과 |
| 4.3 | `mock_lead_vehicle.cpp` (움직이는 앞차 검증노드) | ⬜ | Task4 |
| 4.4 | `path_tracker` 임시 통합 (`/target_velocity` 구독) | ⬜ | Task5 |
| 4.5 | `/target_velocity` 인터페이스 문서 (control 인계) | ⬜ | Task6 |
| 4.6 | 실차 확인 + gain 튜닝 (velocity_gain/distance_gain) | ⬜ | 시뮬에서 |
| — | 문서: `40-acc_design.md`, `41-acc_plan.md` | ✅ | |

## 5. Behavior FSM (미션 로직) — 점수원
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 5.1 | 상태 설계 (FSM 구조·전이) | ⬜ | 신호/차선유지/끼어들기/보행자 |
| 5.2 | 신호등 정지 (traffic light) | ⬜ | ACC 범위 밖이었음 |
| 5.3 | 보행자 급정지 (즉발동, 트랙 갓잡혀도) | ⬜ | ACC 범위 밖이었음 |
| 5.4 | 끼어들기/차선변경 판단 | ⬜ | |
| 5.5 | lattice·ACC와 우선순위 조율 | ⬜ | behavior가 상위 오버라이드 |

## 6. 통합·검증
| ID | 작업 | 상태 | 비고 |
|----|------|------|------|
| 6.1 | 오프라인 end-to-end (mock→planning→차 반응) | ⬜ | Task4·5 완료 후 |
| 6.2 | MORAI 실차 통합 | ⬜ | |
| 6.3 | 속도프로파일 최적화 (곡률기반) | ⬜ | Phase6 |
| 6.4 | 완주 리허설 (제한 15분, 채점=시간+패널티) | ⬜ | |

## 7. 타팀 인터페이스 (내가 소비/생산하는 접점)
| ID | 인터페이스 | 방향 | 타입 | 상태 |
|----|-----------|------|------|------|
| 7.1 | localization `/odom` (ENU pose+twist) | 소비 | `nav_msgs/Odometry` | 🔄 합의됨, 개발중 `/ego_status` 스탠드인 → 교체 예정 |
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
- **지금**: 4.2까지 완료(ACC 노드 동작확인). 커밋 `14fdc85`.
- **다음**: 4.3 `mock_lead_vehicle` → 4.4 `path_tracker` 통합 → 4.5 인터페이스 문서 → (그다음) **5. Behavior FSM**.
- **우선순위 원칙**: 네트워크규정 > localization > planning(충분히) > **behavior(빡세게, 점수원)**.
