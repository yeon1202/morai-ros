# Planning 알고리즘 정리 (MORAI 고속도로 대회)

> 대회 요구: 정적 장애물 회피 + 동적 NPC + 보행자 급출현 회피 + **빠른 완주 = 점수**
> 결론 먼저: **최고 가성비 = FOT, 최고 천장 = MPCC/NMPC.**

---

## 0. 먼저 — Frenet은 알고리즘이 아니다

**Frenet 프레임** = 좌표계. 경로 기준으로 위치를 `s`(경로방향 거리) + `d`(횡방향 offset)로 표현.
lattice도 FOT도 MPC도 **다 Frenet 프레임 위에서** 동작함. "Frenet vs lattice"는 잘못된 비교.

---

## 1. 알고리즘별 간단 설명

### 계열 ① 샘플링/탐색 (단순·튼튼)

- **기본 Lattice** — 기준경로에서 좌우 offset 후보 **경로 몇 개**를 뿌리고, 장애물 충돌검사 후 중앙에 가까운 것 선택. *공간(위치)만* 다룸, 속도는 별도. → 정적 회피·차선변경에 좋음, 동적 약함.
- **Frenet Optimal Trajectory (FOT)** ⭐ — 끝상태(횡 offset `d` + 목표속도 + 시간 `T`)를 샘플링해 **quintic(횡)+quartic(종) 다항식으로 시간축 궤적** 생성. 비용(jerk+시간+속도오차+이탈+충돌)으로 최적 선택. *조향+속도 통합*, **움직이는 장애물 미래위치 예측해 충돌검사**. lattice의 상위호환.
- **State Lattice** — motion primitive(미리 만든 곡선 조각)를 격자로 이어붙여 탐색. 구조화 도로엔 과함.
- **Hybrid A\*** — 연속 상태공간 A* 탐색. **주차/비정형** 공간에 강함, 고속도로엔 부적합.
- **RRT / RRT\*** — 무작위 샘플링으로 트리 확장. 좁고 복잡한 비정형에 쓰나, 도로주행엔 덜 매끄럽고 불안정.

### 계열 ② 최적화 기반 (실제 SOTA) ⭐

- **MPC / NMPC (Nonlinear Model Predictive Control)** — 앞으로 N스텝을 내다보며 **차량 동역학 + 제약(속도·조향·충돌) + 동적장애물**을 하나의 최적화로 풀어 매 주기 다시 계산. 회피+속도+ACC 통합. 현대 자율주행의 주력.
- **MPCC (Model Predictive Contouring Control)** 🏎️ — MPC의 레이싱 특화. **"트랙 진행량 최대화"가 목적함수** → 랩타임 최소화 그 자체. 자율주행 레이싱 우승팀 표준. "빠른 완주=점수"에 가장 직접 부합.
- **Apollo EM Planner** 🏭 — Baidu Apollo 양산 플래너. Frenet에서 **경로/속도를 분리**해 DP(대략해) + 스플라인 QP(정밀해)로 풂. 장애물·교통규칙·부드러움 동시 처리. **완전 오픈소스** = 참고 풍부.
- **iLQR / CILQR** — 궤적을 반복 선형화하며 최적화(제약 포함). 부드럽고 빠름, 일부 양산 스택이 사용.

### 계열 ③ 학습 기반 (연구 SOTA, 대회엔 위험)

- **RL / Imitation / End-to-End** — 신경망이 직접 궤적/행동 출력. 강력하나 **학습 데이터·안전보장·디버깅 난이도** 때문에 대회 데드라인엔 도박.

---

## 2. 대회 기준 순위

| 순위 | 알고리즘 | 이유 | 난이도 |
|---|---|---|---|
| 🥇 현실적 최선 | **FOT** | SOTA급 성능(동적·속도) + **실제 완성 가능** | ★★★ |
| 🚀 최고 천장 | **MPCC**(속도) / **NMPC**(범용) | 이론상 최강, 근데 **솔버·동역학·튜닝** 리스크 | ★★★★★ |
| 🏭 대안 | **Apollo EM Planner** | 오픈소스라 배끼기 좋음, 무겁고 복잡 | ★★★★ |
| ❌ 비추 | RL / end-to-end | 대회엔 불확실성 큼 | - |

---

## 3. 냉정한 현실 체크

> **최고의 알고리즘 = 대회 안에 안정적으로 완성해 "완주"하는 것 중 가장 강한 것.**

MPCC가 이론상 1등이어도 솔버가 실시간에 안 풀리거나 튜닝 어긋나면 → 차 발산 → 충돌 → 탈락.
**안 굴러가는 최강 알고리즘은 잘 굴러가는 FOT한테 진다.**

**추천 전략**: 목표를 MPCC/NMPC로 잡되 **FOT를 먼저 완성해 안전판으로 확보** → Frenet/예측/비용함수 개념 익힌 뒤 MPCC로 상승. MPCC 실패해도 FOT로 완주 보장.

---

## 4. 참고 자료

| 자료 | 용도 |
|---|---|
| [Motion Planning for AD: State of the Art (arXiv 2303.09824)](https://arxiv.org/abs/2303.09824) | 전체 지형도 survey |
| [PythonRobotics — Frenet Optimal Trajectory](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/FrenetOptimalTrajectory/frenet_optimal_trajectory.py) | **FOT 바로 포팅 가능한 구현** |
| [Werling — Optimal Trajectory in Frenét Frame](https://www.semanticscholar.org/paper/Optimal-trajectory-generation-for-dynamic-street-in-Werling-Ziegler/6bda8fc13bda8cffb3bb426a73ce5c12cc0a1760) | FOT 원조 논문 |
| [Curvature-Integrated MPCC for Racing (arXiv 2502.03695)](https://arxiv.org/abs/2502.03695) | 레이싱 시간최적 MPCC 최신 |
| [Baidu Apollo EM Motion Planner (arXiv 1807.08048)](https://arxiv.org/abs/1807.08048) | 양산급 플래너 (오픈소스 Apollo) |
| [2024 CommonRoad Planning Competition (arXiv 2512.19564)](https://arxiv.org/abs/2512.19564) | 벤치마크 결과 |
| 기존 `lattice_planner.py` (Park-chan-young/Highway_Autonomous_Driving_Morai) | 기본 lattice 레퍼런스 |

---

*작성: planning 알고리즘 서치 정리. 선택 시 이 문서 업데이트.*
