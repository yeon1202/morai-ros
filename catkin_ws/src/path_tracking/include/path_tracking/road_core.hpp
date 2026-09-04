#pragma once
//
// road_core : "이 구간에 옆 차로가 있는가" 를 전역경로 인덱스로 답한다.
//
// ROS 를 쓰지 않는 순수 함수다. 시뮬도 마스터도 없이 gtest 로 검증할 수 있어야
// 대회 당일 회귀를 빨리 잡는다 (acc_core.hpp / behavior_core.hpp 와 같은 이유).
//
// ---------------------------------------------------------------------------
// 왜 필요한가 (2026-09-03 실패 기록, logs/percep2)
//
//   경로 417.8m 에서 차선 가장자리 잡동사니(0.33 x 0.19m, 경로에서 1.57m)가
//   전역경로를 막았다고 판정됐다. lattice 는 우측 -3.51m 후보를 골랐다
//   (우측 비용 4 < 좌측 20). 그 지점 링크는 can_move_right_lane=True 였다.
//
//   그런데 12m 앞 429.9m 에서 링크가 A2256W000202 -> A2256W000728 로 바뀌고
//   can_move_right_lane 이 False 가 된다. 차는 인도로 올라갔고, 도로 밖
//   잡동사니에 둘러싸여 ACC 가 목표속도를 0 으로 내렸다. 76초에 멈춘 뒤
//   60초 동안 조향을 40도로 꺾은 채 복구하지 못했다.
//
//   => 자차 위치 하나만 보면 통과된다. 후보가 지나갈 "구간 전체" 를 봐야 한다.
//      그래서 이 헤더의 질의는 점이 아니라 구간 [i0, i1] 을 받는다.
//
//   전역경로 4392점 중 우측 차로가 있는 곳은 25.0%, 좌측은 21.7% 뿐이다.
//   즉 코스의 3/4 에서 우측 회피는 도로 밖이다. 한 번 운이 나빴던 게 아니다.
// ---------------------------------------------------------------------------
//
// 표는 build_lane_table.py 가 미리 만든다:
//   map/mapping_result.csv (웨이포인트 -> link_id)
//   map/link_set.json      (link_id -> 차로 정보, 9MB 라 커밋 안 함)
//        -> map/lane_table.csv (128KB, 이것만 커밋)
//
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <limits>

namespace road {

// "그런 인덱스 없음". 헤더 전용 라이브러리라 네임스페이스 상수로 둔다
// (클래스 static 멤버는 C++14 에서 클래스 밖 정의가 따로 필요하다).
const std::size_t kNoIndex = static_cast<std::size_t>(-1);

// 전역경로 웨이포인트 하나에 붙는 도로 정보.
struct LaneInfo {
  double x          = 0.0;    // [m] 웨이포인트 좌표 (전역, map 프레임)
  double y          = 0.0;
  bool   right_ok   = false;  // 이 지점에서 우측 차로로 나갈 수 있나
  bool   left_ok    = false;  // 좌측 (황색 중앙선 너머일 수 있으니 비용은 별도)
  int    ego_lane   = -1;     // 자차 차로 번호 (1 = 맨 왼쪽)
  double road_width = 0.0;    // [m] 링크 폭
};

// 웨이포인트 인덱스로 조회하는 표. 런타임에는 배열 인덱싱만 한다.
class LaneTable {
 public:
  bool loaded() const { return !rows_.empty(); }
  std::size_t size() const { return rows_.size(); }

  // lane_table.csv 를 읽는다. 헤더 한 줄 + waypoint 당 한 줄.
  // 실패하면 false 를 돌려주고 표는 비어 있는 상태로 남는다.
  bool load(const std::string& csv_path) {
    rows_.clear();
    std::ifstream f(csv_path);
    if (!f.is_open()) return false;

    std::string line;
    if (!std::getline(f, line)) return false;   // 헤더

    while (std::getline(f, line)) {
      if (line.empty()) continue;
      std::stringstream ss(line);
      std::string idx, sx, sy, r, l, lane, width;
      if (!std::getline(ss, idx,   ',')) continue;
      if (!std::getline(ss, sx,    ',')) continue;
      if (!std::getline(ss, sy,    ',')) continue;
      if (!std::getline(ss, r,     ',')) continue;
      if (!std::getline(ss, l,     ',')) continue;
      if (!std::getline(ss, lane,  ',')) continue;
      if (!std::getline(ss, width, ',')) continue;

      LaneInfo v;
      v.x          =  std::atof(sx.c_str());
      v.y          =  std::atof(sy.c_str());
      v.right_ok   = (std::atoi(r.c_str())    != 0);
      v.left_ok    = (std::atoi(l.c_str())    != 0);
      v.ego_lane   =  std::atoi(lane.c_str());
      v.road_width =  std::atof(width.c_str());
      rows_.push_back(v);
    }
    return !rows_.empty();
  }

  // 범위를 벗어난 인덱스는 "옆 차로 없음" 으로 답한다.
  //
  // 왜 보수적으로 답하는가: 이 함수의 답이 "회피해도 되는가" 를 정한다.
  // 모를 때 된다고 하면 도로 밖으로 나간다. 모를 때 안 된다고 하면 회피를
  // 못 할 뿐이고, 그때는 ACC 가 속도를 줄인다. 두 실패의 무게가 다르다.
  LaneInfo at(std::size_t i) const {
    if (i >= rows_.size()) return LaneInfo();
    return rows_[i];
  }

  // (x, y) 에 가장 가까운 웨이포인트 인덱스. 못 찾으면 npos.
  //
  // 전 구간 선형 탐색(4392점)이다. 30Hz 에 프레임당 두 번(구간 시작/끝) 부르면
  // 초당 26만 번인데, 이건 lattice 가 이미 후보마다 하는 충돌 검사보다 훨씬 작다.
  // 창(window)으로 좁히는 최적화는 인덱스를 프레임 간에 들고 다녀야 해서
  // path_manager 가 겪은 "창을 잘못 물면 엉뚱한 데로 튄다" 문제를 다시 만든다.
  // 필요해지면 그때 하되, 지금은 단순한 쪽이 맞다.
  // max_dist 보다 멀면 kNoIndex 를 준다.
  //
  // 거리 제한이 왜 필요한가: 제한이 없으면 코스에서 500m 떨어진 점도 "제일
  // 가까운 웨이포인트" 를 물어온다. 그 웨이포인트의 차로 정보는 그 점과 아무
  // 관계가 없는데, 답이 나오니까 쓰게 된다. "모른다" 와 "아니다" 는 다르다.
  //
  // 기본 3.0m 인 이유: 전역경로가 두 벌이다. path_tracker 는 path_smooth.csv 를
  // 쓰고 이 표는 팀 path_smooth_closed.csv 에서 나왔는데, 4392점 중 3곳에서
  // 최대 0.77m 다르다(경로 895~909m, 1591~1597m). 그 차이는 정상이므로 넉넉히
  // 넘겨야 하고, 그렇다고 차로 폭(3.5m)을 넘길 만큼 크면 안 된다.
  std::size_t nearestIndex(double x, double y, double max_dist = 3.0) const {
    if (rows_.empty()) return kNoIndex;
    std::size_t best = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < rows_.size(); ++i) {
      const double dx = rows_[i].x - x;
      const double dy = rows_[i].y - y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    if (best_d2 > max_dist * max_dist) return kNoIndex;
    return best;
  }

  // 구간 [i0, i1] 전체에서 우측(또는 좌측) 차로가 계속 있는가.
  // 하나라도 없으면 false. 표가 비어 있으면 false (위와 같은 이유).
  bool spanHasRightLane(std::size_t i0, std::size_t i1) const {
    return spanAll(i0, i1, true);
  }
  bool spanHasLeftLane(std::size_t i0, std::size_t i1) const {
    return spanAll(i0, i1, false);
  }

 private:
  bool spanAll(std::size_t i0, std::size_t i1, bool right) const {
    if (rows_.empty()) return false;
    if (i0 > i1) std::swap(i0, i1);
    if (i0 >= rows_.size()) return false;
    if (i1 >= rows_.size()) i1 = rows_.size() - 1;
    for (std::size_t i = i0; i <= i1; ++i) {
      if (right ? !rows_[i].right_ok : !rows_[i].left_ok) return false;
    }
    return true;
  }

  std::vector<LaneInfo> rows_;
};

// "차로 안에서 조금 비켜가기" 의 상한 [m]. 이 이하는 옆 차로가 없어도 허용한다.
//
// 왜 필요한가 (2026-09-03, logs/percep4)
//   처음에는 옆 차로가 없으면 옆으로 나가는 후보를 전부 막았다. 그랬더니 시나리오
//   정적장애물(경로 328.9m) 앞에서 차가 서서 못 갔다. 그 구간은 편도 1차선이라
//   right_ok = left_ok = 0 인데, 기하를 계산하면 차로 안에는 답이 없다:
//
//     인지가 준 물체:  횡 +1.00m, 횡반폭 0.73  ->  차지 구간 +0.27 ~ +1.73m
//     차로 안 한계  :  -0.81 ~ +0.81m  (차로 반폭 1.755 - 차 반폭 0.946)
//     통과에 필요   :  차 중심 <= -1.17m  (SAFE_MARGIN 0.5 포함)
//
//   즉 어떻게 해도 차선을 0.36m 는 넘어야 지나간다. 규정으로 비교하면
//   차선 침범은 3초당 5초인데 정적장애물 충돌은 15초 + 복귀불가 시 주행 불능이고,
//   아예 못 가면 미완주다. 5초 내고 지나가는 쪽이 맞다.
//
// 왜 2.0 인가
//   -1.0 은 부족하다(-1.17 필요). -2.0 은 통과한다.
//   인지가 지금은 이 물체를 1.45 x 0.09 로 작게 보는데, 카메라 매칭이 붙어 실제
//   크기(3.0 x 2.0, θ 6.7도 -> 횡반폭 1.17)로 나오면 필요값이 -1.61 로 커진다.
//   -2.0 은 그때도 견딘다. -1.5 로 하면 그 경우 다시 못 간다.
//   차 바깥쪽이 차로 밖으로 1.19m 나가는데, 인도로 올라갔던 사고 때는 3.5~4.4m
//   였다. 성격이 다른 크기다.
//
// ⚠️ 이 값은 "옆 차로가 없을 때 얼마나 나가도 되는가" 라서, 도로 옆에 갓길이
//    있는지 없는지에 달려 있다. 지도의 width(3.50)는 차로 폭이라 그걸 알려주지
//    않는다. 갓길이 없는 구간이 확인되면 다시 봐야 한다.
const double kNudgeMax = 2.0;

// 이 오프셋이 "차로 안에서 비켜가기" 수준인가 (옆 차로 없이도 허용).
inline bool isNudge(double offset) {
  return std::fabs(offset) <= kNudgeMax;
}

// 회피 오프셋이 이 구간에서 허용되는가.
//
//   offset < 0 : 우측 (LANE_OFFSET 규약)
//   offset > 0 : 좌측
//   offset = 0 : 제자리 - 언제나 허용. 표가 없어도 기준경로는 따라갈 수 있어야 한다.
//
// |offset| <= kNudgeMax 면 표를 보지 않고 허용한다. 그보다 크면(= 옆 차로로 통째로
// 옮기는 것) 그 구간에 실제로 차로가 있어야 한다.
inline bool offsetAllowed(const LaneTable& t, double offset,
                          std::size_t i0, std::size_t i1) {
  if (isNudge(offset)) return true;
  return (offset > 0.0) ? t.spanHasLeftLane(i0, i1)
                        : t.spanHasRightLane(i0, i1);
}

}  // namespace road
