#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <string>
#include <ros/package.h>
#include "path_tracking/road_core.hpp"

using namespace road;

namespace {

// 임시 lane_table.csv 를 만들어 준다. rows = (right_ok, left_ok) 목록.
std::string writeTable(const std::vector<std::pair<int, int> >& rows) {
  std::string path = "/tmp/lane_table_test.csv";
  std::ofstream f(path);
  f << "waypoint_idx,x,y,right_ok,left_ok,ego_lane,road_width,link_id\n";
  for (std::size_t i = 0; i < rows.size(); ++i) {
    // 웨이포인트를 x축 위 1m 간격으로 둔다 (최근접 검색 테스트용)
    f << i << "," << static_cast<double>(i) << ",0.0,"
      << rows[i].first << "," << rows[i].second
      << ",1,3.50,TEST\n";
  }
  return path;
}

}  // namespace

// --- 로딩 ---

TEST(LaneTable, LoadsRows) {
  LaneTable t;
  ASSERT_TRUE(t.load(writeTable({{1, 0}, {1, 0}, {0, 1}})));
  EXPECT_EQ(t.size(), 3u);
  EXPECT_TRUE(t.at(0).right_ok);
  EXPECT_FALSE(t.at(0).left_ok);
  EXPECT_TRUE(t.at(2).left_ok);
  EXPECT_DOUBLE_EQ(t.at(0).road_width, 3.50);
}

TEST(LaneTable, MissingFileIsNotLoaded) {
  LaneTable t;
  EXPECT_FALSE(t.load("/tmp/__no_such_lane_table__.csv"));
  EXPECT_FALSE(t.loaded());
}

// --- 구간 질의 ---

TEST(LaneTable, SpanAllTrue) {
  LaneTable t;
  ASSERT_TRUE(t.load(writeTable({{1, 0}, {1, 0}, {1, 0}})));
  EXPECT_TRUE(t.spanHasRightLane(0, 2));
  EXPECT_FALSE(t.spanHasLeftLane(0, 2));
}

// ★ 이번 실패의 핵심: 구간 안에서 딱 한 점만 False 여도 막아야 한다.
//   417.8m 은 True 였고 429.9m 부터 False 였다. 시작점만 보면 통과된다.
TEST(LaneTable, SpanBlockedBySinglePointInside) {
  LaneTable t;
  ASSERT_TRUE(t.load(writeTable({{1, 0}, {1, 0}, {0, 0}, {1, 0}, {1, 0}})));
  EXPECT_TRUE(t.spanHasRightLane(0, 1));    // 시작 구간만 보면 통과
  EXPECT_FALSE(t.spanHasRightLane(0, 4));   // 전 구간을 보면 막힌다
  EXPECT_FALSE(t.spanHasRightLane(2, 2));
  EXPECT_TRUE(t.spanHasRightLane(3, 4));
}

TEST(LaneTable, SpanHandlesReversedAndClampedRange) {
  LaneTable t;
  ASSERT_TRUE(t.load(writeTable({{1, 0}, {1, 0}, {1, 0}})));
  EXPECT_TRUE(t.spanHasRightLane(2, 0));       // 뒤집힌 범위도 같은 답
  EXPECT_TRUE(t.spanHasRightLane(0, 999));     // 끝을 넘으면 잘라서 본다
  EXPECT_FALSE(t.spanHasRightLane(999, 1000)); // 시작이 밖이면 모른다 -> 막는다
}

// --- 최근접 인덱스 ---

TEST(LaneTable, NearestIndexFindsClosestWaypoint) {
  LaneTable t;
  ASSERT_TRUE(t.load(writeTable({{1, 0}, {1, 0}, {1, 0}, {1, 0}})));
  EXPECT_EQ(t.nearestIndex(0.1, 0.0), 0u);
  EXPECT_EQ(t.nearestIndex(2.4, 0.0), 2u);
  EXPECT_EQ(t.nearestIndex(2.6, 0.0), 3u);
}

// 거리 제한: 경로에서 멀면 "제일 가까운 점" 이 아니라 "모른다" 를 준다.
// 제한이 없으면 500m 밖의 점도 답을 받아가고, 그 답으로 회피 여부를 정하게 된다.
TEST(LaneTable, NearestIndexRejectsFarQuery) {
  LaneTable t;
  ASSERT_TRUE(t.load(writeTable({{1, 0}, {1, 0}, {1, 0}, {1, 0}})));
  EXPECT_EQ(t.nearestIndex(1.0, 2.0), 1u);          // 2m - 기본 제한 3m 안
  EXPECT_EQ(t.nearestIndex(1.0, 5.0), kNoIndex);    // 5m - 밖
  EXPECT_EQ(t.nearestIndex(99.0, 0.0), kNoIndex);   // 경로 끝에서 96m
  EXPECT_EQ(t.nearestIndex(1.0, 5.0, 10.0), 1u);    // 제한을 늘리면 답한다
}

TEST(LaneTable, NearestIndexOnEmptyTableIsNpos) {
  LaneTable t;
  EXPECT_EQ(t.nearestIndex(0.0, 0.0), kNoIndex);
}

// --- 표가 없을 때 (제일 중요한 안전 성질) ---

TEST(LaneTable, EmptyTableBlocksFullLaneChange) {
  LaneTable t;   // load 안 함
  EXPECT_FALSE(t.spanHasRightLane(0, 10));
  EXPECT_FALSE(t.spanHasLeftLane(0, 10));
  // 모를 때 "옆 차로가 있다" 고 하면 도로 밖으로 나간다. 모를 때 "없다" 면
  // 차선 변경을 못 할 뿐이다. 두 실패의 무게가 다르다.
  EXPECT_FALSE(offsetAllowed(t, -3.51, 0, 10));
  EXPECT_FALSE(offsetAllowed(t, +3.51, 0, 10));
  // 다만 차로 안 비켜가기는 표와 무관하게 된다 - 표가 없다고 회피를 통째로
  // 잃으면 정적장애물 미션을 못 한다.
  EXPECT_TRUE(offsetAllowed(t, -2.0, 0, 10));
}

TEST(LaneTable, EmptyTableStillAllowsStayingOnPath) {
  LaneTable t;
  // 표가 없어도 기준경로(offset 0)는 따라갈 수 있어야 한다.
  EXPECT_TRUE(offsetAllowed(t, 0.0, 0, 10));
}

// --- offsetAllowed 부호 규약 ---

// 부호 규약은 kNudgeMax 를 넘는 오프셋에서만 드러난다 (그 이하는 양쪽 다 허용).
TEST(OffsetAllowed, NegativeIsRightPositiveIsLeft) {
  LaneTable t;
  ASSERT_TRUE(t.load(writeTable({{1, 0}, {1, 0}})));   // 우측만 가능
  EXPECT_TRUE(offsetAllowed(t, -3.51, 0, 1))  << "우측 차로가 있다";
  EXPECT_FALSE(offsetAllowed(t, +3.51, 0, 1)) << "좌측 차로는 없다";
  EXPECT_TRUE(offsetAllowed(t,  0.00, 0, 1));
}

// ★ 옆 차로가 없어도 "차로 안에서 비켜가기" 는 해야 한다.
//
// 처음에는 옆 차로가 없으면 옆으로 나가는 후보를 전부 막았다. 그랬더니 시나리오
// 정적장애물(경로 328.9m, 편도 1차선) 앞에서 차가 서서 못 갔다(2026-09-03 실측).
// 기하상 차로 안에는 답이 없고(통과에 -1.17m 필요, 차로 안 한계 -0.81m),
// 차선 침범 5초 < 충돌 15초 < 미완주 라 나가는 쪽이 맞다.
// 근거는 road_core.hpp 의 kNudgeMax 주석에 있다.
TEST(OffsetAllowed, NudgeAllowedEvenWithoutSideLane) {
  LaneTable t;
  ASSERT_TRUE(t.load(writeTable({{0, 0}, {0, 0}})));   // 좌우 차로 둘 다 없음
  EXPECT_TRUE(offsetAllowed(t,  0.0, 0, 1));
  EXPECT_TRUE(offsetAllowed(t, -1.0, 0, 1)) << "차로 안 비켜가기";
  EXPECT_TRUE(offsetAllowed(t, -2.0, 0, 1)) << "kNudgeMax 경계 - 이 물체 통과에 필요";
  EXPECT_TRUE(offsetAllowed(t, +2.0, 0, 1)) << "좌측도 같은 규칙";
}

// 그보다 큰 것(= 옆 차로로 통째로 옮기기)은 여전히 막는다. 인도 사고가 -3.51 이었다.
TEST(OffsetAllowed, FullLaneChangeStillBlockedWithoutSideLane) {
  LaneTable t;
  ASSERT_TRUE(t.load(writeTable({{0, 0}, {0, 0}})));
  EXPECT_FALSE(offsetAllowed(t, -3.00, 0, 1));
  EXPECT_FALSE(offsetAllowed(t, -3.51, 0, 1));
  EXPECT_FALSE(offsetAllowed(t, +3.51, 0, 1));
}

// 경계값 자체를 못 박아둔다. 이 값이 바뀌면 위 두 테스트의 의미가 달라진다.
TEST(OffsetAllowed, NudgeBoundaryIsTwoMeters) {
  EXPECT_DOUBLE_EQ(kNudgeMax, 2.0);
  EXPECT_TRUE(isNudge(-2.0));
  EXPECT_TRUE(isNudge(+2.0));
  EXPECT_FALSE(isNudge(-2.01));
  EXPECT_FALSE(isNudge(+3.51));
}

// ---------------------------------------------------------------------------
// 실제 코스 데이터로 검증한다.
//
// 위 테스트들은 합성 표를 쓴다. 로직은 맞아도 "우리 코스에서 실제로 그 구간이
// 막히는가" 는 별개 질문이고, 그게 2026-09-03 에 차를 인도로 보냈다.
// 표가 다시 만들어질 때(build_lane_table.py) 이 성질이 깨지면 여기서 잡힌다.
// ---------------------------------------------------------------------------

namespace {
std::string realTablePath() {
  return ros::package::getPath("path_tracking") + "/map/lane_table.csv";
}
}  // namespace

TEST(RealCourse, TableLoads) {
  LaneTable t;
  ASSERT_TRUE(t.load(realTablePath())) << realTablePath();
  EXPECT_EQ(t.size(), 4392u);
}

// 실패 재현. 경로 417.8m(idx 840) 에서 우측 회피를 시작했고 435.4m(idx 876) 에
// 도착했다. 시작점만 보면 우측 차로가 있지만, 429.9m(idx 865) 부터 없어진다.
TEST(RealCourse, RightLaneEndsMidwayAtWaypoint865) {
  LaneTable t;
  ASSERT_TRUE(t.load(realTablePath()));

  EXPECT_TRUE(t.at(840).right_ok)  << "417.8m - 회피를 시작한 지점";
  EXPECT_TRUE(t.at(854).right_ok)  << "425.0m - 아직 우측 차로가 있다";
  EXPECT_FALSE(t.at(865).right_ok) << "429.9m - 여기서 링크가 바뀌며 끊긴다";
  EXPECT_FALSE(t.at(876).right_ok) << "435.4m - 차가 실제로 도착한 곳";

  // 시작점 근처만 보면 통과된다 (예전 판정이 이랬다)
  EXPECT_TRUE(t.spanHasRightLane(840, 854));
  // 후보가 지나갈 구간 전체를 보면 막힌다 (지금 판정)
  EXPECT_FALSE(t.spanHasRightLane(840, 876));
  EXPECT_FALSE(offsetAllowed(t, -3.51, 840, 876));
  // 제자리는 언제나 허용
  EXPECT_TRUE(offsetAllowed(t, 0.0, 840, 876));
}

// ★ 실패 재현 2: 시나리오 정적장애물 구간은 편도 1차선이다.
//
// 경로 328.9m(idx 661)에 그 정적장애물이 있는데 좌우 차로가 둘 다 없다.
// 처음 게이트는 여기서 옆으로 나가는 후보를 전부 막아 차가 서서 못 갔다.
// 이제는 차로 안 비켜가기(<= kNudgeMax)는 통과하고, 차선 변경만 막힌다.
TEST(RealCourse, MissionObstacleStretchIsSingleLane) {
  LaneTable t;
  ASSERT_TRUE(t.load(realTablePath()));

  EXPECT_FALSE(t.at(661).right_ok) << "328.9m - 시나리오 정적장애물 자리";
  EXPECT_FALSE(t.at(661).left_ok);
  EXPECT_FALSE(t.spanHasRightLane(643, 683)) << "320~340m 전 구간";

  // 차선 변경은 막힌다
  EXPECT_FALSE(offsetAllowed(t, -3.51, 643, 683));
  EXPECT_FALSE(offsetAllowed(t, -3.00, 643, 683));
  // 비켜가기는 된다 - 이게 없으면 정적장애물 미션을 못 한다
  EXPECT_TRUE(offsetAllowed(t, -2.00, 643, 683))
      << "이 물체 통과에 -1.17m 가 필요하다 (SAFE_MARGIN 포함)";
  EXPECT_TRUE(offsetAllowed(t, -1.00, 643, 683));
  EXPECT_TRUE(offsetAllowed(t,  0.00, 643, 683));
}

// 코스 대부분에 우측 차로가 없다. 이 값이 크게 변하면 표가 잘못 만들어진 것이다.
TEST(RealCourse, MostOfCourseHasNoSideLane) {
  LaneTable t;
  ASSERT_TRUE(t.load(realTablePath()));
  std::size_t r = 0, l = 0;
  for (std::size_t i = 0; i < t.size(); ++i) {
    if (t.at(i).right_ok) ++r;
    if (t.at(i).left_ok)  ++l;
  }
  EXPECT_EQ(r, 1097u) << "우측 차로가 있는 웨이포인트 (25.0%)";
  EXPECT_EQ(l, 953u)  << "좌측 차로가 있는 웨이포인트 (21.7%)";
}

// 좌표로 조회해도 같은 답이 나온다 (lattice 가 쓰는 경로).
TEST(RealCourse, NearestIndexMatchesWaypoint) {
  LaneTable t;
  ASSERT_TRUE(t.load(realTablePath()));
  EXPECT_EQ(t.nearestIndex(-60.060, -53.532), 840u);
  EXPECT_EQ(t.nearestIndex(-60.148, -35.956), 876u);
  // 코스에서 멀면 "모른다". 아무 웨이포인트나 물어오면 안 된다.
  EXPECT_EQ(t.nearestIndex(0.0, 0.0), kNoIndex);
}
