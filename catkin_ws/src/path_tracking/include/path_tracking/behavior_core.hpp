// behavior_core.hpp : behavior FSM 순수 로직 (ROS 비의존, 단위=m/s로 통일)
//
// 종방향은 상태가 없다. 여러 이유(크루즈·곡률·앞차·신호등·보행자·회피불가)가 각자
// "지금 허용되는 최대 속도" 를 내놓고, 그중 가장 낮은 것을 따르면 된다. 속도제한
// 표지판이 여러 개면 제일 낮은 걸 따르는 것과 같다.
//
// 설계: docs/50-behavior_fsm_design.md
#pragma once
#include <cstddef>
#include <vector>

namespace behavior {

// "제한 없음" 을 뜻하는 약속값. 51-traffic_light_brief.md 5절 규약.
// 무한대가 아니라 큰 유한값을 쓰는 이유는, 어떤 경로로 계산돼도 NaN 이 생기지
// 않게 하기 위해서다(inf - inf 같은 것).
constexpr double kNoLimit = 1e6;

// 제약이 끊겼을 때 어떻게 할지.
//
// 정책이 하나가 아닌 이유: 노드가 죽었다고 코스 한복판에 서버리면 곤란하지만,
// 안전이 걸린 제약을 그냥 무시할 수도 없다. 그 사이를 나눈다.
enum class StaleAction {
  Ignore,        // 무시하고 계속 간다 (acc / traffic_light / avoid)
  Conservative,  // stale_value 로 대체해 보수 주행 (pedestrian)
};

// 제약 하나.
struct Limit {
  double value = kNoLimit;   // [m/s] 지금 허용되는 최대 속도
  // 마지막으로 값을 받은 시각 [s]. **음수면 한 번도 못 받았다는 뜻이다.**
  // 시각은 항상 0 이상이므로 별도 bool 없이 이것만으로 구분된다.
  double stamp = -1.0;
  StaleAction on_stale = StaleAction::Ignore;
  double stale_value = kNoLimit;  // Conservative 일 때 쓸 대체값 [m/s]
};

// 합성 결과.
struct Combined {
  double value  = kNoLimit;  // 최종 상한 [m/s]
  int    winner = -1;        // 이긴 제약의 인덱스. -1 이면 유효한 제약이 없음
  int    alive  = 0;         // 유효하게 반영된 제약 수
};

// 살아있는 제약 중 가장 낮은 값을 고른다.
//
// "한 번도 안 옴" 과 "오다가 끊김" 을 구분하는 것이 핵심이다.
//   한 번도 안 옴  = 그 모듈이 아예 없다(예: perception 미개발) -> 무시
//   오다가 끊김    = 돌던 것이 죽었다                          -> 정책 적용
// 둘을 같이 취급하면, perception 이 없는 개발 단계에서 보행자 제약이 계속
// 보수값으로 걸려 차가 영영 느리게 다닌다.
//
// winner 는 진단용이다. 제약이 여섯 개가 되면 "차가 왜 느린가" 를 눈으로 알 수
// 없다. 노드가 이 인덱스로 이름을 찾아 /speed_limit/active 에 내보내면 즉답이 된다.
//
// alive == 0 이면 유효한 제약이 하나도 없다는 뜻이다. 이때 kNoLimit(무제한)을
// 그대로 쓰면 위험하므로, 노드가 발행을 멈추고 경고해야 한다(그러면 path_tracker
// 가 자체 폴백으로 넘어간다).
inline Combined combineLimits(const std::vector<Limit>& limits,
                              double now, double timeout) {
  Combined c;
  for (std::size_t i = 0; i < limits.size(); ++i) {
    const Limit& L = limits[i];
    if (L.stamp < 0.0) continue;   // 한 번도 안 옴

    double v = L.value;
    if (now - L.stamp > timeout) { // 오다가 끊김
      if (L.on_stale == StaleAction::Ignore) continue;
      v = L.stale_value;
    }

    ++c.alive;
    if (v < c.value) {
      c.value  = v;
      c.winner = static_cast<int>(i);
    }
  }
  return c;
}

}  // namespace behavior
