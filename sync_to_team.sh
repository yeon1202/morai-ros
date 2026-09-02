#!/usr/bin/env bash
#
# sync_to_team.sh : 개발용 path_tracking 패키지를 팀 repo 구조로 바꿔서 옮긴다.
#
# 왜 필요한가
#   같은 코드가 두 곳에 있는데 모양이 다르다.
#
#     개발용                                팀 repo
#     catkin_ws/src/path_tracking/          2026_CARSA_AD/autonomous_driving/
#       src/lattice_planner.cpp        ->     src/planning/lattice_planner.cpp
#       include/path_tracking/*.hpp    ->     src/planning/include/planning/*.hpp
#       scripts/ path/ launch/ test/   ->     src/planning/ 아래 그대로
#       map/                           ->     map/            (패키지 루트로)
#
#   팀 repo 는 path_tracking 을 별도 패키지가 아니라 autonomous_driving 안으로
#   병합해서 쓴다. catkin 은 package.xml 을 찾은 디렉터리 아래로 더 안 내려가서,
#   패키지 안에 패키지를 중첩하면 에러도 없이 조용히 무시되기 때문이다.
#
#   그래서 그냥 복사하면 안 되고, 옮길 때마다 파일 배치와 파일 안의 이름
#   몇 가지를 같이 고쳐야 한다. 그걸 손으로 하면 언젠가 하나를 빠뜨린다.
#   이 스크립트가 그 일을 대신하고, 빠뜨린 게 없는지 스스로 검사한다.
#
# 사용법
#   ./sync_to_team.sh              뭐가 바뀌는지 보여주기만 한다 (기본값)
#   ./sync_to_team.sh --apply      실제로 팀 repo 에 반영한다
#   ./sync_to_team.sh --no-build   빌드 검사를 건너뛴다 (빠르게 보고 싶을 때)
#
# 주의
#   이 스크립트는 개발용을 정답으로 치고 팀 repo 를 덮어쓴다. 팀원이 팀 repo 에서
#   planning 코드를 직접 고쳤다면 그게 날아간다. 아래 0단계가 그런 흔적이
#   있는지 확인해서 경고한다.

set -euo pipefail

DEV=${DEV:-/home/yeon/morai-ros/catkin_ws/src/path_tracking}
TEAM=${TEAM:-/home/yeon/2026_CARSA_AD}
CONTAINER=${CONTAINER:-morai-dev}

AD=$TEAM/autonomous_driving
DST_PLANNING=$AD/src/planning
DST_MAP=$AD/map

APPLY=0
BUILD=1
for a in "$@"; do
  case "$a" in
    --apply)    APPLY=1 ;;
    --no-build) BUILD=0 ;;
    -h|--help)  sed -n '3,32p' "$0"; exit 0 ;;
    *) echo "모르는 옵션: $a  (--apply / --no-build / --help)"; exit 2 ;;
  esac
done

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
die() { printf '\n\033[31m✗ %s\033[0m\n' "$*" >&2; exit 1; }

[ -d "$DEV" ]  || die "개발용 폴더가 없다: $DEV"
[ -d "$AD" ]   || die "팀 repo 가 없다: $AD"

STAGE=$(mktemp -d)
trap 'rm -rf "$STAGE"' EXIT

# ─────────────────────────────────────────────────────────────────────
say "0단계 · 팀 repo 에 남의 작업이 있는지 확인"
# ─────────────────────────────────────────────────────────────────────
# 팀원이 팀 repo 에서 planning 을 직접 고쳤다면 이 sync 로 덮어써진다.
# 커밋 이력에 내(seungyeon) 것 아닌 게 있으면 알려준다.
if git -C "$TEAM" rev-parse --git-dir >/dev/null 2>&1; then
  # .gitkeep 은 팀이 빈 폴더 유지용으로 넣은 것이고 이 스크립트가 보호하므로
  # (--exclude='.gitkeep') 세지 않는다. 헛경보가 반복되면 진짜 경고를 무시하게 된다.
  OTHERS=$(git -C "$TEAM" log --format='%an' -- \
             autonomous_driving/src/planning autonomous_driving/map \
             ':(exclude)*/.gitkeep' 2>/dev/null \
           | sort -u | grep -v '^seungyeon$' || true)
  if [ -n "$OTHERS" ]; then
    printf '  ⚠️  planning/map 을 건드린 다른 사람이 있다: %s\n' "$(echo "$OTHERS" | tr '\n' ' ')"
    printf '     이 sync 로 그 작업이 덮어써진다. git log 를 먼저 확인할 것.\n'
  else
    echo "  다른 사람 커밋 없음"
  fi
  DIRTY=$(git -C "$TEAM" status --porcelain -- autonomous_driving/src/planning autonomous_driving/map | wc -l)
  [ "$DIRTY" -gt 0 ] && echo "  참고: 커밋 안 된 변경 $DIRTY 건이 이미 있다"
else
  echo "  (팀 폴더가 git repo 가 아니라 건너뜀)"
fi

# ─────────────────────────────────────────────────────────────────────
say "1단계 · 팀 구조로 재배치 (임시 폴더에서 만든다)"
# ─────────────────────────────────────────────────────────────────────
# 임시 폴더에서 먼저 완성한 뒤 검사를 통과해야 진짜로 옮긴다.
# 실패해도 팀 repo 는 손상되지 않는다.
EXCL=(--exclude='*.bak' --exclude='__pycache__' --exclude='*.pyc')

mkdir -p "$STAGE/planning/include/planning" "$STAGE/map"
rsync -a "${EXCL[@]}" "$DEV/scripts" "$DEV/launch" "$DEV/path" "$DEV/test" "$STAGE/planning/"
rsync -a "${EXCL[@]}" "$DEV/src/"                    "$STAGE/planning/"          # .cpp 를 한 층 위로
rsync -a "${EXCL[@]}" "$DEV/include/path_tracking/"  "$STAGE/planning/include/planning/"
rsync -a "${EXCL[@]}" "$DEV/map/"                    "$STAGE/map/"
echo "  파일 $(find "$STAGE" -type f | wc -l)개 배치"

# ─────────────────────────────────────────────────────────────────────
say "2단계 · 파일 안의 이름 고치기"
# ─────────────────────────────────────────────────────────────────────
# 패키지 이름이 바뀌므로 코드 안에서 그 이름을 부르는 곳도 같이 바꿔야 한다.
#   #include "path_tracking/..."  헤더가 include/planning/ 으로 갔다
#   getPath("path_tracking")      패키지가 autonomous_driving 이 됐다
#   pkg="path_tracking"           launch 가 노드를 찾는 패키지 이름
#   rosrun/roslaunch path_tracking  주석에 적힌 사용법
find "$STAGE" -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.py' -o -name '*.launch' \) -print0 |
xargs -0 sed -i \
  -e 's|#include "path_tracking/|#include "planning/|' \
  -e 's|ros::package::getPath("path_tracking") + "/path/|ros::package::getPath("autonomous_driving") + "/src/planning/path/|' \
  -e 's|ros::package::getPath("path_tracking") + "/map/|ros::package::getPath("autonomous_driving") + "/map/|' \
  -e 's|$(find path_tracking)/path/|$(find autonomous_driving)/src/planning/path/|g' \
  -e 's|$(find path_tracking)/map/|$(find autonomous_driving)/map/|g' \
  -e 's|pkg="path_tracking"|pkg="autonomous_driving"|g' \
  -e 's|rosrun path_tracking |rosrun autonomous_driving |g' \
  -e 's|roslaunch path_tracking |roslaunch autonomous_driving |g'
echo "  치환 완료"

# ─────────────────────────────────────────────────────────────────────
say "3단계 · 검사 ① 못 고친 곳이 남았나"
# ─────────────────────────────────────────────────────────────────────
# 이게 이 스크립트의 핵심 안전장치다. 위 치환 규칙은 "지금 아는 패턴"만 안다.
# 새 코드가 새로운 방식으로 path_tracking 을 부르면 규칙이 못 잡는데,
# 변환이 끝난 뒤에도 그 글자가 남아 있으면 바로 그 경우다. 여기서 멈춘다.
if LEFT=$(grep -rn "path_tracking" "$STAGE" 2>/dev/null); then
  echo "$LEFT" | sed "s|$STAGE/|  |"
  die "치환 규칙이 못 잡은 곳이 있다. 위 줄을 확인하고, 규칙을 늘리기보다
     그 코드에서 폴더 이름을 안 쓰도록 고치는 쪽이 안전하다."
fi
echo "  OK · path_tracking 이라는 이름이 남은 곳 없음"

# ─────────────────────────────────────────────────────────────────────
say "3단계 · 검사 ② 빌드 목록이 맞나"
# ─────────────────────────────────────────────────────────────────────
# 새 .cpp 노드를 추가하면 팀 CMakeLists 에도 add_executable 을 손으로 넣어야
# 한다(이건 자동화하지 않는다 - 팀 파일을 함부로 고치면 안 되므로).
# 빠뜨렸는지만 알려준다.
list_dev()  { grep -oP '^\s*(add_executable|catkin_add_gtest)\(\K[a-z_]+' "$DEV/CMakeLists.txt" | sort -u; }
list_team() { grep -oP '^\s*(add_executable|catkin_add_gtest)\(\K[a-z_]+(?=.*src/planning/)' "$AD/CMakeLists.txt" | sort -u; }
MISSING=$(comm -23 <(list_dev) <(list_team) || true)
EXTRA=$(comm -13 <(list_dev) <(list_team) || true)
if [ -n "$MISSING" ]; then
  printf '  ⚠️  팀 CMakeLists.txt 에 없다 (직접 추가해야 빌드된다):\n'
  echo "$MISSING" | sed 's/^/       /'
  printf '     예)  add_executable(이름 src/planning/이름.cpp)\n'
fi
[ -n "$EXTRA" ] && { printf '  ⚠️  팀에만 있고 개발용엔 없다 (지워진 노드인가?):\n'; echo "$EXTRA" | sed 's/^/       /'; }
[ -z "$MISSING$EXTRA" ] && echo "  OK · 빌드 목록 일치"

# ─────────────────────────────────────────────────────────────────────
say "4단계 · 바뀌는 내용"
# ─────────────────────────────────────────────────────────────────────
# --delete 는 개발용에서 지운 파일을 팀 쪽에서도 지운다는 뜻이다.
# .gitkeep 은 팀이 만든 파일이라 지우면 안 되므로 제외해서 보호한다.
#   map/ 은 --delete 를 쓰지 않는다. autonomous_driving/map/ 은 팀과 공유하는
#   자리라서, 나중에 팀이 자기 지도 파일을 넣었을 때 그걸 지워버리면 안 된다.
SYNC_PLANNING=(rsync -a --delete --exclude='.gitkeep' "$STAGE/planning/" "$DST_PLANNING/")
SYNC_MAP=(rsync -a "$STAGE/map/" "$DST_MAP/")

"${SYNC_PLANNING[@]}" --dry-run --itemize-changes | grep -v '^\.' | sed 's/^/  planning  /' || true
"${SYNC_MAP[@]}"      --dry-run --itemize-changes | grep -v '^\.' | sed 's/^/  map       /' || true
echo "  (윗줄이 없으면 바뀌는 게 없다는 뜻)"

# ─────────────────────────────────────────────────────────────────────
if [ "$BUILD" = 1 ]; then
say "5단계 · 검사 ③ 실제로 빌드되나"
# ─────────────────────────────────────────────────────────────────────
# 팀 repo 사본에 이번 결과를 얹어서 컨테이너에서 빌드해 본다. 팀 repo 자체는
# 안 건드린다. 빌드를 돌려야만 나오는 에러가 있다 - 예를 들어 find_package 한
# 패키지를 package.xml 의 build_depend 에 안 적으면 catkin 이 configure 를
# 거부하는데, 파일만 봐서는 안 보인다.
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "  ⚠️  컨테이너 $CONTAINER 가 안 떠 있어서 건너뛴다 (docker compose up -d)"
else
  WS=/tmp/sync_build
  docker exec "$CONTAINER" rm -rf $WS
  docker exec "$CONTAINER" mkdir -p $WS/src
  docker cp "$TEAM/." "$CONTAINER:$WS/src/" >/dev/null
  docker exec "$CONTAINER" rm -rf $WS/src/.git
  docker cp "$STAGE/planning/." "$CONTAINER:$WS/src/autonomous_driving/src/planning/" >/dev/null
  docker cp "$STAGE/map/."      "$CONTAINER:$WS/src/autonomous_driving/map/" >/dev/null

  # 이 컨테이너엔 OSQP(팀 control 코드용)가 없다. planning 이 목적이므로
  # 그 타겟만 사본에서 빼고 돌린다. 그래서 vehicle_control 은 검사 대상이 아니다.
  # docker exec 는 -i 가 없으면 stdin 을 컨테이너로 넘기지 않는다.
  # 그러면 아래 heredoc 이 python 에 안 들어가서 패치가 조용히 안 걸린다.
  docker exec -i "$CONTAINER" python3 - <<'PY'
p="/tmp/sync_build/src/autonomous_driving/CMakeLists.txt"
s=open(p,encoding="utf-8").read()
s=s.replace("find_package(OsqpEigen REQUIRED)","# find_package(OsqpEigen REQUIRED)  # [sync 검사용 임시]")
s=s.replace("add_executable(vehicle_control src/control/vehicle_control.cpp)","if(FALSE)\nadd_executable(vehicle_control src/control/vehicle_control.cpp)")
s=s.replace("  OsqpEigen::OsqpEigen\n)","  OsqpEigen::OsqpEigen\n)\nendif()")
open(p,"w",encoding="utf-8").write(s)
PY

  echo "  빌드 중... (1~2분)"
  if docker exec "$CONTAINER" bash -lc "cd $WS && source /opt/ros/noetic/setup.bash && catkin_make -j4" >"$STAGE/build.log" 2>&1; then
    echo "  OK · 빌드 성공 (vehicle_control 은 OSQP 미설치라 제외됨)"
    if docker exec "$CONTAINER" bash -lc "cd $WS && source devel/setup.bash && catkin_make run_tests" >"$STAGE/test.log" 2>&1 \
       && grep -q 'PASSED' "$STAGE/test.log"; then
      echo "  OK · 테스트 $(grep -oP '\d+(?= tests\.)' "$STAGE/test.log" | tail -1)개 통과"
    else
      tail -20 "$STAGE/test.log"; die "테스트 실패"
    fi
  else
    tail -25 "$STAGE/build.log"; die "빌드 실패 - 팀 repo 는 안 건드렸다"
  fi
  docker exec "$CONTAINER" rm -rf $WS
fi
fi

# ─────────────────────────────────────────────────────────────────────
if [ "$APPLY" = 1 ]; then
  say "6단계 · 팀 repo 에 반영"
  "${SYNC_PLANNING[@]}"
  "${SYNC_MAP[@]}"
  echo "  완료. git status 로 확인하고 커밋할 것:"
  echo "    cd $TEAM && git status"
else
  say "미리보기만 했다. 실제로 반영하려면:"
  echo "    $0 --apply"
fi
