#!/usr/bin/env bash
# 팔 PS 컨트롤러 조작 실행.  사용:  bash ~/humanoid_controller/arm_joy/run_arm_joy.sh
# 처음 한 번은 자동으로 코드를 넣고 빌드합니다 (다음부터는 바로 실행). 종료: Ctrl+C
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
HC="$HOME/humanoid_controller"
SRC_DST="$HC/examples/src/hardware/motor/example_arm_joy.cpp"
CMAKE="$HC/examples/CMakeLists.txt"

export CMAKE_PREFIX_PATH="$HOME/.local:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"
export PKG_CONFIG_PATH="$HOME/.local/lib/pkgconfig:$PKG_CONFIG_PATH"

# 1) 코드 넣기 (바뀐 경우에만 복사)
cmp -s "$HERE/example_arm_joy.cpp" "$SRC_DST" 2>/dev/null || cp "$HERE/example_arm_joy.cpp" "$SRC_DST"

# 2) 빌드 목록에 없으면 두 줄 추가 (기존 내용은 그대로)
if ! grep -q "example_arm_joy" "$CMAKE"; then
    printf '\nadd_executable(example_arm_joy src/hardware/motor/example_arm_joy.cpp)\ntarget_link_libraries(example_arm_joy PRIVATE motor robot joy)\n' >> "$CMAKE"
fi

# 3) 빌드 (이미 되어 있으면 몇 초)
mkdir -p "$HC/build"
cd "$HC/build"
cmake .. > /dev/null
echo "== 빌드 확인 중..."
if ! make example_arm_joy -j2 > "$HC/build/build_arm_joy.log" 2>&1; then
    tail -n 30 "$HC/build/build_arm_joy.log"
    echo "!! 빌드 실패: 위 내용을 보내주세요 (전체: ~/humanoid_controller/build/build_arm_joy.log)"
    exit 1
fi

# 4) 실행 (Ctrl+C 로 종료)
echo
echo "== 팔 조작 시작: 왼쪽 스틱 = 왼팔, 오른쪽 스틱 = 오른팔, Ctrl+C = 종료 (팔 힘 빠짐, 잡고 누르세요)"
exec ./examples/example_arm_joy ../config/robot/ARTI-D1/motor.yaml ../config/robot/ARTI-D1/joy.yaml
