#!/usr/bin/env bash
# Host-side test runner. No hardware, no PlatformIO -- just g++.
#
#   ./run_tests.sh            run every test
#   ./run_tests.sh maze walls run only the named ones
#
# Every test in this directory is a standalone main() that returns non-zero
# on failure. Each entry below lists the translation units it links; keep it
# in sync when a test grows a new dependency. If a test needs a mock it
# provides the mock itself, so the real module is simply left out of its
# link line (test_motion mocks imu_read_fifo, so ImuIcm45686.cpp is absent).

set -u
cd "$(dirname "$0")"

CXX=${CXX:-g++}
CXXFLAGS="-std=c++17 -Wall -Wextra -Wno-unused-parameter -O1 -g"
INC="-Istubs -I../lib/Control -I../lib/Drivers -I../lib/HAL -I../lib/Maze -I../lib/Application"
L=../lib
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

declare -A SRC=(
  [fifo_decode]="$L/Drivers/ImuIcm45686.cpp"
  [fusion]="$L/Control/Fusion.cpp"
  [profile]="$L/Control/Profile.cpp"
  [path]="$L/Control/Path.cpp"
  [wall_gate]="$L/Control/WallGate.cpp"
  [sweep]=""
  [radio_ring]=""
  [maze]="$L/Maze/Maze.cpp"
  [route]="$L/Application/Route.cpp $L/Maze/Maze.cpp $L/Control/Path.cpp"
  [explore]="$L/Control/Path.cpp"
  [walls]="$L/Control/Walls.cpp"
  [start_gate]=""
  [diag_lane]=""
  [duty_mix]=""
  [big_maze]="$L/Application/Route.cpp $L/Maze/Maze.cpp $L/Control/Path.cpp"
  [wall_align]="$L/Control/WallAlign.cpp"
  [sensors]="$L/Drivers/Sensors.cpp"
  [params]="$L/Application/Params.cpp"
  [lapreport]="$L/Application/LapReport.cpp $L/Application/Params.cpp"
  [ladder]="$L/Application/RunLadder.cpp $L/Application/Params.cpp"
  [goff_table]="$L/Application/GoffTable.cpp $L/Application/Params.cpp"
  [motors]="$L/Drivers/Motors.cpp"
  [odometry]="$L/Control/Control.cpp $L/Control/Path.cpp $L/Control/Fusion.cpp $L/Control/Profile.cpp $L/Drivers/Encoders.cpp $L/Drivers/ImuIcm45686.cpp $L/Drivers/Motors.cpp"
  [velocity_control]="$L/Control/Control.cpp $L/Control/Path.cpp $L/Control/Fusion.cpp $L/Control/Profile.cpp $L/Drivers/Encoders.cpp $L/Drivers/ImuIcm45686.cpp $L/Drivers/Motors.cpp"
  [motion]="$L/Control/Control.cpp $L/Control/Path.cpp $L/Control/Fusion.cpp $L/Control/Profile.cpp $L/Drivers/Encoders.cpp $L/Drivers/Motors.cpp"
  [move]="$L/Control/Control.cpp $L/Control/Path.cpp $L/Control/Fusion.cpp $L/Control/Profile.cpp $L/Drivers/Encoders.cpp $L/Drivers/Motors.cpp"
  [turn]="$L/Control/Control.cpp $L/Control/Path.cpp $L/Control/Fusion.cpp $L/Control/Profile.cpp $L/Drivers/Encoders.cpp $L/Drivers/Motors.cpp"
  [straight_cal]="$L/Control/Control.cpp $L/Control/Path.cpp $L/Control/Fusion.cpp $L/Control/Profile.cpp $L/Drivers/Encoders.cpp $L/Drivers/Motors.cpp"
)

ORDER=(fifo_decode fusion profile path wall_gate sweep radio_ring maze route explore walls start_gate diag_lane duty_mix big_maze wall_align sensors params lapreport ladder goff_table motors \
       odometry velocity_control motion move turn straight_cal)

if [ $# -gt 0 ]; then ORDER=("$@"); fi

pass=0; fail=0; failed=()
for t in "${ORDER[@]}"; do
  printf '%-20s ' "$t"
  if ! $CXX $CXXFLAGS $INC ${SRC[$t]} "test_$t.cpp" -o "$OUT/$t" > "$OUT/$t.build" 2>&1; then
    echo "BUILD FAIL"; sed 's/^/    /' "$OUT/$t.build" | head -20
    fail=$((fail+1)); failed+=("$t"); continue
  fi
  if "$OUT/$t" > "$OUT/$t.run" 2>&1; then
    echo "pass"; pass=$((pass+1))
  else
    echo "FAIL"; sed 's/^/    /' "$OUT/$t.run" | tail -20
    fail=$((fail+1)); failed+=("$t")
  fi
done

echo
echo "$pass passed, $fail failed"
[ $fail -eq 0 ] || { echo "failed: ${failed[*]}"; exit 1; }
