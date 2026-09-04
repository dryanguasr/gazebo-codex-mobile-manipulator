#!/usr/bin/env bash
set -Eeo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUTPUT_DIR="${1:-$ROOT/results/verified/experiments}"
DURATION_S="${DURATION_S:-30}"
mkdir -p "$OUTPUT_DIR"

source /opt/ros/jazzy/setup.bash
cd "$ROOT"
python3 scripts/cad/prepare_poppy_assets.py
python3 scripts/cad/validate_meshes.py
colcon build --symlink-install >"$OUTPUT_DIR/build.log" 2>&1
source install/setup.bash
set -u

run_case() {
  local label=$1
  local tracking=$2
  ros2 launch mobile_manipulator sim.launch.py \
    tracking_enabled:="$tracking" \
    target_mode:=moving \
    metrics_enabled:=true \
    metrics_output_dir:="$OUTPUT_DIR" \
    run_label:="$label" \
    duration_s:="$DURATION_S" \
    target_distance_m:=1.2 \
    >"$OUTPUT_DIR/${label}_launch.log" 2>&1

  if grep -q '\[Err\]' "$OUTPUT_DIR/${label}_launch.log"; then
    echo "Gazebo reported an error in case $label" >&2
    grep '\[Err\]' "$OUTPUT_DIR/${label}_launch.log" >&2
    return 1
  fi
  test -s "$OUTPUT_DIR/${label}.csv"
  test -s "$OUTPUT_DIR/${label}_summary.json"
  for _ in $(seq 1 40); do
    if ! pgrep -f "gz sim.*ball_arena.sdf" >/dev/null; then
      break
    fi
    sleep 0.25
  done
  if pgrep -f "gz sim.*ball_arena.sdf" >/dev/null; then
    pkill -f "gz sim.*ball_arena.sdf" || true
    echo "Gazebo server remained 10 s after case $label" >&2
    return 1
  fi
}

# A is the same moving target with perception active but base tracking disabled.
run_case A false
# B changes one factor only: the visual tracker commands the base.
run_case B true

python3 scripts/compare_experiments.py "$OUTPUT_DIR"
echo "A/B experiment passed. Evidence: $OUTPUT_DIR"
