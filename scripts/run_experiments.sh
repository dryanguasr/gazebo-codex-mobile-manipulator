#!/usr/bin/env bash
set -Eeo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUN_ID="${RUN_ID:-tracking_metric_reference_$(date -u +%Y%m%dT%H%M%SZ)}"
OUTPUT_DIR="${1:-$ROOT/results/verified/$RUN_ID}"
DURATION_S="${DURATION_S:-30}"
if [[ -e "$OUTPUT_DIR" ]]; then
  echo "Refusing to overwrite experiment directory: $OUTPUT_DIR" >&2
  exit 1
fi
mkdir -p "$OUTPUT_DIR/tracking_A" "$OUTPUT_DIR/tracking_B"

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
  local case_dir="$OUTPUT_DIR/$label"
  ros2 launch mobile_manipulator sim.launch.py \
    tracking_enabled:="$tracking" \
    target_mode:=moving \
    metrics_enabled:=true \
    metrics_output_dir:="$case_dir" \
    run_label:="$label" \
    duration_s:="$DURATION_S" \
    target_distance_m:=1.2 \
    >"$case_dir/${label}_launch.log" 2>&1

  if grep -q '\[Err\]' "$case_dir/${label}_launch.log"; then
    echo "Gazebo reported an error in case $label" >&2
    grep '\[Err\]' "$case_dir/${label}_launch.log" >&2
    return 1
  fi
  test -s "$case_dir/${label}.csv"
  test -s "$case_dir/${label}_summary.json"
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
run_case tracking_A false
# B changes one factor only: the visual tracker commands the base.
run_case tracking_B true

python3 scripts/compare_experiments.py "$OUTPUT_DIR"
echo "A/B experiment passed. Evidence: $OUTPUT_DIR"
