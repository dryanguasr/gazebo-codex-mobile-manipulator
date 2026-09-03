#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUTPUT_DIR="${1:-$ROOT_DIR/captures}"
DURATION_S="${2:-60}"
WORK_DIR="$ROOT_DIR/results/video_generation"
SOURCE_DIR="$WORK_DIR/source"
LOG_DIR="$WORK_DIR/logs"
CAMERA_SDF="$ROOT_DIR/scripts/video/eagle_camera.sdf"

mkdir -p "$OUTPUT_DIR" "$SOURCE_DIR" "$LOG_DIR"
rm -f "$SOURCE_DIR/isometric_source.mp4" "$SOURCE_DIR/robot_source.mp4"

source /opt/ros/jazzy/setup.bash
cd "$ROOT_DIR"
colcon build --symlink-install >"$LOG_DIR/build.log" 2>&1
source install/setup.bash
set -u

LAUNCH_PID=""
BRIDGE_PID=""

stop_group() {
  local pid="$1"
  [[ -z "$pid" ]] && return 0
  kill -TERM -- "-$pid" 2>/dev/null || true
  for ((attempt = 0; attempt < 40; attempt++)); do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.25
  done
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  set +e
  stop_group "$BRIDGE_PID"
  stop_group "$LAUNCH_PID"
}
trap cleanup EXIT

wait_for_topic() {
  local topic="$1"
  local attempts="${2:-120}"
  for ((i = 0; i < attempts; i++)); do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      return 0
    fi
    sleep 0.5
  done
  echo "Timed out waiting for $topic" >&2
  return 1
}

setsid ros2 launch mobile_manipulator sim.launch.py   tracking_enabled:=true target_mode:=trefoil   >"$LOG_DIR/simulation.log" 2>&1 &
LAUNCH_PID=$!

wait_for_topic /camera/image_raw 240
wait_for_topic /ball/debug 120

setsid ros2 run ros_gz_bridge parameter_bridge   '/eagle/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'   >"$LOG_DIR/eagle_bridge.log" 2>&1 &
BRIDGE_PID=$!

ros2 run ros_gz_sim create   -world ball_arena   -name eagle_camera   -file $CAMERA_SDF   -x -3 -y -5 -z 7.5 -R 0 -P 0.814 -Y 0.7854   >$LOG_DIR/eagle_spawn.log 2>&1

wait_for_topic /eagle/image_raw 120

for ((i = 0; i < 120; i++)); do
  if ros2 control list_controllers 2>/dev/null |
      grep -Eq '^arm_controller[[:space:]]+joint_trajectory_controller/JointTrajectoryController[[:space:]]+active'; then
    break
  fi
  sleep 0.5
done

ros2 topic pub --once /arm_controller/joint_trajectory   trajectory_msgs/msg/JointTrajectory   "{joint_names: [poppy_m1_joint, poppy_m2_joint, poppy_m3_joint, poppy_m4_joint, poppy_m5_joint, poppy_m6_joint], points: [{positions: [0.25, -0.35, 0.30, -0.25, 0.20, 0.45], time_from_start: {sec: 2}}]}"   >"$LOG_DIR/arm_pose.log" 2>&1
sleep 2

python3 scripts/record_tracking_videos.py   --output-dir "$SOURCE_DIR"   --duration "$DURATION_S"   >"$LOG_DIR/recorder.log" 2>&1

cleanup
trap - EXIT

encode_video() {
  local source="$1"
  local destination="$2"
  local temporary="${destination%.mp4}.new.mp4"
  ffmpeg -v error -y -i "$source"     -an -c:v libx264 -preset medium -crf 18     -pix_fmt yuv420p -r 60 -movflags +faststart "$temporary"
  mv "$temporary" "$destination"
}

encode_video   "$SOURCE_DIR/isometric_source.mp4"   "$OUTPUT_DIR/seguimiento_vista_isometrica_60fps.mp4"
encode_video   "$SOURCE_DIR/robot_source.mp4"   "$OUTPUT_DIR/seguimiento_perspectiva_robot_60fps.mp4"

python3 - "$OUTPUT_DIR" "$DURATION_S" <<'PY'
import json
from pathlib import Path
import subprocess
import sys

output_dir = Path(sys.argv[1])
duration = float(sys.argv[2])
expected_frames = round(duration * 60)
for name, expected_size in (
    ("seguimiento_vista_isometrica_60fps.mp4", (1280, 720)),
    ("seguimiento_perspectiva_robot_60fps.mp4", (960, 720)),
):
    path = output_dir / name
    probe = subprocess.run(
        [
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries",
            "stream=codec_name,width,height,r_frame_rate,avg_frame_rate,nb_frames,duration",
            "-of", "json", str(path),
        ],
        check=True, capture_output=True, text=True,
    )
    stream = json.loads(probe.stdout)["streams"][0]
    assert stream["codec_name"] == "h264", stream
    assert (int(stream["width"]), int(stream["height"])) == expected_size, stream
    assert stream["r_frame_rate"] == "60/1", stream
    assert stream["avg_frame_rate"] == "60/1", stream
    assert int(stream["nb_frames"]) == expected_frames, stream
    assert abs(float(stream["duration"]) - duration) < 0.02, stream
    print(
        f"{path}: {stream['width']}x{stream['height']}, "
        f"{stream['nb_frames']} frames, 60 fps, H.264"
    )
PY

if grep -F '[Err]' "$LOG_DIR/simulation.log"; then
  echo "Gazebo reported an [Err] line" >&2
  exit 1
fi

echo "Tracking videos generated successfully in $OUTPUT_DIR"
