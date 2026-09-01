#!/usr/bin/env bash
set -Eeo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RESULTS="${1:-$ROOT/results/verified/diagnostic}"
CAPTURES="$RESULTS/captures"
mkdir -p "$RESULTS" "$CAPTURES"

source /opt/ros/jazzy/setup.bash
cd "$ROOT"

colcon build --symlink-install >"$RESULTS/build.log" 2>&1
source install/setup.bash
set -u
xacro src/mobile_manipulator/urdf/mobile_manipulator.urdf.xacro \
  >"$RESULTS/robot.urdf"
check_urdf "$RESULTS/robot.urdf" >"$RESULTS/check_urdf.txt" 2>&1

ros2 launch mobile_manipulator sim.launch.py \
  tracking_enabled:=false \
  target_mode:=static \
  metrics_enabled:=false \
  >"$RESULTS/launch.log" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -TERM "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  pkill -f "gz sim.*ball_arena.sdf" 2>/dev/null || true
}
trap cleanup EXIT

wait_for_publisher() {
  local topic=$1
  local attempts=${2:-80}
  for _ in $(seq 1 "$attempts"); do
    if ros2 topic info "$topic" 2>/dev/null |
      grep -E 'Publisher count: [1-9]' >/dev/null; then
      return 0
    fi
    sleep 0.25
  done
  echo "Required publisher missing: $topic" >&2
  return 1
}

wait_for_service() {
  local service=$1
  for _ in $(seq 1 120); do
    if ros2 service list 2>/dev/null |
      grep -Fx "$service" >/dev/null; then
      return 0
    fi
    sleep 0.25
  done
  echo "Required service missing: $service" >&2
  return 1
}

wait_for_service /controller_manager/list_controllers
for topic in \
  /clock \
  /joint_states \
  /camera/image_raw \
  /camera/camera_info \
  /ball/measurement \
  /base_controller/odom \
  /tf; do
  wait_for_publisher "$topic"
done

ros2 topic list -t >"$RESULTS/topics.txt"
ros2 service call \
  /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers \
  '{}' >"$RESULTS/controllers.txt"
grep -Eq "name='base_controller'.*state='active'" "$RESULTS/controllers.txt"
grep -Eq "name='arm_controller'.*state='active'" "$RESULTS/controllers.txt"
grep -Eq "name='joint_state_broadcaster'.*state='active'" \
  "$RESULTS/controllers.txt"

ros2 param get /controller_manager use_sim_time >"$RESULTS/use_sim_time.txt"
ros2 param get /base_controller use_sim_time >>"$RESULTS/use_sim_time.txt"
ros2 param get /robot_state_publisher use_sim_time >>"$RESULTS/use_sim_time.txt"
grep -c 'Boolean value is: True' "$RESULTS/use_sim_time.txt" |
  grep -qx '3'

timeout 10 ros2 topic echo --once /clock >"$RESULTS/clock.txt"
timeout 10 ros2 topic echo --once /joint_states >"$RESULTS/joint_states.txt"
timeout 10 ros2 topic echo --once /camera/camera_info \
  >"$RESULTS/camera_info.txt"
timeout 10 ros2 topic echo --once /ball/measurement \
  >"$RESULTS/ball_measurement.txt"
timeout 10 ros2 topic echo --once /base_controller/odom \
  >"$RESULTS/odom_before.txt"

if timeout 6 ros2 run tf2_ros tf2_echo odom base_footprint \
  >"$RESULTS/tf.txt" 2>&1; then
  :
elif [[ $? -ne 124 ]]; then
  exit 1
fi
grep -q 'Translation:' "$RESULTS/tf.txt"

if timeout 4 ros2 topic pub -r 20 \
  /base_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_footprint}, twist: {linear: {x: 0.20}}}' \
  >"$RESULTS/base_command.txt" 2>&1; then
  :
elif [[ $? -ne 124 ]]; then
  exit 1
fi
sleep 1
timeout 10 ros2 topic echo --once /base_controller/odom \
  >"$RESULTS/odom_after.txt"
if timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint \
  >"$RESULTS/tf_after.txt" 2>&1; then
  :
elif [[ $? -ne 124 ]]; then
  exit 1
fi
grep -q 'Translation:' "$RESULTS/tf_after.txt"

timeout 10 ros2 topic pub --once \
  /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [arm_base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, left_finger_joint, right_finger_joint], points: [{positions: [0.2, -0.3, 0.5, -0.2, 0.02, -0.02], time_from_start: {sec: 2}}]}' \
  >"$RESULTS/arm_command.txt" 2>&1
sleep 3
timeout 10 ros2 topic echo --once /joint_states \
  >"$RESULTS/joint_states_after_arm.txt"

timeout 12 ros2 run mobile_manipulator evidence_capture \
  --ros-args -p output_dir:="$CAPTURES" \
  >"$RESULTS/camera_capture.txt" 2>&1

python3 scripts/validate_diagnostic.py "$RESULTS"
echo "Diagnostic passed. Evidence: $RESULTS"
