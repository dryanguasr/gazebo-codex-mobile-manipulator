#!/usr/bin/env bash
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULTS="$ROOT/results/diagnostic"
CAPTURES="$ROOT/captures/png"
mkdir -p "$RESULTS" "$CAPTURES"
source /opt/ros/jazzy/setup.bash
cd "$ROOT"
colcon build --symlink-install > "$RESULTS/build.log" 2>&1
source install/setup.bash
xacro src/mobile_manipulator/urdf/mobile_manipulator.urdf.xacro > "$RESULTS/robot.urdf"
check_urdf "$RESULTS/robot.urdf" > "$RESULTS/check_urdf.txt" 2>&1
ros2 launch mobile_manipulator sim.launch.py > "$RESULTS/launch.log" 2>&1 &
LAUNCH_PID=$!
cleanup() { kill "$LAUNCH_PID" 2>/dev/null || true; wait "$LAUNCH_PID" 2>/dev/null || true; }
trap cleanup EXIT
for n in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -q '/controller_manager/list_controllers'; then break; fi
  sleep 1
done
ros2 control list_controllers -c /controller_manager > "$RESULTS/controllers.txt" 2>&1 || true
ros2 topic list > "$RESULTS/topics.txt" 2>&1 || true
timeout 8 ros2 topic echo --once /joint_states > "$RESULTS/joint_states.txt" 2>&1 || true
timeout 8 ros2 topic echo --once /odom > "$RESULTS/odom.txt" 2>&1 || true
timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint > "$RESULTS/tf.txt" 2>&1 || true
timeout 12 ros2 run mobile_manipulator evidence_capture > "$RESULTS/camera_capture.txt" 2>&1 || true
timeout 5 ros2 topic pub --once /base_controller/cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: base_footprint}, twist: {linear: {x: 0.15}, angular: {z: 0.25}}}' > "$RESULTS/base_command.txt" 2>&1 || true
timeout 5 ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: [arm_base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, left_finger_joint, right_finger_joint], points: [{positions: [0.2, -0.3, 0.5, -0.2, 0.02, -0.02], time_from_start: {sec: 2}}]}' > "$RESULTS/arm_command.txt" 2>&1 || true
sleep 3
ros2 control list_controllers -c /controller_manager > "$RESULTS/controllers_after_commands.txt" 2>&1 || true
grep -Ei 'error|failed|exception' "$RESULTS/launch.log" > "$RESULTS/errors.txt" || true
