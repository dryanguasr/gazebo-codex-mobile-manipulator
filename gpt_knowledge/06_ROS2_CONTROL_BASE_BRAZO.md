# ros2_control, base, brazo y joint states

## Controladores

Archivo: `src/mobile_manipulator/config/controllers.yaml`.

### `joint_state_broadcaster`

Publica `/joint_states`. La validación final mostró 10 joints: 4 ruedas, 4 articulaciones del brazo y 2 dedos.

### `base_controller`

Tipo `diff_drive_controller/DiffDriveController`.

Izquierda: `front_left_joint`, `rear_left_joint`. Derecha: `front_right_joint`, `rear_right_joint`.

Parámetros centrales:

- `wheel_separation: 0.60`
- `wheel_radius: 0.115`
- `publish_rate: 50.0`
- `base_frame_id: base_footprint`
- `odom_frame_id: odom`
- `enable_odom_tf: true`
- `cmd_vel_timeout: 0.5`

Comando: `/base_controller/cmd_vel`, tipo `geometry_msgs/msg/TwistStamped`.

### `arm_controller`

Tipo `joint_trajectory_controller/JointTrajectoryController`.

Joints: `arm_base_yaw`, `shoulder_pitch`, `elbow_pitch`, `wrist_pitch`, `left_finger_joint`, `right_finger_joint`. Interfaz de comando: posición.

## Inspeccionar controladores

```bash
ros2 service call \
  /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers \
  '{}'
```

Deben aparecer activos los tres controladores.

## Joint states

```bash
ros2 topic echo --once /joint_states
```

Pedir al estudiante identificar nombres y posiciones antes de explicar todo el mensaje.

## Mover la base

Ejemplo temporal:

```bash
ros2 topic pub -r 20 \
  /base_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_footprint}, twist: {linear: {x: 0.20}}}'
```

Detener con `Ctrl+C`. Un único mensaje puede expirar por `cmd_vel_timeout`; por eso el diagnóstico publica repetidamente durante unos segundos.

## Brazo/pinza

Consignas verificadas:

- `arm_base_yaw = 0.2`
- `shoulder_pitch = -0.3`
- `elbow_pitch = 0.5`
- `wrist_pitch = -0.2`
- `left_finger_joint = 0.02`
- `right_finger_joint = -0.02`

El validador exige que cada joint llegue a ±0.03 rad o m de la consigna.

## Cadena conceptual

Base:

```text
TwistStamped → DiffDriveController → velocidades de ruedas
→ gz_ros2_control → física Gazebo → estados → odometría/TF
```

Brazo:

```text
JointTrajectory → JointTrajectoryController → posiciones
→ gz_ros2_control → modelo → /joint_states
```

Mover joints no equivale a manipulación autónoma. IK, MoveIt y pick-and-place quedan fuera del hito.
