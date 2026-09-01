# Referencia de código y fuentes autoritativas

## Commit validado

`251a1f6ca761c85449af9aeaf162c1fa8aa78e47`

`main` auditado posteriormente: `8f75a8980b69aff3b19ff55d4def4fd0e9d63421`, con cambio únicamente documental del marcador SHA.

## Archivos esenciales

### `launch/sim.launch.py`

Inicia Gazebo server, genera `robot_description`, spawnea el robot, bridgea `/clock`, imagen, `CameraInfo` y `set_pose`, activa tres controladores e inicia detector/tracker/target/métricas. Implementa cleanup específico del mundo.

### `config/controllers.yaml`

Define `joint_state_broadcaster`, `base_controller` y `arm_controller`.

### `urdf/mobile_manipulator.urdf.xacro`

Base 4WD, brazo, pinza, cámara, sensor y `ros2_control`.

### `worlds/ball_arena.sdf`

Física, suelo, luz, plugins y esfera roja estática de radio 0.12 m.

### `ball_detector.py`

Funciones puras:

- `focal_length_from_fov`
- `estimate_sphere_distance`
- `depth_to_range`

Usa `CameraInfo` como primera opción y fallback derivado de FOV.

### `visual_tracker.py`

Parámetros por defecto:

```text
target_distance_m = 1.2
linear_gain = 0.7
angular_gain = 1.8
max_linear_speed_mps = 0.45
max_angular_speed_radps = 1.2
distance_deadband_m = 0.04
horizontal_deadband = 0.02
alignment_slowdown = 0.8
measurement_timeout_s = 0.3
```

### `target_trajectory.py`

```text
centre_x_m = 2.0
height_m = 0.12
longitudinal_amplitude_m = 0.45
lateral_amplitude_m = 0.65
angular_frequency_rad_s = 0.25
update_rate_hz = 20
```

### `metrics_logger.py`

CSV:

`timestamp_s`, `elapsed_s`, `valid_detection`, `horizontal_error`, `estimated_distance_m`, `target_distance_m`, `distance_target_error_m`, `linear_command_mps`, `angular_command_radps`, `robot_x_m`, `robot_y_m`, `robot_yaw_rad`, `target_x_m`, `target_y_m`, `ground_truth_camera_distance_m`, `estimation_error_m`.

### `test/test_algorithms.py`

Seis tests de focal, esfera, depth→range, trayectoria, clamp y resumen de métricas.

### Diagnóstico

- `scripts/run_diagnostic.sh`
- `scripts/validate_diagnostic.py`
- fuente numérica: `results/verified/diagnostic/summary.json`

### Experimento

- `scripts/run_experiments.sh`
- `scripts/compare_experiments.py`
- fuente numérica: `results/verified/experiments/comparison.json`

## Datos canónicos

```text
odom topic: /base_controller/odom
TF: odom -> base_footprint
diagnostic base displacement: 0.370 m
CameraInfo fx: 554.383 px
initial estimated ball range: 1.633 m

A detection: 100%
B detection: 100%
A target-distance MAE: 0.535 m
B target-distance MAE: 0.088 m
B steady-state MAE: 0.083 m
B horizontal RMS: 0.034
B robot displacement: 0.368 m
B/A improvement: 83.6%
```

## Web externa

Para nombres/configuración específicos del ejemplo, no reemplazar estos datos por tutoriales genéricos. La web puede complementar documentación o errores no cubiertos; preferir documentación oficial ROS 2/Gazebo y advertir si corresponde a otra distribución/versión.
