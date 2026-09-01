# Arquitectura del sistema validado

## Objetivo

El robot móvil 4WD observa una esfera roja, detecta su centro, estima el rango, genera velocidades lineal/angular, mueve la base e intenta mantener la esfera centrada y a **1.2 m**. La evaluación usa ground truth separado del controlador.

## Flujo

```text
Gazebo / ball_arena
├─ /clock ───────────────────────────────────────────> ROS 2 sim time
├─ /camera/image_raw ─┐
├─ /camera/camera_info├─> ball_detector ─> /ball/measurement
│                     │                         │
│                     └─> /ball/debug           ▼
│                                           visual_tracker
│                                                │
│                                  /base_controller/cmd_vel
│                                                ▼
└─ gz_ros2_control <──────────────────── base_controller
          │                                     │
          ├─ /joint_states                      ├─ /base_controller/odom
          └─ /tf: odom -> base_footprint        └─ wheel commands

target_trajectory ── SetEntityPose ──> target_ball
        └─ /target/ground_truth ───────────────> metrics_logger
/ball/measurement, /cmd_vel y /odom ───────────> metrics_logger
```

## Componentes

| Función | Archivo |
|---|---|
| Mundo | `src/mobile_manipulator/worlds/ball_arena.sdf` |
| Robot | `src/mobile_manipulator/urdf/mobile_manipulator.urdf.xacro` |
| Controladores | `src/mobile_manipulator/config/controllers.yaml` |
| Launch | `src/mobile_manipulator/launch/sim.launch.py` |
| Percepción | `mobile_manipulator/ball_detector.py` |
| Control | `mobile_manipulator/visual_tracker.py` |
| Objetivo | `mobile_manipulator/target_trajectory.py` |
| Métricas | `mobile_manipulator/metrics_logger.py` |

Las rutas de nodos parten de `src/mobile_manipulator/`.

## Nodos principales

`robot_state_publisher`, `parameter_bridge`, `controller_manager`, `ball_detector`, `visual_tracker`, `target_trajectory` y, cuando se habilita, `metrics_logger`.

## Topics principales

- `/clock`
- `/camera/image_raw`
- `/camera/camera_info`
- `/ball/measurement`
- `/ball/debug`
- `/base_controller/cmd_vel`
- `/base_controller/odom`
- `/joint_states`
- `/target/ground_truth`
- `/tf`, `/tf_static`

## `/ball/measurement`

Tipo `geometry_msgs/Vector3Stamped`:

- X: error horizontal normalizado respecto al semiancho.
- Y: error vertical normalizado respecto a la semialtura.
- Z: rango 3D cámara–centro de esfera en metros.
- `NaN`: detección inválida.

## Frames

`odom`, `base_footprint`, `base_link`, `camera_link`.

TF validado: `odom -> base_footprint`.

## Controladores

- `joint_state_broadcaster`: estados articulares.
- `base_controller`: `DiffDriveController` sobre las cuatro ruedas.
- `arm_controller`: `JointTrajectoryController` de cuatro articulaciones y dos dedos.

## Separación de privilegios

`visual_tracker` solo se suscribe a `/ball/measurement`. No recibe pose de Gazebo ni `/target/ground_truth`.

`metrics_logger` sí recibe ground truth porque su función es evaluar. Esta separación debe conservarse en todas las explicaciones y extensiones del tutorial.
