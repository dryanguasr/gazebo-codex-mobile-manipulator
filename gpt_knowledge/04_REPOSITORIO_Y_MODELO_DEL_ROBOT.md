# Repositorio y modelo del robot

## Árbol relevante

```text
README.md
docs/
scripts/
  run_diagnostic.sh
  validate_diagnostic.py
  run_experiments.sh
  compare_experiments.py
src/mobile_manipulator/
  config/controllers.yaml
  launch/sim.launch.py
  mobile_manipulator/
    ball_detector.py
    visual_tracker.py
    target_trajectory.py
    metrics_logger.py
    evidence_capture.py
  test/test_algorithms.py
  urdf/mobile_manipulator.urdf.xacro
  worlds/ball_arena.sdf
results/verified/
  diagnostic/
  experiments/
```

## Base móvil

Modelo docente 4WD:

- `base_link`: 0.72 × 0.52 × 0.16 m;
- masa declarada: 18 kg;
- radio de rueda: 0.115 m;
- separación configurada: 0.60 m;
- cuatro joints continuos comandados en velocidad.

Ruedas: `front_left_joint`, `rear_left_joint`, `front_right_joint`, `rear_right_joint`.

El `DiffDriveController` agrupa ambas ruedas izquierdas y ambas derechas como dos lados de una base diferencial.

## Brazo y pinza

Cuatro GDL rotacionales: `arm_base_yaw`, `shoulder_pitch`, `elbow_pitch`, `wrist_pitch`.

Pinza: `left_finger_joint` y `right_finger_joint`, ambos prismáticos.

El hito solo valida control articular; no incluye IK, MoveIt ni pick-and-place.

## Cámara

`camera_link` está fija al frente de `base_link` con offset X 0.38 m y Z 0.10 m respecto a la base. Sensor:

- 640×480 px;
- FOV horizontal 1.047 rad;
- 30 Hz;
- near 0.05 m;
- far 20 m.

## Esfera objetivo

En `ball_arena.sdf`:

- nombre `target_ball`;
- radio 0.12 m;
- roja;
- `static=true`;
- pose inicial `(2, 0, 0.12)`.

Se reposiciona con `SetEntityPose`. Es una decisión docente para tener trayectoria exacta sin rodadura/rebotes.

## Mundo

Incluye gravedad, física con paso máximo 0.001 s, suelo, iluminación, sistemas de física/comandos/scene/sensores y render OGRE2.

## ros2_control

Plugin de sistema: `gz_ros2_control/GazeboSimSystem`.

- ruedas: comando velocidad + estado posición/velocidad;
- brazo/pinza: comando posición + estado posición.

## Qué enseñar al abrir el Xacro

No explicar todo línea por línea. Pedir al estudiante localizar primero:

1. `base_link`;
2. una rueda y su joint;
3. una articulación del brazo;
4. `camera_link`;
5. bloque `<ros2_control>`;
6. sensor `<camera>`.

Relacionar después cada bloque con lo que observa en Gazebo o ROS 2.

La geometría es intencionalmente sencilla y **no representa el diseño mecánico final del robot cosechador**.
