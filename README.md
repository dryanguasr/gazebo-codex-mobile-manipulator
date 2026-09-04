# Seguimiento visual con ROS 2 Jazzy y Gazebo

Ejemplo reproducible para estudiantes de ingeniería mecatrónica: un robot móvil
4WD observa una esfera roja con una cámara monocular, estima su rango usando
los intrínsecos de `CameraInfo` y controla la base para mantener una distancia
de referencia. El manipulador integra ahora seis motores y geometría CAD
oficial de Poppy Ergo Jr, incluida su pinza rotativa, sin perder el seguimiento
visual medible.

## Estado verificado

El flujo completo fue validado en ROS 2 Jazzy y Gazebo Sim 8:

- build, Xacro/URDF y spawn correctos;
- `joint_state_broadcaster`, `base_controller` y `arm_controller` activos;
- cámara, `CameraInfo`, `/clock`, odometría y TF operativos;
- trayectoria estática, móvil o de cinco lóbulos, suave y determinista;
- detector HSV y estimación monocular sin usar pose privilegiada de Gazebo;
- controlador visual proporcional parametrizado;
- brazo Poppy CAD-first con visuales y colisiones separadas;
- seis joints auditados contra CAD, guía y URDF oficial;
- ensamblaje 1:1 validado visualmente en home y dos poses;
- FK independiente de la punta comparada contra TF;
- base compacta con ruedas, inercia, odometría y cámara coherentes;
- diagnóstico estricto y experimento A/B reproducibles;
- siete pruebas unitarias.

En la corrida A/B verificada, el seguimiento redujo el MAE de distancia objetivo
de **0.529 m** a **0.149 m** (mejora de **71.8%**), con 100% de detección y RMS
horizontal de 0.025. Los archivos fuente están
en [`results/verified/`](results/verified/).

## Arquitectura resumida

```text
Gazebo target_ball ──imagen + CameraInfo──> ball_detector
        │                                      │
        │ set_pose                             │ /ball/measurement
        ▼                                      ▼
target_trajectory                         visual_tracker
        │ /target/ground_truth                 │ TwistStamped
        │ (solo evaluación)                    ▼
        └──────────────────> metrics_logger  base_controller
                                  ▲              │
                                  └──── odom ────┘
```

El controlador solo recibe `/ball/measurement`, calculado desde la imagen. La
pose solicitada a Gazebo se publica en `/target/ground_truth` exclusivamente
para evaluar las métricas. La descripción completa está en
[`docs/architecture.md`](docs/architecture.md).

## Entorno validado

- Ubuntu 24.04 en WSL 2;
- ROS 2 Jazzy `ros-base` 0.11.0;
- Gazebo Sim 8.11.0;
- `ros_gz_sim` 1.0.22;
- `gz_ros2_control` 1.2.19;
- `diff_drive_controller` 4.40.1;
- Python 3.12.3.

La regresión automatizada funciona headless. La aceptación geométrica añade
capturas Gazebo y revisión visual; el overlay nativo de collision no pudo
capturarse bajo WSLg y se conserva una alternativa reproducible etiquetada.

## Instalación

Con ROS 2 Jazzy configurado según la documentación oficial:

```bash
sudo apt update
sudo apt install python3-rosdep ros-jazzy-ros-gz ros-jazzy-gz-ros2-control

cd ~/proyectos/gazebo-tutorial/gazebo-codex-mobile-manipulator
sudo rosdep init  # omitir si rosdep ya fue inicializado
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` lee las dependencias declaradas en
[`package.xml`](src/mobile_manipulator/package.xml); no se requiere un entorno
virtual Python.

## Compilación y pruebas

```bash
source /opt/ros/jazzy/setup.bash
cd ~/proyectos/gazebo-tutorial/gazebo-codex-mobile-manipulator
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

El resultado esperado es `7 tests, 0 errors, 0 failures`.

## Lanzamiento interactivo

Esfera estática y seguimiento activo:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_manipulator sim.launch.py
```

Esfera con trayectoria determinista:

```bash
ros2 launch mobile_manipulator sim.launch.py target_mode:=moving
```

Trayectoria rápida de cinco lóbulos, con una vuelta cada 30 s y su recorrido
marcado como una línea punteada sobre el piso:

```bash
ros2 launch mobile_manipulator sim.launch.py target_mode:=trefoil
```

En una ventana de un minuto la esfera completa exactamente dos vueltas. Los
valores predeterminados son cinco lóbulos, radio de 1.15 m y velocidad máxima
aproximada de 0.60 m/s. Se parametrizan mediante `trefoil_lobes`,
`trefoil_radius_m` y `trefoil_lap_period_s`. El seguidor admite hasta 0.90 m/s
y 2.5 rad/s; las ruedas tienen un límite explícito de 12 rad/s.

El estudiante debería observar:

- `/camera/image_raw` y `/camera/camera_info`;
- la detección en `/ball/measurement` y la imagen anotada `/ball/debug`;
- comandos `TwistStamped` en `/base_controller/cmd_vel`;
- pose cambiante en `/base_controller/odom`;
- TF `odom -> base_footprint`;
- aproximación y giro de la base para conservar `target_distance_m=1.2`.

Los argumentos principales pueden verse con:

```bash
ros2 launch mobile_manipulator sim.launch.py --show-args
```

Las ganancias, saturaciones, umbrales HSV, radio de esfera y distancia objetivo
son parámetros ROS 2 declarados por sus respectivos nodos.

## Diagnóstico reproducible

```bash
bash scripts/run_diagnostic.sh
```

Este comando recompila y falla si no puede demostrar una condición obligatoria.
Verifica controladores, joint states, carga de meshes sin errores, cámara,
intrínsecos, detector, `/clock`, `/base_controller/odom`, TF, desplazamiento
de la base y dos poses de los seis joints Poppy con tolerancia numérica. También
valida transforms oficiales, escala 1:1, landmarks de punta y acuerdo entre FK
independiente y TF. La evidencia queda en
[`results/verified/diagnostic/`](results/verified/diagnostic/).

## Experimento A/B

```bash
bash scripts/run_experiments.sh
```

- A: misma trayectoria, percepción y métricas, pero tracking desactivado.
- B: cambia un único factor: `visual_tracker` controla la base.

Cada condición dura 30 s y produce CSV, JSON, resumen legible y log. El
comparador falla si la trayectoria no se mueve en ambos ejes, la detección cae
por debajo de 90%, B no mueve el robot, el error estacionario supera 0.20 m o
B no reduce a la mitad el error de A. Resultados:
[`results/verified/experiments/comparison.json`](results/verified/experiments/comparison.json).

## Estructura

```text
src/mobile_manipulator/
  config/controllers.yaml        controladores ros2_control
  launch/sim.launch.py           composición y argumentos del sistema
  mobile_manipulator/            percepción, control, trayectoria y métricas
  meshes/poppy_ergo_jr/         CAD fuente, visual, collision y manifest
  test/test_algorithms.py        pruebas de geometría y funciones puras
  urdf/mobile_manipulator.urdf.xacro
  worlds/ball_arena.sdf
scripts/
  cad/                            CAD, validación mecánica y previews collision
  run_diagnostic.sh              aceptación de integración
  run_experiments.sh             comparación A/B
  validate_diagnostic.py
  compare_experiments.py
docs/
  cad_import_tutorial.md
  cad_import_troubleshooting.md
  cad_import_final_report.md
  mechanical_assembly_validation.md
  mechanical_assembly_closure_report.md
  architecture.md
  tutorial_handoff.md
  experiment_log.md
  final_report.md
results/verified/                evidencia textual, CSV e imágenes
```

## Limitaciones y extensiones

- La esfera es estática en términos físicos y se reposiciona mediante
  `SetEntityPose`; esto hace la trayectoria determinista y evita dinámica
  innecesaria para la lección.
- El detector presupone una esfera roja de radio conocido y una cámara pinhole.
- La odometría se usa como pose del robot en la evaluación; no se incorpora
  localización global ni ruido de sensores.
- El brazo y la pinza se validan por control articular, FK/TF, geometría y
  evidencia visual, pero no hay IK, MoveIt ni pick-and-place autónomo.
- Navegación, manipulación, múltiples objetos, calibración real y control
  avanzado quedan deliberadamente fuera de este corte.

Para estudiar el estado actual, empezar por
[`docs/mechanical_assembly_closure_report.md`](docs/mechanical_assembly_closure_report.md),
seguir con [`docs/mechanical_assembly_validation.md`](docs/mechanical_assembly_validation.md)
y [`docs/cad_import_tutorial.md`](docs/cad_import_tutorial.md).
Para el sistema perceptivo previo, continuar con
[`docs/tutorial_handoff.md`](docs/tutorial_handoff.md).
