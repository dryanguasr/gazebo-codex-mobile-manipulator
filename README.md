# Seguimiento visual con ROS 2 Jazzy y Gazebo

Ejemplo reproducible para estudiantes de ingeniería mecatrónica: un robot móvil
4WD observa y sigue una esfera con visión monocular y, en un world separado,
ejecuta pick-and-place determinista con su brazo Poppy Ergo Jr. El sistema
conserva percepción y tracking B3 medibles, geometría CAD oficial 1:1 y añade un
nivel A1 asistido por simulador con contacto bilateral condicionado.

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
- pick-and-place A1 con máquina de estados, gate y evaluación independientes;
- diez corridas nominales y siete negativas verificadas;
- 60 pruebas automatizadas.

La campaña A1 obtuvo **10/10 grasp y 10/10 placement**, con lift medio de
55.19 mm y error de depósito medio de 2.30 mm. Es un MVP
`attach_conditioned`, explícitamente asistido por simulador; A2 físico sin
attach no levantó el objeto en aquella revisión y se conserva como FAIL histórico.

**Corrección del gripper:** el contacto físico actualizado pasó dos repeticiones
sin attach, con elevación de 62,8–62,9 mm y error de depósito de 2,1–3,6 mm.
Se corrigieron colisiones fuera del CAD, precarga, representación del contacto
y detección de pérdida. [Diagnóstico y reproducción](docs/gripper_fix_report.md).

En la campaña A/B final, el tracking redujo el MAE de distancia objetivo de
**0.632 m** a **0.055 m** (mejora de **91.37%**), con 100% de referencias
geométricas válidas. La referencia usa la pose aceptada del objetivo y TF
odométrico hasta `camera_link`; no es ground truth independiente del
simulador. Evidencia:
[`results/verified/tracking_metric_reference_v3_pick_A1_20260908/`](results/verified/tracking_metric_reference_v3_pick_A1_20260908/).

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

El resultado esperado es `60 tests, 0 errors, 0 failures`.

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

## Pick-and-place nivel A

Para observar el agarre físico corregido, sin unión asistida:

~~~bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_manipulator pick_and_place.launch.py \
  attach_enabled:=false grasp_mode:=physical_contact \
  output_dir:="$PWD/results/manual/gripper_fisico" \
  run_id:=gripper_fisico seed:=402 capture_evidence:=true evidence_fps:=60
~~~

Una corrida A1 asistida con captura:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_manipulator pick_and_place.launch.py \
  output_dir:="$PWD/results/manual/pick_a1" run_id:=manual \
  seed:=901 source_sha:="$(git rev-parse HEAD)" capture_evidence:=true
```

La base se congela y el brazo recorre
`IDLE → FREEZE_BASE → OPEN → PREGRASP → APPROACH → CLOSE → VERIFY_GRASP →
LIFT → HOLD → TRANSFER → LOWER → RELEASE → RETREAT → DONE`.
Attach solo se autoriza con contactos frescos en ambos dedos, geometría, cierre,
estado, base detenida y TF válidos. La pose real de Gazebo se usa en ese gate y
en evaluación, no para elegir trayectorias.

Para reproducir la campaña y las siete negativas, consulte
[`docs/pick_and_place_tutorial.md`](docs/pick_and_place_tutorial.md). Resultados:
[`A1`](results/verified/pick_A1_20260908/),
[`negativas`](results/verified/pick_A1_negative_20260908/) y
[`A2`](results/verified/pick_A2_20260908/).

Videos representativos del ciclo completo y sus operaciones, grabados desde
Gazebo a 60 fps: [captures/ensamblaje_inspeccion/](captures/ensamblaje_inspeccion/).
La [corrección visual del ensamblaje](docs/assembly_render_fix.md) resuelve
instancias de servomotor superpuestas y añade colores para inspección.
Los [videos A1 anteriores](captures/pick_and_place/) son históricos y no
demuestran retención física correcta.

## Diagnóstico reproducible

```bash
bash scripts/run_diagnostic.sh
```

Este comando recompila y falla si no puede demostrar una condición obligatoria.
Verifica controladores, joint states, carga de meshes sin errores, cámara,
intrínsecos, detector, `/clock`, `/base_controller/odom`, TF, desplazamiento
de la base y dos poses de los seis joints Poppy con tolerancia numérica. También
valida transforms oficiales, escala 1:1, landmarks de punta, acuerdo entre FK
independiente y TF y la extrínseca efectiva de `camera_link` derivada del
URDF. La evidencia vigente queda en
[`results/verified/diagnostic_pick_A1_20260908_retry1/`](results/verified/diagnostic_pick_A1_20260908_retry1/).

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
[`results/verified/tracking_metric_reference_v3_pick_A1_20260908/comparison.json`](results/verified/tracking_metric_reference_v3_pick_A1_20260908/comparison.json).

La campaña histórica de `results/verified/experiments/` se conserva sin
reescribir, pero su distancia física está marcada como afectada en
[`INCIDENT.md`](results/verified/experiments/INCIDENT.md).

## Estructura

```text
src/mobile_manipulator/
  config/controllers.yaml        controladores ros2_control
  launch/sim.launch.py           seguimiento B3
  launch/pick_and_place*.py       manipulación A1/A2
  config/pick_place_a1.yaml       poses y gates congelados
  worlds/pick_and_place.sdf       escena de manipulación separada
  mobile_manipulator/            percepción, control, supervisor, gate y métricas
  meshes/poppy_ergo_jr/         CAD fuente, visual, collision y manifest
  test/test_algorithms.py        pruebas de geometría y funciones puras
  urdf/mobile_manipulator.urdf.xacro
  worlds/ball_arena.sdf
scripts/
  cad/                            CAD, validación mecánica y previews collision
  run_diagnostic.sh              aceptación de integración
  run_experiments.sh             comparación A/B
  validate_diagnostic.py
tools/
  run_pick_place_campaign.py      diez intentos nominales
  run_pick_place_negative_tests.py siete negativas
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
  pick_and_place_tutorial.md
  pick_and_place_troubleshooting.md
  pick_and_place_final_report.md
results/verified/                evidencia textual, CSV, imágenes y video
```

## Limitaciones y extensiones

- La esfera es estática en términos físicos y se reposiciona mediante
  `SetEntityPose`; esto hace la trayectoria determinista y evita dinámica
  innecesaria para la lección.
- El detector presupone una esfera roja de radio conocido y una cámara pinhole.
- La odometría se usa como pose del robot en la evaluación; no se incorpora
  localización global ni ruido de sensores.
- A1 usa attach temporal después de un gate bilateral; valida la secuencia y
  seguridad, no la fuerza/fricción de un agarre real.
- El A2 histórico falló; el contacto físico corregido pasó dos repeticiones de
  esta escena. No equivale a validación en hardware ni agarre general.
- No hay IK, MoveIt, percepción del cilindro ni planificación general.
- Navegación, múltiples objetos, calibración real y Sim2Real quedan fuera.

Para el hito actual, empezar por
[`docs/pick_and_place_final_report.md`](docs/pick_and_place_final_report.md),
continuar con
[`docs/pick_and_place_tutorial.md`](docs/pick_and_place_tutorial.md) y
[`docs/pick_and_place_troubleshooting.md`](docs/pick_and_place_troubleshooting.md).
La validación mecánica y el tutorial CAD se conservan como baseline.
