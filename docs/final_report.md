# Informe final del hito de seguimiento visual

> **Documento histórico:** este informe cierra el hito perceptivo anterior. El estado mecánico y las métricas actuales están en [mechanical_assembly_closure_report.md](mechanical_assembly_closure_report.md).


## Resultado

El ejemplo base ROS 2 Jazzy + Gazebo quedó cerrado como sistema reproducible,
medible y pedagógicamente explicable. La cámara detecta una esfera roja, estima
su rango con intrínsecos coherentes, un controlador visual mueve la base y una
trayectoria determinista permite comparar resultados.

El alcance no incluye pick-and-place. El brazo y la pinza siguen operativos y
fueron validados mediante control articular.

## Auditoría de aceptación

| # | Criterio | Estado | Evidencia autoritativa |
|---:|---|---|---|
| 1 | Workspace compila | PASS | `diagnostic/build.log` |
| 2 | Robot aparece en Gazebo | PASS | `launch.log`: entity creation |
| 3 | Controladores activos | PASS | `controllers.txt` |
| 4 | Joint states | PASS | `joint_states*.txt` |
| 5 | Cámara Gazebo→ROS | PASS | `camera_info.txt` y capturas |
| 6 | Odometría y topic identificado | PASS | `/base_controller/odom`, `summary.json` |
| 7 | TF odom→base_footprint | PASS | `tf.txt` y `tf_after.txt` |
| 8 | Trayectoria determinista | PASS | spans X/Y en A y B |
| 9 | Detector desde imagen | PASS | `ball_measurement.txt` y CSV |
| 10 | Geometría coherente | PASS | `fx=554.383 px` y MAE de rango |
| 11 | Tracker usa solo percepción | PASS | grafo y código |
| 12 | Seguimiento conserva distancia | PASS | error estacionario 0.083 m |
| 13 | Evidencia cuantitativa | PASS | CSV, resúmenes y comparación |
| 14 | Flujo repetible | PASS | scripts con exit code y validadores |
| 15 | README/docs autosuficientes | PASS | README, arquitectura y handoff |

## Hallazgos clave de auditoría

La configuración original de odometría no estaba rota. El topic correcto del
controlador Jazzy es `/base_controller/odom` y los frames ya estaban configurados
como `odom` y `base_footprint`. Los problemas reales eran:

- el diagnóstico consultaba `/odom`;
- faltaba puentear `/clock`;
- `joint_state_broadcaster` no se activaba explícitamente;
- un build fallido podía quedar oculto y reutilizar `install/` anterior.

Las correcciones y la secuencia de descubrimiento están preservadas en
`docs/experiment_log.md`.

## Resultado del diagnóstico

`scripts/run_diagnostic.sh` verificó:

- desplazamiento de base: **0.666 m**;
- topic: **`/base_controller/odom`**;
- TF final consistente con odometría dentro de **0.02 m**;
- focal CameraInfo: **554.383 px**;
- rango inicial estimado: **1.630 m**;
- tres controladores activos;
- seis posiciones de brazo/pinza alcanzadas dentro de **±0.03**;
- imagen cruda y anotada guardadas;
- cierre sin procesos Gazebo huérfanos.

Resumen: `results/verified/diagnostic/summary.json`.

## Resultado del seguimiento

Definición del A/B:

- A: esfera móvil, detector y métricas activos; tracking desactivado.
- B: mismo sistema, misma ruta y duración; tracking activado.

| Métrica | A | B |
|---|---:|---:|
| Muestras tras warmup | 667 | 698 |
| Tasa de detección | 100.0% | 100.0% |
| MAE de estimación de rango | 0.097 m | 0.016 m |
| RMSE de estimación de rango | 0.111 m | 0.018 m |
| RMS horizontal | 0.528 | 0.034 |
| MAE de distancia objetivo | 0.535 m | 0.088 m |
| MAE estacionario | 0.568 m | 0.083 m |
| Actividad de comando | 0% | 100% |
| Desplazamiento del robot | ~0 m | 0.368 m |
| Span objetivo X/Y | 0.900/1.011 m | 0.900/1.011 m |

B redujo el MAE de distancia objetivo en **83.6%** frente a A. El comparador
aprobó todos los umbrales de detección, geometría, trayectoria, actividad,
movimiento, error horizontal y error estacionario.

## Validez de la estimación

La cámara simulada configura 640×480 px y FOV horizontal 1.047 rad. El valor
teórico `fx = width / (2 tan(FOV/2))` coincide con el `CameraInfo` observado.

El detector calcula profundidad desde el radio aparente y después la convierte
a rango 3D usando `fx, fy, cx, cy` y el rayo del centro detectado. El ground
truth combina la pose aceptada del objetivo, odometría y extrínseco de cámara,
y llega únicamente al logger.

El MAE en A es mayor porque la esfera recorre zonas oblicuas de la imagen, pero
permanece bajo 0.15 m. B mantiene el objetivo centrado y logra 0.016 m.

## Reproducibilidad

```bash
source /opt/ros/jazzy/setup.bash
cd gazebo-codex-mobile-manipulator
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
bash scripts/run_diagnostic.sh
bash scripts/run_experiments.sh
```

Cada script conserva evidencia bajo `results/verified/`. Las pruebas obligatorias
no usan `|| true`; la tolerancia solo aparece en cleanup idempotente.

## Limitaciones conocidas

- Segmentación específica para rojo con iluminación controlada.
- Esfera de radio conocido y cámara ideal sin distorsión.
- Ground truth del objetivo es la última pose aceptada por `SetEntityPose`.
- La pose del robot para evaluación procede de odometría simulada.
- La esfera no tiene dinámica de rodadura: se reposiciona suavemente.
- Sin manipulación autónoma, navegación, SLAM ni percepción aprendida.

Estas limitaciones son decisiones docentes, no resultados ocultos.

## Handoff para generación del GPT tutorial

### Commit SHA final validado

`251a1f6ca761c85449af9aeaf162c1fa8aa78e47`

El valor identifica el commit de código, documentación y evidencia sobre el que
se ejecutó la validación final. Si existe un commit posterior que solo sustituye
este marcador por el SHA, ambos contienen la misma implementación validada.

### Árbol resumido

```text
README.md
docs/
  architecture.md
  experiment_log.md
  final_report.md
  tutorial_handoff.md
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

### Comandos exactos

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
ros2 launch mobile_manipulator sim.launch.py target_mode:=moving
bash scripts/run_diagnostic.sh
bash scripts/run_experiments.sh
```

### Nodes, topics, frames y controladores

- Nodes: `robot_state_publisher`, `parameter_bridge`, `controller_manager`,
  `ball_detector`, `visual_tracker`, `target_trajectory` y `metrics_logger`.
- Topics principales: `/clock`, `/camera/image_raw`,
  `/camera/camera_info`, `/ball/measurement`, `/ball/debug`,
  `/base_controller/cmd_vel`, `/base_controller/odom`, `/joint_states`,
  `/target/ground_truth`, `/tf` y `/tf_static`.
- Frames relevantes: `odom`, `base_footprint`, `base_link` y `camera_link`.
- Controladores: `joint_state_broadcaster`, `base_controller`,
  `arm_controller`.

### Resultados cuantitativos

- Detección B: 100%.
- MAE rango B: 0.016 m.
- RMS horizontal B: 0.034.
- MAE distancia objetivo B: 0.088 m.
- Error estacionario B: 0.083 m.
- Mejora B/A: 83.6%.
- Diagnóstico de base: 0.666 m de movimiento con odom/TF coherentes.

### Evidencias

- `results/verified/diagnostic/summary.json`.
- `results/verified/diagnostic/controllers.txt`.
- `results/verified/diagnostic/tf_after.txt`.
- `results/verified/experiments/comparison.json`.
- `results/verified/experiments/A.csv` y `B.csv`.
- Capturas en `results/verified/diagnostic/captures/`.

### Problemas conocidos y extensiones

Los supuestos de color/radio/cámara ideal y odometría simulada deben explicarse
antes de generalizar a hardware. Las extensiones futuras recomendadas son
calibración real, ruido/oclusiones, estimación filtrada, Nav2 y finalmente
manipulación. MoveIt, IK y pick-and-place quedan deliberadamente fuera.

### Archivos que debería consumir ChatGPT

Usar primero `README.md`, `docs/architecture.md`,
`docs/tutorial_handoff.md` y este informe. Después leer launch, Xacro, mundo,
controladores, los cuatro nodos principales, ambos scripts y los dos JSON de
resumen. `experiment_log.md` aporta el razonamiento de depuración; CSV e imágenes
sirven como evidencia, no como sustituto del código.
