# Transferencia para generar un tutorial

> **Documento histórico:** este handoff describe el hito perceptivo previo. Para el estado actual, empezar por [mechanical_assembly_closure_report.md](mechanical_assembly_closure_report.md) y después [cad_import_tutorial.md](cad_import_tutorial.md).


## Uso previsto

Este documento permite que otra instancia de ChatGPT audite el repositorio
desde GitHub y lo convierta en material docente de ROS 2 Jazzy + Gazebo sin
información de la sesión original. Toda afirmación de funcionamiento enlaza con
código o evidencia versionada.

## Qué funciona realmente

1. El workspace compila con `colcon build --symlink-install`.
2. Xacro expande y `check_urdf` valida el robot.
3. El robot aparece en `ball_arena` con base 4WD, brazo, pinza y cámara.
4. Los tres controladores quedan activos y `/joint_states` publica 10 joints.
5. Imagen y `CameraInfo` cruzan de Gazebo a ROS 2.
6. `/clock` gobierna nodos y controladores con tiempo simulado.
7. `/base_controller/odom` y TF `odom -> base_footprint` funcionan.
8. La esfera acepta una condición estática y una trayectoria 2D determinista.
9. El detector obtiene centro y rango exclusivamente desde cámara.
10. El tracker corrige ángulo y distancia con un control proporcional.
11. La condición B sigue la esfera y supera los umbrales cuantitativos.
12. El brazo y la pinza alcanzan seis consignas; no hacen pick-and-place.

La prueba primaria de integración es
`results/verified/diagnostic/summary.json`. La prueba primaria de seguimiento es
`results/verified/experiments/comparison.json`.

## Mapa de implementación

| Función didáctica | Implementación | Evidencia |
|---|---|---|
| Modelo y frames | `urdf/mobile_manipulator.urdf.xacro` | `diagnostic/robot.urdf`, `check_urdf.txt` |
| Mundo y objetivo | `worlds/ball_arena.sdf` | logs de A/B |
| Lanzamiento y bridges | `launch/sim.launch.py` | `topics.txt`, `launch.log` |
| ros2_control | `config/controllers.yaml` | `controllers.txt` |
| Intrínsecos y HSV | `ball_detector.py` | `camera_info.txt`, `ball_measurement.txt` |
| Control visual | `visual_tracker.py` | `B.csv` |
| Trayectoria | `target_trajectory.py` | spans X/Y en resúmenes A/B |
| Métricas | `metrics_logger.py` | CSV y JSON A/B |
| Aceptación | `run_diagnostic.sh`, `validate_diagnostic.py` | `summary.json` |
| Comparación | `run_experiments.sh`, `compare_experiments.py` | `comparison.json` |
| Funciones puras | `test/test_algorithms.py` | `colcon test`: 6/6 |

Las rutas relativas parten de `src/mobile_manipulator/` salvo las de `scripts/`
y `results/verified/`.

## Comandos reproducibles

Instalación desde un clon:

```bash
source /opt/ros/jazzy/setup.bash
cd gazebo-codex-mobile-manipulator
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build y unit tests:

```bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Lección interactiva:

```bash
ros2 launch mobile_manipulator sim.launch.py target_mode:=moving
```

Diagnóstico de aceptación:

```bash
bash scripts/run_diagnostic.sh
```

Experimento medido:

```bash
bash scripts/run_experiments.sh
```

Los scripts son no interactivos, funcionan headless y terminan los procesos
Gazebo asociados al mundo. Un fallo obligatorio produce exit code distinto de
cero. Los `|| true` restantes están confinados a cleanup de procesos que pueden
haber terminado previamente.

## Secuencia pedagógica sugerida

1. **Modelo y simulación:** identificar links, joints, sensor y plugins.
2. **Bridge y reloj:** comparar Gazebo Transport con topics ROS 2 y explicar
   `use_sim_time`.
3. **Control:** activar broadcasters/controladores y observar interfaces.
4. **Percepción:** segmentar HSV y visualizar `/ball/debug`.
5. **Geometría:** derivar `fx` del FOV, leer `CameraInfo` y obtener rango.
6. **Control visual:** mapear errores de imagen/rango a `TwistStamped`.
7. **Objetivo determinista:** separar estímulo experimental de respuesta.
8. **Evaluación:** distinguir señal de control de ground truth.
9. **A/B:** cambiar un único factor y discutir MAE, RMS y settling.

## Conceptos que ilustra cada etapa

- Descripción URDF/Xacro y árbol TF.
- Sensores Gazebo y `ros_gz_bridge`.
- Sim time, timestamps y sincronización conceptual.
- `ros2_control`, controller manager e interfaces de joints.
- Topics tipados, QoS de sensores y mensajes stamped.
- Cámara pinhole, intrínsecos y estimación monocular.
- Control proporcional, saturación, deadband y watchdog.
- Parámetros ROS 2 frente a constantes mágicas.
- Ground truth reservado para evaluación.
- Experimentos reproducibles y criterios de aceptación.

## Decisiones de diseño

- **Esfera estática reposicionada:** `SetEntityPose` genera una ruta exacta sin
  introducir rodadura, rebotes o un plugin C++ innecesario.
- **Radio conocido:** permite enseñar rango monocular antes de abordar
  calibración/estimación de escala.
- **`Vector3Stamped`:** mantiene un ejemplo pequeño. X/Y son errores normalizados,
  Z es rango; NaN representa medición inválida.
- **Control P:** su relación causa-efecto es visible y ajustable.
- **Odometría como pose del robot en métricas:** suficiente en simulación corta;
  no pretende sustituir localización.
- **Headless:** elimina la GUI como dependencia de aceptación.

## Problemas encontrados y solución

| Problema real | Diagnóstico | Solución |
|---|---|---|
| Se consultaba `/odom` | Jazzy publicaba `/base_controller/odom` | Diagnóstico y docs usan el topic real |
| TF ausente/timestamps cero | Gazebo tenía `/clock`, ROS no | Bridge explícito de `/clock` y `use_sim_time=true` |
| Joint states inconsistentes | Broadcaster no se activaba en limpio | Spawner explícito de `joint_state_broadcaster` |
| Build dependía de caché | `glob('launch/*')` capturaba `__pycache__` | Globs limitados por extensión |
| `fx=320` arbitrario | FOV 60° implica ~554 px | `CameraInfo.K` y fallback derivado |
| Error con bola descentrada | Profundidad se llamaba rango | Conversión profundidad→rango con rayo pinhole |
| Diagnóstico ocultaba fallos | Build y topics críticos tenían tolerancia | Script estricto y validador numérico |
| Servidores Gazebo huérfanos | Wrapper no terminaba el proceso hijo | Handler `OnShutdown` con patrón del mundo |
| Espera infinita aparente | `grep -q` provocaba BrokenPipe con `pipefail` | Consumir toda la salida antes de evaluar |

## Resultados medidos

Condiciones de 30 s, warmup de 5 s, misma trayectoria:

| Métrica | A: sin tracking | B: con tracking |
|---|---:|---:|
| Detección válida | 100.0% | 100.0% |
| MAE estimación de rango | 0.097 m | 0.016 m |
| RMS horizontal | 0.528 | 0.034 |
| MAE distancia objetivo | 0.535 m | 0.088 m |
| Error estacionario | 0.568 m | 0.083 m |
| Desplazamiento robot | ~0 m | 0.368 m |
| Comando activo | 0% | 100% |

La mejora B frente a A en MAE de distancia objetivo fue 83.6%. A no es un
benchmark de control alternativo: es la línea base causal con tracking
desactivado. Su error de estimación mayor se debe a que la esfera visita regiones
más oblicuas de la imagen; aun así queda por debajo del umbral de 0.15 m.

## Simplificaciones docentes

- Cámara ideal sin distorsión ni ruido.
- Color, iluminación y radio de esfera conocidos.
- Odometría ideal de encoder simulado.
- Teletransporte suave del objetivo en vez de dinámica de una bola rodante.
- Un solo objetivo, sin oclusiones deliberadas.
- Ganancias fijas, sin planificador ni estimador de estado.

Estas elecciones son explícitas y permiten atribuir los errores a percepción y
control antes de introducir realismo adicional.

## Fuera de alcance deliberado

- Pick-and-place, IK, MoveIt y planificación de manipulación.
- SLAM, Nav2 o localización global.
- Detección aprendida o múltiples clases.
- Calibración de cámara física y distorsión.
- Robustez a oclusión, ruido y latencia real.
- GUI, capturas externas y PDF.

## Archivos recomendados para generar conocimiento

Consumir, en este orden:

1. `README.md` para el flujo de usuario.
2. `docs/architecture.md` para conceptos y límites.
3. `docs/final_report.md` para la auditoría de aceptación.
4. Los cuatro nodos principales de percepción/control/trayectoria/métricas.
5. `sim.launch.py`, `controllers.yaml`, Xacro y mundo.
6. Scripts de diagnóstico/experimento y validadores.
7. `comparison.json` y `diagnostic/summary.json` para cifras.
8. `docs/experiment_log.md` para la historia de depuración.

No inferir funcionalidad de los vídeos o imágenes por sí solos: las fuentes
autoritativas son el código, los JSON y los scripts que los reproducen.
