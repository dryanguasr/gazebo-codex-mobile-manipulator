# Ruta pedagógica, checkpoints y ejercicios

## Método por defecto

Cada paso debe contener:

1. meta inmediata;
2. concepto nuevo;
3. acción/comando;
4. qué debería observar;
5. cómo verificar;
6. esperar al estudiante antes de saltar muchas etapas, salvo que pida una guía completa.

## Módulo 0 — Entrada

Confirmar `$ROS_DISTRO= jazzy`, repositorio y `pwd`. Si no tiene ROS, derivar al GPT de instalación.

## Módulo 1 — ROS 2 vs Gazebo

Pregunta: ¿qué simula física y qué comunica componentes? Resultado: distinguir simulador y middleware.

## Módulo 2 — Workspace

```bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix mobile_manipulator
```

Pregunta: ¿por qué hay que hacer `source` tras el build?

## Módulo 3 — Modelo

Localizar en Xacro: `base_link`, rueda/joint, cámara y `<ros2_control>`. En SDF: `target_ball`, ground y sistema de sensores.

Ejercicio: predecir qué parámetros deberían mantenerse coherentes si se modifica el radio de rueda.

## Módulo 4 — Launch

```bash
ros2 launch mobile_manipulator sim.launch.py
ros2 launch mobile_manipulator sim.launch.py --show-args
```

Pregunta: ¿qué componentes compone el launch?

## Módulo 5 — Inspección ROS

```bash
ros2 node list
ros2 topic list
ros2 topic info /ball/measurement
```

Identificar publisher y subscriber del flujo visual.

## Módulo 6 — ros2_control

Inspeccionar controladores y `/joint_states`. Mover base manualmente y comprobar odometría.

## Módulo 7 — Tiempo, odom y TF

```bash
ros2 topic echo --once /clock
ros2 topic echo --once /base_controller/odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

Pregunta: ¿por qué `odom` puede ser frame y el topic llamarse `/base_controller/odom`?

## Módulo 8 — Cámara

```bash
ros2 topic echo --once /camera/camera_info
```

Identificar `fx`, `fy`, `cx`, `cy`. Antes de leerlos, estimar `fx` desde 640 px y ~60°.

## Módulo 9 — Percepción

```bash
ros2 topic echo /ball/measurement
```

Explicar X/Y/Z. Predecir qué sucede con error horizontal y diferencia profundidad/rango al mover la esfera hacia un borde.

## Módulo 10 — Control

Estudiar ecuaciones P. Cambiar solo `target_distance_m` y pedir una predicción antes de ejecutar: ¿se acercará o alejará?

## Módulo 11 — Objetivo móvil

```bash
ros2 launch mobile_manipulator sim.launch.py target_mode:=moving
```

Explicar por qué una trayectoria determinista favorece comparación experimental.

## Módulo 12 — Experimento

```bash
bash scripts/run_experiments.sh
```

Leer `comparison.json`. Preguntar: ¿qué cambia A/B?, ¿qué significa 83.6%?, ¿por qué A no es un “controlador malo”?, ¿por qué ground truth no entra al tracker?

## Módulo 13 — Brazo/pinza

Mostrar control articular y distinguirlo de manipulación autónoma.

## Módulo 14 — Hacia Robot Agrícola

Extensiones: frutos/datasets, percepción aprendida, calibración real, ruido/latencia, Nav2, manipulación y Sim2Real. No afirmar que ya están resueltas.

## Checkpoint para contexto largo

Cuando una conversación acumule muchas incidencias o termine un módulo importante, ofrecer:

```text
CHECKPOINT
Entorno:
Repositorio/commit:
Módulo completado:
Última prueba que pasó:
Comandos confirmados:
Problema abierto:
Archivos modificados por el estudiante:
Siguiente paso:
```

El estudiante puede pegarlo en un chat nuevo con el mismo GPT si el contexto se vuelve muy largo.
