# Troubleshooting basado en problemas reales

## Método general

Ante un fallo: 1) identificar capa; 2) pedir error literal; 3) inspeccionar; 4) formular hipótesis; 5) hacer una prueba discriminante; 6) corregir poco; 7) verificar; 8) volver al tutorial.

Capas: entorno, build, overlay, Gazebo, bridge, `ros2_control`, TF/tiempo, cámara, percepción, control, experimento.

## `mobile_manipulator` no encontrado

```bash
pwd
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix mobile_manipulator
```

Causa frecuente: falta `source install/setup.bash`. No reinstalar ROS como primer paso.

## Build falla pero el launch parece funcionar

Peligro de overlay anterior. Si `colcon build` falla, no considerar válida la ejecución hasta resolverlo.

## `__pycache__` durante instalación

Problema histórico de un glob demasiado amplio en `setup.py`. La versión validada limita extensiones. Si aparece, comprobar primero commit/versión.

## No aparece `/odom`

```bash
ros2 topic list | grep odom
```

Topic validado: `/base_controller/odom`.

## TF no funciona

```bash
ros2 topic info /clock
ros2 param get /base_controller use_sim_time
ros2 topic info /base_controller/odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

Problema histórico real: faltaba bridge de `/clock`.

## `/joint_states` sin publisher

Comprobar controladores. Históricamente faltaba activar explícitamente `joint_state_broadcaster`; la versión actual lo spawnea.

## Cámara no publica

```bash
ros2 topic info /camera/image_raw
ros2 topic info /camera/camera_info
```

Revisar sensor Xacro, bridge y sistema de sensores del mundo. No tocar HSV si no existe imagen.

## `CameraInfo` no llega

```bash
ros2 topic echo --once /camera/camera_info
```

`fx` esperado ≈554.38 px. Existe fallback por FOV, pero la ruta validada usa `CameraInfo`.

## Distancia incorrecta

Separar detección, radio aparente, intrínsecos y diferencia profundidad/rango. Problemas históricos: `fx=320` arbitrario y comparación profundidad óptica vs rango Euclídeo.

## Detecta pero no se mueve

```bash
ros2 topic echo /ball/measurement
ros2 topic echo /base_controller/cmd_vel
```

Comprobar `tracking_enabled`, rango finito, deadbands, `base_controller` activo y timeout.

## Gira en sentido contrario

No cambiar signos a ojo. Revisar signo de `measurement.vector.x`, convención de yaw y `-angular_gain * horizontal_error`; probar con esfera claramente a un lado.

## A/B falla

Leer `A_summary.json`, `B_summary.json`, `comparison.json` y logs. Identificar el umbral concreto. No bajar umbrales antes de entender el comportamiento.

## Gazebo queda vivo

```bash
pgrep -af "gz sim"
```

La versión actual hace cleanup específico por mundo. No usar `killall` indiscriminado si existen otras simulaciones.

## `BrokenPipeError`

Problema histórico de `grep -q` + `pipefail`. El script actual consume toda la salida. Si reaparece, comprobar versión del script.

## Cuándo hacer build limpio

Solo con evidencia de caché/artefactos obsoletos. Explicar antes:

```bash
rm -rf build install log
colcon build --symlink-install
```

No convertirlo en ritual automático.
