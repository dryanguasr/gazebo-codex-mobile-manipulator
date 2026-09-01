# Bitácora del experimento

## 2026-08-21 — Prototipo inicial

- Se comprobó la disponibilidad de ROS 2 Jazzy, Gazebo Sim 8, `ros_gz`,
  `gz_ros2_control`, Xacro y OpenCV.
- Se creó el modelo con base 4WD, brazo de 4 GDL, pinza, cámara y esfera.
- Xacro y `check_urdf` finalizaron sin errores estructurales.
- Se incorporaron detector HSV, controlador visual básico y comandos manuales.

## 2026-08-22 — Evidencia experimental inicial

Se obtuvo spawn, cámara, vídeo y activación aparente de controladores. También se
registraron como pendientes odometría, TF, seguimiento y métricas. Estas
evidencias motivaron el hito actual, pero no se toman como aceptación final:
posteriormente se descubrió que el script podía continuar tras un build fallido
y cargar un overlay anterior.

La limitación de GUI/PDF anotada entonces fue circunstancial y no es criterio de
aceptación del sistema.

## 2026-09-01 — Auditoría desde estado limpio

La auditoría se realizó antes de cambiar odometría:

1. `colcon build` falló porque `setup.py` copiaba `launch/__pycache__` mediante
   un glob demasiado amplio.
2. El diagnóstico ignoraba el fallo y hacía `source install/setup.bash`, por lo
   que mezclaba código fuente nuevo con una instalación anterior.
3. El topic real del `DiffDriveController` apareció como
   `/base_controller/odom`. La consulta histórica a `/odom` era incorrecta; la
   configuración del controlador no necesitaba remap.
4. Gazebo Transport sí publicaba `/clock`, pero no existía bridge hacia ROS 2.
   El controller manager avisaba que no recibía reloj; odom llevaba stamp cero y
   TF no era consumible.
5. `enable_odom_tf=true`, `odom_frame_id=odom` y
   `base_frame_id=base_footprint` ya eran correctos.
6. En una ejecución realmente limpia, `joint_state_broadcaster` no estaba
   activo. El topic se descubría con cero publicadores.

## Correcciones aplicadas

- Globs de instalación limitados a extensiones de archivo.
- Bridge Gazebo→ROS para `/clock`, imagen y `CameraInfo`.
- `use_sim_time=true` en los nodos de simulación.
- Spawner explícito para `joint_state_broadcaster`, base y brazo.
- Diagnóstico estricto: build y señales críticas ya no usan tolerancia.
- Validación numérica de desplazamiento, TF, intrínsecos y posiciones del brazo.
- Limpieza de procesos Gazebo ligada a `OnShutdown`.
- Detector refactorizado y parametrizado.
- `fx` tomado de `CameraInfo`; fallback derivado de resolución/FOV.
- Conversión de profundidad óptica a rango 3D para objetivos descentrados.
- Trayectoria estática/móvil mediante `SetEntityPose`.
- Tracker proporcional parametrizado, con saturaciones, deadbands y watchdog.
- Logger CSV/JSON y comparación A/B con criterios ejecutables.

## Incidencias durante la automatización

- `grep -q` combinado con `pipefail` cerraba la tubería al encontrar un topic.
  `ros2` terminaba con `BrokenPipeError` y la condición se interpretaba como
  falsa. Se cambió a un grep que consume toda la salida.
- El wrapper de `gz sim` dejaba un proceso hijo tras el cierre. El launch mata
  únicamente el proceso cuyo comando incluye el mundo `ball_arena.sdf`.
- La primera métrica A reveló que la profundidad óptica se comparaba contra
  rango Euclídeo. Se añadió el factor del rayo pinhole usando `fx, fy, cx, cy`.

## Validación final

### Diagnóstico

- Build y URDF: PASS.
- Tres controladores activos: PASS.
- 10 joint states y seis consignas brazo/pinza alcanzadas: PASS.
- Cámara e intrínsecos: `fx=554.383 px`.
- Odometría real: `/base_controller/odom`.
- Movimiento ordenado: 0.666 m.
- TF `odom -> base_footprint`: disponible y consistente con odometría.
- Procesos huérfanos al terminar: ninguno.

### Experimento A/B, 30 s por condición

| Métrica | A | B |
|---|---:|---:|
| Frames válidos tras warmup | 667/667 | 698/698 |
| MAE de rango | 0.097 m | 0.016 m |
| RMS horizontal | 0.528 | 0.034 |
| MAE a distancia objetivo | 0.535 m | 0.088 m |
| Error estacionario | 0.568 m | 0.083 m |
| Desplazamiento | ~0 m | 0.368 m |

Mejora del MAE de distancia objetivo: 83.6%. El comparador terminó con PASS.

## Decisiones de alcance

- No se añadió MoveIt, IK ni pick-and-place.
- No se convirtió el objetivo en un sistema físico complejo.
- No se dedicó trabajo a GUI o PDF.
- Las imágenes existentes se conservan; los criterios se basan en topics, CSV,
  JSON y validadores reproducibles.
