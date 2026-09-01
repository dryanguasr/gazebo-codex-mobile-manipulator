# Build, launch y diagnóstico reproducible

## Preparar entorno

Desde la raíz del repositorio:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Compilar

```bash
colcon build --symlink-install
source install/setup.bash
```

El segundo `source` incorpora al shell el workspace compilado. Debe repetirse al abrir una terminal nueva.

## Pruebas unitarias

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Resultado validado: **6 tests, 0 errors, 0 failures**.

Cubren focal desde FOV, rango de esfera, profundidad→rango, trayectoria determinista, saturación y resumen de métricas. La integración completa se comprueba con scripts aparte.

## Launch

Esfera estática y tracking activo:

```bash
ros2 launch mobile_manipulator sim.launch.py
```

Esfera móvil:

```bash
ros2 launch mobile_manipulator sim.launch.py target_mode:=moving
```

Ver argumentos:

```bash
ros2 launch mobile_manipulator sim.launch.py --show-args
```

Argumentos centrales: `tracking_enabled`, `target_mode`, `metrics_enabled`, `metrics_output_dir`, `run_label`, `duration_s`, `target_distance_m`.

## Topics esperados

Durante una ejecución sana deben existir, entre otros:

`/clock`, `/joint_states`, `/camera/image_raw`, `/camera/camera_info`, `/ball/measurement`, `/base_controller/cmd_vel`, `/base_controller/odom`, `/tf`.

## Diagnóstico de aceptación

```bash
bash scripts/run_diagnostic.sh
```

El script recompila, expande Xacro, ejecuta `check_urdf`, lanza Gazebo sin tracking, espera servicios/topics obligatorios, comprueba tres controladores, `use_sim_time`, cámara, odometría/TF, movimiento de base, seis posiciones de brazo/pinza, captura imágenes y ejecuta `validate_diagnostic.py`.

Un fallo obligatorio produce exit code distinto de cero. Los `|| true` restantes están limitados a cleanup idempotente.

## Evidencia

`results/verified/diagnostic/`:

- `summary.json`;
- `controllers.txt`;
- `topics.txt`;
- `odom_before.txt`, `odom_after.txt`;
- `tf.txt`, `tf_after.txt`;
- `camera_info.txt`;
- `ball_measurement.txt`;
- `joint_states*.txt`;
- `captures/`.

## Resultado autoritativo

- odom: `/base_controller/odom`;
- TF: `odom -> base_footprint`;
- desplazamiento: **0.370 m**;
- focal: **554.383 px**;
- rango inicial: **1.633 m**;
- status: `passed`.

## Advertencia sobre overlays

La auditoría encontró que una versión anterior podía continuar tras un build fallido y luego reutilizar `install/` antiguo.

Regla: **si `colcon build` falla, no validar una ejecución posterior basada en ese overlay hasta corregir el build.**

No recomendar borrar `build/ install/ log/` como reflejo automático. Hacerlo solo si existe evidencia de artefactos obsoletos y explicar qué se elimina.
