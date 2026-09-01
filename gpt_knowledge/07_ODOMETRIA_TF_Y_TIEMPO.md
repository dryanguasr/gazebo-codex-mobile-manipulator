# Reloj de simulación, odometría y TF

## Hallazgo clave

La configuración de odometría no estaba rota. La versión histórica consultaba `/odom`, pero el topic real del controlador en este ejemplo es:

`/base_controller/odom`

Los frames son `odom` y `base_footprint`. Frames y topics son conceptos distintos.

## `/clock`

Gazebo posee su reloj. `sim.launch.py` lo puentea a ROS 2 y los nodos usan `use_sim_time=true`.

Sin este bridge, la auditoría observó timestamps cero/incoherentes y TF no consumible de forma fiable.

Verificar:

```bash
ros2 topic echo --once /clock
ros2 param get /controller_manager use_sim_time
ros2 param get /base_controller use_sim_time
ros2 param get /robot_state_publisher use_sim_time
```

Estado validado de los parámetros: `True`.

## Odometría

```bash
ros2 topic echo --once /base_controller/odom
```

Tipo: `nav_msgs/msg/Odometry`.

Contiene pose/velocidad y usa `odom` como frame padre y `base_footprint` como hijo.

## TF

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

El diagnóstico exige que la traslación TF final coincida con la pose de odometría dentro de 0.02 m.

## Resultado autoritativo

Según `results/verified/diagnostic/summary.json`:

- X inicial ≈ 0;
- X final ≈ 0.370 m;
- TF X final = 0.370 m;
- desplazamiento = **0.370 m**.

Una cifra narrativa de 0.666 m quedó desactualizada en dos documentos del repositorio; no usarla.

## Frames

- `base_footprint`: referencia planar de la base.
- `base_link`: cuerpo físico, unido por joint fijo.
- `camera_link`: cámara frontal.
- `odom`: marco local incremental.

## Odometría no es localización global

En un robot real puede acumular error por deslizamiento, radios/separación mal calibrados, cuantización y deformaciones. Este tutorial no implementa SLAM ni localización global.

## Pregunta pedagógica

**Si el frame se llama `odom`, ¿por qué el topic puede llamarse `/base_controller/odom`?**

Porque un frame nombra un sistema de coordenadas y un topic nombra un canal de comunicación; no hay obligación de que compartan nombre.
