# Arquitectura técnica

## Propósito y frontera del sistema

El sistema demuestra un lazo de seguimiento visual monocular en simulación:
Gazebo genera imagen y dinámica de la base; ROS 2 detecta una esfera, estima su
rango y ordena velocidades para mantenerla centrada y a 1.2 m. La pose perfecta
del objetivo nunca entra al controlador. Solo `metrics_logger` recibe ground
truth para comparar estimación y desempeño.

La manipulación autónoma queda fuera del corte. El brazo y la pinza se conservan
para demostrar que los controladores articulares coexisten con el lazo móvil.

## Componentes

| Componente | Archivo | Responsabilidad |
|---|---|---|
| Mundo | `worlds/ball_arena.sdf` | Física, suelo, luz y esfera roja |
| Robot | `urdf/mobile_manipulator.urdf.xacro` | Base 4WD, brazo, pinza, cámara y `ros2_control` |
| Controladores | `config/controllers.yaml` | Joint states, base diferencial y trayectoria articular |
| Composición | `launch/sim.launch.py` | Gazebo, spawn, bridges, spawners y nodos |
| Percepción | `ball_detector.py` | Segmentación HSV, centro en imagen y rango |
| Tracking | `visual_tracker.py` | Control proporcional lineal/angular |
| Objetivo | `target_trajectory.py` | Trayectoria estática o móvil por `SetEntityPose` |
| Evaluación | `metrics_logger.py` | CSV por frame y resumen estadístico |

## Flujo de información

```text
ball_arena.sdf
  ├─ /clock ───────────────────────────────> reloj ROS 2
  ├─ /camera/image_raw ─┐
  ├─ /camera/camera_info├─> ball_detector ─> /ball/measurement
  │                     │                         │
  │                     └─> /ball/debug           ▼
  │                                           visual_tracker
  │                                                │
  │                                  /base_controller/cmd_vel
  │                                                ▼
  └─ gz_ros2_control <──────────────────── base_controller
          │                                     │
          ├─ /joint_states                      ├─ /base_controller/odom
          └─ /tf: odom -> base_footprint        └─ wheel commands

target_trajectory ── SetEntityPose ──> target_ball
        └─ /target/ground_truth ───────────────> metrics_logger
/ball/measurement, /cmd_vel y /odom ───────────> metrics_logger
```

## Nodos y topics

| Nodo | Suscribe | Publica o invoca |
|---|---|---|
| `robot_state_publisher` | `/robot_description` interno | `/tf`, `/tf_static` |
| `parameter_bridge` | Gazebo Transport | `/clock`, imagen, `CameraInfo`; bridge de `set_pose` |
| `ball_detector` | `/camera/image_raw`, `/camera/camera_info` | `/ball/measurement`, `/ball/debug` |
| `visual_tracker` | `/ball/measurement` | `/base_controller/cmd_vel` |
| `target_trajectory` | reloj y parámetros | `/target/ground_truth`, servicio `/world/ball_arena/set_pose` |
| `metrics_logger` | medición, comando, odom, ground truth | CSV y resúmenes |
| `controller_manager` | descripción y comandos | servicios y estado de controladores |

`/ball/measurement` usa `geometry_msgs/Vector3Stamped`:

- `vector.x`: error horizontal normalizado respecto al semiancho de imagen;
- `vector.y`: error vertical normalizado respecto a la semialtura;
- `vector.z`: rango cámara-centro de esfera en metros;
- un valor no finito indica detección inválida.

## Reloj, odometría y TF

Gazebo publica `/clock` en su transporte. El bridge lo expone como
`rosgraph_msgs/msg/Clock` y todos los nodos de la simulación usan
`use_sim_time=true`.

El topic real de Jazzy es `/base_controller/odom`, no `/odom`. El
`diff_drive_controller` integra las posiciones de las cuatro ruedas y publica:

- frame padre `odom`;
- frame hijo `base_footprint`;
- `nav_msgs/msg/Odometry` en `/base_controller/odom`;
- TF `odom -> base_footprint` porque `enable_odom_tf=true`.

El diagnóstico compara odometría antes/después de una orden y verifica que la
traslación TF final coincide con odometría dentro de 2 cm.

## Geometría de cámara

La cámara Gazebo tiene 640×480 px y FOV horizontal 1.047 rad. El valor teórico
es:

```text
fx = width / (2 tan(FOVx / 2)) ≈ 554.38 px
```

El detector usa preferentemente `CameraInfo.K`: `fx`, `fy`, `cx` y `cy`. Solo
si `CameraInfo` todavía no llegó deriva `fx` del ancho y FOV configurado.

Para una esfera de radio conocido `R` y radio aparente `r`:

```text
alpha = atan(r / fx)
Z = R / sin(alpha)
x_ray = (u - cx) / fx
y_ray = (v - cy) / fy
D = Z sqrt(1 + x_ray² + y_ray²)
```

`Z` es profundidad óptica y `D` es rango 3D cámara-centro. Esta última
conversión es importante cuando el objetivo se aleja del eje óptico. Supuestos:
cámara pinhole sin distorsión, esfera completa visible y radio real 0.12 m.

## Trayectoria determinista

`target_trajectory` usa tiempo de simulación y el servicio público
`ros_gz_interfaces/srv/SetEntityPose`. La esfera se declara estática para evitar
dinámica no necesaria y cada pose aceptada se publica como ground truth de
evaluación.

Modo estático:

```text
x(t) = centre_x
y(t) = 0
```

Modo móvil:

```text
phase = omega t
x(t) = centre_x + A_long sin(phase)
y(t) = A_lat sin(phase / 2)
```

Valores por defecto: `centre_x=2.0 m`, `A_long=0.45 m`,
`A_lat=0.65 m` y `omega=0.25 rad/s`. La ruta fuerza tanto regulación
longitudinal como corrección angular y se repite para el mismo tiempo simulado.

## Control visual

`visual_tracker` no conoce la pose de Gazebo. Con error horizontal `e_h` y
error de rango `e_d = D - D_ref`:

```text
omega_cmd = saturate(-K_angular e_h)
v_cmd = saturate(K_linear e_d × alignment_scale)
```

`alignment_scale` reduce avance cuando la bola está descentrada. Deadbands,
ganancias, saturaciones, timeout y distancia objetivo son parámetros ROS 2. Si
la medición es inválida o caduca, el nodo publica una orden nula.

## Métricas y separación de privilegios

Por cada imagen, `metrics_logger` registra timestamp, detección, error horizontal,
rango estimado, referencia, comando, odometría, objetivo aceptado y error frente
a ground truth. El ground truth combina la pose objetivo aceptada por Gazebo con
la odometría y el offset conocido de cámara. Nunca se publica hacia
`visual_tracker`.

El resumen calcula:

- tasa de detección;
- MAE y RMSE de estimación de rango;
- RMS de error horizontal;
- MAE de distancia objetivo;
- MAE estacionario en el último 25%;
- actividad de comandos, tiempo de primera detección y settling;
- spans de trayectoria y desplazamiento del robot.

La comparación A/B conserva mundo, trayectoria, detector y métricas. Solo cambia
`tracking_enabled=false/true`, lo que hace interpretable la diferencia.

## Controladores y articulaciones

| Controlador | Tipo | Interfaces |
|---|---|---|
| `joint_state_broadcaster` | `JointStateBroadcaster` | estados disponibles |
| `base_controller` | `DiffDriveController` | velocidad de cuatro ruedas |
| `arm_controller` | `JointTrajectoryController` | posición de 4 GDL y 2 dedos |

Los spawners se ejecutan explícitamente después del spawn. El diagnóstico ordena
seis posiciones y verifica que cada joint llega a ±0.03 rad o m.
