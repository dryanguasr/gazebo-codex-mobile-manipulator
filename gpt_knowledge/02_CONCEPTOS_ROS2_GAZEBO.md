# Conceptos ROS 2 + Gazebo usados en el tutorial

## ROS 2 y Gazebo

ROS 2 es el middleware que comunica y coordina componentes. Gazebo Sim 8 aporta mundo, física, robot y sensores simulados. **ROS 2 no es el simulador y Gazebo no sustituye a ROS 2.** Se conectan mediante `ros_gz_bridge` y `gz_ros2_control`.

## Workspace y package

El repositorio es un workspace de `colcon`. El código ROS está en `src/`. Tras compilar aparecen `build/`, `install/` y `log/`, que son artefactos generados.

El paquete principal es `mobile_manipulator`; su manifiesto es `src/mobile_manipulator/package.xml`.

## Node

Un nodo es un proceso/componente ROS con una responsabilidad. En este ejemplo destacan `ball_detector`, `visual_tracker`, `target_trajectory`, `metrics_logger`, `robot_state_publisher` y `controller_manager`.

## Topic

Canal tipado y asíncrono. Ejemplos: `/camera/image_raw`, `/ball/measurement`, `/base_controller/cmd_vel`, `/base_controller/odom`, `/joint_states`.

Un frame llamado `odom` **no implica** que el topic de odometría se llame `/odom`.

## Mensajes relevantes

- `sensor_msgs/Image`: imagen.
- `sensor_msgs/CameraInfo`: intrínsecos.
- `geometry_msgs/Vector3Stamped`: medición visual.
- `geometry_msgs/TwistStamped`: comando de velocidad.
- `nav_msgs/Odometry`: odometría.
- `geometry_msgs/PoseStamped`: ground truth del objetivo para evaluación.

## Publisher y subscriber

Ejemplo: `ball_detector` publica `/ball/measurement`; `visual_tracker` se suscribe y publica `/base_controller/cmd_vel`.

## Service

Comunicación petición/respuesta. `target_trajectory` usa `/world/ball_arena/set_pose` para pedir a Gazebo una nueva pose de la esfera.

## Launch

`launch/sim.launch.py` arranca Gazebo, instancia el robot, crea bridges, activa controladores e inicia los nodos del ejemplo.

## URDF, Xacro y SDF

- URDF: árbol cinemático del robot.
- Xacro: forma estructurada/parametrizable de generar URDF.
- SDF: descripción del mundo Gazebo.

Archivos: `urdf/mobile_manipulator.urdf.xacro` y `worlds/ball_arena.sdf`.

## ros2_control

Separa controladores ROS de interfaces de hardware/simulación. En este ejemplo: `joint_state_broadcaster`, `base_controller` y `arm_controller`; Gazebo aporta `gz_ros2_control`.

## TF y frames

TF representa relaciones entre marcos. Frames principales: `odom`, `base_footprint`, `base_link`, `camera_link`. El controlador diferencial publica `odom -> base_footprint`.

## Odometría

Estimación incremental de pose a partir del movimiento de ruedas. Topic validado: `/base_controller/odom`. No equivale a localización global ni a SLAM.

## Tiempo simulado

Gazebo publica el reloj; el bridge lo expone en `/clock`. Los nodos usan `use_sim_time=true`, de modo que timers y timestamps dependen del simulador.

## Ground truth

Información perfecta del simulador que un robot real normalmente no conocería. `/target/ground_truth` solo entra a `metrics_logger`. **Nunca debe entrar a `visual_tracker`.** Este límite es central para una evaluación honesta.
