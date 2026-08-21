# Bitácora del experimento

## 2026-08-21 — Etapa 0: auditoría

- Objetivo: verificar versiones y compatibilidad antes de crear el robot.
- Resultado inicial: Ubuntu 24.04.4 LTS, WSL2 kernel 6.18.33.2; Gazebo instalado, pero ROS 2, Xacro y OpenCV ausentes.
- Decisión: tras autorización humana, se añadió el origen ROS oficial y se instalaron solo ROS Jazzy base, ros_gz, gz_ros2_control, controladores, Xacro, cv_bridge, ros_gz_image, ros-dev-tools y python3-opencv; no hubo `apt upgrade`.
- Resultado posterior: Gazebo Sim 8.14.0 CLI, ROS Jazzy, ros_gz 1.0.22, gz_ros2_control 1.2.19, ros2_controllers 4.40.1, Xacro 2.1.1 y OpenCV 4.6.0.
- Fallo: `sudo -n` requirió contraseña; corrección: el humano ejecutó la instalación autorizada.
- Validación: `ros2`, `gz_ros2_control`, `diff_drive_controller`, `joint_trajectory_controller`, `xacro`, `cv_bridge` y OpenCV están resolubles.
- Para guía futura: explicar la fuente ROS Jazzy/Noble, el `source /opt/ros/jazzy/setup.bash`, y que `xacro --version` no es una comprobación válida.

## Arquitectura planificada

