# Informe parcial de progreso

## Estado al 22 de agosto de 2026

El prototipo ya se instancia en Gazebo Sim, publica estados articulares y entrega una imagen de la camara frontal a ROS 2. La base diferencial y el brazo se cargan y activan mediante ros2_control.

La evidencia reproducible se conserva en `results/diagnostic/` y `captures/png/`. La captura frontal real esta en `captures/png/front_camera.png`. El video `captures/diagnostic_front_camera_60fps.mp4` usa H.264 y fue verificado con 60 fps, 3 segundos y 180 fotogramas.

## Arquitectura

Xacro describe la base 4WD, brazo, pinza y camara. Gazebo aporta fisica, sensor y plugin gz_ros2_control. ROS 2 puentea la camara y ejecuta detector, tracker y controladores. El script `scripts/run_diagnostic.sh` recompila y ejercita esta cadena.

## Resultado del diagnostico

- Spawn: exitoso.
- Camara: imagen guardada.
- Estados articulares: recibidos.
- Controladores base y brazo: activados.
- Pendiente: odometria/TF comprobable, metrica A/B, seguimiento visual y agarre.

## Riesgos conocidos

La sesion headless de WSL no permite una captura 3D externa de GUI. El PDF no se genero porque el flujo obligatorio de PDF requiere el marcador local `container_tools/mark_artifact_operation_started.mjs`, ausente en esta sesion; no se omite su verificacion visual.

## Proximo hito

Restaurar el marcador de artefacto PDF, generar el informe con esta evidencia y renderizar cada pagina para inspeccion visual.
