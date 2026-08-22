# Informe parcial de progreso

## Estado al 22 de agosto de 2026

El prototipo ya se instancia en Gazebo Sim, publica estados articulares y entrega una imagen de la camara frontal a ROS 2. La base diferencial y el brazo se cargan y activan mediante ros2_control.

La evidencia reproducible se conserva en `results/diagnostic/` y `captures/png/`. La captura frontal real esta en `captures/png/front_camera.png`.

## Arquitectura

Xacro describe la base 4WD, brazo, pinza y camara. Gazebo aporta fisica, sensor y plugin gz_ros2_control. ROS 2 puentea la camara y ejecuta detector, tracker y controladores. El script `scripts/run_diagnostic.sh` recompila y ejercita esta cadena.

## Resultado del diagnostico

- Spawn: exitoso.
- Camara: imagen guardada.
- Estados articulares: recibidos.
- Controladores base y brazo: activados.
- Pendiente: odometria/TF comprobable, metrica A/B, seguimiento visual y agarre.

## Riesgos conocidos

La sesion headless de WSL no permite una captura 3D externa de GUI. Ademas, no fue posible instalar FFmpeg ni bibliotecas de PDF porque sudo requiere contrasena interactiva. Por ello este archivo Markdown sustituye temporalmente al PDF solicitado; no debe confundirse con una validacion final.

## Proximo hito

Tras habilitar las dependencias, generar MP4 H.264 a 60 fps desde PNGs reales, crear el PDF con esta evidencia y renderizar cada pagina para inspeccion visual.
