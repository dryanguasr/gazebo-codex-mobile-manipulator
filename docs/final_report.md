# Informe final provisional

## Estado

El entorno ROS 2 Jazzy/Gazebo Harmonic quedó instalado y auditado. Se creó un primer modelo integrado Xacro con base 4WD, brazo de cuatro GDL, pinza y soporte de cámara; también configuración inicial de ros2_control, mundo con bola y nodos clásicos de detector/control.

## Validado

`colcon build` y la expansión Xacro desde el overlay son correctas. `check_urdf` produce solo avisos de materiales sin definición RGBA; no se considera validación de Gazebo ni de control.

## Pendiente crítico

No se ha implementado el sensor de cámara SDF, lanzamiento/spawn, bridge, activación de controladores, pruebas físicas, métricas A ni diagnóstico B. Por tanto, este repositorio no cumple todavía el objetivo completo y no debe presentarse como experimento finalizado.

## Recomendación

Completar y validar primero el spawn y `gz_ros2_control`; resolver los avisos visuales; añadir cámara/bridge; solo después medir prueba A. No usar pose de Gazebo en el controlador.
