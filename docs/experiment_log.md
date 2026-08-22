# Bitacora del experimento

## 2026-08-21 - Auditoria inicial

- ROS 2 Jazzy, Gazebo Sim 8, ros_gz, gz_ros2_control, controladores, Xacro y OpenCV quedaron instalados sin actualizar paquetes ajenos.
- El modelo integra base 4WD, brazo de 4 GDL, pinza, camara frontal y una esfera objetivo.
- La expansion Xacro y `check_urdf` se ejecutaron correctamente. Los avisos de materiales sin definicion RGBA no bloquean la cinematica, pero se deben normalizar para la presentacion final.

## 2026-08-22 - Diagnostico posterior a suspension

| Componente | Estado | Evidencia |
|---|---|---|
| Compilacion | Correcta | `results/diagnostic/build.log` |
| Xacro y URDF | Correcto | `robot.urdf`, `check_urdf.txt` |
| Spawn en Gazebo | Correcto | `launch.log`: Entity creation successful |
| Controladores base y brazo | Activos | `launch.log`: configuracion y activacion correctas |
| Joint states | Correcto | `joint_states.txt` contiene los 10 joints |
| Camara y bridge | Correctos | `front_camera.png`, `camera_capture.txt` |
| Odometria y TF | Pendiente de evidencia | el proceso headless no publico `/odom` durante esta corrida |
| CLI de controladores | No disponible | falta el plugin `ros2controlcli`; se conserva el log del manager |
| Captura 3D externa | Pendiente | WSL headless no dispone de una ventana/renderizador GUI para screenshot |
| Video MP4 60 fps y PDF | Pendientes | FFmpeg y ReportLab no estan instalables sin la contrasena de sudo |

### Correcciones aplicadas

1. Se reemplazaron temporizadores de 5/9 s por eventos de finalizacion de Xacro/spawn y los spawners esperan hasta 120 s el servicio de `controller_manager`.
2. Se agrego `gz-sim-sensors-system` al mundo: la camara ahora genera la imagen ROS capturada.
3. Se retiro el spawner redundante de `joint_state_broadcaster`: el plugin ya lo deja activo y el intento paralelo fallaba al reconfigurarlo desde estado activo.
4. Se agrego `scripts/run_diagnostic.sh` y el nodo `evidence_capture` para repetir la prueba y guardar artefactos locales.

### Pruebas ejecutadas

- Se publico un `TwistStamped` fijo (avance 0.15 m/s, giro 0.25 rad/s) en `/base_controller/cmd_vel`.
- Se publico una trayectoria articular determinista para los seis joints de brazo/pinza.
- La base acepto el comando; el log registra la recepcion. La prueba aun no incluye metrica de desplazamiento porque falta odometria en esta ejecucion.

### Limitaciones y siguientes pasos

1. Instalar `ffmpeg`, `python3-reportlab`, `python3-pypdf` y `python3-pdfplumber` con una sesion sudo autorizada; despues codificar la secuencia PNG a H.264 60 fps y generar/verificar el PDF.
2. Activar/puentear `/clock` y verificar `/odom` y TF de forma repetible.
3. Implementar trayectorias deterministas de la bola y mediciones A/B; la pose de Gazebo se usara solo como referencia de error.
4. Validar seguimiento visual repetible y luego pick-and-place.
