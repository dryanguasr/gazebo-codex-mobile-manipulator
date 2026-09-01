# Configuración recomendada del GPT

## Nombre sugerido

**Tutor ROS 2 + Gazebo — Robot Agrícola**

Alternativa genérica: **Tutor ROS 2 + Gazebo — Robótica Móvil**.

## Descripción corta

Tutor paso a paso para aprender ROS 2 Jazzy y Gazebo mediante un manipulador móvil 4WD con cámara, percepción visual, ros2_control, TF, odometría y seguimiento medible.

## Qué subir como Knowledge

Subir los archivos numerados `00_...` a `13_...` de esta carpeta.

`INSTRUCCIONES_GPT.md` está pensado para copiarse en el campo **Instructions**, no como fuente principal de conocimiento.

## Capacidades sugeridas

- Búsqueda web: **activada**, como respaldo para documentación y errores externos.
- Análisis de datos/código: **activado**, útil para CSV/JSON y archivos que suban los estudiantes.
- Generación de imágenes: opcional.
- Actions externas: no necesarias en la primera versión.

## Iniciadores de conversación

- `Quiero empezar el tutorial desde cero.`
- `Ya tengo ROS 2 instalado, verifiquemos si estoy listo.`
- `Explícame cómo se comunican ROS 2 y Gazebo en este proyecto.`
- `Tengo un error ejecutando el simulador.`
- `Quiero analizar los resultados del experimento A/B.`

## Frontera visible

El GPT presupone WSL + ROS 2 Jazzy instalados. Para esa etapa deriva al GPT **Soporte Instalaciones Robótica**.

El hito principal termina en seguimiento visual medible. Brazo/pinza se muestran como control articular; pick-and-place, Nav2, SLAM y Sim2Real real quedan como extensiones.
