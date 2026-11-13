<div align="center">

# Mobile Manipulator · Visual Tracking in Gazebo
### Percepción monocular, ROS 2 y control móvil reproducible

**ROS 2 Jazzy · Gazebo Sim 8 · Python · Xacro / URDF · Poppy Ergo Jr**

[Arquitectura](docs/architecture.md) · [Resultados verificados](results/verified/) · [Tutorial de integración](docs/tutorial_handoff.md) · [Documentación completa](README_TECHNICAL.md)

</div>

## El problema

Un robot móvil 4WD observa una esfera roja, estima su distancia **a partir de imagen y CameraInfo** y ajusta su movimiento para mantener una separación objetivo. Un brazo inspirado en Poppy Ergo Jr añade geometría CAD y control articular, sin mezclar la posición privilegiada de Gazebo con la medida usada por el controlador.

## Qué hay implementado

| Componente | Evidencia |
| --- | --- |
| Percepción | Detector HSV, intrínsecos de cámara, medición y vista de depuración |
| Seguimiento | Control proporcional, odometría, TF, límites de velocidad |
| Manipulación | Seis articulaciones, geometría CAD y comparación FK/TF |
| Validación | Diagnóstico de integración, prueba A/B, telemetría y pruebas unitarias |

La comparación A/B documentada redujo el **MAE de distancia** de 0,529 m a 0,149 m (71,8 %) en el escenario de prueba; ambas condiciones usaron la misma trayectoria. **Es una mejora del simulador y de ese experimento, no una validación en hardware.** [Datos originales](results/verified/experiments/comparison.json).

```text
Cámara + CameraInfo → detector → medición → controlador → base móvil
                              │                         │
           Referencia Gazebo ─┴──► evaluación ◄── odometría
                 (solo para métricas)
```

## Reproducir

Requiere Ubuntu 24.04, ROS 2 Jazzy y Gazebo; seguir la [instalación completa](README_TECHNICAL.md#instalación). Tras preparar ROS y el workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
ros2 launch mobile_manipulator sim.launch.py
```

[Diagnóstico completo](README_TECHNICAL.md#diagnóstico-reproducible) · [Experimento A/B](README_TECHNICAL.md#experimento-ab) · [Reporte de integración](docs/final_report.md).

## Límites y siguientes pasos

La esfera de radio conocido es controlada por un escenario determinista y se observa con cámara monocular; no hay calibración real, navegación global ni manipulación autónoma *pick-and-place*. El brazo ha sido evaluado geométrica y articularmente, pero no como sistema físico real. [Ver limitaciones completas](README_TECHNICAL.md#limitaciones-y-extensiones).

> Ejemplo reproducible pensado para docencia e investigación. [README original íntegro](README_TECHNICAL.md).
