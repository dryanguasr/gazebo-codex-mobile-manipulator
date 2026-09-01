# Índice y estado validado — GPT Tutorial ROS 2 + Gazebo

## Propósito

Este paquete sustenta un GPT tutorial para estudiantes de ingeniería mecatrónica que **ya tienen WSL 2 y ROS 2 Jazzy instalados**. El caso conductor es un manipulador móvil 4WD simulado en Gazebo Sim 8 que detecta una esfera roja con una cámara monocular y la sigue manteniendo una distancia de referencia.

El tutorial no pretende enseñar todo ROS 2 ni todo Gazebo. Su meta es recorrer un lazo completo:

**Gazebo → cámara → bridge → ROS 2 → percepción → control → base móvil → odometría/TF → métricas.**

## Repositorio de referencia

- Repositorio: `dryanguasr/gazebo-codex-mobile-manipulator`.
- Commit de implementación y evidencia validada: `251a1f6ca761c85449af9aeaf162c1fa8aa78e47`.
- `main` auditado al preparar este paquete: `8f75a8980b69aff3b19ff55d4def4fd0e9d63421`.
- El commit `8f75a898...` solo sustituye en `docs/final_report.md` el marcador del SHA validado; no cambia implementación ni evidencia.

## Entorno validado

- Ubuntu 24.04 sobre WSL 2.
- ROS 2 Jazzy.
- Gazebo Sim 8.11.0.
- `ros_gz_sim` 1.0.22.
- `gz_ros2_control` 1.2.19.
- `diff_drive_controller` 4.40.1.
- Python 3.12.3.
- Paquete `mobile_manipulator`, versión 0.2.0.

## Estado funcional validado

Se verificó: build; Xacro/URDF; spawn; tres controladores activos; 10 joint states; cámara y `CameraInfo`; `/clock`; odometría en `/base_controller/odom`; TF `odom -> base_footprint`; detector HSV; estimación monocular de rango; trayectoria estática/móvil determinista; controlador visual P basado solo en percepción; experimento A/B; métricas CSV/JSON; control articular del brazo/pinza; seis pruebas unitarias.

## Resultados autoritativos

### Diagnóstico

Fuente: `results/verified/diagnostic/summary.json`.

- estado: `passed`;
- topic de odometría: `/base_controller/odom`;
- TF: `odom -> base_footprint`;
- desplazamiento comprobado: **0.370 m**;
- focal observada: **554.383 px**;
- rango inicial estimado: **1.633 m**;
- seis consignas de brazo/pinza comprobadas.

### Experimento A/B

Fuente: `results/verified/experiments/comparison.json`.

| Métrica | A: tracking off | B: tracking on |
|---|---:|---:|
| Muestras tras warmup | 667 | 698 |
| Detección | 100.0% | 100.0% |
| MAE rango | 0.097 m | 0.016 m |
| RMSE rango | 0.111 m | 0.018 m |
| RMS horizontal | 0.528 | 0.034 |
| MAE distancia objetivo | 0.535 m | 0.088 m |
| Error estacionario MAE | 0.568 m | 0.083 m |
| Actividad de comando | 0% | 100% |
| Desplazamiento robot | ~0 m | 0.368 m |
| Span objetivo X/Y | 0.900/1.011 m | 0.900/1.011 m |

Mejora B/A en MAE de distancia objetivo: **83.6%**.

## Corrección documental detectada durante la auditoría

`docs/final_report.md` y `docs/experiment_log.md` conservan una frase que reporta **0.666 m** de desplazamiento de diagnóstico. El artefacto generado por el validador y versionado en `summary.json` reporta **0.370 m**, consistente con `odom_before.txt`, `odom_after.txt`, `tf_after.txt` y el propio validador.

Para el GPT, **0.370 m es el valor autoritativo**. Es una inconsistencia narrativa menor y no invalida el hito funcional.

## Jerarquía de fuentes

Ante discrepancias usar, en este orden:

1. JSON verificado en `results/verified/`;
2. código y validadores que generan/comprueban esos JSON;
3. archivos de conocimiento de este paquete;
4. README/documentación narrativa;
5. conocimiento general de ROS 2/Gazebo;
6. web externa.

No reemplazar nombres de topics, frames, controladores, archivos o cifras validadas por alternativas genéricas sin advertirlo.

## Ruta pedagógica

0. Prerrequisitos.
1. ROS 2 frente a Gazebo.
2. Workspace y repositorio.
3. URDF/Xacro/SDF.
4. Launch y spawn.
5. Nodes/topics/messages.
6. `ros2_control`.
7. `/clock`, odometría y TF.
8. Cámara y bridge.
9. Percepción monocular.
10. Control visual.
11. Objetivo móvil y seguimiento.
12. Métricas y A/B.
13. Brazo/pinza.
14. Extensiones hacia robot agrícola.

## Fuera de alcance

Instalación inicial de WSL/ROS; SLAM/Nav2; IK/MoveIt/pick-and-place; percepción aprendida; calibración física; ruido/oclusiones/latencia real; GUI como requisito.

Para la instalación inicial usar el GPT **Soporte Instalaciones Robótica**:
`https://chatgpt.com/g/g-696a59fbc5748191801b7fb3896b7925-soporte-instalaciones-robotica`
