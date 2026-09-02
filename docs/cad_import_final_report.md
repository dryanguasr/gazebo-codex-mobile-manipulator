# Informe final: importación CAD de Poppy Ergo Jr

## Resultado

El brazo genérico de cuatro GDL y la pinza prismática fueron reemplazados por una cadena de seis motores basada en los STEP/STL oficiales de Poppy Ergo Jr. Los originales, derivados visuales y colisiones están físicamente separados, sus hashes y transformaciones quedan registrados y Gazebo carga todas las geometrías sin errores.

Commit de implementación validado: `VALIDATED_IMPLEMENTATION_COMMIT`.

Fuente Poppy: `poppy-project/poppy-ergo-jr`, commit `97ce599be8c717843c45ebf48341f2ebf8f250b3`, hardware CC BY-SA 4.0.

## Assets y herramientas

```text
src/mobile_manipulator/meshes/poppy_ergo_jr/
  asset_manifest.json
  source/hardware/
    LICENSE.md
    README.md
    STEP/{base,U_parts,lateral_parts}.step
    STEP/tools/gripper.step
    STL/{base,disk_support,long_U,short_U,horn2horn,side2side,support_camera}.stl
    STL/tools/{gripper-fixation,gripper-fixed_part,gripper-rotative_part}.stl
  visual/
    poppy_mount.stl
    poppy_link_1.stl ... poppy_link_6.stl
    poppy_camera_support.stl
  collision/
    poppy_mount_convex.stl
    poppy_link_6_convex.stl
```

Herramientas registradas en el manifest: Python 3.12.3, NumPy 1.26.4, SciPy 1.11.4/Qhull. No se requirió edición gráfica. El pipeline acepta STL binario o ASCII, escribe STL binario determinista y usa una grilla de 1 mm antes del hull.

Los STEP AP214 declaran metros. Los STL usan milímetros y se escalan 0.001. Las correcciones de origen y rotación están en `asset_manifest.json`.

## Links, joints, masas e inercia

| Link | CAD principal | Collision | Masa kg | COM aproximado m |
|---|---|---|---:|---|
| mount | base | convex hull | 0.0283 | 0, 0, 0.025 |
| link 1 | long_U | box | 0.0223 | 0, 0, 0.025 |
| link 2 | horn2horn + side2side | box | 0.0216 | 0, 0, 0.041 |
| link 3 | short_U | box | 0.0186 | 0, 0, 0.030 |
| link 4 | horn2horn + side2side | box | 0.0216 | 0, 0, 0.041 |
| link 5 | fixation + mordaza fija | dos boxes | 0.0244 | 0.025, 0, 0.045 |
| link 6 | mordaza rotativa | hull 92 tri | 0.00614 | 0.030, -0.0035, 0 |

Las masas combinan volumen STL a 1240 kg/m³ y 16.7 g por XL-320. Las inercias son aproximaciones de caja en kg·m² y pasaron positividad y desigualdades triangulares.

| Joint | Parent → child | Axis | Origin m | Límite rad |
|---|---|---|---|---|
| m1 | mount → 1 | 0 0 1 | 0 0 0.0328 | ±2.618 |
| m2 | 1 → 2 | 0 1 0 | 0 0 0.024 | ±1.571 |
| m3 | 2 → 3 | 1 0 0 | 0 0 0.054 | ±1.571 |
| m4 | 3 → 4 | 0 1 0 | 0 0 0.045 | ±1.571 |
| m5 | 4 → 5 | 1 0 0 | 0 0 0.048 | ±1.571 |
| m6 | 5 → 6 | 0 1 0 | 0 0 0.058 | 0 a 1.20 |

## Visual frente a collision

La mordaza móvil visual tiene 32168 triángulos. Su convex hull collision tiene 92, conserva una envolvente aproximada de 80 x 27 x 34 mm y elimina dientes, agujeros y concavidades. Ratio: 0.0029. Los brackets usan primitivas para evitar contactos costosos.

La autocolisión está desactivada. Los colliders no bloquearon ninguno de los seis joints en dos poses separadas.

## Plataforma móvil y cámara

No se cambió la base. Sus 0.72 x 0.52 m admiten la huella Poppy de 0.15 m. Se conservaron masa, inercia, ruedas, radio 0.115 m, separación 0.60 m y sensor frontal. El mount se fija en `-0.05 0 0.08`.

El soporte oficial de cámara fue inspeccionado y convertido, pero no sustituye la cámara frontal porque habría alterado el experimento de tracking.

## Comandos para reproducir

```bash
cd ~/proyectos/gazebo-tutorial/gazebo-codex-mobile-manipulator
python3 scripts/cad/prepare_poppy_assets.py
python3 scripts/cad/validate_meshes.py

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
xacro src/mobile_manipulator/urdf/mobile_manipulator.urdf.xacro >/tmp/mobile.urdf
check_urdf /tmp/mobile.urdf
colcon test --event-handlers console_direct+
colcon test-result --verbose
bash scripts/run_diagnostic.sh
bash scripts/run_experiments.sh
```

## Evidencia de validación

- Build: PASS.
- Tests: 7, 0 errores, 0 fallos.
- Xacro/check_urdf: PASS, cadena mount + seis links.
- Spawn: PASS y sin líneas `[Err]` de Gazebo.
- Meshes instalados: PASS bajo `install/.../share/mobile_manipulator/meshes`.
- Controllers: joint_state_broadcaster, base_controller y arm_controller activos.
- Joint states: seis joints Poppy presentes.
- Pose 1: PASS, máximo error 2.75e-11 rad.
- Pose 2: PASS, máximo error 3.33e-11 rad.
- Detección de joint bloqueado: PASS, todos cambian más de 0.15 rad.
- Base: PASS, desplazamiento 0.676 m.
- TF: PASS, odom → base_footprint coincide con odometría.
- Cámara: PASS, 640 x 480, fx 554.383 px.
- Detector: PASS, rango inicial 1.630 m.
- A/B: PASS, 100% detección.
- MAE objetivo A/B: 0.528 / 0.043 m.
- Mejora B/A: 92.0%.

Evidencia: `results/verified/diagnostic/summary.json`, `launch.log`, poses y `results/verified/experiments/comparison.json`.

## Problemas encontrados

1. Objetos Git LFS no descargados.
2. Diferencia de unidad entre STEP y STL.
3. Falso nonmanifold por tolerancia de auditoría.
4. Entorno ROS no cargado para Xacro.
5. Interpretación inicial incorrecta de centros articulares.
6. `package://` convertido a `model://` no resoluble por Gazebo.
7. Diagnóstico previo demasiado débil para detectar meshes ausentes y joints bloqueados.

Las correcciones y validación están desarrolladas en `cad_import_tutorial.md` y `cad_import_troubleshooting.md`.

## Limitaciones

Las masas no proceden de balanza ni del modelo completo con cables/remaches. Los servos son cajas con dimensiones oficiales. No se evaluó contacto contra objetos ni precisión de la punta. La validación visual fue headless; por ello se usaron carga de meshes, bounds, TF, states y movimiento. No se añadió MoveIt, IK ni manipulación.

## Auditoría de aceptación

Los 20 criterios se cubren con originals y licencia versionados, manifest reproducible, carpetas separadas, hull deliberado, Xacro Poppy, seis joints controlados, dos poses, spawn, base/cámara/tracking, diagnóstico y A/B. El estado Git final debe comprobarse después del commit de handoff con `git status --short`.

## Handoff para GPT tutorial

Leer, en este orden:

1. `docs/cad_import_final_report.md`
2. `docs/cad_import_tutorial.md`
3. `docs/cad_import_troubleshooting.md`
4. `third_party/poppy_ergo_jr/README.md`
5. `src/mobile_manipulator/meshes/poppy_ergo_jr/asset_manifest.json`
6. `scripts/cad/prepare_poppy_assets.py`
7. `scripts/cad/validate_meshes.py`
8. `src/mobile_manipulator/urdf/mobile_manipulator.urdf.xacro`
9. `src/mobile_manipulator/config/controllers.yaml`
10. `scripts/run_diagnostic.sh`
11. `scripts/validate_diagnostic.py`
12. `results/verified/diagnostic/summary.json`
13. `results/verified/experiments/comparison.json`
14. `docs/architecture.md`
15. `README.md`

Para un tutorial de estudiantes, convertir primero la auditoría de unidades/orígenes y la separación visual/collision en ejercicios independientes; después introducir frames, inercia y control.
