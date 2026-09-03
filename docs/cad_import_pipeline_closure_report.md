# Cierre del pipeline CAD → Gazebo

## Estado validado

El baseline técnico preservado es `d146b36586c0b73a5892290b56d6d9c7b532da9d`; `aa1ef71901b46f2b4caf2a6cc17db494b88a9be7` solo documentaba ese SHA. Este cierre se valida sobre ROS 2 Jazzy y Gazebo Sim 8 sin rediseñar la base 4WD, el brazo Poppy de seis joints, percepción HSV, `visual_tracker` ni el experimento A/B.

Código del repositorio: Apache-2.0. Hardware Poppy, CAD oficiales y derivados: CC BY-SA 4.0; la licencia y README de hardware siguen instalándose junto a los assets de runtime.

## Conversión STEP real y reproducible

La pieza didáctica es el ensamblaje de montaje `src/mobile_manipulator/meshes/poppy_ergo_jr/source/hardware/STEP/base.step` (SHA-256 `c6f222b8cb2bd227412fad26ee7f5eeb8a1c56c1555b45d4d3072bd74c164e5a`). Es un STEP AP214 que declara metros y contiene los sólidos base, soporte de disco y soporte de cámara.

Herramienta: **Gmsh 4.12.1 con OpenCASCADE**. Dependencia de la ruta STEP: `sudo apt update && sudo apt install gmsh`; NumPy y SciPy están declarados en `package.xml` (`python3-numpy`, `python3-scipy`) y comprobados por `scripts/cad/check_cad_dependencies.py`.

Comando reproducible:

```bash
python3 scripts/cad/convert_step_example.py
```

El script invoca Gmsh en modo CLI con `-2 -clscale <factor> -format stl`, partiendo únicamente del STEP. Registra input, herramienta/versión, parámetros, formato, unidades, escala y output en `results/verified/cad_step_conversion/summary.json`. En esta combinación de STEP/Gmsh las coordenadas importadas fueron de magnitud mm; por ello se aplicó una escala de salida explícita `0.001` tras tessellar. El STL oficial jamás se entrega a Gmsh: solo se abre después como referencia de validación.

## Comparación geométrica

Referencia oficial: agregado de `base.stl`, `disk_support.stl` y `support_camera.stl`. La triangulación no tiene que ser idéntica; la aceptación exige orientación coherente, diferencia de bounds/extensiones ≤ 1 mm y volumen relativo ≤ 2 %.

| Mesh | Factor | Triángulos | Extensiones XYZ (m) | Volumen firmado (m³) | Máx. error extensión | Estado |
|---|---:|---:|---|---:|---:|---|
| STL oficial agregado | — | 156 324 | 0.150000, 0.150000, 0.034200 | 4.4606756e-05 | — | referencia |
| STEP coarse | 1.0 | 23 482 | 0.150000, 0.148483, 0.034200 | 4.4378407e-05 | 0.001516719 m | FAIL didáctico |
| STEP fine | 0.5 | 26 330 | 0.150000, 0.150000, 0.034200 | 4.4746408e-05 | 0.000000017 m | PASS |

La variante fine es watertight, mantiene el signo de volumen/orientación y difiere 0.313 % en volumen. La referencia agregada tiene 120 aristas no manifold por reunir sólidos independientes, por lo que no se usa su flag de watertight como criterio de rechazo. Gmsh avisó de 12 elementos inválidos en dos superficies pero terminó con 0 errores; el warning queda registrado y el resultado pasó los controles geométricos. La comparación coarse/fine enseña la compensación: menor factor/tolerancia implica más triángulos, fichero mayor y bounds más fieles.

Outputs versionados:

- `source/derived_step/base_step_gmsh_coarse.stl`
- `source/derived_step/base_step_gmsh_fine.stl`
- `results/verified/cad_step_conversion/summary.json`

## Fuente frente a runtime

`source/` conserva STEP, STL oficial y derivados para trazabilidad y docencia; no es necesario para ejecutar Gazebo. `setup.py` instala solo `visual/*.stl`, `collision/*.stl`, `asset_manifest.json` y los ficheros CC BY-SA de atribución en `licenses/`. Tras borrar los artefactos previos y ejecutar `colcon build --symlink-install`, se comprobó que `install/.../meshes/poppy_ergo_jr/source` no existe, mientras que los ocho visuales, dos collisions, manifest y licencia sí existen.

## Collision y validación CAD

Las tres estrategias están presentes y documentadas: primitivas `box` para links intermedios, convex hull para `poppy_mount_link` y para `poppy_link_6`, y mesh detallado para visual. El ejemplo de mordaza mantiene 32 168 triángulos visuales frente a 92 de collision (ratio 0.0029). `validate_meshes.py` ahora valida además el artefacto STEP: status, política de no usar STL como input, output, número de triángulos, dimensión razonable, orientación, tolerancia geométrica y URI. Conserva comprobaciones de archivos ausentes, escala, collision simplificada, masa/inercia y URI rotas con errores diagnósticos.

## Evidencia visual

WSLg expone `DISPLAY=:0` y `WAYLAND_DISPLAY=wayland-0`, pero el helper Windows de automatización falló antes de inicializar con `windows sandbox failed: helper_unknown_error: setup refresh had errors`. No se declara por tanto una inspección GUI directa ni se inventan capturas de collision overlay.

Como alternativa limitada y reproducible se extrajeron frames de las corridas Gazebo headless ya validadas:

- `captures/cad_import/robot_pose_a_isometric.png` (t=5 s)
- `captures/cad_import/robot_pose_b_isometric.png` (t=35 s)
- `captures/cad_import/robot_close_tracking.png` (t=18 s)

Proceden de los vídeos de cámara isométrica/perspectiva de 60 fps. Complementan, pero no reemplazan, la inspección GUI. La evidencia de collision disponible es numérica y exportable: los STL de `collision/`, sus bounds/triángulos en el manifest y las comprobaciones de `validate_meshes.py`; queda pendiente una captura de overlay de colisiones en una sesión con GUI funcional.

## Regresión ROS/Gazebo

Orden ejecutado:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 scripts/cad/prepare_poppy_assets.py
python3 scripts/cad/convert_step_example.py
python3 scripts/cad/validate_meshes.py
bash scripts/run_diagnostic.sh
bash scripts/run_experiments.sh
```

Resultados: build correcto; 7 tests, 0 errores/fallos/skips; conversión STEP PASS; CAD validator PASS. Diagnóstico PASS: spawn y carga de meshes, seis joints, dos poses (error máximo `3.326416919691155e-11` rad), base/odom/TF con 0.676 m, cámara fx 554.383 px, detector y tracking activos.

Experimento final PASS: A obtuvo 100 % de detección y MAE de distancia objetivo 0.5283678 m; B obtuvo 100 %, MAE 0.0425082 m, desplazamiento 0.3122548 m y mejora 91.9548 %. Los criterios del comparador siguen satisfechos.

## Problemas reales y límites restantes

Se observaron Gmsh ausente de PATH en la instalación base, interpretación mm-like del STEP por Gmsh y warnings de elementos inválidos sin error final; están separados de problemas comunes adicionales en el troubleshooting. La única limitación abierta es visual: faltó un overlay GUI de collision porque el helper de captura falló; el procedimiento y evidencia alternativa quedan explícitos, sin equipararlos a una auditoría GUI.

## Handoff para actualización del tutorial ChatGPT

Orden recomendado de lectura:

1. `docs/cad_import_pipeline_closure_report.md`
2. `results/verified/cad_step_conversion/summary.json`
3. `scripts/cad/convert_step_example.py` y `scripts/cad/check_cad_dependencies.py`
4. `scripts/cad/validate_meshes.py`, `scripts/cad/prepare_poppy_assets.py` y `asset_manifest.json`
5. `docs/cad_import_tutorial.md` y `docs/cad_import_troubleshooting.md`
6. `src/mobile_manipulator/setup.py`, `package.xml`, Xacro y `results/verified/diagnostic/summary.json`
7. `results/verified/experiments/comparison.json` y `captures/cad_import/`
