# Cierre del pipeline CAD → Gazebo

## Estado validado


> Actualización: el pipeline CAD de este documento permanece válido, pero la geometría y las métricas fueron superadas por el cierre mecánico basado en el baseline 70c5d6fc30ea60e6a96166c816fa8106855000c7.
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

Este informe documentó el cierre inicial del pipeline. La auditoría humana posterior descubrió un ensamblaje mecánico incorrecto pese a los PASS sintácticos y de control. Ese fallo se corrigió y su evidencia autoritativa está en [mechanical_assembly_validation.md](mechanical_assembly_validation.md).

Las nuevas capturas Gazebo de home, dos poses, detalle del brazo 1:1 y robot compacto están en captures/mechanical_assembly/. También se generaron vistas overlay/collision-only reproducibles. Gazebo GUI abrió bajo WSLg, pero la automatización no logró capturar el overlay nativo; la alternativa está etiquetada y no se equipara con inspección GUI interactiva.

## Regresión ROS/Gazebo

El pipeline STEP de este informe sigue pasando sin cambios conceptuales. Tras la corrección mecánica se repitieron build, 7 tests, diagnóstico y A/B. El diagnóstico actual añade FK independiente contra TF y pasa con errores menores a 0.48 mm. La base compacta desplazó .244 m con odom/TF coherentes; cámara y detector permanecen operativos.

El experimento posterior obtuvo 100 % de detección en A y B; el MAE objetivo cambió de .528910 m a .149327 m, mejora 71.77 %, con desplazamiento B de .745751 m. El comparador sigue en PASS. Los valores de este párrafo sustituyen las métricas históricas del cierre inicial.

## Problemas reales y límites restantes

Los problemas de Gmsh, unidades AP214 y warnings de tessellation aquí descritos siguen siendo reales. La limitación visual quedó parcialmente cerrada mediante cámaras Gazebo y un preview collision reproducible; continúa faltando una captura verificable del overlay nativo de la GUI. La auditoría posterior también documentó el falso positivo geométrico y un proceso Gazebo huérfano durante shutdown. Consulte el informe mecánico para causa, corrección y límites actuales.

## Handoff para actualización del tutorial ChatGPT

Este informe debe leerse como base del pipeline CAD, seguido por:

1. docs/mechanical_assembly_closure_report.md;
2. docs/mechanical_assembly_validation.md;
3. docs/cad_import_tutorial.md y docs/cad_import_troubleshooting.md;
4. results/verified/cad_step_conversion/summary.json y results/verified/mechanical_assembly/summary.json;
5. results/verified/diagnostic/summary.json y results/verified/experiments/comparison.json.

El informe mecánico posterior es la autoridad para frames, base, evidencia visual y métricas de regresión actuales.
