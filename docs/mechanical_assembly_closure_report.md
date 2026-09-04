# Informe de cierre: ensamblaje mecánico y escala del manipulador móvil

## Estado validado

Baseline recibido: 70c5d6fc30ea60e6a96166c816fa8106855000c7. El pipeline CAD → tessellation → visual/collision → URDF/Gazebo construido en ese commit se preservó. Este hito modifica frames, composición geométrica, física de la plataforma y validación integrada; Poppy continúa en metros reales a escala 1:1.

Entorno: ROS 2 Jazzy, Gazebo Sim 8 y Gmsh 4.12.1/OpenCASCADE. Código del repositorio: Apache-2.0. CAD, hardware y derivados Poppy: CC BY-SA 4.0.

## Problema inicial y causa raíz

La inspección humana mostró dos fallos que los gates anteriores no podían detectar:

- un Ergo Jr 1:1 visualmente diminuto sobre una base de .72 x .52 m;
- brackets y servos aparentemente desensamblados aunque los seis joints respondían.

La causa mecánica fue encadenar m2–m6 casi exclusivamente con offsets sobre Z y asignar ejes locales aproximados. Esas transformaciones formaban un árbol URDF válido, pero no reproducían los cambios de frame de 90°, los desplazamientos X/Y ni la pertenencia rígida de la pinza. Además, varios meshes CAD se habían dejado en el frame de exportación en vez de reexpresarlos en el frame físico del link.

## Evidencia utilizada

Se triangularon:

- los STEP/STL oficiales versionados, commit de hardware 97ce599be8c717843c45ebf48341f2ebf8f250b3;
- la [guía mecánica oficial](https://docs.poppy-project.org/en/assembly-guides/ergo-jr/mechanical-construction);
- el [URDF oficial](https://github.com/poppy-project/poppy_ergo_jr_description), commit 7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b.

Del modelo oficial se portaron únicamente transforms verificados. Los meshes visuales y collision siguen siendo los derivados propios del CAD; no se reemplazaron por assets del paquete ROS oficial. Las cotas entre ejes se confirmaron en centros de horns/CAD y el orden y orientación se contrastaron con la guía.

## Joints y frames corregidos

| Joint | Parent → child | origin xyz (m) | origin rpy (rad) | axis |
|---|---|---|---|---|
| m1 | mount → link1 | 0 0 .0327993216 | 0 0 0 | 0 0 1 |
| m2 | link1 → link2 | 0 0 .0240006784 | 0 -pi/2 0 | 0 0 -1 |
| m3 | link2 → link3 | .054 0 0 | 0 0 0 | 0 0 -1 |
| m4 | link3 → link4 | .045 0 0 | 0 -pi/2 0 | 0 0 -1 |
| m5 | link4 → link5 | 0 -.048 0 | 0 -pi/2 0 | 0 0 1 |
| m6 | link5 → link6 | 0 -.058 0 | 0 -pi/2 0 | 0 0 -1 |

Los links 2–6 se reexpresaron mediante matrices documentadas en prepare_poppy_assets.py y asset_manifest.json. La mordaza fija pertenece a link 5 junto al cuerpo m6; la móvil pertenece a link 6. Se añadieron landmarks fijos poppy_fixed_tip y poppy_moving_tip para validar FK/TF.

## Plataforma elegida y física

Se evaluaron .36 x .28 m con rueda .065 m, .40 x .30 con .070 m y .44 x .32 con .075 m. Se eligió .40 x .30 m porque conserva margen de ruedas, cámara y mount sin hacer que la plataforma domine visualmente al brazo.

| Elemento | Configuración final |
|---|---|
| base | .40 x .30 x .10 m, 6.0 kg |
| inercia base | Ixx .050, Iyy .085, Izz .125 kg·m² |
| ruedas | radio .070 m, ancho .045 m, masa .45 kg |
| inercia rueda | axial .000627, radial .001103 kg·m² |
| posición ruedas | x ±.140 m, y ±.1725 m |
| diff-drive | radio .070 m, separación .345 m |
| mount brazo | x −.030, z .050 m sobre base |
| cámara | x .225, z .050 m; 640x480 y FOV preservados |

Visual, collision, masa, tensor, posición de ruedas, parámetros de odometría, mount y altura se cambiaron de forma conjunta. No se modificaron las ganancias del tracker ni el detector HSV.

## Validación geométrica y cinemática

El nuevo scripts/cad/validate_mechanical_assembly.py comprueba:

- los seis parent/child, origins y axes contra la referencia auditada;
- ausencia de escala de mesh distinta de 1:1;
- URI existentes, masas positivas e inercias válidas;
- collision simplificada frente al visual;
- FK independiente para home y dos poses.

FK desde poppy_mount_link:

| Pose | XYZ de poppy_moving_tip (m) | FK vs TF posición | FK vs TF orientación |
|---|---|---:|---:|
| home | 0, -.158, .1558 | gate estático | gate estático |
| pose 1 | -.016193, -.130942, .130429 | .000474 m | .000781 |
| pose 2 | -.045117, -.158248, .178372 | .000462 m | .000562 |

Los gates son 2 mm y 0.003 de distancia cuaternión. El validador mecánico produjo PASS con seis joints, tres poses y cero fallos. El resultado queda en results/verified/mechanical_assembly/summary.json; la comparación ROS queda en results/verified/diagnostic/summary.json.

## Evidencia visual

El detalle completo, incluida la captura preservada del fallo, está en [mechanical_assembly_validation.md](mechanical_assembly_validation.md).

Evidencias principales:

- home, dos poses y close-up del brazo 1:1 en captures/mechanical_assembly/arm_1to1_*.png;
- robot compacto en home y dos poses en compact_robot_*.png;
- comparación visual/collision, detalle de gripper y collision-only en collision_*_alternative_offscreen.png.

Las cámaras de evidencia se ejecutaron dentro de Gazebo. Visualmente, los brackets son continuos, los servos permanecen dentro de sus soportes, los pivotes coinciden y la pinza abre/cambia de posición sin piezas flotantes ni interpenetraciones groseras.

Gazebo GUI pudo abrir bajo WSLg, pero la automatización de Windows no pudo capturar ni activar de forma verificable el overlay nativo: falló con helper_unknown_error: setup refresh had errors y una captura X11 fue negra. Por ello las collisions se mostraron con un URDF estático reproducible creado por generate_collision_preview.py; es evidencia geométrica válida, pero no se afirma que sea un screenshot del overlay nativo.

## Collision y autocolisión

Se preservan tres estrategias: primitivas para links intermedios, convex hull para mount/link 6 y mesh detallado solo para visual. La mordaza móvil conserva 32 168 triángulos visuales y 92 de collision. Los previews confirman alineación; la autocolisión continúa deshabilitada. No se alejaron links para evitar solapes deliberados de uniones vecinas.

## Regresión ROS/Gazebo

Orden final:

~~~bash
python3 scripts/cad/check_cad_dependencies.py
python3 scripts/cad/prepare_poppy_assets.py
python3 scripts/cad/convert_step_example.py
python3 scripts/cad/validate_meshes.py
python3 scripts/cad/validate_mechanical_assembly.py

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
bash scripts/run_diagnostic.sh
bash scripts/run_experiments.sh
~~~

Resultados observados:

- dependencias NumPy/SciPy y herramientas CAD: PASS; Gmsh se ejecutó desde una distribución local explícita en esta sesión;
- conversión STEP real y comparación STL: PASS;
- CAD meshes, escala, URI, masas, inercias y simplificación: PASS;
- build y 7 tests: 0 errores, fallos o skips;
- diagnóstico: PASS; seis joints y dos poses; movimiento base .244 m con odom/TF coherentes; cámara 640x480, fx 554.383; detector activo;
- experimento A: detección 100 %, MAE objetivo .52891048 m, desplazamiento aproximadamente cero;
- experimento B: detección 100 %, MAE objetivo .14932676 m, RMS horizontal .02497811, desplazamiento .74575067 m y comandos activos 99.14 %;
- mejora A→B: 71.7671 %, comparador PASS.

## Problemas reales encontrados

1. Los validadores anteriores daban falso positivo porque no tenían landmarks ni FK independiente.
2. La primera inferencia de frames trataba offsets de CAD como una cadena sobre Z; la guía y el URDF oficial mostraron los cambios de frame ausentes.
3. Los meshes repetidos debían rotarse al frame de cada cuerpo rígido, no solo desplazar los joints.
4. La plataforma antigua ocultaba visualmente la escala real del Poppy.
5. El cierre normal del servidor Gazebo dejó un proceso huérfano en la primera corrida A/B; se añadió espera acotada, detección explícita y SIGKILL solo para el proceso estrechamente identificado.
6. WSLg permitió lanzar GUI, pero el helper de automatización/captura no pudo inicializarse.
7. La base rosdep del host no estaba inicializada y sudo exigió contraseña; se verificaron las declaraciones package.xml y los módulos instalados, pero no se declara una instalación rosdep limpia en esta sesión.

## Limitaciones restantes

- No hubo medición metrológica sobre un robot físico ni identificación dinámica.
- Las cajas de los XL-320 siguen siendo aproximaciones basadas en dimensiones y masa del fabricante.
- No existe captura verificable del overlay nativo de collision de la GUI; se conserva el render alternativo reproducible y se documenta la diferencia.
- La autocolisión queda fuera de alcance y permanece deshabilitada.
- No se añadieron MoveIt, IK de usuario, Nav2, SLAM ni pick-and-place.

## Handoff para actualización del tutorial ChatGPT

Orden recomendado:

1. docs/mechanical_assembly_closure_report.md;
2. docs/mechanical_assembly_validation.md;
3. results/verified/mechanical_assembly/summary.json y results/verified/diagnostic/summary.json;
4. src/mobile_manipulator/urdf/mobile_manipulator.urdf.xacro y src/mobile_manipulator/config/controllers.yaml;
5. scripts/cad/prepare_poppy_assets.py, validate_mechanical_assembly.py y generate_collision_preview.py;
6. docs/cad_import_tutorial.md y docs/cad_import_troubleshooting.md;
7. docs/cad_import_pipeline_closure_report.md y results/verified/cad_step_conversion/summary.json;
8. results/verified/experiments/comparison.json y captures/mechanical_assembly/.

La actualización docente debe conservar la secuencia: CAD/B-rep → tessellation → visual/collision → frames físicos/cuerpos rígidos → masa/inercia → Xacro/instalación → Gazebo → sintaxis + control + geometría mecánica + física.
