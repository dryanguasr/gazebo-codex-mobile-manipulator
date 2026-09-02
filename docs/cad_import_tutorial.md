# Tutorial reproducible: de CAD de Poppy Ergo Jr a Gazebo Sim

## Objetivo y alcance

Este documento reconstruye un brazo Poppy Ergo Jr desde los CAD mecánicos oficiales y lo monta sobre el robot móvil 4WD. No parte de un URDF existente. El flujo separa originales, visuales y colisiones, define frames y física, conecta seis joints a `ros2_control` y valida que la percepción y la base sigan funcionando.

El resultado usa cinco articulaciones para orientar el brazo y el motor m6 para abrir la mordaza rotativa oficial. No incluye MoveIt, IK, pick-and-place, navegación ni SLAM.

## 1. Obtener y fijar la fuente

La fuente primaria es `poppy-project/poppy-ergo-jr`, commit `97ce599be8c717843c45ebf48341f2ebf8f250b3`. El hardware es CC BY-SA 4.0. La tabla completa de rutas y OID LFS está en `third_party/poppy_ergo_jr/README.md`.

Una reproducción con Git LFS puede usar:

```bash
git clone https://github.com/poppy-project/poppy-ergo-jr.git /tmp/poppy-ergo-jr
git -C /tmp/poppy-ergo-jr checkout 97ce599be8c717843c45ebf48341f2ebf8f250b3
git -C /tmp/poppy-ergo-jr lfs pull --include="hardware/STEP/**,hardware/STL/**"
```

En esta máquina `git lfs env` falló porque el helper LFS no estaba disponible. La recuperación reproducible fue descargar cada OID mediante:

```text
https://media.githubusercontent.com/media/poppy-project/poppy-ergo-jr/
97ce599be8c717843c45ebf48341f2ebf8f250b3/<ruta-oficial>
```

Cada descarga se comparó con el OID SHA-256 antes de versionarla.

## 2. Auditoría de formatos, unidades y orígenes

Se inspeccionaron `hardware/STEP`, `hardware/STL`, herramientas y documentación de montaje. Los STEP son AP214, declaran longitud en metros y contienen coordenadas del orden de `0.054`. Los STL oficiales contienen la misma geometría en milímetros: la pieza equivalente alcanza aproximadamente `54`. Por ello solo los STL reciben escala `0.001`.

Los STL son binarios, watertight y con normales almacenadas. Sus pivotes no son uniformes. Algunas piezas empiezan en el eje de un horn; otras conservan un offset de ensamblaje. Esto es útil para medir, pero no permite insertarlas todas con `origin="0 0 0"` sin análisis.

| Pieza STL | Extensión original mm | Triángulos | Interpretación |
|---|---:|---:|---|
| `base` | 56.0 x 57.0 x 26.6 | 102888 | soporte fijo y alojamiento de m1 |
| `long_U` | 34.2 x 19.8 x 33.9 | 27638 | primer bracket, gira con m1 |
| `horn2horn + side2side` | 34.2 x 19.8 x 44.9 | 28966 | pareja lateral repetida |
| `short_U` | 28.2 x 20.0 x 10.0 | 6748 | travesaño situado a 44-54 mm |
| `gripper-fixation` | 20.0 x 24.0 x 28.2 | 9388 | unión entre m5 y cuerpo m6 |
| `gripper-fixed_part` | 5.1 x 92.0 x 24.0 | 7442 | mordaza fija |
| `gripper-rotative_part` | 27.0 x 79.9 x 34.2 | 32168 | mordaza accionada por m6 |

La guía oficial confirma el orden físico: base+m1, `long_U`+m2, laterales+m3, `short_U`+m4, laterales+m5 y fijación/pinza+m6. Los horns deben quedar hacia el mismo lado y la marca de cero hacia arriba.

## 3. Formar los links

Se agruparon piezas por el cuerpo rígido al que permanecen unidas durante el movimiento:

| Link | CAD fuente | Visual | Collision | Masa kg | Joint padre | Observaciones |
|---|---|---|---|---:|---|---|
| `poppy_mount_link` | `base.stl` + cuerpo m1 | `poppy_mount.stl` + box de servo | convex hull 134 tri | 0.0283 | fijo a `base_link` | pieza impresa y estator m1 |
| `poppy_link_1` | `long_U.stl` + cuerpo m2 | 27638 tri + box | box | 0.0223 | m1 | primer bracket |
| `poppy_link_2` | dos laterales + cuerpo m3 | 28966 tri + box | box | 0.0216 | m2 | dos piezas repetidas |
| `poppy_link_3` | `short_U.stl` + cuerpo m4 | 6748 tri + box | box | 0.0186 | m3 | conserva offset CAD |
| `poppy_link_4` | dos laterales + cuerpo m5 | 28966 tri + box | box | 0.0216 | m4 | repetición deliberada |
| `poppy_link_5` | fijación + mordaza fija + cuerpo m6 | 16830 tri + box | dos boxes | 0.0244 | m5 | mordaza fija no gira con m6 |
| `poppy_link_6` | mordaza rotativa | 32168 tri | convex hull 92 tri | 0.00614 | m6 | parte móvil real de la pinza |

Los cuerpos XL-320 no estaban entre los STL seleccionados. Se representan con cajas visuales de 24 x 36 x 27 mm, dimensiones y masa de 16.7 g tomadas del manual ROBOTIS. Esta aproximación se identifica en el Xacro y no se atribuye al CAD Poppy.

## 4. Generar visual meshes

Se eligió STL binario porque los originales ya son triangulaciones oficiales, el robot usa un color uniforme y Gazebo Sim los soporta. DAE habría sido útil para materiales por cara; OBJ no aportaba una ventaja y habría añadido archivos MTL. No se simplificaron los visuales.

Ejecute:

```bash
python3 scripts/cad/prepare_poppy_assets.py
```

El script:

1. verifica SHA-256 de STEP y STL;
2. lee STL sin depender de Blender o FreeCAD;
3. convierte milímetros a metros;
4. centra la base en XY y coloca su cara inferior en Z=0;
5. conserva los offsets de brackets que representan ejes físicos;
6. rota la fijación con `Rx(-90°)`;
7. rota ambas mordazas con `Rz(-90°)`;
8. coloca la mordaza fija a 58 mm del frame m5;
9. escribe visuales deterministas y `asset_manifest.json`.

No hubo corrección gráfica manual.

## 5. Generar collision meshes

Las colisiones intermedias usan cajas que siguen la envolvente del bracket y servo. Esto evita enviar decenas de miles de triángulos al motor de contacto. La base fija y mordaza móvil usan convex hull.

| Link 6 | Triángulos | Extensión m |
|---|---:|---|
| visual | 32168 | 0.0799 x 0.0270 x 0.0342 |
| collision | 92 | 0.0800 x 0.0270 x 0.0340 |

La colisión conserva alcance y volumen general, pero sacrifica dientes, huecos, redondeos y concavidades. Su relación collision/visual es 0.0029. El hull se construye después de cuantizar puntos a una grilla de 1 mm. Esto demuestra por qué `<visual>` y `<collision>` no son intercambiables.

Verificación:

```bash
python3 scripts/cad/validate_meshes.py
```

Para inspección con GUI en Gazebo, abra el menú de visualización y active la vista de collision geometry. En este hito se validó sin GUI mediante carga sin errores, bounds, watertight hulls y dos movimientos completos.

## 6. Frames y joints

Las distancias se obtuvieron de extremos, centros de horn y offsets repetidos en STEP/STL; la guía de montaje fijó el orden y orientación. Tras terminar el modelo CAD-first, se hizo una auditoría final contra `poppy_ergo_jr_description` commit `7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b`. No se copiaron meshes ni Xacro. Esa comparación detectó que la primera lectura trataba varios offsets como longitud total del bracket; se volvieron a medir y se corrigieron.

| Joint | Parent | Child | Tipo | Axis | Origin m | Límites rad | Fuente |
|---|---|---|---|---|---|---|---|
| `poppy_m1_joint` | mount | link 1 | revolute | 0 0 1 | 0 0 0.0328 | ±2.618 | base, horn m1, rango XL-320 |
| `poppy_m2_joint` | link 1 | link 2 | revolute | 0 1 0 | 0 0 0.024 | ±1.571 | `long_U` y cuerpo m2 |
| `poppy_m3_joint` | link 2 | link 3 | revolute | 1 0 0 | 0 0 0.054 | ±1.571 | laterales y centro m3 |
| `poppy_m4_joint` | link 3 | link 4 | revolute | 0 1 0 | 0 0 0.045 | ±1.571 | offset `short_U` |
| `poppy_m5_joint` | link 4 | link 5 | revolute | 1 0 0 | 0 0 0.048 | ±1.571 | laterales y centro m5 |
| `poppy_m6_joint` | link 5 | link 6 | revolute | 0 1 0 | 0 0 0.058 | 0 a 1.20 | fijación y pivote de mordaza |

Los signos se eligieron para que las dos poses de diagnóstico cambien alternadamente de sentido. La comprobación numérica detectaría un joint invertido porque compararía la posición observada con el comando con signo.

## 7. Masa e inercia

Para plástico se usó densidad PLA aproximada de 1240 kg/m³ multiplicada por el volumen cerrado del STL. A cada link se agregó la masa oficial del XL-320 correspondiente. Los centros de masa se colocaron dentro de la envolvente collision. Los tensores se aproximaron como cajas:

```text
Ixx = m (y² + z²) / 12
Iyy = m (x² + z²) / 12
Izz = m (x² + y²) / 12
```

`validate_meshes.py` exige masa positiva, inercias principales positivas y desigualdades triangulares. No se observaron warnings críticos de física en Gazebo. La limitación es que cables, remaches, placa y distribución interna del servo no se modelan.

## 8. Montaje sobre la base móvil

El mount se fija en `(-0.05, 0, 0.08)` respecto de `base_link`, sobre la cara superior. No fue necesario agrandar la plataforma: 0.72 x 0.52 m frente a 0.15 m del disco Poppy. Por ello se conservaron visual, collider, masa, inercia, ruedas, separación 0.60 m, cámara y estabilidad ya validadas.

## 9. Xacro, rutas e instalación

`setup.py` instala recursivamente `meshes/`, incluyendo fuente, visual, collision y manifest. En ROS/Gazebo Sim 8, `package://mobile_manipulator/...` fue convertido a `model://mobile_manipulator/...` y Gazebo no lo resolvió. El robot spawneaba y los joints se movían, pero las piezas CAD no se cargaban.

La corrección fue usar:

```xml
<mesh filename="file://$(find mobile_manipulator)/meshes/..."/>
```

Xacro expande `$(find ...)` al share instalado antes de enviar el URDF a Gazebo. El diagnóstico ahora falla ante cualquier línea `[Err]` de Gazebo, evitando un falso positivo similar.

## 10. ros2_control

`controllers.yaml` enumera exactamente `poppy_m1_joint` a `poppy_m6_joint`. El bloque `ros2_control` expone comando y estado de posición para los seis. Se eliminaron `arm_base_yaw`, `shoulder_pitch`, `elbow_pitch`, `wrist_pitch` y los dos dedos prismáticos docentes.

## 11. Validación

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
bash scripts/run_diagnostic.sh
bash scripts/run_experiments.sh
```

Resultados observados:

- 7 tests, 0 errores;
- Xacro y `check_urdf` correctos;
- spawn sin `[Err]` de Gazebo;
- controladores de estados, base y brazo activos;
- seis joints presentes;
- pose 1 y pose 2 alcanzadas, error máximo menor que 3.4e-11 rad;
- base desplazada 0.676 m con odom y TF coherentes;
- cámara 640 x 480, fx 554.383 px;
- detector y seguimiento visual operativos;
- A/B con 100% de detección;
- tracking redujo el MAE objetivo de 0.528 m a 0.043 m, mejora 92.0%.

La terminación del servidor en el experimento aparece como `[ERROR] ... exit code -15` del launcher porque el cierre envía SIGTERM deliberadamente. No es un error `[Err]` del motor Gazebo.

## 12. Errores reales y correcciones

1. Git LFS no estaba operativo: se descargaron OID oficiales y se verificaron hashes.
2. STEP y STL no usaban la misma magnitud numérica: se confirmó metro en STEP y milímetro en STL.
3. El auditor redondeaba meshes en metros a 1e-6 y fusionó aristas pequeñas: se cambió a 1e-9.
4. Xacro sin entorno ROS no encontraba metadatos ni el paquete: se cargaron `/opt/ros/jazzy/setup.bash` e `install/setup.bash`.
5. Los primeros offsets de joint confundían bounds con centros de eje: la auditoría final obligó a medir de nuevo.
6. `package://` no cargaba en Gazebo Sim: se usó URI absoluta expandida por Xacro y un gate de log.
7. El diagnóstico antiguo aceptaba “controller active” y una sola pose: ahora prueba dos configuraciones, error y desplazamiento de cada joint.

## 13. Limitaciones

No hay medición metrológica del robot físico ni identificación dinámica. Las cajas de servo son aproximaciones visuales. La autocolisión del modelo permanece desactivada; las colisiones se validaron por simplicidad, bounds y movimiento sin bloqueo. No se generó captura 3D porque la validación disponible fue headless. El soporte de cámara Poppy está auditado, pero el sensor frontal existente se conserva para no cambiar el experimento.
