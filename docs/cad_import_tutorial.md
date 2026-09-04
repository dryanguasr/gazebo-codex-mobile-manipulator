# Tutorial generalizable: de CAD o mesh a un robot mecánicamente válido en Gazebo

## Objetivo

Este tutorial cubre dos entradas habituales:

- Ruta 1: el fabricante ya entrega STL/OBJ/DAE.
- Ruta 2: solo existe CAD STEP/B-rep y hay que tessellarlo de forma reproducible.

Ambas convergen en la misma cadena:

CAD/B-rep → tessellation → mesh visual → collision simplificada → cuerpos rígidos y frames → masa/inercia → URDF/Xacro → instalación ROS → Gazebo → validación.

El Poppy Ergo Jr integrado aquí es el caso de estudio. Sus meshes proceden del CAD oficial, permanecen a escala física 1:1 y se montan sobre una base 4WD compacta. No se implementan MoveIt, IK, pick-and-place, Nav2 ni SLAM.

## 1. Fijar y atribuir la fuente

La fuente mecánica Poppy es poppy-project/poppy-ergo-jr, commit 97ce599be8c717843c45ebf48341f2ebf8f250b3. El inventario, rutas y OID LFS están en third_party/poppy_ergo_jr/README.md.

~~~bash
git clone https://github.com/poppy-project/poppy-ergo-jr.git /tmp/poppy-ergo-jr
git -C /tmp/poppy-ergo-jr checkout 97ce599be8c717843c45ebf48341f2ebf8f250b3
git -C /tmp/poppy-ergo-jr lfs pull --include="hardware/STEP/**,hardware/STL/**"
~~~

Si Git LFS no está disponible, descargue cada objeto por su ruta en media.githubusercontent.com y verifique SHA-256 contra el OID. No trabaje con el pequeño puntero textual como si fuera un STL.

El código del repositorio es Apache-2.0. El hardware Poppy y sus derivados CAD conservan CC BY-SA 4.0. La licencia y atribución acompañan a los assets instalados.

## 2. Preflight reproducible

En una instalación ROS limpia:

~~~bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 scripts/cad/check_cad_dependencies.py
~~~

package.xml declara python3-numpy y python3-scipy, requeridos por preparación y validación. La ruta STEP usa además Gmsh/OpenCASCADE:

~~~bash
sudo apt update
sudo apt install gmsh
~~~

No presuponga una herramienta porque esté casualmente en PATH. El preflight identifica cada módulo/binario ausente y su instalación.

## 3. Ruta 1 — ya dispongo de STL/OBJ/DAE

### 3.1 Inspeccionar antes de importar

Registre para cada mesh:

- formato y tamaño;
- número de triángulos;
- bounding box y dimensiones XYZ;
- unidades asumidas y una cota física conocida;
- origen y orientación de ejes;
- watertightness, normales y volumen si son fiables;
- hash y licencia.

STL no almacena unidades. En este Poppy los STEP declaran metros, pero los STL oficiales contienen coordenadas en milímetros. Por ello prepare_poppy_assets.py aplica 0.001 a esos STL y escribe los derivados en metros.

### 3.2 Elegir escala y origen

Gazebo/URDF usa SI. Compare una longitud conocida antes de decidir la escala; no aplique 0.001 por costumbre. El frame de un link debe ubicarse según la mecánica —normalmente en un eje de joint o interfaz de montaje—, no necesariamente en el centro del bounding box.

Registre cada transformación como matriz o rotación/traslación en un script. Evite correcciones manuales opacas en una GUI y evite aplicar la misma traslación tanto al mesh como a visual/origin.

### 3.3 Separar visual y collision

El visual puede conservar detalle, materiales y curvas. Collision debe representar contacto con el menor coste razonable:

1. primitivas box/cylinder/sphere para cuerpos sencillos;
2. convex hull para envolventes robustas;
3. mesh simplificado cuando la forma de contacto no cabe en una primitiva o un único hull.

Poppy usa boxes en links intermedios, hull convexo en mount y link 6, y mesh detallado solo para visual. La mordaza móvil ilustra el coste: 32 168 triángulos visuales frente a 92 de collision. No reutilice automáticamente el visual como collision.

### 3.4 Preparar los assets

~~~bash
python3 scripts/cad/prepare_poppy_assets.py
python3 scripts/cad/validate_meshes.py
~~~

El script verifica hashes, convierte unidades, reexpresa cada pieza en su frame físico, combina los cuerpos rígidos, genera hulls y actualiza asset_manifest.json. El validador explica qué comprobó y falla ante archivos ausentes, escala absurda, bounds incoherentes, collision no simplificada, masa/inercia inválidas o URI rotas.

## 4. Ruta 2 — parto de CAD STEP/B-rep

### 4.1 Qué es tessellation

STEP describe sólidos y superficies B-rep; no es una lista de triángulos y Gazebo no lo usa directamente como mesh. Tessellation aproxima esas superficies mediante triángulos.

La resolución es una decisión:

- menor tolerancia o menor tamaño objetivo → más detalle y triángulos, mayor disco/coste;
- mayor tolerancia o mayor tamaño objetivo → menos triángulos y coste, con riesgo de perder curvaturas, agujeros o bounds.

Genere al menos dos resoluciones para una pieza representativa y comparelas.

### 4.2 Conversión real del ejemplo

Este repositorio convierte source/hardware/STEP/base.step sin utilizar el STL oficial como entrada:

~~~bash
python3 scripts/cad/convert_step_example.py
~~~

Conceptualmente ejecuta:

~~~bash
gmsh source/hardware/STEP/base.step -2 -clscale 0.5 -format stl -o base_raw.stl
~~~

El script registra input/hash, herramienta y versión, parámetros de tessellation, formato, unidades, transformaciones y ruta de salida. En esta combinación concreta de STEP AP214 y Gmsh 4.12.1, Gmsh expuso coordenadas con magnitud de milímetros aunque el STEP declara metros; el script aplica 0.001 después de tessellar. Es un hallazgo específico, no una regla para otros CAD.

| Variante | clscale | Triángulos | Máximo error de extensión vs STL oficial | Estado |
|---|---:|---:|---:|---|
| coarse | 1.0 | 23 482 | 1.516719 mm | FAIL didáctico |
| fine | 0.5 | 26 330 | 0.000017 mm | PASS |

El mesh fine mide 0.150 x 0.150 x 0.0342 m, es watertight y difiere 0.313 % en volumen respecto a la referencia. La aceptación permite hasta 1 mm por extensión y 2 % por volumen.

### 4.3 Validar sin exigir triangulación idéntica

El STL oficial se abre solo después de convertir y funciona como referencia. Compare:

- unidades y dimensiones XYZ;
- min/max del bounding box y origen;
- orientación/signo de volumen;
- volumen, si ambos meshes son cerrados o la política está justificada;
- triángulos y coherencia visual.

No exija igualdad byte a byte: dos tessellators pueden describir la misma superficie con triángulos distintos. El resultado auditado está en results/verified/cad_step_conversion/summary.json.

Para otro STEP, compruebe si contiene múltiples sólidos. Separe cada cuerpo que deba moverse con un joint distinto; no fusione mecánicamente un ensamblaje por conveniencia del exportador.

## 5. Formar links: la pregunta mecánica

Antes de escribir joints, pregunte para cada pieza:

¿Qué permanece rígidamente unido cuando este motor gira?

En Poppy:

| Link | Cuerpo rígido |
|---|---|
| poppy_mount_link | base impresa + estator m1 |
| poppy_link_1 | long_U + cuerpo m2 |
| poppy_link_2 | par lateral + cuerpo m3 |
| poppy_link_3 | short_U + cuerpo m4 |
| poppy_link_4 | segundo par lateral + cuerpo m5 |
| poppy_link_5 | fijación, cuerpo m6 y mordaza fija |
| poppy_link_6 | mordaza móvil |

Un mesh bien orientado asignado al link incorrecto sigue siendo un fallo.

Los STL Poppy se reexpresan en los frames de link: Ry(+90°) para links 2/3, Rx(+90°) para link 4, Ry(-90°) @ Rz(180°) y traslación al pivote para link 5, y Rz(180°) para link 6. Estas decisiones quedan en el manifest.

## 6. Auditar frames y joints

Use conjuntamente CAD, guía mecánica y una referencia cinemática autoritativa. Para Poppy se auditó el URDF oficial commit 7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b, sin copiar sus meshes.

| Joint | origin xyz (m) | origin rpy (rad) | axis |
|---|---|---|---|
| m1 | 0 0 .0327993216 | 0 0 0 | 0 0 1 |
| m2 | 0 0 .0240006784 | 0 -pi/2 0 | 0 0 -1 |
| m3 | .054 0 0 | 0 0 0 | 0 0 -1 |
| m4 | .045 0 0 | 0 -pi/2 0 | 0 0 -1 |
| m5 | 0 -.048 0 | 0 -pi/2 0 | 0 0 1 |
| m6 | 0 -.058 0 | 0 -pi/2 0 | 0 0 -1 |

La cadena anterior usaba casi solo offsets Z y aun así pasaba check_urdf y ros2_control. La explicación y tabla comparativa completa están en [mechanical_assembly_validation.md](mechanical_assembly_validation.md).

## 7. Masa e inercia

Use masa positiva, centro de masa dentro de una envolvente plausible y tensor positivo definido que cumpla desigualdades triangulares. Para una caja homogénea:

~~~text
Ixx = m (y² + z²) / 12
Iyy = m (x² + z²) / 12
Izz = m (x² + y²) / 12
~~~

Los links Poppy combinan volumen PLA aproximado con 16.7 g por XL-320. Es una hipótesis docente, no identificación dinámica. Las cajas de servo son aproximaciones visuales basadas en dimensiones del fabricante.

## 8. Integración Xacro e instalación ROS

Las dimensiones principales de la base son propiedades Xacro. La configuración elegida conserva Poppy 1:1 y reduce el vehículo:

- base 0.40 x 0.30 x 0.10 m, 6.0 kg;
- rueda radio 0.070 m, ancho 0.045 m;
- wheel separation 0.345 m;
- cámara x=0.225 m, z=0.050 m;
- mount de brazo x=-0.030 m, z=0.050 m.

controllers.yaml usa el mismo radio y separación que la geometría. La inercia, posiciones de ruedas, odometría, cámara y mount se actualizaron junto con el visual/collision.

Las URI usan file://$(find mobile_manipulator)/meshes/... porque Gazebo Sim 8 no resolvió package:// en esta integración. El URDF expandido debe apuntar a un archivo instalado existente.

SOURCE ASSETS y RUNTIME ASSETS tienen propósitos distintos:

- source/ conserva STEP, STL oficial y derivados didácticos en Git;
- visual/, collision/, asset_manifest.json y atribución son lo que Gazebo necesita.

setup.py instala solo los recursos de runtime y licencia, no los STEP pesados.

## 9. Validación en cuatro niveles

### 9.1 Sintaxis

~~~bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
xacro src/mobile_manipulator/urdf/mobile_manipulator.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
~~~

### 9.2 Cinemática y control

Compruebe los seis joints, controladores activos, dos poses con signos distintos y error contra consigna. Añada landmarks de punta y compare FK independiente con TF; joint_states no detecta por sí solo un frame físicamente equivocado.

~~~bash
python3 scripts/cad/validate_mechanical_assembly.py
bash scripts/run_diagnostic.sh
~~~

Para las dos poses, FK y TF de poppy_moving_tip concuerdan dentro de 0.48 mm y 0.00079 de distancia cuaternión.

### 9.3 Geometría mecánica

Inspeccione home, dos poses, tres joints consecutivos y gripper en dos posiciones. Compruebe brackets unidos, servos dentro de soportes, horns/ejes plausibles, continuidad, ausencia de gaps y ausencia de interpenetraciones groseras. Compare la topología con CAD y guía oficial.

Las capturas están en captures/mechanical_assembly/. El fallo anterior se conserva en captures/cad_import/robot_pose_a_isometric.png como ejemplo de falso positivo.

### 9.4 Física y simulación

Active collisions o genere el preview reproducible:

~~~bash
python3 scripts/cad/generate_collision_preview.py --mode overlay
python3 scripts/cad/generate_collision_preview.py --mode collision-only
~~~

Compruebe alineación, masa/inercia, movimiento sin bloqueo, odom/TF, cámara, detector y experimento completo. La autocolisión permanece deshabilitada; no aleje artificialmente piezas reales para evitar contacto entre vecinos.

## 10. Regresión completa

Orden recomendado:

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

Estado validado: 7 tests sin fallos; CAD/STEP/mecánica PASS; diagnóstico PASS; odometría y TF coherentes; cámara 640x480; detector activo. El experimento A/B mantuvo 100 % de detección y el tracking redujo el MAE objetivo de 0.528910 m a 0.149327 m, mejora 71.77 %.

## 11. Evidencia y límites

Las capturas de poses proceden de cámaras reales de Gazebo. Gazebo GUI abrió bajo WSLg, pero el helper de automatización no pudo capturar de forma verificable el overlay nativo; se generó una representación alternativa estática, reproducible y explícitamente etiquetada. No equivale a interacción GUI directa.

No hubo metrología física ni identificación dinámica. No se modelan cables/tornillería y la autocolisión está fuera de alcance. Estas limitaciones no invalidan la cadena geométrica, pero delimitan lo demostrado.

## 12. Regla de aceptación

No declare PASS porque el XML parsea o porque el joint se mueve. Exija coherencia simultánea en:

1. sintaxis;
2. cinemática/control;
3. geometría mecánica;
4. física/simulación.

Solo entonces el procedimiento es reutilizable para otro CAD.
