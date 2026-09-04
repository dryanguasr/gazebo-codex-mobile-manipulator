# Troubleshooting de importación CAD y ensamblaje mecánico

Este documento separa hechos observados durante la integración de Poppy de problemas comunes que no aparecieron en esta máquina. Cada incidencia incluye síntoma, causa, diagnóstico, corrección y gate de cierre.

## Incidencias observadas

### STL descargado como puntero Git LFS

**Síntoma:** el supuesto STL pesa pocos cientos de bytes y comienza con version https://git-lfs.github.com/spec/v1.

**Causa:** el clone obtuvo el puntero, no el objeto LFS.

**Diagnóstico:** compare wc -c, el campo size y el OID del puntero.

**Corrección:** ejecute git lfs pull o descargue la ruta desde media.githubusercontent.com/media/repo/commit/ruta.

**Validación:** SHA-256 igual al OID y el inspector debe leer triángulos.

### El mesh aparece 1000 veces mayor

**Síntoma:** un bracket de centímetros mide 34 x 20 x 45 en Gazebo.

**Causa observada:** los STL oficiales usan magnitudes de milímetros aunque los STEP declaran metros.

**Diagnóstico:** contraste LENGTH_UNIT/CARTESIAN_POINT del STEP, bounds STL y una cota conocida.

**Corrección:** aplique 0.001 solo a esos STL; no vuelva a escalar el derivado ya escrito en metros.

**Validación:** bounds físicos en metros y ausencia de scale distinto de 1:1 en Xacro.

### El auditor informó non-manifold por redondeo

**Síntoma:** un mesh originalmente cerrado parecía tener aristas inválidas.

**Causa:** redondear coordenadas métricas a 1e-6 fusionó detalles geométricos distintos.

**Corrección:** use 1e-9 para identidad topológica y una tolerancia distinta para generar hulls.

**Validación:** el hull es watertight y no contiene boundary edges.

### Xacro no encuentra ROS o el paquete

**Síntoma:** PackageNotFoundError al expandir.

**Causa:** no se cargó /opt/ros/jazzy/setup.bash o install/setup.bash.

**Corrección:**

~~~bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
~~~

**Validación:** xacro y check_urdf pasan, y ros2 pkg prefix localiza mobile_manipulator.

### Spawn y control pasan, pero Gazebo no carga CAD

**Síntoma:** joint_states funciona y el log contiene Unable to find file with URI model:// o Failed to load geometry.

**Causa:** Gazebo Sim 8 reinterpretó package:// sin un resource path compatible.

**Corrección:** use file://$(find mobile_manipulator)/meshes/... y asegure que setup.py instala visual/collision.

**Validación:** el URDF expandido apunta a archivos existentes y el diagnóstico rechaza cualquier [Err] de carga.

### Joints responden, pero el robot parece desarmado

**Síntoma:** los seis joints alcanzan consignas, pero brackets no coinciden, hay piezas flotantes o las articulaciones giran fuera del horn.

**Causa observada:** origins/ejes formaban una cadena matemática válida, pero encadenaban m2–m6 casi solo sobre Z e ignoraban rotaciones de 90° y desplazamientos X/Y de la mecánica. Además, varios meshes seguían en el frame CAD equivocado.

**Por qué pasó:** check_urdf valida sintaxis/topología; ros2_control y joint_states validan interfaces/estado. Ninguno demuestra que el frame corresponda al pivote físico.

**Diagnóstico:** audite joint por joint y cuerpo rígido por cuerpo rígido usando simultáneamente CAD, guía de montaje y URDF oficial. Compare centros de horns, ejes, landmarks, FK y una vista cercana en varias poses.

**Corrección final:** el registro autónomo alineó las piezas impresas, pero no
pudo demostrar contacto motor/horn porque esos sólidos no existen en el CAD
propio. El gate quedó en FAIL y se activó B3: joints, visual origins, tip frames
y siete DAE exactos del commit oficial fijado.

**Validación:** `validate_official_consolidation.py` compara 6 joints, 7
visuales y 35 FK; el diagnóstico compara tip/tool con TF. La captura del fallo
permanece en `captures/cad_import/robot_pose_a_isometric.png`.

**Lección:** un URDF válido y joints funcionales pueden representar un ensamblaje mecánicamente falso.

### La base hacía parecer diminuto al brazo 1:1

**Síntoma:** Poppy parecía un microbrazo sobre un vehículo sobredimensionado.

**Causa:** base .72 x .52 m y ruedas de .115 m frente a un Ergo Jr real de unos decímetros.

**Corrección:** se mantuvo Poppy 1:1 y se eligió base .40 x .30 x .10 m, rueda .070 x .045 m y separación .345 m; se actualizaron masa, inercias, ruedas, odometría, cámara y mount como conjunto.

**Validación:** capturas del robot compacto, movimiento lineal, odom/TF y experimento A/B.

### Gmsh no estaba en PATH

**Síntoma:** check_cad_dependencies.py marca gmsh ausente.

**Causa:** Gmsh no es dependencia transitiva de ROS; solo lo requiere la ruta STEP.

**Corrección reproducible:** `sudo apt update && sudo apt install gmsh`.
`package.xml` declara ahora `gmsh`. En esta sesión se reutilizó una
distribución local explícita 4.12.1.

**Incidencia adicional observada:** el binario local se encontró, pero falló con
`libgmsh.so.4.12: cannot open shared object file`. Se declaró explícitamente
su directorio en `LD_LIBRARY_PATH`; una instalación apt no requiere ese paso.

### STEP en metros, salida Gmsh con magnitud mm

**Síntoma:** base.step produce bounds cercanos a 150 en vez de .150 m.

**Causa observada:** interpretación de unidades de este AP214 por Gmsh 4.12.1.

**Corrección:** convert_step_example.py tessella primero y aplica 0.001 al output, registrándolo en summary.json. No replique esa escala sin medir otro STEP.

### rosdep local no estaba inicializado

**Síntoma:** rosdep install informa que primero deben ejecutarse sudo rosdep init y rosdep update.

**Causa observada:** la base rosdep del host WSL no había sido inicializada; sudo no disponía de credencial no interactiva en esta sesión.

**Tratamiento:** no se presentó el comando como PASS. `package.xml` declara
NumPy, SciPy, OpenCV, Matplotlib, Pillow y Gmsh. En un host nuevo, inicialice
`rosdep` una vez y repita la instalación antes de preparar assets.

**Limitación:** esta sesión verificó claves y módulos presentes, pero no una instalación rosdep desde cero.


### Tessellation coarse incumple tolerancia

**Síntoma:** clscale 1.0 difiere 1.516719 mm de la referencia.

**Corrección:** clscale 0.5 produce 26 330 triángulos y error máximo de extensión de 0.000017 mm. Coarse se conserva como ejemplo docente; fine es la variante aceptada.

### Gmsh avisó elementos inválidos

**Síntoma:** 12 elementos inválidos en dos superficies, con finalización de 0 errores.

**Tratamiento:** el warning no se ocultó. Se exigieron watertightness, bounds, orientación y volumen; fine pasó. En otro STEP, un warning combinado con geometría abierta o fuera de tolerancia debe ser FAIL.

### Gazebo quedó huérfano al cerrar una corrida A/B

**Síntoma:** la siguiente corrida detectó un servidor Gazebo previo pese a que el launcher había terminado.

**Causa:** el proceso hijo ignoró TERM/INT durante el shutdown.

**Corrección:** el runner espera un intervalo acotado, vuelve a comprobar un patrón estrecho del world_file y usa KILL solo sobre ese servidor si persiste.

**Validación:** corrida A/B completa posterior sin procesos residuales y comparador PASS.

### GUI abre, pero no se pudo capturar el overlay nativo

**Síntoma:** WSLg mostró Gazebo, pero el helper de automatización falló con helper_unknown_error: setup refresh had errors; una captura X11 resultó negra.

**Corrección alternativa final:** `render_mechanical_assembly.py` lee los DAE
y transforms exactos, añade collisions cian y genera 17 vistas offscreen.

**Limitación:** esta evidencia comprueba alineación geométrica, pero no se presenta como captura del overlay interactivo nativo.

## Problemas comunes adicionales no observados

### Piezas impresas alinean, pero el ensamblaje autónomo sigue en FAIL

**Síntoma:** diez registros tienen residuales de landmarks < 0,067 mm, pero el
gate global no pasa.

**Causa observada:** el CAD propio contiene brackets y pinza, no sólidos
autoritativos completos de XL-320, horns y tornillería. Una caja aproximada no
prueba el contacto entre superficies.

**Diagnóstico:** separar “componente disponible alineado” de “ensamblaje
completo demostrado”. Revisar `decision.json`, no solo la tabla de piezas.

**Corrección:** activar B3 y usar los visuales oficiales fijados, conservando
CAD-derived y manifest como material docente.

**Validación:** comparación oficial/final con error posicional 0 y residual
angular máximo numérico de 2,98e-8 rad.

### Dos README colisionan durante la instalación

**Síntoma:** `colcon build --symlink-install` falla con `Errno 17 File exists`
al instalar `README.md`.

**Causa observada:** README de hardware y README oficial se aplanaban en un
mismo directorio `licenses/`.

**Corrección:** instalar por separado en `licenses/hardware/` y
`licenses/official/`.

**Validación:** build PASS; los dos conjuntos y sus licencias quedan instalados,
mientras `source/` no se copia.

### Las etiquetas open/closed estaban invertidas

**Síntoma:** las capturas nominales no correspondían al movimiento físico de la
mordaza.

**Causa observada:** se asumió la semántica del valor articular sin medir las
caras del DAE.

**Corrección:** fijar `m6=0 rad` como cerrado (gap ≈ 2,1 mm) y
`m6=1,20 rad` como abierto; regenerar tabla, FK y capturas.

**Lección:** nombres y límites no sustituyen una inspección geométrica.

### Evidencia offscreen no es una captura GUI

**Síntoma:** el helper `view_image`/automatización de ventana falló sobre WSL
con `helper_unknown_error`.

**Tratamiento:** se inspeccionaron montajes temporales de los PNG y se conservó
un render reproducible que consume los assets/runtime transforms exactos.

**Limitación:** esta evidencia demuestra geometría evaluada, no interacción con
el overlay nativo de la GUI. Gazebo se validó aparte mediante el diagnóstico.

### FreeCAD o su módulo Python no está disponible

La GUI puede existir aunque import FreeCAD falle desde Python del sistema. Use FreeCAD CLI/su intérprete o el flujo Gmsh documentado; registre versión y PYTHONPATH, sin mezclar entornos silenciosamente.

### STEP no abre o contiene múltiples solids

Revise versión AP, unidades y reparación del B-rep. Exporte explícitamente cada sólido/cuerpo rígido que corresponda a un link; no fusione piezas que deban moverse entre sí.

### Tessellation excesivamente densa o gruesa

Una malla densa aumenta disco y coste; una gruesa pierde curvas, agujeros y bounds. Genere al menos dos resoluciones y compare triángulos, tamaño, cotas y apariencia. La resolución de visual y la de collision son decisiones distintas.

### Mesh no encontrado después de instalar

La ruta puede existir en src pero faltar en install/share. Revise setup.py/CMake, reconstruya y resuelva la URI desde el URDF expandido. source/ no se instala por diseño; visual/, collision/, manifest y licencia sí.

### Normales invertidas

Caras que desaparecen o iluminación interior indican winding inconsistente. Calcule normales y volumen firmado, corrija el orden de vértices y renderice desde ambos lados.

### Materiales o texturas ausentes

En DAE/OBJ, revise rutas relativas, MTL y archivos instalados. Pruebe desde install, no solo desde src.

### Collision demasiado compleja

Un visual de decenas de miles de triángulos usado como collider reduce el real-time factor. Use primitivas, hulls o descomposición/simplificación. Compare ratio; Poppy link 6 usa 92/32168.

### Collider flotante o joint bloqueado

Active collisions, revise origin y cuerpo rígido. Un collider puede estar alineado en el mesh local y mal ubicado por el joint. Pruebe joints de uno en uno y no habilite autocolisión hasta entender solapes vecinos.

### Pieza desplazada o flotante

Busque una transformación aplicada tanto durante export como en visual/origin, o una pieza asignada al link incorrecto. Mantenga una sola fuente de verdad para cada reframe y valide landmarks.

### Joint invertido

Ordene una variación pequeña positiva y observe TF, no solo joint_states. Corrija el signo del axis o el frame completo según la mecánica; documente la convención.

### Masa o inercia inválida

Masa cero, unidades kg·mm² o tensor no positivo definido pueden desestabilizar Gazebo. Calcule en SI, verifique eigenvalores/desigualdades triangulares y ubique el centro de masa dentro de una envolvente plausible.

### Diferencias de ejes entre CAD y Gazebo

Z-up/Y-up y transforms de exportación pueden rotar el modelo. Use un cubo o landmarks de referencia y una matriz explícita; compare bounds y ejes en todas las herramientas.

## Checklist de diagnóstico

1. ¿El archivo es el objeto real y su hash coincide?
2. ¿Las dimensiones corresponden a una cota conocida en metros?
3. ¿Visual y collision están separados?
4. ¿Cada pieza pertenece al cuerpo rígido correcto?
5. ¿Cada joint coincide con el centro y eje físico del horn?
6. ¿FK independiente coincide con TF para varias poses?
7. ¿Home y dos poses se ven ensamblados?
8. ¿Collisions están alineadas y no flotan?
9. ¿Masa/inercia y odometría usan la misma geometría?
10. ¿Cámara, detector, tracking y A/B siguen pasando?
