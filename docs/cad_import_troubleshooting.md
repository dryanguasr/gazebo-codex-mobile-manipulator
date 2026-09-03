# Troubleshooting de importación CAD

Los primeros casos son incidencias observadas durante este hito. La segunda sección reúne fallos previsibles que no se observaron aquí.

## Incidencias observadas

### El archivo STL mide solo unos cientos de bytes

**Síntoma:** `head pieza.stl` muestra `version https://git-lfs.github.com/spec/v1`.

**Posibles causas:** el clone descargó el puntero Git LFS y no el objeto.

**Cómo inspeccionar:** compare `wc -c`, `git show HEAD:ruta` y el campo `size` del puntero.

**Corrección:** ejecute `git lfs pull` o descargue la ruta desde `media.githubusercontent.com/media/<repo>/<commit>/<ruta>`.

**Cómo validar:** `sha256sum` debe coincidir con el OID LFS; `inspect_poppy_meshes.py` debe leer triángulos.

### El mesh aparece 1000 veces mayor

**Síntoma:** un bracket de centímetros reporta extensiones 34 x 20 x 45 en URDF/Gazebo.

**Posibles causas:** el STL usa milímetros aunque el STEP usa metros.

**Cómo inspeccionar:** revise `CARTESIAN_POINT` y `LENGTH_UNIT` del STEP; compare con bounds del STL.

**Corrección:** aplique 0.001 solo al STL. No aplique otra escala al derivado ya escrito en metros.

**Cómo validar:** `validate_meshes.py` exige extensiones entre 5 mm y 200 mm.

### Un mesh derivado parece no manifold aunque el original era watertight

**Síntoma:** el primer auditor informó aristas nonmanifold en la base convertida.

**Posibles causas:** redondear coordenadas en metros a 1e-6 fusionó detalles distintos de una micra.

**Cómo inspeccionar:** ejecute el auditor con distintas precisiones y compare número de vértices.

**Corrección:** use 1e-9 para identidad topológica en metros y una tolerancia separada para convex hull.

**Cómo validar:** el hull debe ser watertight y no tener boundary edges.

### Xacro falla con PackageNotFoundError

**Síntoma:** `xacro` no encuentra sus metadatos o `mobile_manipulator`.

**Posibles causas:** no se cargó ROS o el workspace instalado.

**Cómo inspeccionar:** examine `PYTHONPATH`, `AMENT_PREFIX_PATH` y `ros2 pkg prefix mobile_manipulator`.

**Corrección:**

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**Cómo validar:** `xacro ... | check_urdf /dev/stdin` o genere un archivo intermedio.

### Xacro y spawn pasan, pero no se ve el CAD

**Síntoma:** controladores y joint states funcionan; el log dice `Unable to find file with URI model://...` y `Failed to load geometry`.

**Posibles causas:** Gazebo convirtió `package://` a `model://` sin un resource path compatible.

**Cómo inspeccionar:** busque `[Err]`, `SystemPaths`, `MeshManager` y `SceneManager` en `launch.log`.

**Corrección:** use `file://$(find mobile_manipulator)/meshes/...` y asegure que `setup.py` instala los archivos.

**Cómo validar:** el URDF expandido debe contener una ruta absoluta existente y `run_diagnostic.sh` debe pasar su gate de log.

### El joint gira, pero el eje no coincide con la mecánica

**Síntoma:** el primer ensamblaje tenía offsets plausibles y control correcto, pero los centros no coincidían con horns.

**Posibles causas:** se usó el tamaño total del bracket como distancia entre ejes; el origen STL era de ensamblaje.

**Cómo inspeccionar:** compare extremos STEP/STL, centros circulares, guía de montaje y transform acumulado.

**Corrección:** vuelva a medir el centro del eje. En este modelo se corrigieron a 32.8, 24, 54, 45, 48 y 58 mm.

**Cómo validar:** dos poses con signos opuestos deben llegar numéricamente y los TF deben formar una cadena continua.

## Problemas comunes adicionales no observados

### Mesh no encontrado después de instalar

**Síntoma:** la ruta existe en `src/` pero no bajo `install/.../share/<paquete>`.

**Posibles causas:** `setup.py` o CMake no incluye subdirectorios; se añadió el archivo después del build.

**Cómo inspeccionar:** `find install/<paquete>/share/<paquete>/meshes -type f -o -type l`.

**Corrección:** instale cada directorio de assets y reconstruya.

**Cómo validar:** resuelva la URI del URDF expandido y ejecute `test -e`.

### Formato no soportado

**Síntoma:** CAD abre en el modelador, pero Gazebo no carga STEP.

**Posibles causas:** URDF/Gazebo espera una malla, no un B-rep STEP.

**Cómo inspeccionar:** revise extensión y log de MeshManager.

**Corrección:** triangule a STL/DAE/OBJ con tolerancia documentada.

**Cómo validar:** registre bounds y conteo de triángulos antes de integrar.

### Normales invertidas

**Síntoma:** caras desaparecen o la iluminación parece interior.

**Posibles causas:** winding inconsistente tras rotar o combinar triángulos.

**Cómo inspeccionar:** calcule `cross(v1-v0, v2-v0)` y volumen firmado.

**Corrección:** oriente faces hacia fuera y regenere normales.

**Cómo validar:** render desde ambos lados y compruebe volumen firmado consistente.

### Materiales o texturas ausentes

**Síntoma:** DAE/OBJ aparece blanco.

**Posibles causas:** URI de textura relativa rota, MTL no instalado o material URDF sobrescrito.

**Cómo inspeccionar:** abra DAE/MTL como texto y resuelva cada ruta desde el share instalado.

**Corrección:** instale texturas conservando jerarquía o use material uniforme explícito.

**Cómo validar:** pruebe desde `install/`, no solo desde `src/`.

### Collision mesh demasiado complejo y Gazebo lento

**Síntoma:** baja el real-time factor al mover el brazo.

**Posibles causas:** se reutilizó el visual de decenas de miles de triángulos como collider.

**Cómo inspeccionar:** compare conteos en manifest y active visualización de colisiones.

**Corrección:** use primitivas, hull voxelizado o varios hulls convexos.

**Cómo validar:** mida el ratio; link 6 usa 92/32168.

### Autocolisiones o contactos espurios

**Síntoma:** el brazo vibra o no alcanza una pose.

**Posibles causas:** colliders adyacentes se solapan, origen equivocado o autocolisión habilitada.

**Cómo inspeccionar:** muestre collisions, consulte contactos y pruebe joints uno por uno.

**Corrección:** reduzca envelopes, separe colliders vecinos y defina política de self-collision explícita.

**Cómo validar:** ejecute dos poses y compruebe error final, velocidad y contactos.

### Link desplazado o pieza flotante

**Síntoma:** una pieza está separada aunque el joint state es correcto.

**Posibles causas:** translation aplicada en el mesh y otra vez en `<visual><origin>`.

**Cómo inspeccionar:** compute bounds en frame local y TF acumulado.

**Corrección:** elija un solo lugar para cada corrección; este pipeline hornea escala/reframe y deja origin visual cero.

**Cómo validar:** compare min/max del asset con origen del joint hijo.

### Joint invertido

**Síntoma:** llega al valor negativo cuando se ordena positivo.

**Posibles causas:** axis con signo contrario o frame rotado.

**Cómo inspeccionar:** ordene una pose pequeña y compare `/joint_states`.

**Corrección:** cambie el signo de `axis` o redefina el frame, documentando la convención.

**Cómo validar:** use poses con signos alternos; no basta que el controller esté activo.

### Piezas flotantes en un link compuesto

**Síntoma:** una mitad del bracket se mueve con el link equivocado.

**Posibles causas:** clasificación rígida incorrecta.

**Cómo inspeccionar:** pregunte qué piezas están unidas al rotor y cuáles al estator de cada servo.

**Corrección:** mueva cada mesh al link del cuerpo rígido correcto. La mordaza fija pertenece a link 5 y la rotativa a link 6.

**Cómo validar:** anime solo m6.

### Inercia inválida o masa irreal

**Síntoma:** Gazebo ignora el link, lanza warnings o la dinámica explota.

**Posibles causas:** masa cero, tensor negativo o unidades kg·mm² usadas como kg·m².

**Cómo inspeccionar:** eigenvalores, desigualdades triangulares y centro de masa.

**Corrección:** recalcule en SI con una envolvente simple.

**Cómo validar:** `validate_meshes.py` y log sin warnings críticos.

### Xacro válido, robot visualmente incorrecto

**Síntoma:** XML y árbol pasan, pero escala/orientación es absurda.

**Posibles causas:** los validadores sintácticos no interpretan apariencia.

**Cómo inspeccionar:** bounds, TF, ejes y log de carga; use GUI cuando esté disponible.

**Corrección:** valide cada link aislado antes del ensamblaje.

**Cómo validar:** combine inspección visual con dos poses numéricas.

### Diferencias Blender/CAD/Gazebo

**Síntoma:** la pieza cambia de eje vertical o escala entre herramientas.

**Posibles causas:** Z-up/Y-up, unidades de escena y export transforms.

**Cómo inspeccionar:** exporte un cubo de referencia y registre matriz de transformación.

**Corrección:** aplique una matriz explícita en script, no una rotación manual sin registro.

**Cómo validar:** bounds y landmarks deben coincidir en los tres entornos.

## Incidencias observadas al cerrar la ruta STEP → mesh

### Gmsh no estaba instalado como herramienta del sistema

**Síntoma:** `scripts/cad/check_cad_dependencies.py` informa que `gmsh` no está en `PATH`.

**Causa observada:** Gmsh no es una dependencia transitiva de ROS ni de `rosdep`; es una herramienta CAD opcional para la ruta 2.

**Corrección reproducible:** `sudo apt update && sudo apt install gmsh`. El preflight mantiene NumPy/SciPy como obligatorios para los scripts actuales y marca Gmsh como requerido solo al convertir STEP.

### El STEP AP214 declara metros, pero Gmsh 4.12.1 expuso coordenadas con magnitud de mm

**Síntoma:** `base.step` produce bounds cercanos a 150 unidades, cuando el ensamblaje físico mide cerca de 0.15 m.

**Causa observada:** diferencia de interpretación/unidades entre este archivo AP214 y Gmsh 4.12.1.

**Corrección:** `convert_step_example.py` tessella primero y aplica escala explícita `0.001` al STL de salida; registra esa decisión, bounds y tolerancias en `summary.json`. No copie esta escala a otro STEP sin comprobar una cota conocida.

### Tessellation coarse no cumple la tolerancia geométrica

**Síntoma:** con `-clscale 1.0`, la mayor diferencia de extensión frente al STL de referencia es 1.52 mm.

**Corrección:** para este ejemplo se usa `-clscale 0.5`, que produce 26 330 triángulos y una diferencia máxima de 0.000000017 m. La variante coarse se conserva como demostración de la decisión de resolución, no como mesh de runtime.

### Warnings de elementos inválidos al tessellar

**Síntoma:** Gmsh informó 12 elementos inválidos en dos superficies, aunque finalizó con `0 errors`.

**Corrección y validación:** no se ocultó el warning: queda en el reporte. El mesh fine fue watertight, conservó bounds/orientación y pasó volumen/tolerancias. En otro CAD, warnings o un mesh no watertight deben investigarse antes de integrarlo.

## Problemas comunes adicionales no observados en este cierre

### FreeCAD o su módulo Python no está disponible

Instale `freecad` si su distribución lo ofrece o prefiera el flujo Gmsh documentado. `import FreeCAD` desde el Python del sistema puede fallar aunque la GUI exista, porque FreeCAD usa su propio intérprete/módulos; ejecute su CLI o documente `PYTHONPATH` en vez de mezclar intérpretes silenciosamente.

### STEP no abre o contiene múltiples sólidos

Compruebe la versión AP, unidades y reparación del B-rep en una herramienta CAD. Para múltiples sólidos, exporte cada link rígido o el ensamblaje elegido de manera explícita; no deje que el exportador combine piezas que deben moverse con joints distintos.

### Tessellation demasiado densa o demasiado gruesa

Una malla densa aumenta disco, carga y coste de collision; una gruesa pierde curvatura/bordes. Genere al menos dos resoluciones, compare triángulos, tamaño y bounds frente a cotas conocidas, y mantenga el visual y collision como decisiones separadas.

### Mesh desplazado, eje distinto o rutas rotas tras instalar

Ponga el frame del link sobre el eje de joint y registre la matriz de corrección. Tras `colcon build --symlink-install`, inspeccione el share instalado y pruebe las URI Xacro/Gazebo; los CAD de `source/` no se instalan por diseño, pero visual/collision, manifest y atribución sí.
