# Ensamblaje y colores de inspección — 2026-09-09

## Qué ocurría en el segundo 23

La separación visual reportada era real, aunque las transformaciones de los
joints estuvieran bien. Con gz-common5-graphics 5.8.0, el lector de las mallas
Collada reutilizaba la primera transformación para geometrías instanciadas
varias veces. En section_4.dae, los dos cuerpos XL-320 se superponían.

En coordenadas de esa malla:

| Cuerpo | Centro Y previsto por el CAD | Centro Y cargado anteriormente |
|---|---:|---:|
| Primer XL-320 | 9 mm | 9 mm |
| Segundo XL-320, unión a la muñeca | 49 mm | 9 mm |

El desplazamiento del segundo cuerpo era 40 mm. También había instancias
repetidas de remaches mal posicionadas. El gripper no estaba desprendido en la
cadena cinemática, pero faltaba la pieza renderizada que debía mostrar la unión.
Las pruebas anteriores auditaban joints y CAD fuente; no detectaban cómo el
lector de Gazebo reconstruía las instancias. Se corrige esa carencia de prueba.

## Cambios

Se mantienen intactos los siete DAE oficiales. A partir de ellos se generan
siete visuales derivados con las transformaciones aplicadas a cada instancia
y nombres de geometría únicos. Se conserva cada triángulo en sus coordenadas
evaluadas: no se añadieron puentes inventados ni se movieron articulaciones.

También se incorporan normales explícitas y materiales de inspección:

- mordaza fija: verde;
- mordaza móvil: magenta;
- cuerpos de servomotor: gris grafito;
- discos de salida y remaches: grises metálicos;
- eslabones impresos: colores diferenciados.

Los archivos derivados mantienen licencia GPL-3.0-only y registran procedencia,
hashes, colores y correspondencia de las piezas en su manifest. No se cambian
colisiones, masas, inercias, órdenes de cierre ni trayectorias de la corrección
de agarre anterior.

El modelo Poppy conserva su mecánica de una mordaza fija y una móvil. Un cierre
en espejo sería un cambio de gripper, no una corrección de este ensamblaje.

## Verificación

La auditoría nueva usa el mismo gz::common::MeshManager instalado que consume
Gazebo, extrae sus vértices y los compara con el CAD original transformado:

- antes: FAIL, conservado como prueba negativa;
- después: PASS en 7 mallas, 123 instancias y 378222 triángulos;
- error máximo por coordenada: aproximadamente 5e-14 m (tolerancia 1e-10 m);
- normales explícitas presentes para los vértices cargados;
- regresión mecánica: PASS, 6 joints y 3 poses FK;
- 60 pruebas automáticas: 0 fallos.

La nueva corrida de contacto físico, sin attach, pasó todos los criterios:
elevación de 62,85 mm, retención de 3,21 s y depósito con error XY de 3,36 mm.
Se revisó visualmente el segundo 23 y la unión del servomotor ahora es visible.

[Resultados](../results/verified/assembly_render_20260909/).
[Videos](../captures/ensamblaje_inspeccion/).
[Fotograma corregido del segundo 23](../captures/ensamblaje_inspeccion/segundo_23_corregido.jpg).

## Reproducir y auditar

~~~bash
python3 scripts/cad/build_gazebo_visuals.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator
source install/setup.bash
python3 scripts/cad/verify_gazebo_visuals.py \
  --output results/manual/runtime_visuals.json
python3 scripts/cad/validate_mechanical_assembly.py \
  --output results/manual/mechanical_visuals.json
ros2 launch mobile_manipulator pick_and_place.launch.py \
  attach_enabled:=false grasp_mode:=physical_contact \
  capture_evidence:=true evidence_fps:=60 \
  output_dir:="$PWD/results/manual/assembly_visuals" run_id:=assembly_visuals seed:=402
~~~

La auditoría del lector requiere g++, pkg-config y los headers/bibliotecas de
gz-common5-graphics del entorno Gazebo de desarrollo.

El video es CFR 60 fps y conserva timestamps simulados. Repite el último frame
recibido ante huecos de entrega, sin interpolar movimiento: 1843 imágenes fuente
y 2005 frames codificados. El detalle de muñeca es un recorte ampliado de los
segundos 20–26, no otra simulación ni geometría generada.
