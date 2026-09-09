# Pick-and-place móvil: traslado, giro y entrega

Este ejercicio amplía el pick-and-place estacionario conservando el CAD,
las colisiones del gripper y sus colores de inspección. La base 4WD lleva la
pieza entre dos estaciones separadas, gira y se detiene antes de depositarla.

## Alcance y simplificaciones

- Recogida: centro del cilindro en **(-0,044; -0,225; 0,1625) m**.
- Entrega: centro objetivo en **(1,063; 0,694; 0,1625) m**.
- Separación horizontal: **1,439 m**, frente a unos 89 mm del ejercicio anterior.
- Ruta actual: salida recta hacia +X, esquina redondeada y aproximación recta
  hacia +Y, hasta **(0,838; 0,738; 90°)**. La esquina une (0,488; 0)
  con (0,838; 0,35), sin detener el carro para girar.
- Rejilla visual: celdas de **20 cm**, líneas principales cada metro;
  las líneas no añaden colisiones ni alteran la fricción del piso.

Por solicitud del ejercicio, se asume suficiente fricción para que la pieza
no gire respecto al gripper. Se representa con **DetachableJoint**, una unión
rígida temporal con la mordaza fija, no con una modificación de la pose de
la pieza. Fija los seis grados de libertad relativos durante la retención.
La orientación mundial sí cambia cuando gira el carro: lo que permanece fijo
es la orientación **respecto al gripper**, como ocurriría con un agarre firme.

Durante el final de APPROACH se permite que la mordaza fija toque la pieza
aún apoyada: desplazamiento del objeto ≤10 mm, altura conservada ±2 mm,
distancia al gripper ≤30 mm, mordaza móvil abierta y base quieta. No se permite
contacto prematuro de la mordaza móvil ni empujar una pieza fuera del soporte.

La unión se autoriza solo tras contacto bilateral fresco y persistente,
cierre, alineación, TF y base detenida. Se retira en RELEASE, después de
descender sobre el soporte. La pieza vuelve entonces a la dinámica de
contacto y gravedad. El RESET inicial elimina el auto-attach del plugin
antes de empezar la medición.

Esto es **agarre asistido por simulación**, no una validación de fuerzas de
fricción reales. Conserva contactos para verificar recogida y depósito; no
elimina toda la carga de cálculo del solver ni promete una aceleración medida.
El ejercicio anterior de contacto físico sin attach sigue disponible.

La navegación utiliza **localización simulada**: la pose mundial del robot
publicada por Gazebo corrige la deriva del skid-steer. El controlador ordena
velocidades a las ruedas; no teletransporta el robot ni la pieza. No se presenta
como SLAM, Nav2, evitación de obstáculos o navegación basada en percepción.
La pose del objeto se usa para el gate y para evaluar, no para planificar la ruta.

## Reproducción

Desde la raíz del repositorio:

~~~bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator
source install/setup.bash
ros2 launch mobile_manipulator pick_and_place_mobile.launch.py \
  output_dir:="$PWD/results/manual/pick_mobile" run_id:=pick_mobile \
  seed:=501 capture_evidence:=true evidence_fps:=60 \
  source_sha:="$(git rev-parse HEAD)"
~~~

Use un directorio de salida nuevo por corrida. Si hay cambios sin commit,
añada `source_dirty:=true`. El launch es headless y termina automáticamente
después del estado terminal y el cierre de los archivos.

Configuración: [pick_place_mobile.yaml](../src/mobile_manipulator/config/pick_place_mobile.yaml).
Mundo: [pick_and_place_mobile.sdf](../src/mobile_manipulator/worlds/pick_and_place_mobile.sdf).
El mundo se regenera de forma determinista con:

~~~bash
python3 scripts/build_mobile_world.py
~~~

## Secuencia y responsables

~~~text
OPEN → PREGRASP → APPROACH → CLOSE → VERIFY_GRASP
                                      │ contacto válido + unión temporal
                                      ▼
LIFT → HOLD → FOLD → NAVIGATE → DOCK_BASE → TRANSFER → LOWER
              │            │                         │
          avanzar/girar   detenerse                  ▼
                                      RELEASE → RETREAT → DONE
~~~

El supervisor es el único emisor nominal de comandos al brazo y a la base.
La ruta suave limita la velocidad lineal a **0,10 m/s** (antes 0,06) y
angular a 0,45 rad/s. Las rampas limitan las aceleraciones a 0,05 m/s² y
0,30 rad/s². Una curva Bézier de grado cinco enlaza las rectas con curvatura
nula en ambos extremos. El seguimiento continuo evita la parada intermedia;
la llegada reduce progresivamente la velocidad y verifica posición y orientación.

El controlador usa un punto virtual 6 cm detrás del origen de la base para
mejorar el seguimiento con skid-steer. Es una transformación geométrica de
la localización, no una modificación de la pose física ni una estimación exacta
del centro de giro. Las pruebas incluyen desplazamientos del pivote de 0 a 14 cm.
La ruta redondeada corresponde a este escenario conocido: parte de (0,0,0)
y termina mirando hacia +Y. Con `smooth_transport: true`, el último elemento
de `route` es el objetivo; los waypoints intermedios pertenecen al modo antiguo.
`PlanarRoute` conserva ese modo de parar/alinear para comparación.

Antes de arrancar, **FOLD** recoge el brazo en
`[-0.05806242, 0.15, -1.05, 0, 0.9, 0]` rad. La extensión horizontal del
gripper respecto al montaje pasa de unos **215 a 113 mm**, con su centro
aproximadamente a **296 mm del suelo**, conservando la pieza vertical.
El supervisor exige alcanzar esa postura antes de NAVIGATE; el evaluador
comprueba posiciones articulares frescas durante todo el transporte.

Los movimientos nominales del brazo pasan de 2,5 a **2,2 s** y usan
interpolación quíntica, con velocidad y aceleración nulas en sus extremos.
La aproximación inicial conserva sus puntos de despeje y escala sus tiempos.
No se acortan las ventanas de contacto, cierre, retención o estabilidad del
depósito. La unión temporal sigue fijando la orientación relativa de la pieza.

La base queda inmóvil en recogida y depósito. DOCK_BASE exige reposo continuo
antes de permitir mover el brazo hacia la entrega. Se reutiliza la postura de recogida
para entregar: el punto local (-0,044; -0,225) se transforma por el giro de 90°
y la traslación: (0,838 + 0,225; 0,738 - 0,044) = (1,063; 0,694).
Esto deja 14 mm nominales entre el pedestal y la cara exterior de la rueda,
frente a los 2 mm que daba trasladar directamente la postura antigua de entrega.

TF: **world → odom → base_footprint → brazo**. La corrección world→odom
compensa el deslizamiento de la odometría sin cambiar la dinámica.
El gate y evaluador comparan el objeto y el gripper en world.

Un contacto robot/pedestal detectado por los sensores de las estaciones,
localización u odometría obsoleta durante navegación, pérdida de unión,
cancelación o timeout detienen la base. La recuperación móvil mantiene el
brazo en su posición; no abre sobre el piso ni ordena volver a home con carga.

## Evidencia y criterios

El evaluador usa la pose real de Gazebo, independiente de los comandos enviados.
Comprueba recogida, elevación ≥50 mm, retención ≥3 s, recorrido de base ≥1,3 m,
cambio de orientación ≥80°, llegada a ≤5 mm y ≤1°, reposo durante manipulación,
retención durante el transporte, liberación y estabilidad del objeto ≥2 s.
Se mide también la variación de orientación pieza/gripper: máximo permitido 1°,
incluido el desfase temporal de las mediciones.

La tolerancia de posición de la pieza sigue siendo 30 mm, pero no basta por
sí sola: también debe quedar a la altura del soporte, vertical y estable.
No se acepta un objeto en el piso cerca del objetivo.

Puede auditar los datos de una corrida nominal sin ROS ni Gazebo:

~~~bash
python3 tools/verify_mobile_pick_place.py results/manual/pick_mobile
~~~

El auditor vuelve a calcular recorrido, giro y retención desde samples.csv,
y comprueba la secuencia observada y los criterios del evaluador.

Cada corrida produce run.json, eventos y muestras, más dos grabaciones de
cámara real de Gazebo cuando `capture_evidence:=true`:

- `media/`: vista general fija para inspeccionar estaciones, rejilla y recorrido.
- `detail/media/`: cámara ligada al carro para inspeccionar la pieza y muñeca.

Ambas solicitan 60 Hz. El MP4 usa 60 fps CFR y tiempo simulado; si falta una
imagen se repite la anterior, sin interpolación de movimiento. Los manifests
registran imágenes recibidas, frames y duración. Una marca lateral blanca y una marca superior oscura en el cilindro permiten
inspeccionar también el giro alrededor de su eje; son visuales, sin colisiones.
Las anotaciones identifican
explícitamente `attach_conditioned`.

## Prueba de cancelación segura

~~~bash
ros2 launch mobile_manipulator pick_and_place_mobile.launch.py \
  output_dir:="$PWD/results/manual/pick_mobile_cancel" run_id:=mobile_cancel \
  seed:=505 negative_scenario:=cancel_mobile_route capture_evidence:=false
~~~

Se inyecta la cancelación tres segundos simulados después de iniciar NAVIGATE.
El resultado esperado es CANCELLED, sin detach ni apertura del gripper:
la pieza permanece elevada. El run.json marca el pick-and-place como failed
porque no hubo entrega; eso es correcto para esta prueba negativa. Sus
aserciones de seguridad se registran por separado, no se contabiliza como
una corrida nominal exitosa.

## Hallazgo de integración

La primera corrida instrumentada detectó contacto entre la rueda delantera
derecha y el pedestal. Se canceló y se conservó como intento de desarrollo,
no como éxito. El problema se corrigió en la aproximación y postura de entrega,
sin reducir la separación entre estaciones ni retirar las colisiones.
Se añadieron sensores de estación al supervisor y evaluador para que una
colisión equivalente detenga la maniobra y quede registrada.

## Revisión de cierre: transporte recogido y suave

Dos corridas con las mismas fuentes runtime pasan todos los criterios.
El ciclo queda en **55–57 s**, frente a unos 90 s de la maniobra anterior.
La pieza no se libera durante el repliegue, la curva ni el despliegue.

| Métrica | Sin grabación (604) | Con grabación (603) |
|---|---:|---:|
| Ciclo simulado | 56.68 s | 55.28 s |
| Transporte NAVIGATE | 21.80 s | 21.66 s |
| Recorrido real | 1.5199 m | 1.5198 m |
| Error de llegada de la base | 1.54 mm | 1.49 mm |
| Error de depósito | 0.36 mm | 0.59 mm |
| Variación pieza/gripper | 0.700° | 0.690° |
| Error articular máximo en transporte | 0.0018 rad | 0.0039 rad |
| Attach / detach | 1 / 1 | 1 / 1 |

Pasaron también la cancelación con carga (0,485 mm de desplazamiento posterior,
sin soltar la pieza), la regresión estacionaria sin attach, 89 pruebas
automatizadas, el lint y la auditoría mecánica. Los cinco videos entregables
se revisaron por decodificación y por fotogramas de recogida, repliegue, curva,
llegada y entrega.

La cámara cercana se elevó 10 cm y su campo horizontal pasó de 0,85 a 1,0 rad
para incluir el brazo recogido. El clip de depósito amplía una región de la
vista general, mostrando también el soporte completo. La vista general original
y el ciclo cercano completos se conservan.

[Resultados y trazabilidad](../results/verified/mobile_smooth_20260909/README.md) ·
[Videos actuales a 60 fps CFR](../captures/pick_place_movil_suave/README.md).

## Evidencia anterior: recorrido con parada para girar

Dos corridas nominales completas pasaron todos los criterios, una de ellas
con las dos cámaras activas. Las fuentes runtime permanecieron idénticas
entre ambas; sus SHA-256 se conservan con la evidencia.

| Métrica | Sin grabación, seed 504 | Con grabación, seed 506 |
|---|---:|---:|
| Recorrido real del carro | 1,6823 m | 1,6822 m |
| Orientación final | 89,777° | 89,775° |
| Error de llegada del carro | 1,55 mm | 1,58 mm |
| Elevación de la pieza | 62,66 mm | 62,62 mm |
| Error del depósito | 2,14 mm | 2,26 mm |
| Variación máxima pieza/gripper | 0,355° | 0,367° |
| Estabilidad después de soltar | 4,67 s | 4,69 s |
| Attach / detach | 1 / 1 | 1 / 1 |

No hubo contactos indebidos en ninguna corrida nominal. También pasaron la
cancelación con carga (desplazamiento observado tras cancelar: 0,154 mm),
la regresión estacionaria de contacto físico sin attach, las 77 pruebas
automatizadas, el lint y la auditoría mecánica de seis articulaciones.

[Resultados, muestras y fuentes](../results/verified/mobile_pick_place_20260909/README.md) ·
[Resumen de criterios](../results/verified/mobile_pick_place_20260909/summary.json).

Los [cinco videos entregables](../captures/pick_place_movil/README.md) incluyen
ciclo general, ciclo cercano al gripper, recogida, traslado/giro y depósito.
Se verificaron los cinco por decodificación completa y ffprobe: H.264 a 60 fps CFR.
La vista general recibió 3438 imágenes para 5389 frames y la cercana 4723
para 5390; los frames faltantes se completan por repetición, no por generación
ni interpolación. No se garantiza que los 60 frames de cada segundo sean nuevos.

El alcance educativo de ambas variantes es: ruta conocida,
localización simulada y retención por unión temporal. No afirma autonomía
perceptiva, evitación de obstáculos ni validación dinámica de fricción.
