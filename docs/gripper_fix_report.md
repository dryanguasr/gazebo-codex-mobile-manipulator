# Corrección del agarre físico del gripper

Actualización visual posterior: [ensamblaje y colores de inspección](assembly_render_fix.md).
La corrección física de este informe se conserva. El nuevo trabajo resuelve
cómo Gazebo carga las instancias visuales y reemplaza los videos de inspección.

## Diagnóstico

El video anterior señalaba un problema real de agarre. Que solo se mueva una
mordaza, en cambio, es correcto para este Poppy Ergo Jr: m6 acciona la mordaza
móvil y la otra pertenece a link 5. No se añadió un segundo dedo ficticio.

Había tres problemas:

1. El modo de manipulación sustituía los dedos por esferas invisibles de radio
   3 mm situadas en los frames históricos de punta, no en las superficies del
   CAD. Aplicando también el origen del visual, las caras interiores reales
   están en z=+14 mm de link 5 y x=-14 mm de link 6. El contacto de las esferas
   podía ocurrir fuera de esas superficies y la unión asistida ocultaba el fallo.
2. Durante lift/transfer/place se copiaba el ángulo medido de m6 como nueva
   consigna. Eso eliminaba el error de cierre y la precarga de retención.
   El backend convierte el error de posición en una orden de velocidad
   ([implementación de gz_ros2_control](https://github.com/ros-controls/gz_ros2_control/blob/jazzy/gz_ros2_control/src/gz_system.cpp)).
3. La representación del cilindro mediante una sola colisión permitía un
   contacto numérico tipo pivote: podía girar y escapar al cambiar la postura,
   aunque se hubieran registrado contactos bilaterales al cerrar.

La auditoría inicial de longitud sin aplicar el origen del visual no era válida;
el desfase relevante está en las caras interiores, no en una supuesta falta de
longitud del dedo fijo.

## Corrección

- Colisiones de las mordazas alineadas con el CAD: cajas distales de
  24 x 30 x 3,1 mm. La auditoría de 18 puntos contra los triángulos transformados
  obtiene un desfase máximo de 0,564 mm por la aproximación de la superficie
  detallada. Sustituye a los contactos invisibles separados del material.
- Frames de contacto nuevos; se conservan los frames históricos de herramienta
  y punta para no alterar el diagnóstico/tracking existente.
- Centro de agarre recalibrado para el cilindro de diámetro 30 mm. m6 contacta
  alrededor de 0,04 rad y mantiene la consigna de cierre 0 rad al transportar.
- Trayectorias predefinidas que conservan el eje vertical de la pinza también
  entre waypoints. No se añadió IK en línea ni control de trayectoria mediante
  la pose privilegiada del objeto.
- El volumen de colisión del cilindro se representa mediante tres cilindros
  coaxiales contiguos de 15 mm. La unión geométrica sigue siendo exactamente
  el cilindro original de 30 x 45 mm; masa, inercia, fricción y visual no cambian.
  Las parejas de colisión distribuyen el contacto a lo largo de su altura.
  No hay adhesión, reposicionamiento del objeto ni DetachableJoint en A2.
- Verificación continua de contacto bilateral, desviación del centro <=8 mm e
  inclinación <=10 grados durante lift/hold/transfer/lower. La pose de Gazebo se
  usa explícitamente para seguridad y evaluación, no para elegir trayectorias.
- Pérdida persistente o heartbeat ausente: cancelar movimiento, mantener la
  postura y terminar FAILED. No abrir ni barrer hacia home con carga incierta.
- Aislamiento de resultados asíncronos de acciones anteriores: un resultado
  tardío no puede completar o hacer fallar el movimiento que lo reemplazó.
- Evaluación más estricta: el hold requiere objeto elevado y retenido; la
  liberación física requiere apertura observada; la estabilidad requiere altura
  de soporte y cilindro vertical, no solo proximidad XY.

No se modificaron las siete mallas DAE oficiales ni las articulaciones m1-m6.

## Evidencia

Resultados posteriores a la corrección, con **attach desactivado**:

| Corrida | Seed | Elevación | Error XY depósito | Error máximo centro pinza/objeto | Hold | Estabilidad |
|---|---:|---:|---:|---:|---:|---:|
| run_01 | 402 | 62,92 mm | 3,58 mm | 3,19 mm | 3,20 s | 2,50 s |
| run_02 | 403 | 62,77 mm | 2,09 mm | 2,90 mm | 3,20 s | 2,49 s |

Ambas pasan todos los criterios, sin contactos prohibidos y con cero attach.
La prueba negativa run_03 usa deliberadamente la colisión única anterior:
detectó pérdida de retención e inició RECOVER en 0,260 s, terminando FAILED
sin transferir ni abrir. Ese fallo esperado no se cuenta como intento nominal.
Son dos repeticiones de una escena determinista, no una garantía de robustez
general ni una validación en hardware. Hubo además un piloto completo correcto
y cuatro pruebas de ajuste fallidas; sus resultados se resumen en la evidencia.

- [Resultados y procedencia](../results/verified/gripper_fix_20260908/)
- [Videos corregidos a 60 fps](../captures/gripper_corregido/)
- [Auditoría geométrica](../tools/audit_gripper_contact.py)
- 50 pruebas automáticas: 0 fallos, 0 errores.
- Los videos A1 anteriores se conservan como históricos; sus éxitos asistidos
  no deben interpretarse como prueba de retención física correcta.

Las corridas se ejecutaron sobre el árbol modificado de ac364f1.
Cada repetición guarda huellas SHA-256 de los archivos usados; source_dirty=true
se conserva de forma explícita, sin atribuir las mediciones al commit anterior.

## Reproducir

Desde el repositorio, con ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator
source install/setup.bash
ros2 launch mobile_manipulator pick_and_place.launch.py \
  attach_enabled:=false grasp_mode:=physical_contact \
  output_dir:="$PWD/results/manual/gripper_fisico" \
  run_id:=gripper_fisico seed:=402 capture_evidence:=true evidence_fps:=60
```

La variante `pick_and_place_a2.launch.py` también selecciona contacto físico.
El lanzamiento A1 por defecto continúa siendo explícitamente asistido; no debe
usarse para demostrar agarre por fricción. No ejecutar dos mundos a la vez.

```bash
python3 tools/audit_gripper_contact.py
colcon test --packages-select mobile_manipulator --event-handlers console_direct+
colcon test-result --verbose
```
