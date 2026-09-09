# Informe final: pick-and-place nivel A

Versión estacionaria e histórica. Para el ejercicio con traslado, giro y rejilla,
consulte [Pick-and-place móvil](mobile_pick_and_place.md).

Nota posterior: el problema de agarre del video A1 fue corregido. Consulte
[el informe del gripper](gripper_fix_report.md) y los
[videos físicos actualizados](../captures/gripper_corregido/).
Los resultados A1/A2 de este documento corresponden a la revisión histórica.

## Resultado

El nivel A1 quedó implementado y aceptado como **MVP asistido por simulador**.
La campaña oficial completó diez intentos consecutivos sin reemplazos: 10/10
grasp, 10/10 placement, 10/10 ejecuciones limpias, cero attach fuera del gate,
cero detach fuera de release y cero residuos. El nivel A2 sin unión temporal no
levantó el objeto y queda formalmente rechazado; esa limitación no se oculta
detrás del éxito de A1.

El seguimiento visual B3, el ensamblaje Poppy 1:1, la base compacta y el pipeline
CAD permanecen operativos. El paquete conserva exactamente siete DAE oficiales.

## Alcance entregado

- world independiente `pick_and_place.sdf` y launch A1/A2 separados;
- cilindro físico de 30 mm × 45 mm, 30 g, inercia y fricción explícitas;
- base inmóvil durante toda la manipulación;
- supervisor determinista con timeouts, watchdog de pared y recuperación segura;
- gate A1 condicionado por contacto bilateral fresco, geometría, cierre,
  estado autorizado, base detenida y TF válido;
- evaluador que lee pose real de Gazebo y decide lift, clearance, hold,
  placement, estabilidad, contactos y deriva;
- inicialización verificada anterior al intervalo medido;
- runners de campaña nominal y siete negativas;
- evidencia de Gazebo en JSON/JSONL/CSV/logs, seis capturas y un MP4;
- A2 acotado, sin attach y sin ajuste post hoc;
- regresión CAD, build, tests, diagnóstico y A/B de tracking.

La fuente funcional evaluada por A1 fue
`1d03fc59d05a7de8b95bd01e40a0db0e70bcd8ef`. Los resultados A1, negativos,
A2, compatibilidad geométrica, diagnóstico y tracking se añadieron en commits
posteriores independientes para no alterar la procedencia registrada.

## Arquitectura de confianza

El supervisor controla únicamente trayectorias articulares predefinidas y
publica cero a la base. No lee la pose real del objeto para seleccionar
movimientos: `ground_truth_used_for_control=false`.

A1 sí usa la pose real de Gazebo dentro del contrato geométrico de la unión y el
evaluador la usa para medir. Los artefactos lo declaran como
`ground_truth_used_for_attachment_gate=true` y
`ground_truth_used_for_evaluation=true`. Por tanto, A1 no certifica percepción,
IK ni agarre por fricción.

La liberación nominal solo existe en `RELEASE`. Ante cancelación o fallo con el
objeto unido, `RECOVER` baja primero; si bajar falla, no libera. No existe attach
por proximidad ni por tiempo.

## Resultados A1

Evidencia: `results/verified/pick_A1_20260908/`.

| Métrica, 10 corridas | Mínimo | Media | Máximo |
|---|---:|---:|---:|
| lift (m) | 0.055047 | 0.055190 | 0.055327 |
| error de placement XY (m) | 0.002105 | 0.002303 | 0.002733 |
| hold (s) | 3.201 | 3.2025 | 3.209 |
| estabilidad (s) | 4.554 | 4.6651 | 4.700 |
| deriva de base (m) | 5.82e-14 | 9.02e-14 | 1.49e-13 |
| máximo error articular observado (rad) | 0.00881 | 0.04572 | 0.25081 |

El máximo articular es una muestra instantánea del feedback durante trayectoria,
no el error final: todos los estados de movimiento cumplieron la tolerancia
final de 0.035 rad. Hubo dos reintentos transitorios en toda la campaña, nunca
más de uno por corrida, y ambos quedaron registrados.

Aceptación:

| Gate | Resultado |
|---|---:|
| intentos exactos | 10 |
| válidos / éxito | 10 / 10 |
| grasp | 10 / 10 |
| placement | 10 / 10 |
| lift ≥50 mm | 10 / 10 |
| hold ≥3 s | 10 / 10 |
| dentro de 30 mm | 10 / 10 |
| contactos prohibidos | 0 |
| attach/detach por corrida | 1 / 1 |
| inicialización válida | 10 / 10 |
| residuos | 0 |

El video real de Gazebo está en
`a1_01/media/pick_and_place_a1.mp4`: MPEG-4, 960×720, 15 fps, 829 frames y
55.27 s. El manifiesto enlaza pregrasp, approach, contact, lift, deposit y
release; no hay imágenes generativas ni reutilizadas.

## Pruebas negativas

Evidencia: `results/verified/pick_A1_negative_20260908/`. Las siete fueron
aceptadas y ninguna produjo éxito falso:

| Escenario | Terminal / causa | Attach |
|---|---|---:|
| objeto ausente | `FAILED / state_timeout:VERIFY_GRASP` | 0 |
| objeto 0.24 m incompatible | `FAILED / state_timeout:APPROACH` | 0 |
| proximidad sin contactos válidos | `FAILED / state_timeout:VERIFY_GRASP` | 0 |
| TF inválido | `FAILED / state_timeout:VERIFY_GRASP` | 0 |
| controlador no disponible | `FAILED / state_timeout:FREEZE_BASE` | 0 |
| base en movimiento | `FAILED / base_speed_limit` | 0 |
| cancelar durante transporte | `CANCELLED / explicit_cancel` | 1, luego detach seguro |

Las inicializaciones requeridas fueron válidas y no hubo residuos.

## Resultado A2

Evidencia: `results/verified/pick_A2_20260908/`.

Se ejecutó una configuración de las dos autorizadas, seed 801, con
`attach_enabled=false`. El gate bilateral físico se verificó, pero el cilindro
no abandonó el pedestal:

- lift y clearance máximos: 0.00065056 m;
- error de placement: 0.0868995 m;
- estabilidad: 0 s;
- cinco contactos prohibidos con el soporte de recogida;
- cero attach y cero detach;
- `run.status=failed`.

El fallo fue concluyente para esa física y trayectoria. Se detuvo antes de la
segunda configuración para evitar calibración a posteriori. Decisión:
`retain_A1_as_assisted_MVP`.

## Regresiones finales

El diagnóstico aprobado
`results/verified/diagnostic_pick_A1_20260908_retry1/` verificó controladores,
seis joints, cámara, odom, TF, dos poses del brazo, FK de punta y tool y
extrínseca de cámara. Resultado `passed`; errores TF/FK de tool <0.8 mm y
errores articulares <2.1e-10 rad. El primer intento se conserva como INVALID
por descubrimiento DDS/TF efímero y motivó probes reintentables acotados.

La comparación
`results/verified/tracking_metric_reference_v3_pick_A1_20260908/` pasó:

| Métrica | A: tracking off | B: tracking on |
|---|---:|---:|
| referencia válida | 100 % | 100 % |
| MAE distancia objetivo | 0.632345 m | 0.054581 m |
| RMS horizontal | 0.476436 | 0.024939 |
| desplazamiento robot | ~0 m | 0.402936 m |

Mejora de MAE: 91.37 %. La referencia usa la pose aceptada del objetivo y TF
odom→cámara, no ground truth independiente leído del estado del simulador.

También pasaron conversión STEP con Gmsh, validación de meshes (relación
collision/visual link 6 = 0.0029), consolidación oficial/final (6 joints,
7 visuales, 35 filas FK) y validación mecánica (6 joints, 3 poses). El FAIL del
alineador autónomo se mantiene como limitación documentada de B3; la decisión
final oficial es la autoridad.

## Incidencias y trazabilidad

1. Se corrigió antes de manipulación la referencia métrica de tracking para
   depender de TF real, sin cambiar ganancias.
2. Se neutralizó la unión inicial de DetachableJoint con un reset verificable.
3. Se añadió gestión exacta del grupo de procesos para impedir contaminación
   entre corridas.
4. Se acotó a uno el reintento de rechazo transitorio del action.
5. Se endurecieron los probes TF del diagnóstico conservando el intento inválido.
6. Se registró A2 como fallo físico, no como éxito por alcanzar `DONE`.

Consulte `pick_and_place_troubleshooting.md` para evidencia y respuesta
operativa.

## Clasificación y siguiente paso

- **Geometría:** cerrada para este hito; Poppy 1:1, siete DAE, B3 preservado.
- **Métrica:** corregida y validada; no es GT independiente para tracking.
- **A1:** aceptado como MVP determinista asistido por simulador.
- **A2:** no aceptado; requiere rediseño físico previo y preregistrado.
- **Percepción/IK:** futuros niveles B/C, fuera de este corte.

No se debe avanzar directamente a percepción autónoma. El próximo experimento
debe preregistrar como máximo dos cambios físicos concretos (por ejemplo,
geometría de contacto o perfil de cierre), repetir A2 sin attach y conservar
todos los fallos. Solo un A2 físicamente válido habilita IK y después percepción.
