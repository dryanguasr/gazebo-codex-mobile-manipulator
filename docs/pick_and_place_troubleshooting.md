# Troubleshooting de pick-and-place

Esta guía distingue hechos observados en las corridas verificadas de síntomas
comunes aún hipotéticos. No convierta una previsión en incidencia sin SHA, log
y prueba de cierre.

## Incidencias observadas

### DetachableJoint aparecía unido al crear el objeto

- **Síntoma:** el plugin podía adjuntar el hijo inmediatamente al aparecer,
  antes de `VERIFY_GRASP`.
- **Causa comprobada:** comportamiento de inicialización de DetachableJoint en
  Gazebo Sim 8, no un contacto autorizado.
- **Corrección:** fase previa `INITIALIZE_RESET`: publicar detach varias veces,
  exigir feedback `detached` y pose fresca, y guardar `initialization.json`.
- **Cierre:** las diez corridas nominales y las negativas aplicables tienen
  inicialización válida. Esta fase está excluida del intervalo medido.

Si falta o falla `initialization.json`, la corrida es inválida; no la cuente ni
la reemplace silenciosamente.

### Wrapper de Gazebo podía dejar procesos de una corrida

- **Síntoma:** wrappers y descendientes sobrevivían a la terminación del launch,
  con riesgo de contaminar la siguiente semilla.
- **Causa comprobada:** el cierre del wrapper no representaba necesariamente el
  cierre de todo su grupo de procesos.
- **Corrección:** `managed_gazebo` reenvía señales y los runners gestionan el
  grupo exacto, con TERM acotado, escalamiento estrecho y auditoría de residuos.
- **Cierre:** 10/10 nominales, 7/7 negativas y A2 terminaron con
  `process_residue=[]`.

Nunca mate todos los procesos ROS o Gazebo del usuario. Identifique la corrida y
su grupo exacto.

### Rechazo transitorio del primer goal articular

- **Síntoma:** el action server podía rechazar el primer goal durante una breve
  ventana de activación y aceptarlo decenas de milisegundos después.
- **Causa comprobada:** disponibilidad transitoria del controlador al arranque.
- **Corrección:** un único reintento del mismo goal después de 0.25 s, registrado
  como `trajectory_goal_rejected_retry`.
- **Cierre:** la campaña nominal acepta como máximo un reintento interno; nunca
  repite externamente una corrida fallida para fabricar diez éxitos.

Más de un rechazo termina en `goal_rejected` y recuperación.

### Probe TF efímero en el diagnóstico final

- **Síntoma:** el primer diagnóstico de este hito no descubrió
  `base_footprint` dentro de una única ventana de 4 s, aunque una consulta previa
  de punta había devuelto TF válido.
- **Causa comprobada:** descubrimiento DDS transitorio del proceso nuevo
  `tf2_echo`.
- **Corrección:** `capture_tf` permite hasta tres procesos independientes,
  acotados a 4 s cada uno; no relaja tolerancias ni reutiliza TF viejo.
- **Cierre:** `diagnostic_pick_A1_20260908_retry1/summary.json` es PASS con FK/TF
  submilimétrica. El primer intento queda preservado y marcado `INVALID` en
  `diagnostic_pick_A1_20260908/STATUS.md`.

### Métrica de tracking desacoplada de la geometría real

- **Síntoma:** una campaña histórica reconstruía la cámara con extrínsecos
  anteriores a la plataforma compacta.
- **Causa comprobada:** geometría duplicada como constantes en el logger.
- **Corrección:** referencia mediante pose aceptada del objetivo y TF odométrico
  fresco hasta `camera_link`, con fallos explícitos en vez de fallback.
- **Cierre:** la comparación final
  `tracking_metric_reference_v3_pick_A1_20260908` obtuvo 100 % de referencias
  válidas y 91.37 % de mejora. Esta referencia no es GT independiente del estado
  real de Gazebo.

### A2 físico no levantó el objeto

- **Síntoma:** hubo gate bilateral, pero el cilindro no se separó del pedestal.
- **Evidencia:** lift y clearance máximos de 0.00065056 m, cinco contactos
  prohibidos, error de depósito de 0.08690 m y estabilidad 0 s.
- **Clasificación:** `run.status=failed`, aunque el supervisor alcanzó `DONE`.
- **Decisión:** detener tras la primera de hasta dos configuraciones; no afinar
  fricción/solver a posteriori. Se conserva A1 como MVP asistido.

## Diagnóstico rápido por síntoma

### No aparece `run.json`

Revise `launch.log`, la salida del inicializador y el watchdog. Un reloj simulado
sin progreso más de 3 s produce `sim_clock_watchdog`. Verifique `/clock`, plugins
del world y que no exista otra instancia con el mismo world.

### Estado atascado en `FREEZE_BASE`

Compruebe:

~~~bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /base_controller/odom --once
ros2 action list -t
~~~

Deben existir seis joints, odom y
`/arm_controller/follow_joint_trajectory`. La negativa oficial demuestra que un
controlador ausente termina por timeout sin attach.

### Llega a `VERIFY_GRASP` pero no adjunta

Inspeccione los eventos del gate. Todos deben ser simultáneos: dos contactos
frescos, persistencia, geometría, cierre, estado autorizado, base detenida y TF.
La mera proximidad o un temporizador no son criterios válidos.

~~~bash
grep -E 'gate|contact|attach' RESULTS/events.jsonl
~~~

Si solo toca un dedo, corrija pose/profundidad; no aumente la tolerancia
geométrica hasta englobar el objeto sin contacto.

### Cierra pero m6 no llega a cero

Es normal con un objeto entre los dedos. Cero radianes es la consigna de cierre,
no el feedback esperado bajo contacto. El gate acepta hasta 0.78 rad y la
campaña observó aproximadamente 0.69 rad. Sin objeto, sí debe verificarse el
recorrido abierto/cerrado contra el modelo.

### Base en movimiento o deriva

El supervisor publica cero continuamente. Velocidad >0.01 m/s o deriva >0.01 m
provoca recuperación. Busque otro publicador en
`/base_controller/cmd_vel`; no compense la pose con ground truth.

### Cancelación con objeto transportado

La transición correcta es `TRANSFER → RECOVER`; después debe bajar, autorizar
release y terminar `CANCELLED`, con un attach y un detach. Si no se completa el
descenso, el diseño retiene el objeto y registra
`recovery_release_withheld`.

### `DONE` pero evaluación FAIL

`DONE` solo indica que el supervisor completó comandos. La autoridad de
aceptación es `run.json`, que también exige lift, clearance, hold, placement,
estabilidad, ausencia de contactos prohibidos y base inmóvil. Este caso ocurrió
en A2.

### El objeto es demasiado grande o está ausente

El cilindro oficial mide 30 mm. La prueba de 240 mm debe fallar sin attach; el
objeto ausente debe agotar `VERIFY_GRASP`. No use la esfera roja de tracking
(240 mm de diámetro) como pieza manipulable.

### Tool y grasp frame parecen desplazados

`poppy_tool_frame` está en la punta fija y sirve para FK. El gate usa
`poppy_grasp_frame`, centrado en la mordaza. Valide primero:

~~~bash
python3 scripts/cad/validate_official_consolidation.py
python3 scripts/cad/validate_mechanical_assembly.py
bash scripts/run_diagnostic.sh results/verified/diagnostic_NUEVO
~~~

No mueva meshes para hacer coincidir una trayectoria errónea.

### CAD falla al convertir STEP

Instale `gmsh` y ejecute `check_cad_dependencies.py`. El FAIL del alineador
autónomo es esperado únicamente si la decisión final continúa siendo B3
`official_reference_consolidation`; `convert_step_example.py`,
`validate_meshes.py`, el validador oficial/final y el mecánico deben pasar.

## Problemas previstos aún no observados

- ruido o pérdida de detección en un futuro nivel perceptivo;
- pose fuera del workspace o IK sin solución en un futuro nivel B;
- timestamp/frame inválido en una estimación externa;
- inestabilidad física por otro timestep, masa, fricción o solver;
- desgaste/calibración y límites de torque en hardware real;
- oclusión del objeto durante el approach;
- planificación de colisiones al introducir trayectorias no predefinidas.

Esos casos pertenecen a percepción/IK o Sim2Real futuros. A1 no los resuelve y
no debe extrapolarse a ellos.

## Plantilla para una incidencia nueva

~~~text
Estado: OBSERVADO
Corrida y SHA:
Síntoma:
Evidencia:
Causa comprobada:
Corrección:
Validación posterior:
Limitación restante:
~~~
