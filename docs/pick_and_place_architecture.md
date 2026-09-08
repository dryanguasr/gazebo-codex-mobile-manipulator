# Arquitectura implementada de pick-and-place nivel A

## Alcance y compatibilidad

Esta arquitectura ya está implementada. Añade manipulación determinista sin
alterar el flujo B3 de tracking: `sim.launch.py` conserva la esfera de 240 mm,
la base compacta, cámara y colliders originales; `pick_and_place.launch.py` usa
un world separado y activa únicamente allí los pads de contacto de la pinza.

El ensamblaje Poppy permanece 1:1, con seis joints, siete visuales DAE oficiales
y `poppy_tool_frame` validado. El objeto manipulable no es la esfera: es un
cilindro de 30 mm × 45 mm y 30 g.

## Componentes y flujo

~~~text
pick_and_place.launch.py
 ├─ managed_gazebo ── pick_and_place.sdf + soportes + cámara de evidencia
 ├─ robot_state_publisher ── Xacro manipulation_enabled=true
 ├─ ros_gz_bridge
 │   ├─ contactos y pose real del objeto
 │   ├─ imagen de evidencia
 │   └─ attach/detach y estado de la unión
 ├─ controllers ── joint_state, base y arm
 ├─ pick_place_initializer ── RESET verificado, fuera de medición
 ├─ pick_place_supervisor ── única autoridad de m1–m6 y base cero
 ├─ pick_place_attach_gate ── contrato A1 / verificación física A2
 ├─ pick_place_evaluator ── métricas independientes de Gazebo
 └─ pick_place_recorder ── PNG y video
~~~

El launch inicia el objeto y el robot, activa controladores, ejecuta el reset y
solo si este pasa inicia el intervalo medido. Al terminar el supervisor solicita
el cierre del servidor y el wrapper gestiona su grupo exacto de procesos.

## Marcos

~~~text
world/odom → base_footprint → base_link → ... → poppy_link_5
                                            ├→ poppy_tool_frame
                                            └→ poppy_grasp_frame
world → pick_object                  (ground truth de gate/evaluación)
~~~

- `poppy_tool_frame` está en la punta fija y es la referencia de FK/diagnóstico.
- `poppy_grasp_frame` representa el centro efectivo entre dedos y es la
  referencia geométrica del gate.
- `poppy_fixed_tip` y `poppy_moving_tip` verifican la configuración bilateral.
- La base no se realinea en A: las poses articulares son predefinidas.

## Interfaces reales

| Interfaz | Tipo | Propietario y uso |
|---|---|---|
| `/arm_controller/follow_joint_trajectory` | action | supervisor, m1–m6 |
| `/base_controller/cmd_vel` | `TwistStamped` | supervisor publica cero |
| `/base_controller/odom` | `Odometry` | gate de velocidad/deriva y evaluación |
| `/joint_states` | `JointState` | cierre y tolerancia articular |
| `/pick_place/status` | `String` JSON | eventos y transiciones del supervisor |
| `/pick_place/gate_command` | `String` JSON | estado/cierre autorizados |
| `/pick_object/contacts` | `Contacts` | contacto bilateral real |
| `/model/pick_object/pose` | `TFMessage` | GT del gate A1 y evaluación |
| `/pick_object/attach`, `/detach` | `Empty` | comando de unión temporal |
| `/pick_object/joint_state` | `String` | feedback unido/desunido |
| `/pick_place/physical_grasp_verified` | `Bool` | criterio A2 sin attach |
| `/pick_place/cancel` | `Bool` | cancelación observable |
| `/pick_place/evidence/image` | `Image` | evidencia visual real de Gazebo |

## Máquina de estados

Secuencia nominal exacta:

~~~text
IDLE → FREEZE_BASE → OPEN → PREGRASP → APPROACH → CLOSE → VERIFY_GRASP
     → LIFT → HOLD → TRANSFER → LOWER → RELEASE → RETREAT → DONE
~~~

Los terminales alternos son `RECOVER → FAILED` y
`RECOVER → CANCELLED`. Cada estado no terminal tiene timeout simulado de 15 s;
un watchdog de pared de 3 s cubre reloj detenido. Las trayectorias duran 2.5 s,
salvo pregrasp de tres waypoints (1.5, 3.0 y 5.5 s) y close (3 s).

El supervisor es el único dueño de m1–m6. Exige resultado de la action y
tolerancia final de 0.035 rad. Un goal rechazado puede reintentarse una sola vez
tras 0.25 s; todo reintento queda en eventos.

Durante estados posteriores a freeze, velocidad >0.01 m/s o deriva >0.01 m
provoca `RECOVER`. En recuperación, un objeto unido se baja a `grasp` antes de
transfer o a `place` después de iniciarla. Release se autoriza solo tras bajar;
si el descenso falla se retiene el objeto.

## Separación entre control, gate y evaluación

| Dato | Supervisor | Gate A1 | Evaluador |
|---|---:|---:|---:|
| poses articulares configuradas | sí | no | observa |
| joints/odom | sí | sí | sí |
| contactos | no | sí | sí |
| pose real Gazebo del objeto | no | sí | sí |
| attach/detach | solicita contexto | decide/publica | observa |
| resultado físico | no | no | sí |

El supervisor declara `control_source=predefined_joint_trajectories` y
`ground_truth_used_for_control=false`. El gate A1 declara uso de GT porque su
validación geométrica consulta la pose real; el evaluador también. Esta
separación impide presentar A1 como percepción o control autónomo.

## Contrato A1

Attach requiere simultáneamente:

1. estado `VERIFY_GRASP` y cierre comandado;
2. contactos frescos de las collisions exactas del dedo fijo y móvil;
3. persistencia bilateral mínima de 0.15 s;
4. pose del objeto y TF frescos, máximo 0.10 s;
5. error a `poppy_grasp_frame` ≤20 mm y orientación ≤5°;
6. feedback m6≤0.78 rad;
7. base ≤0.01 m/s.

No existe ruta de attach por proximidad o temporizador. Detach nominal solo se
acepta en `RELEASE`. El reset inicial está explícitamente fuera de medición.

## A2

`pick_and_place_a2.launch.py` incluye la misma arquitectura con
`attach_enabled=false` y `grasp_mode=physical_contact`. El gate publica
verificación física bilateral, pero no crea un joint. El evaluador, no el
terminal del supervisor, decide si hubo separación, lift y depósito.

La evaluación acotada demostró contacto bilateral, pero no separación del
soporte. Por ello A2 está rechazado y A1 permanece etiquetado como asistido.

## Persistencia de evidencia

Cada corrida guarda procedencia, inicialización, eventos, muestras y resultado.
El evaluador serializa magnitudes no finitas como `null`; ausencia de dato no
puede convertirse en NaN JSON ni éxito. Los runners verifican SHA limpio, hash
de configuración, cantidad exacta de intentos, logs e inexistencia de procesos
residuales.

## Límites de esta arquitectura

- no hay IK, MoveIt, planificación general ni percepción del cilindro;
- las poses de pick/place son constantes de configuración;
- A1 usa una unión temporal posterior a contacto y no valida fricción;
- A2 aún no pasa;
- tracking y manipulación son regresiones separadas;
- la referencia métrica de tracking usa pose solicitada + TF, no GT real
  independiente del simulador.
