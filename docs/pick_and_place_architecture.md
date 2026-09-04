# Arquitectura propuesta para pick-and-place

## Estado de partida y alcance

Este documento diseña la fase siguiente; **no afirma que pick-and-place esté
implementado**. El punto de partida será el commit estable de consolidación
mecánica, con `poppy_tool_frame`, seis joints, base 4WD, cámara y tracking
existentes.

La esfera roja de seguimiento conserva radio ≈ 0,12 m (diámetro ≈ 0,24 m) y no
es agarrable por la pinza Poppy 1:1. Debe seguir siendo el objetivo de tracking,
separada de un nuevo modelo `pick_object`.

## Auditoría del efector y objeto

La geometría oficial confirma una mordaza fija en link 5 y una rotativa en link
6:

- `m6=0 rad`: cerrado; separación aproximada entre caras internas 2,1 mm;
- `m6=1,20 rad`: abierto;
- dedos de aproximadamente 80–92 mm desde el pivote/soporte;
- apertura no paralela: el ancho útil depende de la profundidad de inserción;
- esfuerzo URDF actual de m6: 0,39 N·m;
- collider móvil: hull convexo de 92 triángulos;
- collider fijo: box simplificada.

Objeto MVP recomendado: cilindro vertical de 30 mm de diámetro, 45 mm de alto y
masa 30 g. Es claramente distinto de la esfera de tracking, admite contacto
lateral y deja margen frente a un cubo de 40 mm. Alternativa posterior: cubo de
30 mm. El siguiente hito debe medir de nuevo la apertura útil a la profundidad
real de pregrasp y no asumir que el AABB es la capacidad de agarre.

## Marcos

Cadena propuesta:

~~~text
world/odom -> base_footprint -> base_link -> ... -> poppy_link_5
                                              -> poppy_tool_frame
world -> pick_object (ground truth, evaluación)
camera_link -> object_measurement (control en Nivel C)
~~~

`poppy_tool_frame` está fijo al centro de la punta del dedo fijo; m6 solo
modifica la mordaza móvil. Las poses objetivo deben llevar sello temporal y
frame. Toda transformación a `base_footprint` debe ocurrir antes de planificar
el brazo.

## Interfaces propuestas

No se añaden aún estas interfaces; son el contrato recomendado:

| Interfaz | Tipo | Uso |
|---|---|---|
| `/pick_and_place/command` | service o action propia | iniciar/cancelar ensayo |
| `/pick_and_place/state` | mensaje/string | estado observable |
| `/pick_object/estimate` | `PoseStamped` | control, Nivel B/C |
| `/arm_controller/joint_trajectory` | topic/action existente | consignas m1–m6 |
| `/base_controller/cmd_vel` | `TwistStamped` | alinear/aproximar base |
| TF a `poppy_tool_frame` | TF2 | pregrasp/grasp y verificación |
| contactos Gazebo | topic/plugin a definir | confirmar contacto físico |
| estado joint m6 | `JointState` | cierre/apertura |
| `/pick_and_place/metrics` | JSONL o mensaje | evidencia por corrida |
| ground truth de Gazebo | servicio/topic de evaluación | métricas, nunca control Nivel C |

Una ROS 2 action es preferible para la tarea completa porque admite feedback,
cancelación y resultado. En el Nivel A basta un nodo determinista con máquina de
estados y un disparador explícito.

## Máquina de estados

| Estado | Entrada/condición de entrada | Acción e interfaz | Condición de salida | Timeout | Fallo y recuperación |
|---|---|---|---|---:|---|
| `IDLE` | controladores activos, modelo y object presentes | publicar estado y esperar command | orden válida | sin timeout | rechazar si faltan TF/controladores |
| `SEARCH_DETECT` | Nivel C y cámara lista | buscar detección estable en `/pick_object/estimate` | N muestras válidas | 10 s | reorientar base; luego `RECOVERY` |
| `ALIGN_BASE` | pose objeto en base disponible | giro mediante `cmd_vel` | error angular bajo gate | 8 s | detener, volver a SEARCH |
| `APPROACH_BASE` | alineación lograda | avance acotado | objeto dentro del workspace de brazo | 10 s | detener; recalcular o RECOVERY |
| `FREEZE_BASE` | precondición de manipulación | publicar velocidad cero y comprobar odom | velocidad lineal/angular estable | 2 s | frenar de nuevo; abortar si deriva |
| `ESTIMATE_OBJECT_POSE` | base quieta | transformar medición a base/tool | pose fresca y covariance/gate válidos | 3 s | nueva observación; no usar GT oculto |
| `PREGRASP` | objetivo alcanzable | trayectoria segura predefinida (A) o IK (B/C) | joints dentro de tolerancia | 6 s | retirar a home |
| `APPROACH_ARM` | pregrasp logrado, gripper abierto | avance final del brazo | tool dentro de error de grasp | 4 s | retroceder y reintentar una vez |
| `CLOSE_GRIPPER` | objeto entre dedos | comandar m6 hacia cerrado con límite | contacto/esfuerzo/posición estable | 3 s | reabrir; ajustar approach |
| `VERIFY_GRASP` | cierre finalizado | comprobar contactos y movimiento relativo | criterio de retención cumplido | 2 s | reabrir y volver a PREGRASP |
| `LIFT` | grasp verificado | elevar con trayectoria vertical/segura | altura mínima alcanzada | 5 s | detener y bajar si hay pérdida |
| `TRANSPORT` | objeto retenido | Nivel A: brazo/base a pose conocida | región de place alcanzada | 12 s | detener y recuperar |
| `PLACE` | sobre región destino | bajar hasta altura de liberación | error de place bajo gate | 5 s | recalcular una vez |
| `OPEN_GRIPPER` | objeto soportado | mandar m6 a 1,20 rad | separación confirmada | 3 s | repetir apertura o abortar seguro |
| `RETREAT` | objeto liberado | elevar/retirar brazo, luego home | distancia segura | 6 s | parada segura |
| `SUCCESS` | objeto colocado y robot seguro | cerrar métricas y resultado action | resultado emitido | 1 s | si registro falla, marcar corrida inválida |
| `RECOVERY` | cualquier fallo recuperable | velocidad cero, abrir pinza, retirar/home | estado seguro | 10 s | `FAILURE` si no se logra |
| `FAILURE` | reintentos agotados | detener actuadores y persistir causa | acknowledgement/cancel | — | intervención o nueva orden |

Todos los timeouts son valores iniciales que deben medirse y ajustarse; no son
resultados experimentales.

## Información de control frente a ground truth

| Dato | Nivel A | Nivel B | Nivel C |
|---|---|---|---|
| pose inicial del objeto | constante configurada | pose suministrada en base | estimación cámara/TF |
| pose del robot | odom/TF | odom/TF | odom/TF |
| pose real de Gazebo del objeto | métricas | métricas | solo métricas |
| contacto | control y métrica | control y métrica | control y métrica |
| detector visual | no requerido | opcional | requerido |
| éxito | contacto + lift/place, verificado con GT | igual | igual, sin GT para decidir acciones |

Cada métrica debe registrar `control_source` y `ground_truth_used_for_control`.
En Nivel C este último debe ser `false`.

## Niveles de implementación

### Nivel A — baseline determinista recomendado

- objeto en pose conocida;
- base estacionaria y congelada;
- secuencia de poses articulares predefinidas;
- validar apertura, approach, contacto, cierre, lift y liberación;
- registrar toda causa de fallo.

Es el siguiente paso recomendado porque aísla física del gripper, collisions y
secuencia de estados antes de introducir IK o ruido de percepción.

### Nivel B — cinemática

- pose objeto expresada en `base_footprint`;
- IK numérica pequeña o analítica específica;
- pregrasp y approach separados;
- validación de workspace y límites.

No se necesita MoveIt como condición inicial. Una solución simple, observable y
verificable es más adecuada para aprender la cadena de frames.

### Nivel C — percepción integrada

- estima `pick_object` desde cámara;
- alinea/aproxima base sin leer ground truth;
- congela la base, vuelve a estimar y ejecuta el brazo;
- usa ground truth únicamente para calcular métricas.

## Agarre físico y attach

### Agarre físico por contacto

Requiere:

- superficies de collision útiles en ambos dedos;
- fricción estática/dinámica suficiente y documentada;
- paso de simulación y solver estables;
- masa e inercia realistas del objeto;
- esfuerzo/velocidad de m6 compatibles;
- ausencia de penetración inicial;
- comprobación de dos contactos o retención relativa durante lift.

Es la validación física final, pero puede ser sensible al solver.

### Attach/detach explícito

Un plugin puede crear una unión temporal cuando se cumplen distancia, contacto
y cierre. Es útil como MVP reproducible **solo si** las métricas dicen
`grasp_mode=attach`. No demuestra fricción ni retención por contacto y nunca
debe presentarse como tal.

Recomendación:

1. Nivel A inicial con attach condicionado por contacto para depurar estados y
   trayectorias;
2. conservar una prueba negativa que impida attach sin contacto;
3. luego ejecutar una variante física sin attach y comparar tasa de éxito;
4. no avanzar a Nivel C hasta entender por qué falla cada variante.

## Riesgos principales

- la pinza rotativa no produce caras paralelas en todo su recorrido;
- el objeto puede quedar fuera del workspace aun si la cámara lo ve;
- el esfuerzo de m6 puede mover el objeto o la base;
- collisions simplificadas válidas para navegación visual pueden ser
  insuficientes para contacto de dedos;
- un tool frame correcto puede seguir no coincidiendo con el punto de contacto
  elegido;
- mezclar frame de cámara, base y world produce grasps aparentemente aleatorios;
- un attach incondicional ocultaría todos los fallos anteriores.

## Gate antes de Nivel B

No implementar IK/percepción hasta que Nivel A demuestre:

- 10 corridas reproducibles;
- ≥ 90 % de grasp y ≥ 90 % de place con semilla/estado inicial registrado;
- lift ≥ 50 mm y retención ≥ 3 s;
- cero attach sin contacto;
- cero colisiones no permitidas;
- logs que distinguen percepción, control y ground truth.
