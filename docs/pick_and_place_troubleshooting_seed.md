# Troubleshooting inicial previsto para pick-and-place

## Naturaleza de este documento

Los casos siguientes son hipótesis de trabajo para el hito futuro. **No fueron
observados todavía** y no se presentan como incidencias resueltas. Cuando
aparezca un fallo real, registrar síntoma, log/captura, causa comprobada,
corrección y prueba de cierre en una sección separada de incidencias observadas.

## Objeto demasiado grande

**Síntoma previsto:** no cabe entre dedos o la mordaza choca antes de rodearlo.

**Diagnóstico:** comparar diámetro en la profundidad real de grasp con el
barrido de la mordaza; no usar solo el AABB global.

**Prevención:** usar `pick_object` cilíndrico de 30 mm. La esfera de tracking
de 240 mm debe ser rechazada por preflight.

## Objeto fuera del workspace

**Síntoma previsto:** el detector ve el objeto pero ninguna pose pregrasp es
alcanzable.

**Diagnóstico:** transformar a `base_footprint`, evaluar distancia/altura y
límites antes de mover.

**Recuperación:** reposicionar base u objeto; no saturar joints ni reutilizar la
última solución.

## IK sin solución

**Síntoma previsto:** solver no converge, devuelve NaN o viola límites.

**Diagnóstico:** registrar target, seed, frame, residuo y límites.

**Recuperación:** probar un pregrasp alternativo acotado o volver a ALIGN_BASE.
No avanzar a APPROACH con una solución inválida.

## Tool frame incorrecto

**Síntoma previsto:** los joints llegan a la pose pero los dedos quedan
desplazados respecto al objeto.

**Diagnóstico:** comparar FK independiente y TF a `poppy_tool_frame`; renderizar
ejes y comprobar que m6 no mueve el frame fijo.

**Prevención:** ejecutar la regresión oficial-vs-final antes de cada experimento.

## Dedos mal alineados

**Síntoma previsto:** un dedo roza el objeto y el otro queda lejos.

**Diagnóstico:** activar collision, revisar el punto de contacto elegido y el
ángulo m6. La pinza es rotativa, no paralela.

**Recuperación:** ajustar profundidad/orientación de grasp; no desplazar meshes
para hacer coincidir una trayectoria errónea.

## Fricción insuficiente

**Síntoma previsto:** el cierre produce contacto pero el objeto cae al levantar.

**Diagnóstico:** registrar coeficientes, fuerzas/contactos, velocidad de lift y
movimiento relativo.

**Recuperación:** cambiar un parámetro por vez dentro de un barrido predefinido.
No marcar attach como validación física.

## Objeto atraviesa collision

**Síntoma previsto:** penetración visible o ausencia de contacto.

**Diagnóstico:** comprobar collision del objeto/dedos, bitmasks, pose inicial,
paso de simulación y mensajes del solver.

**Recuperación:** corregir geometría/pose y repetir una prueba lenta de contacto.

## Gripper cierra pero no retiene

**Síntoma previsto:** m6 llega a cero, pero solo empuja o expulsa el objeto.

**Diagnóstico:** verificar dos superficies de contacto, normal, torque, fricción
y profundidad de inserción. Posición articular no equivale a grasp.

**Recuperación:** reabrir, retirar y reintentar una vez con approach corregido.

## El brazo mueve la base

**Síntoma previsto:** odom cambia durante approach/lift y degrada la pose.

**Diagnóstico:** revisar FREEZE_BASE, masa/inercia, contacto de ruedas y comandos
residuales.

**Recuperación:** velocidad cero, esperar estabilidad y abortar si la deriva
supera el gate. No corregir ocultamente con ground truth.

## El objeto se desprende al levantar

**Síntoma previsto:** VERIFY_GRASP pasa pero se pierde en LIFT/TRANSPORT.

**Diagnóstico:** registrar transform relativo objeto-tool, contactos y aceleración.

**Recuperación:** bajar de forma segura si aún hay contacto; después abrir y
volver a PREGRASP. El evento cuenta como grasp fallido.

## Attach plugin oculta un error físico

**Síntoma previsto:** la tarea pasa aun sin contacto o con dedos alejados.

**Diagnóstico:** prueba negativa con objeto fuera del gate; revisar timestamp de
contacto y creación de joint.

**Prevención:** exigir distancia, contacto y cierre antes del attach, registrar
`grasp_mode=attach_conditioned` y ejecutar variante sin attach.

## Target pose en frame incorrecto

**Síntoma previsto:** movimiento coherente pero hacia un lugar equivocado.

**Diagnóstico:** registrar `frame_id`, sello temporal y transform completo a
`base_footprint`. Rechazar frames vacíos o TF extrapolado.

**Recuperación:** volver a ESTIMATE_OBJECT_POSE; nunca reinterpretar números
como si ya estuvieran en base.

## Percepción y ground truth mezclados

**Síntoma previsto:** Nivel C parece robusto porque lee pose real de Gazebo.

**Diagnóstico:** auditar subscriptions/parámetros y registrar
`ground_truth_used_for_control`.

**Prevención:** separar namespaces/procesos de control y evaluación; una prueba
debe fallar si se habilita GT en Nivel C.

## Apertura/cierre invertidos

**Síntoma previsto:** el estado CLOSE_GRIPPER abre la mordaza.

**Diagnóstico:** inspeccionar geometría y consignas, no inferir semántica del
nombre del joint. En el modelo consolidado: 0 rad es cerrado y 1,20 rad abierto.

**Prevención:** prueba visual y de collision sin objeto antes del primer grasp.

## Rebote o explosión del objeto

**Síntoma previsto:** contacto genera aceleraciones grandes o NaN.

**Diagnóstico:** pose con penetración inicial, inercia demasiado pequeña,
velocidad de cierre, stiffness/damping y timestep.

**Recuperación:** restaurar snapshot, eliminar penetración y reducir velocidad;
documentar cualquier cambio de solver.

## Contacto detectado en el link equivocado

**Síntoma previsto:** la base o un bracket activa el gate de attach.

**Diagnóstico:** registrar nombres de ambas collisions y fase.

**Prevención:** whitelist exclusiva de collisions de dedo fijo, dedo móvil y
`pick_object`; todo otro contacto es espurio.

## Proceso Gazebo huérfano

**Síntoma previsto:** una corrida reutiliza estado o world anterior.

**Diagnóstico:** PID/world exacto y timestamp de inicio.

**Recuperación:** cierre TERM acotado y KILL solo para el proceso estrechamente
identificado, como en el runner A/B existente.

## Plantilla para convertir una previsión en incidencia observada

~~~text
### Título

Estado: OBSERVADO
Corrida/SHA:
Síntoma:
Evidencia:
Causa comprobada:
Corrección:
Validación posterior:
Limitación:
~~~

No mover una previsión a “observado” sin una corrida y evidencia reales.

## Incidencias observadas

### Referencia métrica de tracking con extrínseca anterior al chasis compacto

Estado: **OBSERVADO**

Corrida/SHA: los artefactos de `results/verified/experiments/` producidos antes
de esta corrección, auditados sobre `00c60529268c1786451893119e4bb150777eb701`.

Síntoma: `metrics_logger.py` reconstruía la posición de cámara con
`camera_offset_x_m=0.38` y `camera_height_m=0.51`, mientras el Xacro
consolidado sitúa `camera_link` mediante la cadena
`base_footprint -> base_link -> camera_link`. El PASS A/B y el porcentaje de
mejora histórico no certifican distancias físicas.

Evidencia: comparación directa del logger auditado con
`mobile_manipulator.urdf.xacro`. Los CSV y resúmenes originales se conservan
sin reescritura y están marcados en
`results/verified/experiments/INCIDENT.md`.

Causa comprobada: geometría duplicada como números mágicos en el evaluador,
sin enlazarla al TF publicado por `robot_state_publisher`.

Corrección: la referencia transforma la pose aceptada del objetivo desde su
`frame_id` a `camera_link` usando el TF más reciente y valida su edad contra el
timestamp de la medición. Pose y TF tienen
gates de frescura; una ausencia produce una muestra de referencia inválida, no
un fallback. El diagnóstico compara además el TF de cámara observado con la
cadena fija del URDF.

Validación posterior: **PASS**. Las 11 pruebas unitarias pasan; el diagnóstico
`diagnostic_metric_reference_v2_retry1` midió
`base_footprint -> camera_link = [0.225, 0.0, 0.120] m` con error numérico
menor que 1.4e-17 m; y la campaña
`tracking_metric_reference_v2_20260904_retry1` obtuvo 100% de referencias
válidas en A y B y una mejora B/A de 92.43%. No se modificaron ganancias ni
umbrales del controlador/comparador para compensar el cambio.

Limitación: la pose publicada por `target_trajectory` es la consigna aceptada
por `SetEntityPose`; la referencia usa TF odométrico y no constituye ground
truth independiente leído del estado real de Gazebo.

### Consulta TF en el sello exacto sin historia disponible

Estado: **OBSERVADO Y CORREGIDO**

La primera campaña corregida,
`tracking_metric_reference_v2_20260904`, produjo cero referencias válidas:
el logger pedía una transformación en el sello exacto de la imagen dentro del
mismo executor, antes de disponer de historia TF suficiente. El intento se
conserva como inválido. La corrección consulta el TF más reciente y calcula su
edad contra el sello de la medición; conserva el gate estricto de 0.10 s, sin
extrapolar ni reutilizar transforms obsoletos. El retry posterior pasó con
100% de referencias válidas.
