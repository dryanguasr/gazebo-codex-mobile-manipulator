# Evidencia de corrección del gripper

- run_01 (seed 402): nominal físico, PASS.
- run_02 (seed 403): repetición nominal física, PASS.
- run_03 (seed 404): **prueba negativa deliberada**, usa
  negative_loss_object.sdf, la colisión única anterior. Se esperaba pérdida de
  agarre y FAILED: la detectó y entró a RECOVER en 0,260 s, sin transferir ni abrir.
  El fallo físico del objeto es esperado; no cuenta como una corrida nominal.
- development_attempts.json conserva los resultados del ajuste, incluidos los
  cuatro intentos fallidos. No se ocultan dentro de la tasa nominal.

Cada corrida incluye inicialización, eventos, CSV, resultado, video de cámara y
huellas de los archivos fuente. source_dirty=true indica correctamente que se
probó el árbol modificado, no el commit base sin estos cambios.
Las mallas oficiales y las seis articulaciones pasan la regresión mecánica.

La cámara publica a 60 Hz; los videos son CFR 60 fps con repetición de la última
imagen recibida ante huecos de entrega. Los videos H.264 para inspección están
en [captures/gripper_corregido](../../../captures/gripper_corregido/).
[Informe](../../../docs/gripper_fix_report.md).
