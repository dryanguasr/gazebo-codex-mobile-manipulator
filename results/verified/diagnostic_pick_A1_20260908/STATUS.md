# Estado del diagnóstico

Estado: **INVALID**

SHA ejecutado: a00fc0a4e5f51c8754515c24867f8d0d2d307384.

Los gates CAD, URDF, controladores, cámara, odometría, movimiento de base y la
primera pose articular avanzaron. El proceso se detuvo antes de producir el
resumen final porque una invocación nueva de tf2_echo no descubrió
base_footprint dentro de su única ventana de 4 s. El probe inmediatamente
anterior sí registró la transformación de poppy_moving_tip.

La evidencia se conserva y no cuenta como PASS. La causa observada es
descubrimiento DDS/TF transitorio entre procesos cortos, no una ausencia
persistente del frame. scripts/run_diagnostic.sh se corrigió con tres intentos
acotados por consulta, cada uno con timeout, y el cierre se evalúa en un
directorio de retry independiente.
