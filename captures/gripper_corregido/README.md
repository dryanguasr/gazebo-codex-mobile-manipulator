# Gripper corregido: contacto físico, 60 fps

Actualización posterior: estos videos verifican el agarre, pero conservan el
defecto visual de instancias de servomotor superpuestas. Para inspeccionar el
ensamblaje, usar los [videos coloreados actuales](../ensamblaje_inspeccion/).

Videos de una corrida real de Gazebo **sin attach ni unión fija al objeto**.
La pinza Poppy tiene una mordaza fija y una móvil; el cilindro queda apoyado
entre sus superficies visibles. Se conserva el CAD oficial.

| Video | Contenido | Duración |
|---|---|---:|
| [Ciclo completo](ciclo_completo_60fps.mp4) | Secuencia completa, incluido depósito | 33,13 s |
| [Agarre y elevación](agarre_y_elevacion_60fps.mp4) | Aproximación, cierre bilateral y lift | 9,00 s |
| [Retención y transferencia](retencion_y_transferencia_60fps.mp4) | Pausa con carga y traslado | 7,70 s |
| [Depósito y liberación](deposito_y_liberacion_60fps.mp4) | Descenso, apertura y retirada | 10,13 s |

Todos: H.264, 960 x 720, 60/1 fps verificados con ffprobe y decodificación completa.
Cámara cercana renderizada por Gazebo; no hay animación generada ni interpolación
de movimiento. El grabador conserva el tiempo simulado y repite el último frame
real cuando falta una entrega: 1872 imágenes recibidas y 1988 frames CFR en el
ciclo completo. Por tanto, 60 fps de archivo no significa 60 imágenes distintas
en cada segundo. Las ventanas y hashes están en [manifest.json](manifest.json).

La corrida fuente, seed 402, elevó el objeto 62,92 mm, lo mantuvo 3,20 s y lo
depositó con error XY de 3,58 mm. La evaluación física pasó todos los criterios;
la segunda repetición, seed 403, también pasó.

- [Resultado fuente](../../results/verified/gripper_fix_20260908/run_01/run.json)
- [Diagnóstico y cambios](../../docs/gripper_fix_report.md)
- [Contacto](contacto.png), [retención](retencion.png), [liberación](liberacion.png)

Los [videos anteriores](../pick_and_place/) se conservan como evidencia histórica
asistida; no deben usarse para evaluar la corrección física del gripper.
