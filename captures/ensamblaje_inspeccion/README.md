# Ensamblaje corregido y colores de inspección

Esta es la evidencia visual actualizada. Se corrigió la superposición de
instancias de servomotor que dejaba la muñeca aparentemente desacoplada.
Verde = mordaza fija; magenta = mordaza móvil; gris = servomotores.

| Video | Duración | Vista |
|---|---:|---|
| [Ciclo completo](ciclo_completo_60fps.mp4) | 33,42 s | 960 x 720 |
| [Detalle de muñeca](detalle_muneca_60fps.mp4) | 6 s | Recorte ampliado de los segundos 20–26 |
| [Agarre y elevación](agarre_y_elevacion_60fps.mp4) | 9 s | 960 x 720 |
| [Retención y transferencia](retencion_y_transferencia_60fps.mp4) | 7,7 s | 960 x 720 |
| [Depósito y liberación](deposito_y_liberacion_60fps.mp4) | 10,42 s | 960 x 720 |

Todos H.264 a 60/1 fps, verificados por decodificación y ffprobe.
El detalle de muñeca aplica crop 560 x 320 y ampliación a 1120 x 640;
no añade contenido ni interpola movimiento.

[Antes, segundo 23](segundo_23.jpg) · [Después, segundo 23](segundo_23_corregido.jpg)

La nueva corrida física sin attach pasó todos los criterios. El CAD fuente,
las articulaciones y el agarre se conservan; cambia la representación visual
para que Gazebo cargue las instancias correctamente.

La cámara publica a 60 Hz. El grabador utiliza tiempo simulado y repite la
última imagen si falta una entrega: 1843 imágenes recibidas, 2005 frames CFR.
[Manifest y hashes](manifest.json).
[Diagnóstico](../../docs/assembly_render_fix.md).
[Resultado fuente](../../results/verified/assembly_render_20260909/run_01/run.json).
