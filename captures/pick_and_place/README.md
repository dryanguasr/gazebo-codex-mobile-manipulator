# Videos de pick-and-place a 60 fps

Estos archivos proceden de una corrida nueva de la cámara real de Gazebo,
ejecutada con el commit
`7a083fad9a27fada89f697c33cf886dc4c57df15` y seed 911.

| Archivo | Operación | Duración |
|---|---|---:|
| `ciclo_completo_60fps.mp4` | ciclo A1 completo | 31.43 s |
| `agarre_y_elevacion_60fps.mp4` | pregrasp, contacto bilateral, cierre y lift | 9.00 s |
| `transferencia_60fps.mp4` | hold, transferencia e inicio del descenso | 7.50 s |
| `deposito_y_liberacion_60fps.mp4` | descenso, depósito, apertura y retreat | 8.90 s |

Todos son 960×720 y 60 fps constantes. El ciclo completo conserva la salida
MPEG-4 del grabador; los recortes están codificados en H.264 para reproducción
amplia. `manifest.json` contiene hashes SHA-256, conteos de frames, ventanas
temporales y el resultado del evaluador.

La cámara publica a 60 Hz. El grabador usa el timestamp simulado y repite el
último frame real solo cuando existe un hueco de entrega; no interpola
movimiento ni genera imágenes sintéticas. La diferencia entre duración simulada
y codificada es de aproximadamente 9 ms.

Los PNG `contacto_60fps.png` y `liberacion_60fps.png` identifican las fases
`VERIFY_GRASP` y `RETREAT` usadas al seleccionar los recortes.
