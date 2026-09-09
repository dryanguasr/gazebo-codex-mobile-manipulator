# Pick-and-place móvil — evidencia a 60 fps CFR

Estaciones separadas **1,439 m**, recorrido medido del carro **1,682 m**,
reorientación final **89,8°** y rejilla visual de **20 cm**.
La unión temporal se activa tras verificar contacto bilateral; fija la pieza
respecto al gripper durante el transporte y se retira al depositarla.

| Video | Duración | Cámara |
|---|---:|---|
| [Ciclo completo](ciclo_completo_60fps.mp4) | 89.82 s | overview |
| [Detalle del gripper: ciclo completo](detalle_gripper_60fps.mp4) | 89.83 s | robot_relative |
| [Recogida y elevación](recogida_60fps.mp4) | 12.03 s | robot_relative |
| [Traslado y giro](traslado_y_giro_60fps.mp4) | 57.05 s | overview |
| [Depósito y liberación](deposito_60fps.mp4) | 13.53 s | robot_relative |


H.264 / yuv420p; los cinco archivos se verificaron con ffprobe y decodificación
completa sin errores. No hay aceleración ni interpolación de movimiento.
**60 fps CFR no significa 60 imágenes nuevas cada segundo**: el grabador repite
la última imagen cuando falta una entrega de cámara. La vista general recibió
3438 imágenes y produjo 5389 frames; el detalle recibió 4723 y produjo 5390.
Ambas cámaras solicitan 60 Hz y la temporización procede de sus stamps simulados.

La vista general es fija y permite comparar estaciones y rejilla. La vista
de detalle acompaña al carro: su referencia es móvil, no debe usarse sola
para inferir desplazamiento mundial. Las marcas del cilindro son solo visuales
y permiten observar la orientación pieza/gripper.

La corrida grabada pasó todos los criterios: depósito a 2,3 mm del centro,
estabilidad durante 4,69 s y variación de orientación relativa máxima de 0,37°
(incluye desfase de medición). Se registró un attach y un detach, sin colisiones
indebidas. Esto es agarre asistido y localización simulada, no una prueba de
fricción física ni de navegación perceptiva.

[Entrega final](entrega_final.jpg) · [Pieza sujeta](pieza_sujeta.png) ·
[Llegada](vista_general_llegada.png) · [Manifest y hashes](manifest.json).

[Guía reproducible](../../docs/mobile_pick_and_place.md) ·
[Corrida fuente](../../results/verified/mobile_pick_place_20260909/run_02/run.json) ·
[Campaña y regresiones](../../results/verified/mobile_pick_place_20260909/README.md).
