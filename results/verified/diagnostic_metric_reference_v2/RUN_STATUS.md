# Estado de corrida

Estado: **INVALID / FAIL**

El diagnóstico capturó correctamente
`base_footprint -> camera_link = [0.225, 0.0, 0.120] m`, pero la consulta
`odom -> base_footprint` posterior al movimiento agotó su timeout de cuatro
segundos. No se generó `summary.json` y esta corrida no es evidencia de PASS.

El retry completo aprobado es `../diagnostic_metric_reference_v2_retry1/`.
