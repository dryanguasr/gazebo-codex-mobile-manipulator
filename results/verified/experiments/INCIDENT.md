# Incidencia de referencia métrica

Estado: **AFECTADO — conservar como evidencia histórica**

Los archivos de este directorio fueron generados antes de corregir la
extrínseca usada por `metrics_logger.py`. El evaluador empleó
`camera_offset_x_m=0.38` y `camera_height_m=0.51`, valores que no
corresponden al chasis compacto consolidado en
`00c60529268c1786451893119e4bb150777eb701`.

Consecuencias:

- los CSV originales no se modifican;
- detección HSV, error horizontal, actividad de comandos y movimiento de base
  siguen siendo evidencia útil;
- el MAE de distancia física, el error estacionario y el porcentaje de mejora
  publicados no se usan para aceptar la referencia corregida;
- la campaña de reemplazo se guarda en un directorio nuevo con subdirectorios
  independientes `tracking_A` y `tracking_B`.

La corrección obtiene la geometría de `camera_link` mediante el TF más reciente,
valida su edad contra el sello de la imagen y rechaza poses o transformaciones
ausentes/obsoletas. Su procedencia se declara como referencia basada en pose
aceptada + TF odométrico, no como ground truth independiente del estado real del
simulador.

La campaña sustitutiva aprobada está en
`../tracking_metric_reference_v2_20260904_retry1/`: A y B lograron 100% de
referencias válidas y la mejora corregida fue 92.43%.
