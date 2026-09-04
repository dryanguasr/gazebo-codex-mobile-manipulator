# Corrección de la referencia métrica de tracking

## Resultado

Estado: **PASS**

La referencia geométrica ya no contiene offsets duplicados de cámara. El logger
selecciona una muestra fresca de la pose aceptada del objetivo, transforma esa
posición hasta `camera_link` con TF odométrico y registra explícitamente la
procedencia y las edades. Una referencia ausente, futura u obsoleta queda
inválida; no existe fallback a números mágicos.

Esta señal es una referencia de evaluación basada en consigna aceptada y
odometría, no ground truth independiente del estado real de Gazebo. Nunca entra
al controlador.

## Evidencia

El diagnóstico aprobado
`results/verified/diagnostic_metric_reference_v2_retry1/` verificó:

- transform esperado `base_footprint -> camera_link`:
  `[0.225, 0.000, 0.120] m`;
- transform observado: `[0.225, 0.000, 0.120] m`;
- error de traslación: menor que 1.4e-17 m;
- desplazamiento de base: 0.406 m;
- focal observada: 554.383 px;
- FK independiente frente a TF de herramienta: error menor que 0.784 mm.

La campaña sustitutiva
`results/verified/tracking_metric_reference_v2_20260904_retry1/` produjo:

| Métrica | A sin tracking | B con tracking |
|---|---:|---:|
| Referencias válidas | 100% | 100% |
| Detección válida | 100% | 100% |
| MAE de rango | 0.0461 m | 0.0390 m |
| RMS horizontal | 0.4812 | 0.0251 |
| MAE a distancia objetivo | 0.6319 m | 0.0478 m |
| Error estacionario | 0.6749 m | 0.0181 m |
| Desplazamiento de base | ~0 m | 0.4053 m |

La mejora B/A fue **92.43%**. El comparador mantuvo sus ganancias y umbrales
anteriores.

## Intentos inválidos preservados

- `diagnostic_metric_reference_v2/`: el primer muestreo TF posterior al
  movimiento agotó su timeout. El retry completo pasó.
- `tracking_metric_reference_v2_20260904/`: consultar TF en el sello exacto
  desde el mismo executor dejó 0% de referencias válidas. Se sustituyó por el
  TF más reciente con gate de edad máximo de 0.10 s; el retry completo pasó.

Los directorios inválidos contienen `RUN_STATUS.md` y no deben combinarse con
la evidencia aprobada.

## Reproducibilidad

```bash
bash scripts/run_diagnostic.sh results/verified/diagnostic_metric_reference_v2_retry
bash scripts/run_experiments.sh results/verified/tracking_metric_reference_v2
```

Ambos runners rechazan sobrescribir un directorio existente. El runner A/B crea
subdirectorios independientes `tracking_A` y `tracking_B`.
