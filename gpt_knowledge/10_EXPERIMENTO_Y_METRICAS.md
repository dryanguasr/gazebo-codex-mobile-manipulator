# Trayectoria, experimento A/B y métricas

## Trayectoria determinista

Nodo `target_trajectory`; servicio `/world/ball_arena/set_pose`.

La esfera es estática físicamente y se reposiciona a 20 Hz por defecto.

### Modo `static`

```text
x(t) = centre_x
y(t) = 0
```

### Modo `moving`

```text
phase = omega t
x(t) = centre_x + A_long sin(phase)
y(t) = A_lat sin(phase/2)
```

Valores: `centre_x=2.0 m`, `A_long=0.45 m`, `A_lat=0.65 m`, `omega=0.25 rad/s`, altura `0.12 m`.

## Ground truth

Cada pose aceptada se publica en `/target/ground_truth`. Es solo para evaluación; el tracker no lo consume.

## Logger

`metrics_logger` registra por medición: tiempo, detección, error horizontal, rango estimado, referencia, error de distancia, comandos, odometría, pose objetivo, distancia ground truth y error de estimación.

El ground truth de distancia combina pose objetivo, odometría del robot, yaw, offset conocido de cámara y diferencia vertical.

## Métricas

- tasa de detección;
- MAE/RMSE de rango;
- RMS horizontal;
- MAE de distancia objetivo;
- MAE estacionario en el último 25%;
- actividad de comandos;
- spans de trayectoria;
- desplazamiento neto;
- tiempo a primera detección;
- `settling_time_s` operacional.

**Nota:** el settling implementado busca el primer punto desde el cual al menos 90% de los errores restantes están dentro de la tolerancia. No presentarlo como definición universal de tiempo de establecimiento.

## A/B

```bash
bash scripts/run_experiments.sh
```

A: esfera móvil + percepción + métricas, tracking desactivado.  
B: mismo sistema, tracking activado.

Se cambia un único factor. A no es otro controlador; es línea base sin seguimiento.

Duración: 30 s por condición; warmup: 5 s.

## Criterios automáticos principales

- ≥400 muestras útiles;
- detección ≥90%;
- spans X/Y ≥0.7 m;
- MAE de rango ≤0.15 m;
- A: comando ≤1% y desplazamiento ≤0.05 m;
- B: comando ≥80%, desplazamiento ≥0.25 m, RMS horizontal ≤0.10, error estacionario ≤0.20 m;
- B debe reducir a menos de la mitad el error de A.

## Resultado validado

| Métrica | A | B |
|---|---:|---:|
| Muestras útiles | 667 | 698 |
| Detección | 100% | 100% |
| MAE rango | 0.097 m | 0.016 m |
| RMSE rango | 0.111 m | 0.018 m |
| RMS horizontal | 0.528 | 0.034 |
| MAE distancia objetivo | 0.535 m | 0.088 m |
| Error estacionario | 0.568 m | 0.083 m |
| Comando activo | 0% | 100% |
| Desplazamiento | ~0 m | 0.368 m |
| Span X/Y | 0.900/1.011 m | 0.900/1.011 m |

Mejora B/A: **83.6%**.

Evidencia: `results/verified/experiments/` y especialmente `comparison.json`.

## Pregunta metodológica clave

¿Por qué no permitir al tracker leer la pose perfecta de Gazebo? Porque entonces dejaríamos de evaluar percepción y usaríamos información que un robot real no tendría de esa forma.
