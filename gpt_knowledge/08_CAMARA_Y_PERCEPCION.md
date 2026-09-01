# Cámara, intrínsecos y percepción

## Sensor

- 640×480 px;
- FOV horizontal 1.047 rad;
- 30 Hz;
- cámara pinhole ideal.

Topics: `/camera/image_raw` y `/camera/camera_info`.

## Detector

Nodo `ball_detector`, archivo `mobile_manipulator/ball_detector.py`.

Publica `/ball/measurement` y `/ball/debug`.

## Pipeline HSV

1. BGR → HSV;
2. dos máscaras de rojo;
3. contornos;
4. contorno de mayor área;
5. círculo mínimo;
6. centro `(u,v)` y radio aparente `r`;
7. errores normalizados;
8. profundidad/rango;
9. publicación de medición e imagen anotada.

Parámetros configurables: radio de esfera, radio mínimo en píxeles, saturación/valor mínimos, límites de hue rojo y FOV de fallback.

## Error horizontal

```text
e_x = (u - width/2)/(width/2)
```

Cero significa centrado.

## Intrínsecos

El detector usa preferentemente `CameraInfo.K`: `fx`, `fy`, `cx`, `cy`.

Valor observado: **fx = 554.383 px**.

Con ancho 640 y FOV 1.047:

```text
fx = width / [2 tan(FOV/2)]
```

≈554 px.

La versión inicial usaba `fx=320`; podía ejecutar y producir números, pero la geometría métrica era inconsistente.

## Estimación para esfera conocida

Radio real: `R=0.12 m`.

```text
alpha = atan(r/fx)
Z = R/sin(alpha)
```

`Z` es profundidad óptica.

Para rango Euclídeo cuando la esfera está fuera del eje:

```text
x_ray = (u-cx)/fx
y_ray = (v-cy)/fy
D = Z sqrt(1 + x_ray² + y_ray²)
```

El detector publica `D`, no solo `Z`.

## Medición

`Vector3Stamped`:

- X: error horizontal normalizado;
- Y: error vertical normalizado;
- Z: rango en metros;
- `NaN`: detección inválida.

Si `CameraInfo` aún no llegó, existe un fallback derivado de resolución/FOV; la ruta preferida es `CameraInfo`.

## Validación

- MAE de rango A: 0.097 m.
- MAE de rango B: 0.016 m.
- RMSE B: 0.018 m.

A tiene mayor error porque la esfera recorre zonas más oblicuas de la imagen; aun así queda bajo el umbral de 0.15 m.

## Limitaciones

Color/iluminación/radio conocidos, esfera visible completa, cámara ideal, un solo objetivo y sin oclusiones deliberadas. No extrapolar directamente a detección de frutos reales.
