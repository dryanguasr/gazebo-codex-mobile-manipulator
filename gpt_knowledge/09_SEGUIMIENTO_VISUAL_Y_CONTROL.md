# Seguimiento visual y control

## Nodo

`visual_tracker` recibe `/ball/measurement` y publica `/base_controller/cmd_vel`. No recibe ground truth.

## Referencia y parámetros por defecto

- `target_distance_m = 1.2`
- `linear_gain = 0.7`
- `angular_gain = 1.8`
- `max_linear_speed_mps = 0.45`
- `max_angular_speed_radps = 1.2`
- `distance_deadband_m = 0.04`
- `horizontal_deadband = 0.02`
- `alignment_slowdown = 0.8`
- `measurement_timeout_s = 0.3`

## Control angular

Con error horizontal `e_h`:

```text
omega_cmd = saturate(-K_angular e_h)
```

El signo corresponde a la convención del sistema validado.

## Control lineal

```text
e_d = D - D_ref
v_cmd = saturate(K_linear e_d × alignment_scale)
```

La escala de alineación reduce avance cuando la esfera está descentrada:

```text
alignment_scale = 1 - min(alignment_slowdown, |e_h|)
```

## Deadbands y watchdog

Errores pequeños dentro del deadband se llevan a cero. Si la medición es inválida o pasan más de 0.3 s sin una nueva medición, se publica una orden nula.

## Por qué P

El objetivo no es demostrar el controlador más sofisticado, sino hacer visible la causalidad:

```text
error → comando → movimiento → nuevo error
```

Esto permite enseñar realimentación, ganancia, saturación, deadband y timeout.

## Lazo cerrado

```text
esfera se mueve
→ imagen cambia
→ detector produce e_h y D
→ tracker calcula v y ω
→ base se mueve
→ nueva imagen
```

## Verificación de no privilegio

En `visual_tracker.py` la única entrada de objetivo es `/ball/measurement`. No existe suscripción a `/target/ground_truth`.

## Resultado B

- comandos activos: 100%;
- desplazamiento: 0.368 m;
- RMS horizontal: 0.034;
- MAE distancia objetivo: 0.088 m;
- error estacionario: 0.083 m.

## Límites

No demuestra navegación general, evasión de obstáculos, SLAM, Nav2, control óptimo ni Sim2Real real. Demuestra un lazo visual sencillo, reproducible y medible en simulación.
