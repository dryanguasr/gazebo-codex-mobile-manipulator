# Protocolo experimental ejecutado de pick-and-place

## Propósito y congelamiento

El experimento separó dos niveles sobre la misma escena y poses:

- **A1:** attach temporal solo después del contrato de agarre;
- **A2:** contacto físico con attach deshabilitado.

La configuración de A1 se congeló en
`src/mobile_manipulator/config/pick_place_a1.yaml`. La campaña oficial usa el
hash SHA-256
`798bf4eb4adab93f291b7f922da1ebefbe3527c6675545d1bc08832591685e5b`,
seeds 601–610 y fuente limpia
`1d03fc59d05a7de8b95bd01e40a0db0e70bcd8ef`. No se reemplazaron intentos.

## Unidad experimental

Una corrida comienza después de `INITIALIZE_RESET` verificado y termina en
`DONE`, `FAILED` o `CANCELLED`. El reset no forma parte de las métricas. Cada
intento registra:

- SHA, dirty flag, fecha, ROS, Gazebo, world, seed y hash de configuración;
- modo de grasp y fuente de control;
- poses inicial/final y muestras de base, objeto, tool y grasp frame;
- consignas, feedback y máximo error articular;
- contactos, autorizaciones attach/detach y transiciones;
- lift, clearance, hold, estabilidad, placement y contactos prohibidos;
- causas de timeout/fallo y limpieza del grupo de procesos;
- uso de ground truth en control, gate y evaluación por separado.

Los JSON usan `null` para datos ausentes/no finitos. Un NaN, log faltante,
inicialización inválida, fuente dirty o residuo clasifica la corrida como
`INVALID`.

## Escena congelada

| Parámetro | Valor |
|---|---:|
| objeto | cilindro vertical |
| diámetro / altura | 0.030 / 0.045 m |
| masa | 0.030 kg |
| fricción `mu=mu2` | 0.85 |
| pick XY / superficie Z | (-0.044, -0.225) / 0.140 m |
| place XY / superficie Z | (0.044, -0.213) / 0.140 m |
| paso físico | 0.001 s |
| base, límite velocidad / deriva | 0.01 m/s / 0.01 m |
| lift mínimo | 0.050 m |
| hold mínimo | 3.0 s |
| placement máximo | 0.030 m |
| estabilidad mínima | 2.0 s |

## Gates de aceptación A1

Con exactamente diez intentos consecutivos:

| Gate | Umbral |
|---|---:|
| ejecución sin crash/residuo | 10/10 |
| inicialización válida | 10/10 |
| grasp | ≥9/10 |
| place | ≥9/10 |
| lift en éxito | ≥0.050 m |
| hold en éxito | ≥3.0 s |
| estabilidad | ≥2.0 s |
| error placement XY | ≤0.030 m |
| contactos prohibidos | 0 |
| attach fuera de `VERIFY_GRASP` | 0 |
| detach nominal fuera de `RELEASE` | 0 |
| reintento de goal | ≤1 por corrida |
| GT usado por supervisor | false |

Un terminal `DONE` no basta: el evaluador debe marcar `run.status=passed`.

## Secuencia ejecutada

1. Validación CAD, build y pruebas.
2. Corrida A1 observada con captura real.
3. Diez intentos A1 consecutivos.
4. Siete negativas obligatorias.
5. Una configuración A2 sin attach, de máximo dos autorizadas.
6. Decisión A1/A2 sin tuning posterior.
7. Diagnóstico final y regresión A/B de tracking.

## Resultado nominal A1

`results/verified/pick_A1_20260908/summary.json` marca `accepted=true`:

- 10 intentos, 10 válidos, 10 éxitos;
- 10 grasp y 10 placement;
- lift medio 0.055190 m;
- error de placement medio 0.002303 m;
- hold medio 3.2025 s;
- estabilidad media 4.6651 s;
- deriva máxima de base 1.485e-13 m;
- cero contactos prohibidos y cero residuos;
- dos reintentos transitorios en total, máximo uno por corrida.

La captura se hizo únicamente en el intento 1 y no altera la física de los
otros nueve.

## Matriz negativa ejecutada

| Caso | Estímulo | Resultado esperado y observado |
|---|---|---|
| objeto ausente | `spawn_object=false` | FAIL en verify, attach 0 |
| objeto 0.24 m | SDF incompatible | FAIL en approach, attach 0 |
| proximidad | se invalida contacto fresco | FAIL en verify, attach 0 |
| TF inválido | grasp frame inexistente | FAIL en verify, attach 0 |
| controlador ausente | action name inválido | FAIL en freeze, attach 0 |
| base móvil | inyección de velocidad | FAIL por `base_speed_limit`, attach 0 |
| cancelación | cancel en transfer | bajar, detach y `CANCELLED` |

Resultado agregado:
`results/verified/pick_A1_negative_20260908/summary.json`, 7/7 aceptadas, sin
éxito falso ni residuos.

Estas negativas cubren seguridad del simulador, no percepción/IK futura. El
objeto grande es deliberadamente la dimensión de la esfera de tracking y
demuestra que no es una pieza agarrable.

## Protocolo y resultado A2

A2 reutiliza objeto, poses y seed documentados, pero fija
`attach_enabled=false`. Se autorizaron como máximo dos configuraciones para
evitar un barrido abierto. La primera fue suficiente para rechazar la hipótesis
de retención física nominal:

- contacto bilateral verificado;
- lift 0.00065056 m, menor de 0.050 m;
- no hubo separación del soporte;
- cinco contactos prohibidos;
- error de placement 0.0868995 m;
- estabilidad 0 s;
- attach/detach 0/0 y ningún residuo.

Se detuvo antes de la segunda configuración porque la trayectoria terminó sin
haber levantado el objeto; modificar física después de observarlo habría sido
tuning post hoc. A2 `status=failed` y la decisión es
`retain_A1_as_assisted_MVP`.

## Evidencia y autoridad

~~~text
results/verified/
  pick_A1_20260908/
    provenance.json
    summary.json
    a1_01/ ... a1_10/
    a1_01/media/                 seis fases + MP4 + manifiesto
  pick_A1_negative_20260908/
    summary.json
    negative_*/
  pick_A2_20260908/
    summary.json
    config_1/
~~~

`run.json` y `summary.json` del evaluador tienen autoridad sobre logs visuales y
sobre el terminal del supervisor. Los PNG/MP4 proceden de la cámara de evidencia
de Gazebo y sirven para auditoría, no para decidir éxito.

## Reproducción

~~~bash
python3 tools/run_pick_place_campaign.py \
  --runs 10 --seed-start 601 --timeout-s 180 --capture-index 1 \
  --output-root results/verified/pick_A1_NUEVA

python3 tools/run_pick_place_negative_tests.py \
  --seed-start 701 --timeout-s 180 \
  --output-root results/verified/pick_A1_negative_NUEVA
~~~

Para A2 use `pick_and_place_a2.launch.py` y una carpeta nueva. Toda repetición
debe partir de un commit limpio y conservar resultados fallidos; no sobrescriba
las carpetas oficiales.

## Decisión para el siguiente hito

A1 cumple su gate y puede conservarse como baseline de estados, frames,
trayectorias, evaluación y seguridad. A2 no cumple y bloquea por ahora los
niveles B/C. Un nuevo protocolo debe preregistrar como máximo dos variantes
físicas, cambiar una dimensión causal por vez y mantener attach deshabilitado.
Solo después de un A2 válido se justifica añadir IK y, posteriormente,
percepción del objeto sin GT para control.
