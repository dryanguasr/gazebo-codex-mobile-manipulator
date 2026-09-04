# Plan experimental de pick-and-place

## Propósito

Este plan será ejecutado en el hito posterior. Hoy solo define escenarios,
métricas y gates; no hay resultados de pick-and-place.

El experimento separa tres niveles:

- A: pose conocida, base inmóvil y trayectoria determinista;
- B: pose en frame de base e IK sencilla;
- C: percepción integrada sin ground truth para control.

El siguiente hito debe comenzar exclusivamente por Nivel A.

## Unidad experimental

Una corrida comienza en un snapshot reproducible y termina en `SUCCESS`,
`FAILURE` o timeout. Debe registrar:

- SHA, fecha, ROS/Gazebo, world y seed;
- modo de grasp: `attach_conditioned` o `physical_contact`;
- fuente de pose usada por control;
- pose inicial/final de base, objeto y tool;
- consignas y estados articulares;
- eventos de contacto/attach/detach;
- transiciones, timeout y motivo terminal;
- uso de ground truth para control: booleano obligatorio.

Formato recomendado: un `run.json` resumen y `events.jsonl` temporal por
corrida, más un comparador agregado.

## Objeto y escena Nivel A

Modelo propuesto `pick_object`:

| Propiedad | Valor inicial |
|---|---:|
| forma | cilindro vertical |
| diámetro | 0,030 m |
| altura | 0,045 m |
| masa | 0,030 kg |
| pose | fija, documentada, dentro del workspace |
| collision | cilindro idéntico a la forma |
| visual | color distinto de la esfera de tracking |
| fricción inicial | valor explícito en SDF, por calibrar |

La esfera de radio 0,12 m permanece en la escena de tracking o en un world
separado, pero nunca se declara agarrable.

## Métricas

| Métrica | Definición | Unidad |
|---|---|---|
| tasa de éxito de grasp | corridas que verifican retención / corridas válidas | % |
| tasa de éxito de place | objetos dentro de región final / corridas válidas | % |
| tiempo hasta grasp | command start → `VERIFY_GRASP` PASS | s |
| tiempo total | command start → estado terminal | s |
| error de pregrasp | distancia/orientación tool deseada vs observada | m, rad |
| error de placement | centro objeto final vs target | m |
| altura de lift | máximo Z objeto − Z inicial | m |
| distancia transportada | trayectoria o separación inicio-fin del objeto | m |
| pérdida de objeto | grasp validado seguido de separación no autorizada | bool/conteo |
| contactos espurios | contacto con links/base/suelo fuera de fases permitidas | conteo |
| colisiones no permitidas | eventos contra lista prohibida | conteo |
| reintentos | vuelta a PREGRASP/APPROACH por corrida | conteo |
| uso de ground truth | control leyó pose real de Gazebo | bool |
| reproducibilidad | dispersión de tiempo, error y éxito entre corridas | estadística |
| attach sin contacto | unión creada sin gate de contacto/distancia/cierre | conteo |

Para attach, éxito de grasp exige que el gate de contacto haya pasado antes de
crear la unión. Para contacto físico, exige que el objeto conserve su transform
relativo al tool durante lift y retención.

## Criterios PASS/FAIL Nivel A

Con 10 corridas válidas y mismo escenario:

| Gate | PASS |
|---|---|
| spawn/controladores/TF | 10/10 |
| apertura previa | m6 llega a 1,20 ± 0,03 rad |
| cierre | comando a 0 rad y criterio de contacto/retención |
| error posición pregrasp | ≤ 10 mm |
| error orientación pregrasp | ≤ 5 grados |
| grasp | ≥ 9/10 |
| lift | ≥ 50 mm en cada corrida exitosa |
| retención | ≥ 3 s sin pérdida |
| place | ≥ 9/10 dentro de 30 mm del target |
| contactos espurios | 0 |
| colisiones no permitidas | 0 |
| attach sin contacto | 0 |
| reintentos | ≤ 1 por corrida |
| ground truth en control | permitido y declarado solo en A |
| limpieza | sin procesos Gazebo/ROS huérfanos |

Un timeout, NaN, falta de log o corrida cuyo estado inicial no coincide se marca
`INVALID`; no se elimina silenciosamente ni cuenta como éxito.

## Criterios preliminares Nivel B

Solo después de Nivel A PASS:

- objeto dentro de workspace detectado antes de solicitar IK;
- solución respeta límites de los seis joints;
- error de pregrasp ≤ 10 mm y ≤ 5 grados;
- approach no atraviesa objeto/suelo/base;
- ≥ 9/10 grasp y place con tres poses conocidas distintas;
- ningún dato de cámara necesario para control.

## Criterios preliminares Nivel C

Solo después de Nivel B PASS:

- control usa exclusivamente estimación de cámara + odom/TF;
- `ground_truth_used_for_control=false` verificado por configuración/log;
- error de estimación de pose medido contra GT, no realimentado;
- ≥ 80 % grasp/place en al menos 20 corridas con variación documentada;
- recuperación observable ante detección perdida o pose fuera del workspace.

Estos valores son propuestas iniciales; cualquier cambio debe justificarse antes
de ver los resultados, no después.

## Matriz experimental

### Nivel A1 — attach condicionado

- 10 corridas;
- pose fija;
- base congelada;
- unión solo tras contacto/distancia/cierre;
- objetivo: validar máquina de estados, frames, trayectorias y métricas.

### Nivel A2 — contacto físico

- mismas 10 condiciones y seed cuando sea aplicable;
- sin attach;
- barrido pequeño de fricción/solver previamente definido;
- objetivo: separar fallos de control de fallos de contacto.

### Pruebas negativas obligatorias

1. objeto 240 mm: debe ser rechazado como incompatible antes de mover el brazo;
2. objeto fuera del workspace: debe terminar sin IK/approach;
3. tool frame inexistente: debe fallar en preflight;
4. objeto desplazado fuera del gate de attach: no debe adjuntarse;
5. fricción insuficiente: debe registrar pérdida, no éxito;
6. frame de pose incorrecto: debe ser rechazado por frame_id/timestamp;
7. base no congelada: no debe iniciar approach.

## Análisis

El comparador debe publicar:

- número total/válido/éxito/fallo/invalid;
- intervalos o al menos proporciones y media/desviación;
- distribución de tiempos y errores;
- fallos agrupados por estado y causa;
- comparación attach vs contacto físico;
- evidencia de que GT no fue usado por control cuando corresponda.

No se debe optimizar solo la tasa de éxito: una corrida que usa GT oculto, attach
incondicional o atraviesa collisions es FAIL aunque coloque el objeto.

## Artefactos esperados del siguiente hito

~~~text
results/verified/pick_and_place/
  configuration.json
  runs/<id>/run.json
  runs/<id>/events.jsonl
  comparison.json
captures/pick_and_place/
  pregrasp.png
  grasp.png
  lift.png
  place.png
  failure_cases/
~~~

Los nombres pueden ajustarse, pero deben conservar trazabilidad por corrida y
una salida agregada legible.

## Secuencia de aceptación

1. preflight y regresión del ensamblaje estable;
2. validar objeto/tamaño/collisions;
3. demostrar m6 abierto/cerrado sin objeto;
4. una corrida A1 observada;
5. pruebas negativas de attach;
6. lote A1;
7. una corrida A2 observada;
8. lote A2;
9. comparación y decisión;
10. solo entonces diseñar Nivel B.
