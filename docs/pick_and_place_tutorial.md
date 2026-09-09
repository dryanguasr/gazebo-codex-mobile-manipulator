# Tutorial reproducible de pick-and-place nivel A

Versión estacionaria e histórica. Para el ejercicio con traslado, giro y rejilla,
consulte [Pick-and-place móvil](mobile_pick_and_place.md).

Nota posterior: el problema de agarre del video A1 fue corregido. Consulte
[el informe del gripper](gripper_fix_report.md) y los
[videos físicos actualizados](../captures/gripper_corregido/).
Los resultados A1/A2 de este documento corresponden a la revisión histórica.

## Qué demuestra este hito

El proyecto dispone de un ciclo determinista de manipulación en Gazebo Sim 8,
separado del world de seguimiento visual. El nivel A1 toma un cilindro conocido
desde una pose conocida, lo eleva al menos 50 mm, lo retiene al menos 3 s y lo
deposita dentro de una región de 30 mm. La base permanece inmóvil.

A1 es un MVP **asistido por simulador**: después de comprobar contacto fresco
en ambos dedos, geometría, cierre, estado, velocidad de base y TF, crea una
unión temporal. No demuestra retención puramente física. La variante A2 elimina
esa unión y se documenta por separado.

El seguimiento B3 anterior sigue disponible con `sim.launch.py`. La geometría
normal conserva sus colliders originales; los pads de contacto solo se activan
con `manipulation_enabled:=true`. El ensamblaje mantiene escala Poppy 1:1 y
exactamente siete DAE oficiales.

## Escena y convenciones

`pick_and_place.sdf` contiene dos pedestales estrechos: el azul de recogida en
`(-0.044, -0.225)` m y el verde de depósito en `(0.044, -0.213)` m, ambos con
superficie a `z=0.140` m. El cilindro naranja se define en
`pick_object.sdf`:

| Propiedad | Valor |
|---|---:|
| diámetro | 0.030 m |
| altura | 0.045 m |
| masa | 0.030 kg |
| inercia `ixx=iyy` | 0.00000675 kg·m² |
| inercia `izz` | 0.000003375 kg·m² |
| fricción `mu=mu2` | 0.85 |

La física usa paso de 1 ms. La pinza Poppy es rotativa: `m6=1.20 rad` es
abierta y el comando de cierre es `m6=0 rad`; el objeto detiene físicamente la
mordaza alrededor de 0.69 rad. `poppy_tool_frame` representa la punta fija para
FK y diagnóstico; `poppy_grasp_frame`, centrado entre los dedos, es el marco del
gate de agarre. No son intercambiables.

## Preparación

~~~bash
sudo apt update
sudo apt install python3-rosdep ros-jazzy-ros-gz ros-jazzy-gz-ros2-control gmsh

source /opt/ros/jazzy/setup.bash
cd ~/proyectos/gazebo-tutorial/gazebo-codex-mobile-manipulator
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
~~~

## Una corrida A1 manual

Use una carpeta nueva: el launch no debe sobrescribir evidencia previa.

~~~bash
OUT="$PWD/results/manual/pick_a1_$(date -u +%Y%m%dT%H%M%SZ)"
SHA="$(git rev-parse HEAD)"
DIRTY=false
test -z "$(git status --porcelain)" || DIRTY=true

ros2 launch mobile_manipulator pick_and_place.launch.py \
  output_dir:="$OUT" run_id:=manual_a1 seed:=901 \
  source_sha:="$SHA" source_dirty:="$DIRTY" \
  capture_evidence:=true
~~~

La inicialización `INITIALIZE_RESET` ocurre antes del intervalo medido. Es
obligatoria porque `DetachableJoint` puede aparecer unido al crearse su hijo:
el inicializador manda detach, exige feedback `detached` y pose fresca y deja
`initialization.json`. Solo entonces arrancan gate, evaluador y supervisor.

El recorrido nominal exacto es:

~~~text
IDLE → FREEZE_BASE → OPEN → PREGRASP → APPROACH → CLOSE → VERIFY_GRASP
     → LIFT → HOLD → TRANSFER → LOWER → RELEASE → RETREAT → DONE
~~~

`RECOVER` conduce a `FAILED` o `CANCELLED`. Si el objeto está unido, primero lo
baja a `grasp` o `place`; solo después autoriza `RELEASE`. Si falla ese descenso,
retiene el objeto y registra `recovery_release_withheld`.

## Contrato de estados

| Estado | Entrada y comando | Evidencia para salir | Timeout/recuperación |
|---|---|---|---|
| `IDLE` | nodos iniciados; espera 0.5 s con `auto_start` | orden de inicio | no aplica |
| `FREEZE_BASE` | publica velocidad cero | odom, seis joints y action listos durante 1.5 s | `RECOVER` |
| `OPEN` | trayectoria `home`, m6=1.20 | resultado y error m6 ≤0.03 rad | `RECOVER` |
| `PREGRASP` | tres waypoints de despeje | action OK y joints dentro de 0.035 rad | `RECOVER` |
| `APPROACH` | trayectoria `grasp` | action OK y tolerancia articular | `RECOVER` |
| `CLOSE` | trayectoria `close`, objetivo m6=0 | feedback m6≤0.72 y ≥0.2 s | `RECOVER` |
| `VERIFY_GRASP` | mantiene cierre y publica contexto al gate | A1: joint unido; A2: contacto físico verificado | `RECOVER` sin attach por tiempo |
| `LIFT` | trayectoria `lift`, conserva m6 observado | action OK y joints m1–m5 en tolerancia | bajar y recuperar |
| `HOLD` | mantiene el agarre 3.2 s | A1 sigue unido durante todo el hold | bajar y recuperar |
| `TRANSFER` | trayectoria `transfer` | action OK | bajar a `place` y recuperar |
| `LOWER` | trayectoria `place` | action OK | bajar/retener según resultado |
| `RELEASE` | única liberación nominal; detach y `release_open` | desligado, apertura completa y estabilidad 2.2 s | `RECOVER` |
| `RETREAT` | trayectoria `retreat` | action OK | `RECOVER` |
| `DONE` | emite resultado terminal | cierre gestionado de Gazebo | — |

Cada estado no terminal usa `state_timeout_s=15`. Un watchdog de pared de 3 s
detecta reloj simulado detenido. El supervisor acepta como máximo un reintento
de un goal rechazado transitoriamente; no repite una corrida nominal completa.
En todo estado de manipulación exige velocidad de base ≤0.01 m/s y deriva total
≤0.01 m.

## Gate A1

La unión solo se publica en `VERIFY_GRASP` cuando coinciden todas estas
condiciones:

- contacto con el dedo fijo y el móvil, ambos frescos ≤0.10 s;
- persistencia bilateral ≥0.15 s;
- error del `poppy_grasp_frame` ≤0.020 m y ≤5°;
- cierre comandado y feedback m6≤0.78 rad;
- base detenida ≤0.01 m/s;
- TF válido y fresco ≤0.10 s;
- objeto presente y attach habilitado.

La pose real `/model/pick_object/pose` se declara como ground truth. Se usa en
el gate geométrico A1 y en evaluación, pero el supervisor no la usa para elegir
trayectorias (`ground_truth_used_for_control=false`). Por ello A1 se etiqueta
`attach_conditioned`, `simulator_assisted=true`.

## Campaña oficial y negativas

Ejecute diez intentos consecutivos, sin reemplazar fallos:

~~~bash
python3 tools/run_pick_place_campaign.py \
  --runs 10 --seed-start 601 --timeout-s 180 \
  --capture-index 1 \
  --output-root results/verified/pick_A1_NUEVA
~~~

El runner exige fuente limpia, exactamente diez intentos, inicialización válida,
configuración idéntica, logs completos y ausencia de residuos. Acepta ≥9 grasp
y ≥9 place, pero también exige 10/10 terminaciones limpias.

~~~bash
python3 tools/run_pick_place_negative_tests.py \
  --seed-start 701 --timeout-s 180 \
  --output-root results/verified/pick_A1_negative_NUEVA
~~~

Las siete negativas son: objeto ausente, objeto incompatible de 0.24 m,
proximidad sin contactos válidos, TF inválido, controlador no disponible, base
en movimiento y cancelación durante transporte. Ninguna puede producir un
éxito falso; la cancelación debe bajar y liberar de forma segura.

## A2 físico, sin attach

~~~bash
ros2 launch mobile_manipulator pick_and_place_a2.launch.py \
  output_dir:="$PWD/results/manual/pick_a2" \
  run_id:=manual_a2 seed:=902 source_sha:="$(git rev-parse HEAD)"
~~~

A2 fija `attach_enabled=false` y `grasp_mode=physical_contact`. El protocolo
autoriza como máximo dos configuraciones previamente delimitadas. El resultado
oficial no separó el cilindro del soporte, de modo que se detuvo tras la primera
configuración y se conservó A1 como MVP asistido; no ajuste fricción o solver
después de ver el fallo y no presente `DONE` del supervisor como éxito físico.

## Lectura de artefactos

Cada corrida contiene `initialization.json`, `run.json`, `events.jsonl`,
`samples.csv` y `launch.log`. `run.json` es la decisión del evaluador Gazebo;
los estados terminales del supervisor no sustituyen sus criterios físicos. La
primera corrida oficial añade seis PNG, `media_manifest.json` y un MP4 de la
simulación real en `a1_01/media/`.

Antes de aceptar una campaña compruebe `source_sha`, `source_dirty=false`, hash
de configuración, conteos attach/detach, contactos prohibidos, lift, hold,
placement, estabilidad y `process_residue=[]`.

## Regresión completa

~~~bash
python3 scripts/cad/check_cad_dependencies.py
python3 scripts/cad/prepare_poppy_assets.py
python3 scripts/cad/convert_step_example.py
python3 scripts/cad/validate_meshes.py
python3 scripts/cad/align_poppy_to_official.py
python3 scripts/cad/validate_official_consolidation.py
python3 scripts/cad/validate_mechanical_assembly.py
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
bash scripts/run_diagnostic.sh results/verified/diagnostic_NUEVO
bash scripts/run_experiments.sh results/verified/tracking_NUEVO
~~~

El alineador autónomo puede informar FAIL mientras el método final continúe
siendo `official_reference_consolidation` (B3); los validadores oficial/final y
mecánico deben pasar. Compruebe además que hay exactamente siete DAE en
`meshes/poppy_ergo_jr/official/` y que no queda `gz sim ...ball_arena.sdf`.
