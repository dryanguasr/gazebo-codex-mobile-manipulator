# Siguiente objetivo: pick-and-place nivel A

## Baseline obligatorio

Trabajar sobre el commit técnico validado:

`f814f0cf5c6019b943f122db74243495d1bfb8f4`

Antes de modificar funcionalidad, comprobar que el HEAD contiene también el
commit documental que introduce este archivo. No reabrir la consolidación
mecánica salvo regresión demostrada.

## Objetivo

Implementar y validar un pick-and-place reproducible de nivel A con poses
predefinidas. El sistema debe tomar un objeto conocido desde una pose conocida y
depositarlo en una segunda pose conocida, conservando el robot móvil, cámara,
percepción HSV, visual_tracker, pipeline CAD, diagnósticos y experimento A/B.

Esta etapa no autoriza MoveIt, un solucionador IK para usuario, planificación de
trayectoria general, Nav2, SLAM, plantas, frutos ni Sim2Real.

## Preflight de ensamblaje

Antes de programar el ciclo:

~~~bash
python3 scripts/cad/validate_official_consolidation.py
python3 scripts/cad/validate_mechanical_assembly.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
bash scripts/run_diagnostic.sh
~~~

Criterios de entrada:

- seis joints y controladores activos;
- comparación oficial/final PASS;
- `poppy_tool_frame` disponible;
- error tool FK/TF menor de 2 mm;
- cámara, odometría y TF PASS;
- gripper abierto `m6=1.20 rad`;
- gripper cerrado `m6=0 rad`.

Si falla cualquiera, detener el pick-and-place y registrar la regresión. No
compensarla modificando percepción o tolerancias.

## Objeto de ensayo

Usar inicialmente un cilindro:

- diámetro: 30 mm;
- altura: 45 mm;
- masa: 30 g;
- pose inicial fija, visible y accesible;
- pose de depósito fija y distinta;
- fricción e inercia declaradas.

La pinza Poppy es rotativa. La separación AABB cerrada de 2.1 mm no es su
capacidad útil: el contacto depende de altura/profundidad y del barrido de la
mordaza. Antes de la campaña de diez corridas, hacer una prueba visual y de
contacto sin ajustar ganancias simultáneamente.

## Arquitectura mínima

Separar cuatro responsabilidades:

1. descripción/world: objeto, zona de depósito y frames;
2. supervisor: máquina de estados y timeouts;
3. actuadores: trajectory controller del brazo, m6 y mecanismo de attach si se
   usa;
4. medición: ground truth únicamente para evaluación.

La cámara/percepción existente puede registrar el experimento, pero no debe
mezclarse con ground truth para producir éxito artificial.

Estados mínimos:

`IDLE → FREEZE_BASE → PREGRASP → OPEN → APPROACH → CLOSE → VERIFY_GRASP →
LIFT → TRANSFER → LOWER → RELEASE → RETREAT → DONE`

Cada estado debe tener condición de entrada, comando, condición de salida,
timeout y transición a `RECOVER`.

## Estrategia de agarre

Implementar primero A1, attach condicionado, y etiquetarlo explícitamente como
MVP no físico. Solo permitir attach cuando:

- el gripper está cerrado o cerrándose;
- objeto y `poppy_tool_frame` están dentro de una tolerancia posicional y
  angular;
- la fase actual es `VERIFY_GRASP`;
- se registra el evento con pose y timestamp.

Detach únicamente durante `RELEASE`. Un attach incondicional o por simple
tiempo es FAIL.

Después, si el contacto de Gazebo es estable, evaluar A2 físico sin attach.
Mantener resultados A1 y A2 separados. No presentar A1 como demostración de
fuerza/fricción.

## Poses y seguridad

Definir home, pregrasp, grasp, lift, transfer, place y retreat como parámetros,
no como números dispersos. Las trayectorias deben respetar límites articulares,
velocidad y aceleración configuradas.

Abortar ante:

- timeout del controller;
- error articular fuera de tolerancia;
- objeto ausente;
- distancia tool/objeto excesiva;
- caída o pérdida del objeto;
- estado TF no disponible.

La base debe permanecer inmóvil durante el ciclo inicial. El tracking de esfera
es una regresión separada, no parte del supervisor de manipulación.

## Métricas obligatorias

Por corrida guardar:

- resultado y estado terminal;
- timestamps por estado;
- pose de objeto inicial, en grasp, después de lift y final;
- pose de `poppy_tool_frame`;
- distancia tool/objeto;
- error final respecto al centro de depósito;
- altura libre después de lift;
- pérdida/caída;
- attach/detach y condición que los autorizó;
- máximos errores articulares;
- logs ROS/Gazebo.

Criterios iniciales para diez corridas consecutivas:

- 10/10 terminan sin crash;
- al menos 9/10 pick exitoso;
- al menos 9/10 place exitoso;
- error final XY <= 30 mm;
- objeto estable dentro de la zona durante 2 s;
- cero attach fuera de `VERIFY_GRASP`;
- cero detach fuera de `RELEASE`;
- cero regresiones del diagnóstico base/cámara/TF;
- comparador A/B de tracking sigue PASS.

Añadir pruebas negativas: objeto desplazado fuera de tolerancia, objeto ausente,
controller no disponible y timeout. Deben terminar en `RECOVER`/FAIL, nunca en
éxito falso.

## Evidencia

Guardar en una estructura nueva y versionada:

~~~text
captures/pick_and_place/
results/verified/pick_and_place/
~~~

Incluir como mínimo:

- home/pregrasp;
- aproximación;
- cierre;
- lift con objeto;
- depósito;
- release;
- una secuencia o video completo;
- logs y resumen JSON por corrida;
- resumen agregado de diez corridas;
- evidencia diferenciada A1/A2.

No reutilizar capturas antiguas como si correspondieran al nuevo ciclo.

## Regresión final

Ejecutar al terminar:

~~~bash
python3 scripts/cad/check_cad_dependencies.py
python3 scripts/cad/prepare_poppy_assets.py
python3 scripts/cad/convert_step_example.py
python3 scripts/cad/validate_meshes.py
python3 scripts/cad/align_poppy_to_official.py
python3 scripts/cad/validate_official_consolidation.py
python3 scripts/cad/validate_mechanical_assembly.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
bash scripts/run_diagnostic.sh
bash scripts/run_experiments.sh
~~~

El FAIL autónomo documentado del registro CAD es esperado mientras el método
final siga siendo B3. Los demás gates deben pasar.

## Entregables

- world/modelo del cilindro y zona de depósito;
- supervisor con máquina de estados;
- configuración parametrizada de poses;
- medición independiente;
- script de diez corridas y pruebas negativas;
- evidencias y resultados;
- tutorial específico;
- troubleshooting con incidencias observadas separadas de problemas comunes;
- informe final con commit validado, métricas y limitaciones;
- worktree limpio y push del commit validado.

La guía de arquitectura previa es
`docs/pick_and_place_architecture.md`; el protocolo de evaluación es
`docs/pick_and_place_experiment_plan.md`; los problemas previstos, todavía no
observados, están en `docs/pick_and_place_troubleshooting_seed.md`.
