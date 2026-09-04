# Informe final de consolidación mecánica

## Estado trazable

- baseline anterior al intento: `d5c18317df4e86b80c4dd9a8478b531cc8e82059`;
- commit técnico estable y validado: `f814f0cf5c6019b943f122db74243495d1bfb8f4`;
- CAD de hardware Poppy: `poppy-project/poppy-ergo-jr@97ce599be8c717843c45ebf48341f2ebf8f250b3`;
- descripción oficial: `poppy-project/poppy_ergo_jr_description@7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b`;
- método final: `official_reference_consolidation`;
- fallback aplicado: B3.

Este informe pertenece a un commit documental posterior. El SHA técnico anterior
es el identificador reproducible del modelo, scripts, resultados y capturas
validados; evita la referencia circular de intentar incluir en un commit su
propio SHA.

## Problema y causa raíz

El modelo anterior era sintácticamente válido, publicaba los seis joints y
aceptaba consignas, pero no representaba de forma fiable el ensamblaje físico.
Los offsets casi exclusivamente axiales, las orientaciones inferidas y la
asignación parcial de piezas dejaban brackets, motores y pivotes visualmente
desacoplados.

La causa raíz fue usar bounds globales y consistencia cinemática como sustitutos
de evidencia mecánica local. Esas pruebas no demuestran coincidencia de agujeros,
caras de contacto, horns ni pertenencia a cuerpos rígidos.

## Último intento automatizado

`scripts/cad/align_poppy_to_official.py` registró diez componentes impresos
CAD-derived contra la referencia oficial. Probó las 24 rotaciones propias de
ejes, inició por centros AABB y refinó mediante ICP recortado. Se consumió una
de las dos iteraciones globales permitidas.

| Componente | RMS Chamfer (mm) | P95 (mm) | landmark máx. (mm) |
|---|---:|---:|---:|
| base impresa | 0.505 | 0.836 | 0.017 |
| long U | 0.410 | 0.870 | 0.049 |
| lateral horn 1 | 0.097 | 0.131 | 0.003 |
| lateral body 1 | 0.210 | 0.427 | 0.003 |
| short U | 0.086 | 0.134 | 0.028 |
| lateral horn 3 | 0.097 | 0.131 | 0.003 |
| lateral body 3 | 0.210 | 0.427 | 0.003 |
| fijación gripper | 0.219 | 0.133 | 0.001 |
| mordaza fija | 0.209 | 0.197 | 0.067 |
| mordaza móvil | 0.406 | 0.983 | 0.026 |

Las piezas impresas pasan el umbral de 1.5 mm. El gate autónomo global es
**FAIL**, porque el CAD propio no incluye sólidos autoritativos completos de
XL-320, horns y tornillería. Por tanto no permite demostrar contacto de montaje
ni equivalencia superficial del conjunto completo. Este FAIL no se reinterpretó
como éxito.

## Decisión B3 y procedencia

Se consolidó el runtime con los siete DAE exactos del repositorio oficial y sus
transforms verificados. Los archivos se comprobaron byte a byte contra el commit
fijado. No se sustituyó ni eliminó el pipeline CAD propio:

- `source/`: CAD y referencias docentes;
- `visual/` y `collision/`: derivados propios reproducibles;
- `official/`: gold standard final GPL-3.0-only;
- `asset_manifest.json`: trazabilidad de derivados.

El código conserva Apache-2.0; el hardware y derivados propios, CC BY-SA 4.0;
los DAE y la descripción oficiales, GPL-3.0-only. La instalación mantiene las
licencias separadas y no copia el árbol STEP de origen al runtime.

## Joints y frames finales

| Joint | Parent → child | xyz (m) | rpy (rad) | eje |
|---|---|---|---|---|
| m1 | mount → link 1 | `0 0 0.0327993216120967` | `0 0 0` | `0 0 1` |
| m2 | link 1 → link 2 | `0 0 0.0240006783879033` | `0 -pi/2 0` | `0 0 -1` |
| m3 | link 2 → link 3 | `0.054 0 0` | `0 0 0` | `0 0 -1` |
| m4 | link 3 → link 4 | `0.045 0 0` | `0 -pi/2 0` | `0 0 -1` |
| m5 | link 4 → link 5 | `0 -0.048 0` | `0 -pi/2 0` | `0 0 1` |
| m6 | link 5 → link 6 | `0 -0.058 0` | `0 -pi/2 0` | `0 0 -1` |

La comparación oficial/final pasa para 6 joints, 7 visuales y 35 evaluaciones FK
en home, pose 1, pose 2, gripper abierto y cerrado. Error de posición:
`<=1e-9 m`; mayor residual angular numérico: `2.98e-8 rad`.

Se añadió `poppy_tool_frame` en la punta central del dedo fijo. Es estable
respecto a la apertura de m6:

- cerrado: `m6=0 rad`, gap AABB aproximado 2.1 mm;
- abierto: `m6=1.20 rad`;
- la pinza es rotativa, no paralela.

## Visuales, collisions y escala

Poppy permanece a escala física 1:1. La plataforma compacta se conserva en
`0.40 x 0.30 x 0.10 m`, masa 6 kg, ruedas de radio 0.070 m y separación
0.345 m. Cámara, mount, masa, inercia, diff-drive y odometría son coherentes con
esas dimensiones.

Los visuales finales son los DAE oficiales. Las collisions continúan
simplificadas: cilindros para mount y secciones intermedias, dos boxes en link 5
y el hull CAD-derived de 92 triángulos en link 6. No se habilitó autocolisión.

Las 17 imágenes de `captures/mechanical_assembly_final/` cubren home, dos poses,
gripper abierto/cerrado, seis close-ups, comparación oficial/final y collision
overlay. Son renders técnicos offscreen generados con los mismos DAE y
transforms del Xacro; no se presentan como screenshots de la GUI de Gazebo.
La GUI directa no estuvo disponible de forma fiable en el entorno WSLg.

## Validación ejecutada

Pipeline CAD:

~~~bash
python3 scripts/cad/check_cad_dependencies.py
python3 scripts/cad/prepare_poppy_assets.py
python3 scripts/cad/convert_step_example.py
python3 scripts/cad/validate_meshes.py
python3 scripts/cad/align_poppy_to_official.py
python3 scripts/cad/validate_official_consolidation.py
python3 scripts/cad/validate_mechanical_assembly.py
~~~

Resultado: dependencias PASS con NumPy 1.26.4, SciPy 1.11.4,
Matplotlib 3.6.3, Pillow 10.2.0 y Gmsh 4.12.2 local; STEP→mesh y meshes PASS;
registro autónomo FAIL documentado; consolidación oficial/final y ensamblaje
final PASS.

Build y pruebas:

~~~bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
~~~

Resultado: 1 paquete construido; 7 pruebas, 0 fallos, 0 errores, 0 omitidas.
Una instalación aislada adicional confirmó que no se instala `source/`, que
los siete DAE están presentes y que ambas licencias de hardware/referencia
quedan disponibles.

Diagnóstico:

- estado global: PASS;
- avance de base: 0.680 m, coherente en odometría y TF;
- cámara: `fx=554.3827 px`;
- distancia inicial estimada a la esfera: 1.7919 m;
- seis joints presentes;
- máximo error articular pose 1: `8.38e-11 rad`;
- máximo error articular pose 2: `2.04e-10 rad`;
- máximo error posición tool FK/TF: `0.784 mm`;
- máximo error de cuaternión tool: `7.41e-4`.

Experimento A/B:

| Métrica | A, sin tracking | B, tracking |
|---|---:|---:|
| detecciones válidas | 719 | 700 |
| tasa de detección | 100 % | 100 % |
| MAE distancia al objetivo | 0.52685 m | 0.11995 m |
| RMS horizontal | 0.47468 | 0.02514 |
| desplazamiento robot | ~0 m | 0.40555 m |
| settling | n/a | 5.032 s |

Mejora del error de distancia: **77.23 %**. El comparador A/B pasa.

## Problemas reales encontrados

- Los validadores sintácticos/control admitían un ensamblaje visual incorrecto.
- Faltaba geometría CAD autoritativa de motores/horns para cerrar el intento
  autónomo.
- La biblioteca compartida de Gmsh no estaba en el loader global; se usó la
  distribución local con `PATH` y `LD_LIBRARY_PATH` explícitos.
- `setup.py` intentó instalar dos README con el mismo destino; se separaron
  licencias de hardware y referencia oficial.
- Las etiquetas abierto/cerrado se habían inferido al revés; el render geométrico
  confirmó `m6=0` cerrado y `m6=1.20` abierto.
- La GUI WSLg no fue fiable; se usó evidencia offscreen declarada como tal.

## Limitaciones restantes

- No existe todavía una reconstrucción autónoma completa y demostrable sin los
  visuales oficiales; el resultado final es B3.
- No se validó contacto dinámico de agarre ni self-collision.
- No se implementaron pick-and-place, IK, MoveIt, Nav2 o SLAM.
- El objeto recomendado para la siguiente etapa es un cilindro de 30 mm de
  diámetro, 45 mm de altura y 30 g; su grasp físico aún debe ensayarse.

## Resultado de aceptación

El modelo final es geométrica y cinemáticamente equivalente al gold standard,
Poppy sigue 1:1, el carro compacto y la percepción/tracking continúan
funcionales, y las limitaciones se hacen explícitas. El failure case previo se
conserva como material pedagógico. El hito queda apto para iniciar un
pick-and-place de nivel A sin reabrir el ensamblaje.

## Handoff para actualización del tutorial ChatGPT

Orden recomendado de lectura:

1. `docs/final_mechanical_consolidation_report.md`;
2. `docs/mechanical_alignment_method.md`;
3. `results/verified/mechanical_alignment/decision.json`;
4. `results/verified/mechanical_alignment/alignment_manifest.json`;
5. `results/verified/mechanical_alignment/official_vs_final.md`;
6. `docs/mechanical_assembly_validation.md`;
7. `docs/cad_import_tutorial.md`;
8. `docs/cad_import_troubleshooting.md`;
9. `docs/pick_and_place_architecture.md`;
10. `docs/pick_and_place_experiment_plan.md`;
11. `docs/pick_and_place_troubleshooting_seed.md`;
12. `docs/next_goal_pick_and_place.md`;
13. `results/verified/diagnostic/summary.json`;
14. `results/verified/experiments/comparison.json`;
15. `captures/mechanical_assembly_final/capture_manifest.json`.
