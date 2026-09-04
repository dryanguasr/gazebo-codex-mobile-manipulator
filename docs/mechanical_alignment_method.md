# Método de alineación mecánica y consolidación final

## Alcance y baseline

Este documento registra el último intento automatizado de reconstrucción independiente del Poppy Ergo Jr y el gate que decidió el modelo final.

- baseline anterior a cualquier cambio: `d5c18317df4e86b80c4dd9a8478b531cc8e82059`;
- CAD de hardware: `poppy-project/poppy-ergo-jr@97ce599be8c717843c45ebf48341f2ebf8f250b3`;
- referencia de ensamblaje: `poppy-project/poppy_ergo_jr_description@7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b`;
- máximo permitido: dos iteraciones globales;
- iteraciones realmente usadas: una.

La captura [robot_pose_a_isometric.png](../captures/cad_import/robot_pose_a_isometric.png) conserva el failure case previo: Xacro válido, control activo, bounding boxes plausibles y FK consistente, pero brackets, motores y pivotes visualmente desacoplados.

## Por qué los bounds no bastan

Una caja envolvente solo describe extremos globales. Dos piezas pueden compartir dimensiones y centro, pero diferir en:

- el lado que contiene el horn;
- el centro de un agujero;
- la normal de una cara de contacto;
- el offset entre motor y bracket;
- el cuerpo rígido al que pertenecen;
- una rotación o reflexión local.

Por eso el gate no acepta una alineación únicamente porque sus bounds coincidan.

## Transformaciones que se mantuvieron separadas

Para cada joint/link se auditaron por separado:

1. `T_parent_joint`: `<joint><origin>`;
2. eje del joint: `<axis xyz>`;
3. movimiento `R_axis(q)` y frame del child;
4. `T_link_visual`: `<visual><origin>`;
5. `T_link_collision`: `<collision><origin>`.

El intento autónomo expresa además `T_source_CAD_link` en
[alignment_manifest.json](../results/verified/mechanical_alignment/alignment_manifest.json).
No se aplica esa matriz de nuevo en el Xacro final B3. Los DAE finales conservan
su escena original y solo usan el `visual origin` oficial explícito.

## Algoritmo reproducible del intento A

Ejecute:

~~~bash
python3 scripts/cad/align_poppy_to_official.py
~~~

El script:

1. lee los STL CAD-derived propios en metros;
2. convierte cada escena Collada oficial a triángulos mediante
   `scripts/cad/collada_io.py`;
3. muestrea puntos determinísticamente;
4. prueba las 24 rotaciones rígidas propias formadas por permutaciones/signos de
   ejes;
5. inicia la traslación con centros AABB;
6. refina con 18 iteraciones ICP point-to-point y conserva el 85 % de inliers;
7. mide Chamfer simétrico, P95, máximo y siete landmarks AABB;
8. registra matriz 4x4, traslación, RPY, hashes, confianza y observaciones.

La descripción oficial fue objetivo de medición; sus joints no se usaron como
semilla de registro.

Dependencias: NumPy y SciPy/Qhull declarados en `package.xml`. El parser DAE
usa la biblioteca estándar. Matplotlib y Pillow, también declarados, se usan en
la evidencia offscreen.

## Resultados por componente impreso

| Pieza | RMS Chamfer | P95 | Residual landmark | Resultado |
|---|---:|---:|---:|---|
| base impresa | 0,505 mm | 0,836 mm | 0,017 mm | PASS |
| long U | 0,410 mm | 0,870 mm | 0,049 mm | PASS |
| lateral horn, sección 1 | 0,097 mm | 0,131 mm | 0,003 mm | PASS |
| lateral body, sección 1 | 0,210 mm | 0,427 mm | 0,003 mm | PASS |
| short U | 0,086 mm | 0,134 mm | 0,028 mm | PASS |
| lateral horn, sección 3 | 0,097 mm | 0,131 mm | 0,003 mm | PASS |
| lateral body, sección 3 | 0,210 mm | 0,427 mm | 0,003 mm | PASS |
| fijación de gripper | 0,219 mm | 0,133 mm | 0,001 mm | PASS |
| mordaza fija | 0,209 mm | 0,197 mm | 0,067 mm | PASS |
| mordaza móvil | 0,406 mm | 0,983 mm | 0,026 mm | PASS |

El manifest 4x4 completo es la fuente normativa. Dos hallazgos que los offsets
manuales previos no contenían fueron:

- `short_U`: traslación aproximada `[-0.023972, 0, 0] m`;
- segundo conjunto lateral: traslación aproximada `[0, +0.006000, 0] m` y
  cambio de orientación;
- base impresa: corrección aproximada de `+0.004195 m` en Z.

## Landmarks y límite de la evidencia

Los landmarks disponibles fueron centros/extremos de las superficies registradas,
centros de envolventes de horns impresos, caras de soporte, patrones repetidos y
ejes inferidos de simetría. Todos los componentes impresos quedaron por debajo
de 1,5 mm.

Sin embargo, el conjunto CAD propio no contiene sólidos autoritativos completos
de los XL-320, horns y tornillería. Las cajas grises aproximadas no permiten
probar contacto entre cara del motor, horn y bracket. En consecuencia:

- landmark impreso ≤ 1,5 mm: PASS;
- eje joint/horn ≤ 1,0 mm usando el frame oficial: PASS;
- error angular ≤ 1 grado usando el eje oficial: PASS;
- geometría autoritativa de contacto motor/horn disponible: FAIL;
- equivalencia visual independiente a nivel de superficie: FAIL.

El resultado del intento A es por ello **FAIL**. No fallaron las piezas impresas;
falló la suficiencia del conjunto fuente para demostrar todo el ensamblaje.

## Gate y fallback B3

[decision.json](../results/verified/mechanical_alignment/decision.json) fija:

~~~json
{
  "autonomous_attempt_status": "FAIL",
  "method_final": "official_reference_consolidation",
  "fallback_stage": "B3"
}
~~~

Se activó B3 inmediatamente, sin una segunda iteración global indefinida. Los
siete DAE oficiales se copiaron sin modificación geométrica y con hashes
fijados. La procedencia es:

| Link final | Asset runtime | Procedencia |
|---|---|---|
| `poppy_mount_link` | `official/base.dae` | oficial GPL-3.0-only |
| `poppy_link_1` | `official/long_U.dae` | oficial GPL-3.0-only |
| `poppy_link_2` | `official/section_1.dae` | oficial GPL-3.0-only |
| `poppy_link_3` | `official/section_2.dae` | oficial GPL-3.0-only |
| `poppy_link_4` | `official/section_3.dae` | oficial GPL-3.0-only |
| `poppy_link_5` | `official/section_4.dae` | oficial GPL-3.0-only |
| `poppy_link_6` | `official/gripper.dae` | oficial GPL-3.0-only |

Las mallas propias de `visual/`, `collision/` y todos los CAD de `source/`
se conservan como material docente y para reproducibilidad. No se presentan como
los visuales finales.

El código permanece Apache-2.0; el hardware/derivados propios mantiene CC BY-SA
4.0; el conjunto oficial DAE y la copia de su Xacro permanecen
GPL-3.0-only con licencia instalada.

## Transforms finales

### Joints: `T_parent_joint` y eje

| Joint | Parent → child | xyz (m) | rpy (rad) | eje |
|---|---|---|---|---|
| m1 | mount → link 1 | `0 0 0.0327993216120967` | `0 0 0` | `0 0 1` |
| m2 | link 1 → link 2 | `0 0 0.0240006783879033` | `0 -pi/2 0` | `0 0 -1` |
| m3 | link 2 → link 3 | `0.054 0 0` | `0 0 0` | `0 0 -1` |
| m4 | link 3 → link 4 | `0.045 0 0` | `0 -pi/2 0` | `0 0 -1` |
| m5 | link 4 → link 5 | `0 -0.048 0` | `0 -pi/2 0` | `0 0 1` |
| m6 | link 5 → link 6 | `0 -0.058 0` | `0 -pi/2 0` | `0 0 -1` |

### Visual: `T_link_visual`

| Link | xyz (m) | rpy (rad) |
|---|---|---|
| mount | `0 0 0` | `0 0 0` |
| link 1 | `0 0 0` | `0 0 0` |
| link 2 | `0 0 0` | `0 +pi/2 0` |
| link 3 | `0 0 0` | `0 +pi/2 0` |
| link 4 | `0 0 0` | `0 pi 0` |
| link 5 | `0 -0.058 0` | `0 -pi/2 0` |
| link 6 | `0 0 0` | `0 0 0` |

### Collision: `T_link_collision`

- mount: cilindro `r=.08, l=.03`, origen `0 .038 .015`;
- link 1: cilindro `r=.02, l=.02`, origen `0 0 .01`;
- links 2 y 3: cilindro `r=.02, l=.05`, origen `.025 0 0`,
  `rpy=0 pi/2 0`;
- link 4: cilindro `r=.02, l=.05`, origen `0 -.025 0`,
  `rpy=pi/2 0 0`;
- link 5: dos boxes para motor/fijación y dedo fijo;
- link 6: hull convexo propio de 92 triángulos.

Estas colisiones son deliberadamente simplificadas; no copian la triangulación
visual ni habilitan autocolisión.

## Comparación oficial versus final

Ejecute:

~~~bash
python3 scripts/cad/validate_official_consolidation.py
~~~

El resultado es PASS para 6 joints, 7 visuales y 35 filas FK en home, pose 1,
pose 2, gripper abierto y cerrado. Las posiciones son idénticas dentro de
`1e-9 m`; el mayor residual angular numérico observado es
`2.98e-8 rad`.

`poppy_tool_frame` está fijado a la punta central del dedo fijo y no gira al
abrir m6. Esto lo hace estable para expresar futuras poses de pregrasp y grasp.

Convención verificada del gripper:

- cerrado: `m6=0 rad`, separación AABB entre caras internas ≈ 2,1 mm;
- abierto: `m6=1,20 rad`, mordaza móvil apartada;
- no es una pinza paralela: la apertura útil depende del punto de contacto.

## Evidencia visual y collision

Las 17 imágenes y sus consignas están inventariadas en
[capture_manifest.json](../captures/mechanical_assembly_final/capture_manifest.json).

- [official_home.png](../captures/mechanical_assembly_final/official_home.png) y
  [final_home.png](../captures/mechanical_assembly_final/final_home.png);
- referencia/final para pose 1 y pose 2;
- [final_home_close.png](../captures/mechanical_assembly_final/final_home_close.png);
- [final_gripper_open.png](../captures/mechanical_assembly_final/final_gripper_open.png) y
  [final_gripper_closed.png](../captures/mechanical_assembly_final/final_gripper_closed.png);
- seis close-ups consecutivos;
- [official_vs_final_overlay.png](../captures/mechanical_assembly_final/official_vs_final_overlay.png);
- [final_collision_overlay.png](../captures/mechanical_assembly_final/final_collision_overlay.png).

El renderer offscreen consume exactamente los DAE fijados y los transforms
oficiales usados por el Xacro. No se presenta como captura de GUI. La carga real
de Gazebo, TF, controladores y sensores se valida por separado en el diagnóstico.

## Resultado

El intento autónomo es un failure case útil y explícito; la consolidación final
B3 es estable, reproducible y geométricamente idéntica al gold standard. La
lección generalizable es doble:

1. registration puede recuperar frames de componentes disponibles;
2. ninguna métrica debe suplir geometría autoritativa ausente en una unión
   mecánica crítica.
