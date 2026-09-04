# Informe de cierre anterior y consolidación posterior

## Estado de este documento

Este archivo conserva el cierre del hito iniciado en
`70c5d6fc30ea60e6a96166c816fa8106855000c7`, cuyo commit técnico fue
`a9292f8faae9ca843e5464c22a99acd297edced6`.

La conclusión “ensamblaje CAD-derived final aprobado” quedó **supersedida** por
la inspección humana posterior y por el último gate automatizado. El informe
autoritativo actual es
[final_mechanical_consolidation_report.md](final_mechanical_consolidation_report.md).

## Qué corrigió correctamente el hito anterior

Se mantienen válidos:

- joint origins/axes oficiales m1–m6;
- pertenencia rígida de mordaza fija a link 5 y móvil a link 6;
- Poppy a escala 1:1;
- base compacta 0,40 × 0,30 × 0,10 m;
- ruedas de radio 0,070 m y separación 0,345 m;
- masa/inercia de plataforma;
- cámara, odometría, TF, detector, tracker y experimento A/B;
- pipeline STEP/STL y separación visual/collision.

La corrección anterior resolvió el árbol cinemático casi vertical y la
desproporción del carro. Es un paso histórico útil, no trabajo descartado.

## Qué seguía sin demostrarse

Los visuales CAD-derived se habían reexpresado con transforms heurísticos y
cajas aproximadas de servo. Aunque bounds, joints, FK y poses eran plausibles,
una vista cercana seguía mostrando dudas en:

- contacto bracket–motor;
- horn–eje;
- caras y orificios de montaje;
- duplicación/aproximación del cuerpo del servo.

El CAD propio no incluye sólidos completos de XL-320, horn y tornillería. Por
eso no era posible convertir una alineación de brackets en una prueba completa
del ensamblaje.

## Último intento y decisión

El hito final partió de
`d5c18317df4e86b80c4dd9a8478b531cc8e82059` y ejecutó:

~~~bash
python3 scripts/cad/align_poppy_to_official.py
~~~

Diez componentes impresos quedaron en PASS, con RMS Chamfer de 0,086–0,505 mm
y residual de landmarks máximo de 0,067 mm. El gate global quedó en **FAIL**
porque faltaba geometría autoritativa de contacto.

`decision.json` fijó:

- `autonomous_attempt_status=FAIL`;
- `method_final=official_reference_consolidation`;
- `fallback_stage=B3`.

No se hizo una segunda iteración de offsets ni se maquilló el FAIL.

## Modelo final

El Xacro usa los siete DAE exactos del repositorio oficial fijado en
`7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b`. Se conservan por separado:

- código: Apache-2.0;
- CAD/derivados hardware: CC BY-SA 4.0;
- DAE/Xacro oficial incorporado: GPL-3.0-only.

Los CAD-derived propios siguen versionados e instalados como material docente;
no se presentan como visual runtime final.

La comparación oficial/final produce PASS en:

- 6 joints;
- 7 visuales;
- 35 filas FK;
- home, dos poses, abierto y cerrado;
- `poppy_tool_frame` frente a `fixed_tip`.

Error de posición máximo: 0 m dentro de la precisión del comparador. Residual
angular máximo: 2,98e-8 rad por redondeo numérico.

## Collisions y plataforma

Se mantuvo visual de alta fidelidad separado de collision simplificada:
cilindros en mount/links 1–4, boxes en link 5 y hull convexo de 92 triángulos en
link 6. La autocolisión sigue deshabilitada.

La base compacta del hito anterior permanece sin variantes ambiguas:

| Parámetro | Valor |
|---|---:|
| base | 0,40 × 0,30 × 0,10 m |
| masa | 6,0 kg |
| inercia | 0,050 / 0,085 / 0,125 kg·m² |
| ruedas | 0,070 × 0,045 m |
| wheel separation | 0,345 m |
| cámara | X 0,225; Z 0,050 m |
| mount | X −0,030; Z 0,050 m |

## Regresión actual

- CAD/STEP: PASS;
- build: 1 paquete;
- tests: 7/7 PASS;
- diagnóstico: PASS;
- tool FK/TF: error de posición máximo 0,784 mm;
- odometría/TF: PASS, desplazamiento 0,680 m;
- cámara/detector: PASS;
- A/B: PASS, 100 % detección y 77,2323 % de mejora del MAE;
- limpieza de procesos: PASS.

Las cifras completas viven en `results/verified/`; este documento no debe
usarse para recuperar métricas del hito anterior.

## Evidencia

La captura incorrecta histórica permanece en
`captures/cad_import/robot_pose_a_isometric.png`.

La evidencia final está en `captures/mechanical_assembly_final/` e incluye
referencia/final para tres poses, gripper abierto/cerrado, seis close-ups,
overlay oficial/final y collision overlay. Son renders técnicos reproducibles,
no una captura GUI nativa; Gazebo real se validó por diagnóstico headless.

## Handoff para actualización del tutorial ChatGPT

Leer en este orden:

1. `docs/final_mechanical_consolidation_report.md`;
2. `docs/mechanical_alignment_method.md`;
3. `results/verified/mechanical_alignment/decision.json`;
4. `results/verified/mechanical_alignment/alignment_manifest.json`;
5. `docs/mechanical_assembly_validation.md`;
6. este archivo, únicamente como historia de la primera corrección;
7. `docs/cad_import_tutorial.md` y troubleshooting.
