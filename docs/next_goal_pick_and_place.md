# Hito cerrado: pick-and-place nivel A

## Estado

El objetivo descrito originalmente en este archivo se completó. La implementación
funcional está en el commit
`1d03fc59d05a7de8b95bd01e40a0db0e70bcd8ef` y su campaña oficial en
`results/verified/pick_A1_20260908/`.

Resultado:

- A1 asistido: 10/10 grasp, 10/10 place y 10/10 ejecuciones limpias;
- siete pruebas negativas aceptadas, sin attach falso;
- A2 físico sin attach: FAIL documentado, sin separación del pedestal;
- diagnóstico final: PASS;
- seguimiento B3 posterior: PASS, mejora MAE 91.37 %;
- Poppy 1:1, base compacta y siete DAE oficiales preservados.

A1 queda explícitamente clasificado como `attach_conditioned` y
`simulator_assisted=true`. No representa agarre físico, percepción del objeto ni
IK.

## Dónde continuar

- Arquitectura ejecutada: `docs/pick_and_place_architecture.md`.
- Tutorial de reproducción: `docs/pick_and_place_tutorial.md`.
- Protocolo y resultados: `docs/pick_and_place_experiment_plan.md`.
- Incidencias observadas: `docs/pick_and_place_troubleshooting.md`.
- Informe de cierre: `docs/pick_and_place_final_report.md`.

## Próximo objetivo permitido

El siguiente hito es rediseñar y evaluar A2 sin attach. Antes de ejecutar,
preregistre como máximo dos configuraciones y una hipótesis causal por cambio.
La autoridad seguirá siendo el evaluador de Gazebo: contacto bilateral o
terminal `DONE` no bastan si el objeto no se separa, no alcanza 50 mm o toca el
soporte fuera de fases permitidas.

No avance aún a MoveIt, IK general, Nav2, SLAM, percepción autónoma, plantas,
frutos ni Sim2Real. Esas extensiones solo se justifican después de que A2 pase
de forma reproducible.

## Baseline que no debe regresionarse

- `sim.launch.py` y la lección B3 de seguimiento;
- base compacta, cámara, odom y TF;
- seis joints y geometría Poppy oficial 1:1;
- exactamente siete DAE en el directorio oficial;
- método final CAD `official_reference_consolidation`;
- `poppy_tool_frame` para FK y `poppy_grasp_frame` para el gate;
- diagnóstico, 34 tests y experimento A/B.

## Gate propuesto para un futuro A2

Conservar objeto, world, poses, evaluador y seeds siempre que la hipótesis no
requiera cambiarlos. Para cada configuración:

1. fuente limpia e inicialización válida;
2. attach y detach exactamente cero;
3. contacto bilateral fresco y persistente;
4. separación del soporte;
5. lift ≥50 mm y hold ≥3 s;
6. place dentro de 30 mm y estabilidad ≥2 s;
7. cero contactos prohibidos y cero residuos;
8. todos los fallos conservados, sin reemplazo ni tuning post hoc.

Si las dos configuraciones fallan, cierre el hito como A2 no aceptado y documente
qué variable debe rediseñarse antes de otro experimento.
