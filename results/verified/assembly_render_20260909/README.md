# Ensamblaje renderizado: validación 2026-09-09

- runtime_vertices_before.json: FAIL esperado al leer los DAE instanciados anteriores.
- runtime_vertices_after.json: PASS, todos los vértices cargados por Gazebo
  coinciden con el CAD transformado (7 mallas, 123 instancias, 378222 triángulos).
- mechanical_regression.json: PASS, articulaciones y poses FK conservadas.
- run_01: ciclo físico sin attach, PASS; incluye video, eventos y CSV.
- source_fingerprints.json: procedencia del árbol modificado, registrada después
  de la corrida. No se atribuyen estos resultados al commit base sin los cambios.

[Informe](../../../docs/assembly_render_fix.md).
[Videos y segundo 23](../../../captures/ensamblaje_inspeccion/).
