# Validación de pick-and-place móvil

- `run_01/`: corrida nominal sin grabación, seed 504; todos los criterios PASS.
- `run_02/`: repetición con dos cámaras a 60 Hz, seed 506; todos los criterios PASS, ambas vistas completas.
- `negative_cancel/`: cancelación con carga, seed 505; aserciones de seguridad PASS.
- `stationary_physical_regression/`: regresión sin attach, seed 402; PASS.
- `development/`: intentos de integración conservados con sus resultados originales.
- `source_fingerprints.json`: SHA-256 de fuentes/modelos/configuración runtime.
- `validation.log`, `lint.log`, `mechanical_regression.json`: pruebas y auditorías.

La base de las fuentes es el commit cc0e344; las corridas incluyen cambios
sin commit identificados por fingerprints. La navegación usa localización
simulada y el agarre una unión temporal condicionada a contacto bilateral.
No se presentan como navegación perceptiva ni agarre físico sin asistencia.

El resultado failed/CANCELLED de la negativa no representa un fallo del criterio
de seguridad: consulte `negative_cancel/assertions.json`. El cilindro se
mantuvo elevado y unido al gripper; el desplazamiento observado después de
cancelar fue de 0,154 mm.

[Guía y reproducción](../../../docs/mobile_pick_and_place.md).


## Cierre de la campaña

- Dos corridas nominales PASS y auditorías independientes de muestras PASS.
- Cancelación con carga PASS; versión estacionaria física PASS.
- 77 pruebas automatizadas, lint y auditoría mecánica PASS.
- Cinco MP4 H.264 a 60 fps CFR, verificados por decodificación completa.
- Revisión visual de inicio, sujeción, llegada y retirada tras el depósito.

[Resumen cuantitativo](summary.json) ·
[Videos y explicación de frames repetidos](../../../captures/pick_place_movil/README.md).
