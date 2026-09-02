# Poppy Ergo Jr: licencia y trazabilidad

Este directorio registra la procedencia de los CAD usados por el manipulador móvil. Los originales versionados viven en `src/mobile_manipulator/meshes/poppy_ergo_jr/source/`; no se presentan como trabajo propio.

## Fuente fijada

- Proyecto: Poppy Ergo Jr, Poppy Project / Inria.
- Repositorio: https://github.com/poppy-project/poppy-ergo-jr
- Revisión: `97ce599be8c717843c45ebf48341f2ebf8f250b3`.
- Fecha del commit: 2022-04-30.
- Licencia de hardware: Creative Commons Attribution-ShareAlike 4.0 International.
- Copia íntegra de la licencia: `source/hardware/LICENSE.md`.
- Documentación de montaje consultada: https://docs.poppy-project.org/en/assembly-guides/ergo-jr/mechanical-construction
- Dimensiones y masa del XL-320: https://emanual.robotis.com/docs/en/dxl/x/xl320/

Los binarios oficiales están almacenados con Git LFS. Los SHA-256 de esta tabla son los OID LFS oficiales y también se verifican antes de preparar los assets.

| Ruta oficial | SHA-256 | Uso |
|---|---|---|
| `hardware/STEP/base.step` | `c6f222b8cb2bd227412fad26ee7f5eeb8a1c56c1555b45d4d3072bd74c164e5a` | B-rep de referencia para base, unidades y ejes |
| `hardware/STEP/U_parts.step` | `099ea2c65bb806e280d8e2a06b30f002be026a2dc6f72785559c794e6248c056` | B-rep de piezas U |
| `hardware/STEP/lateral_parts.step` | `f4a3494e56e9543655446a23b73b3e583146f86940e4dd88ba044188f36d28ed` | B-rep de laterales |
| `hardware/STEP/tools/gripper.step` | `f67540cef54d364f57d19afdf3019e4fe453319d8034a073f4e73564e3c8e9ee` | B-rep de la pinza |
| `hardware/STL/base.stl` | `c3150095267a94d0df530167b9bb22d22d00ec74918969fdbf33aa83f77ca63b` | soporte impreso del brazo |
| `hardware/STL/long_U.stl` | `8f6b02d1be0517bb018c28496cc8d4129be730b838f8db11ea8e93f3711fe0fe` | primer bracket |
| `hardware/STL/short_U.stl` | `aeb94f32e01d08db7a00f137fec3b970a2d7247b4b3447dea5c84f4382c1efd1` | bracket corto |
| `hardware/STL/horn2horn.stl` | `cc6d40d692c6d12e4ad16f9566cf90b6e401c1a46848564d8dda83072ed75dbb` | lateral derecho repetido |
| `hardware/STL/side2side.stl` | `0527075781cdb4221049aa5d0fef7d218e132986196dadcee7b82862985c4d02` | lateral izquierdo repetido |
| `hardware/STL/tools/gripper-fixation.stl` | `52663bd2b410975eaf59751bdbd981dd51ce24b9356cf13acf8508e975b61c4c` | unión m5-m6 |
| `hardware/STL/tools/gripper-fixed_part.stl` | `78d22d532083d36677f54f75575d5bd9a2208222eafda29a12f7cb0144b368a6` | mordaza fija |
| `hardware/STL/tools/gripper-rotative_part.stl` | `c4afe084b66d845d64fa19e9c82c457b171f6f85538500aa532bec783ca1eac6` | mordaza móvil |
| `hardware/STL/support_camera.stl` | `f26f269ffd4ca92fc5d4615e3f7bdcd20519e8a5909ce94ace47295ecbd72969` | auditado; no sustituye la cámara frontal |
| `hardware/STL/disk_support.stl` | `45941750fa44a9b625d471589e919e69b5d0e881c435da8f733b7bb797fe8827` | comparación de huella de montaje |

## Selección y exclusiones

Se conservaron las piezas necesarias para la cadena de seis motores con pinza, además del disco y soporte de cámara usados para decidir si era necesario modificar la plataforma. No se incorporaron `4dofs-*`, lámpara, portalápiz ni tornillo del portalápiz porque no participan en esta configuración.

La base móvil existente mide 0.72 x 0.52 m y ya admite sobradamente la huella Poppy de 0.15 m del disco; no se cambiaron masa, inercia, ruedas ni `wheel_separation`. La cámara de seguimiento permanece en el frente de la plataforma. El soporte Poppy se conserva como material auditado, pero mover el sensor allí habría cambiado el experimento perceptivo.

## Transformaciones

`scripts/cad/prepare_poppy_assets.py` verifica los OID, lee los STL binarios, aplica escala 0.001 porque los STL están en milímetros, recompone las piezas de cada link y escribe STL binario en metros. Los STEP declaran metros y muestran coordenadas como `0.054`; los STL equivalentes muestran `54`. Esta diferencia fue comprobada, no supuesta.

Las rotaciones y traslaciones exactas, versiones de Python/NumPy/SciPy, límites geométricos y conteos de triángulos quedan en `asset_manifest.json`. La pinza se reorienta con `Rx(-90°)` y `Rz(-90°)`; la mordaza fija se desplaza 58 mm al eje m6. Los collision meshes se obtienen con convex hull tras voxelizar vértices a 1 mm.

Los derivados continúan sujetos a CC BY-SA 4.0. El código original del repositorio mantiene su licencia Apache-2.0; esta separación evita confundir ambas licencias.
