# Registro automático de piezas CAD contra la referencia oficial

| Pieza | Link | RMS Chamfer (mm) | P95 (mm) | Landmark máx. (mm) | Estado |
|---|---|---:|---:|---:|---|
| mount_printed_base | poppy_mount_link | 0.505 | 0.836 | 0.017 | PASS |
| link_1_long_u | poppy_link_1 | 0.410 | 0.870 | 0.049 | PASS |
| link_2_horn_side | poppy_link_2 | 0.097 | 0.131 | 0.003 | PASS |
| link_2_body_side | poppy_link_2 | 0.210 | 0.427 | 0.003 | PASS |
| link_3_short_u | poppy_link_3 | 0.086 | 0.134 | 0.028 | PASS |
| link_4_horn_side | poppy_link_4 | 0.097 | 0.131 | 0.003 | PASS |
| link_4_body_side | poppy_link_4 | 0.210 | 0.427 | 0.003 | PASS |
| link_5_gripper_fixation | poppy_link_5 | 0.219 | 0.133 | 0.001 | PASS |
| link_5_fixed_jaw | poppy_link_5 | 0.209 | 0.197 | 0.067 | PASS |
| link_6_moving_jaw | poppy_link_6 | 0.406 | 0.983 | 0.026 | PASS |

## Auditoría de joints

| Joint | Parent → child | xyz (m) | rpy (rad) | Eje | Residuo eje (mm/°) | Estado |
|---|---|---|---|---|---:|---|
| m1 | poppy_mount_link → poppy_link_1 | `0 0 0.0327993216` | `0 0 0` | `0 0 1` | 0.000 / 0.000 | PASS |
| m2 | poppy_link_1 → poppy_link_2 | `0 0 0.0240006784` | `0 -1.57079633 0` | `0 0 -1` | 0.000 / 0.000 | PASS |
| m3 | poppy_link_2 → poppy_link_3 | `0.054 0 0` | `0 0 0` | `0 0 -1` | 0.000 / 0.000 | PASS |
| m4 | poppy_link_3 → poppy_link_4 | `0.045 0 0` | `0 -1.57079633 0` | `0 0 -1` | 0.000 / 0.000 | PASS |
| m5 | poppy_link_4 → poppy_link_5 | `0 -0.048 0` | `0 -1.57079633 0` | `0 0 1` | 0.000 / 0.000 | PASS |
| m6 | poppy_link_5 → poppy_link_6 | `0 -0.058 0` | `0 -1.57079633 0` | `0 0 -1` | 0.000 / 0.000 | PASS |

Gate autónomo: **FAIL**.

Método final: **official_reference_consolidation**. El FAIL se debe a geometría mecánica ausente, no se maquilla con bounds plausibles.
