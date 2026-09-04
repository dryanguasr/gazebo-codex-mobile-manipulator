# Pinned official assembly reference

This directory preserves the authoritative URDF/Xacro and upstream
`package.xml` used to interpret the Poppy Ergo Jr assembly. The files are copied
unchanged from
[poppy-project/poppy_ergo_jr_description](https://github.com/poppy-project/poppy_ergo_jr_description)
at commit `7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b`.

The corresponding Collada meshes live once, without duplication, in
`../../official/`. They are used both by the registration audit and, after the
autonomous gate failed for missing servo/horn solids, by the final runtime model.

These reference files are GPL-3.0-only as declared by the upstream
`package.xml`. They are distinct from the Apache-2.0 repository code and from
the CC BY-SA 4.0 hardware CAD derivatives.
