# Poppy Ergo Jr official runtime visuals

These seven Collada files are copied without geometric edits from
[poppy-project/poppy_ergo_jr_description](https://github.com/poppy-project/poppy_ergo_jr_description)
at commit `7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b`:

- `base.dae`
- `long_U.dae`
- `section_1.dae`
- `section_2.dae`
- `section_3.dae`
- `section_4.dae`
- `gripper.dae`

Upstream declares the package as GPLv3 in `package.xml`. These files remain
GPL-3.0-only; the complete license text is in `LICENSE_GPL-3.0.txt`. Their
SHA-256 hashes are recorded by
`results/verified/mechanical_alignment/alignment_manifest.json`.

They are runtime assets only because the bounded autonomous reconstruction could
align the available printed CAD parts but could not reconstruct or verify the
missing XL-320 servo bodies, horns and fasteners. The final Xacro therefore uses
the official visuals under the explicitly documented B3 fallback. The repository
code remains Apache-2.0. The original hardware CAD and its derived teaching
meshes remain separately attributed under CC BY-SA 4.0.
