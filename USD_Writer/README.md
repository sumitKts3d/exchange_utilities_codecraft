# USD_Writer — CAD / PRC to OpenUSD (HOOPS Exchange 2026.8)

Loads any file HOOPS Exchange reads (PRC, STEP, CATIA V5, JT, SolidWorks, NX, …) and writes it as OpenUSD. After
writing, it reopens the file with OpenUSD and checks meshes, triangles, PMI, cameras, units and the world bounding box
against the HOOPS Exchange model, for every configuration it wrote ("Check: OK").

```
USD_Writer <input CAD file> <output.usda|usdc|usd|usdz> [options]
```

| Option | Meaning |
| --- | --- |
| `--force` | overwrite an existing output |
| `--lod extralow\|low\|medium\|high\|extrahigh` | B-rep tessellation level (default `medium`) |
| `--no-instancing` | write every occurrence in full |
| `--layout monolithic\|parts\|layers` | `monolithic` (default): one file. `parts`: each part and each shared sub-assembly in its own file under `<output stem>_parts/`, referenced from the assembly file. `layers`: one layer per kind of data (geometry, looks, PMI, views) under `<output stem>_layers/`, stacked by the main file (see below) |
| `--payloads` | with `--layout parts`: the assembly loads its parts as payloads, so a viewer can open it without loading the geometry |
| `--skip-hidden` | do not write hidden objects (default: written with `visibility = invisible`) |
| `--pmi guide\|default\|none` | PMI and views: `guide` (default) writes the markups with `purpose = guide` (shown only when a viewer displays guides), `default` always draws them, `none` does not read PMI or views |
| `--all-views` | also write the views of nested parts that show no annotation (SolidWorks parts carry their standard views: hundreds in a large assembly) |
| `--default-configuration` | write only the default configuration of a multi-configuration file (default: all, as a variant set) |
| `--configuration <name>` | write only the named configuration |
| `--dump-tree` | print the USD prim tree read back from the written file |
| `--quiet` / `--verbose` | fewer messages / also print the HOOPS Exchange reader messages |

Exit codes: `0` ok, `1` usage (bad option, output exists), `2` input (missing, unreadable, no geometry), `3` unsupported
output format, `4` USD write failed, `5` verification failed.

## Mapping

| PRC (HOOPS Exchange) | USD |
| --- | --- |
| model-file unit (mm per unit) | `metersPerUnit`; `upAxis = Z` |
| product occurrences (own, prototype and external-data rules of the SDK samples) | `Xform` prims; valid unique names, original CAD name in `displayName`; location as `xformOp:transform` |
| identical sub-trees or parts used two or more times | written once under the abstract `/Prototypes` class, referenced by `instanceable` prims (sub-assemblies are shared, not only parts) |
| tessellated representation items (through `RiSet`) | `Mesh`: triangles from every `A3DTessFaceData` layout, indexed face-varying normals, `subdivisionScheme = none`, `extent`, `doubleSided` for open shells, representation-item transform as `xformOp:transform` |
| B-rep face identity | `primvars:hoops:faceId` (uniform int: the PRC face of each triangle) |
| resolved colours (cascaded attributes, per face) | `displayColor` / `displayOpacity`, constant or per triangle |
| PRC materials | `UsdPreviewSurface` under `/<root>/Looks` (diffuse, specular workflow, emissive, roughness from shininess, opacity); per-face materials as `GeomSubset`s (`materialBind`, non-overlapping) |
| hidden / removed | `visibility = invisible` / not written |
| PRC attributes, persistent ids | custom attributes `userProperties:<title>:<key>` (int, double, int64 or string) |
| product configurations (SolidWorks, CATIA, NX, Creo…) | variant set `configuration` on the root prim, one variant per configuration (each loaded separately with its own unit), default configuration selected; names listed in `hoops:configurations` |
| PMI markups (tessellated) | an `Xform` per markup under a `PMI` scope of the occurrence or part that owns it: leader and frame lines as linear `BasisCurves` (`Lines`), text as stroked glyph outlines (`Text`, the font's glyphs through HOOPS Exchange), filled areas as `Mesh` (`Fill`), the markup colour as `displayColor`; `hoops:pmi:type`, `:subType`, `:label`, `:text` (the string, also kept when the glyphs cannot be drawn), `:screenAligned`, `:linked` (the linked geometry) |
| views (`A3DMkpView`) | `Camera` prims under a `Views` scope: transform from position / target / up, perspective or orthographic, field of view as apertures (focal length 50), clipping range; `hoops:view:isDefault`, `:isAnnotationView`, `:hasCamera` and `hoops:view:annotations` (relationship to the view's markups) |
| per-view PMI visibility | variant set `view` on the root prim: `All` plus one variant per annotated view that hides the markups not in that view (single-configuration files) |
| output `.usdz` | `.usdc` packaged with `UsdUtilsCreateNewUsdzPackage` |

## `--layout parts`

```
micro.usda                      assembly: structure, placements, PMI, views, configurations (variant sets)
micro_parts/HOUSING_1_.usda     one file per part: default prim, metersPerUnit, upAxis, its meshes and materials
micro_parts/SCREW_TOP_1_.usda   referenced 4 times: the referencing prims are instanceable
```

- A part (an occurrence with geometry) or a sub-assembly used twice or more is written once to its own file and
  referenced with a relative asset path (`@./micro_parts/X.usda@`), so the folder can be moved or shared as a whole.
  Parts used twice or more are `instanceable`; a shared sub-assembly file references its part files (`@./Y.usda@`).
- Each part file opens on its own: it carries its own `Looks` (materials are repeated in each part that uses them).
- Part files are `.usda` next to a `.usda` output and `.usdc` otherwise. With `.usdz`, the part files are packaged
  in the archive.
- Sub-trees that carry PMI or views stay in the assembly file, because the views and the `view` variants address
  their markups by path.
- Configurations: part files are matched by content (geometry, colours, names, placements), so a part that is the
  same in several configurations is written once (Caliper: 20 files for 2 configurations, not 84).
- `--no-instancing`: a part used several times is still one file, referenced (not instanced) by each occurrence.
- Writing is staged in `<output>.partial/` and moved into place at the end. An existing `<stem>_parts/` folder needs
  `--force`, and is replaced only if it holds nothing but `.usda` / `.usdc` files.
- Verification adds: all asset paths resolve, the layers used are the main file plus exactly the part files written,
  every part file opens on its own (default prim, units, no composition errors).

### `--payloads`

The assembly file refers to its part files as payloads instead of references (references between part files stay
references). Opening the assembly with "load none" gives the structure, names, placements, PMI and cameras without
any part geometry; loading a prim (or all) brings its geometry in. Instancing is unchanged. Verification opens the
file unloaded (no part meshes), then loads everything and compares with the full model.

## `--layout layers`

```
model.usda                      stage metadata, default prim, and the layer stack:
model_layers/views.usda         cameras ("Views" scopes)                          strongest
model_layers/pmi.usda           PMI markups and the "view" variant set (per-view PMI visibility)
model_layers/looks.usda         materials (Looks)
model_layers/geometry.usda      structure, meshes, prototypes, configuration variants, attributes   weakest
```

- Each layer holds only its own data; prims of the structure appear in the other layers as `over`s. Muting a layer
  (for example the PMI one) removes exactly that data, without touching the files.
- Configurations: each layer carries its part of every configuration variant.
- Empty layers are not written (no PMI: no `pmi` layer). Every layer has the stage units, so each opens on its own.
- `.usdz`: the layers are packaged in the archive. Muting does not reach a layer inside a package; the verification
  removes the layer from the stack instead.
- Verification adds the asset-path and standalone checks of `--layout parts`, then mutes the `pmi` and `views`
  layers one at a time: the PMI (or cameras) must disappear and the geometry must stay unchanged.
- `--layout layers` and `--layout parts` are exclusive for now.

## Build

Same requirements as USD_Reader: HOOPS Exchange 2026.8.0 with a valid `include/hoops_license.h`, CMake ≥ 3.21,
Visual Studio with the C++ workload, and an OpenUSD package (`<HOOPS Exchange>/bin/usd`, else `USD_ROOT`; see `../usd_common/OpenUSD.cmake`).
Build `Release`.

Build from the repository root as described in the [top-level README](../README.md#build): the repository is
cloned as `<HOOPS Exchange>/tools/plugins`, so HOOPS Exchange is found without any path setting.

```
cmake -S . -B build
cmake --build build --config Release --target USD_Writer USD_Reader
ctest --test-dir build -C Release -R "^USD_Writer\." --output-on-failure
```

OpenUSD is taken from `<HOOPS Exchange>/bin/usd`; if that folder does not exist, from the folder given by
`-DUSD_ROOT=<OpenUSD folder>` (or the `USD_ROOT` environment variable). Configure fails with an explanation if neither exists
(`-DHOOPS_PLUGINS_BUILD_USD=OFF` builds the QIF plugins only).

The round-trip tests use `USD_Reader` from this repository (build both targets).

## Tests (82)

v1:

- CAD to each USD flavour: PRC → `.usda`, STEP → `.usdc`, JT → `.usdz`, CATIA V5 assembly → `.usd`; SolidWorks assembly
  (inch units), `--lod high`.
- Assembly with repeated parts (`_micro engine.prc`): 2 prototypes, 8 instances; `--no-instancing`; `--skip-hidden`.
- `usdchecker` (OpenUSD's validator) on every written file.
- Round trip CAD → USD → PRC with USD_Reader (helloworld, micro engine, CATIA assembly).
- Round trip USD → PRC (USD_Reader) → USD for instancing, point instancer, materials and primitives: the world
  bounding box comes back unchanged and is checked by `usdchecker`.
- Error paths: usage, missing input, unreadable input, bad output extension, existing output, unknown option, bad
  `--lod`; non-ASCII input and output paths.

v2:

- PMI and views (`viewswithpmis.prc`): 36 markups, 6 cameras, 4 view variants; `--pmi default` checked through
  `--dump-tree` (markup text, cameras with their annotation counts); `--pmi none`; CATIA V5 PMI (`CV5_Sample.CATPart`)
  and `pmi_Sample.prc`; `usdchecker` on each.
- Nested part views: 30 skipped by default on `Assem2.SLDASM`, all 41 cameras with `--all-views`.
- SolidWorks assembly mixing inch and mm documents, with PMI (`DiskBrakeAssembly-01.SLDASM`).
- Configurations (`CaliperSubassmebly-01.SLDASM`: `Default`, `test sub`): variant set checked per configuration;
  `--configuration "test sub"`, `--default-configuration`; unknown configuration, file without configurations,
  bad `--pmi`.
- Round trips through USD_Reader: the PMI file (guide prims and cameras skipped, geometry back) and the
  `test_sub` configuration selected with `--variant`.

`--layout parts`:

- Micro engine: 15 part files, 21 references, 8 instanceable; writing again over it; with `--no-instancing`.
- Configurations (Caliper) and `.usdz` with PMI (CATIA V5 Aquo bottle); `usdchecker` on each.
- Round trip of the multi-file micro engine through USD_Reader.
- A `_parts` folder holding other files is not replaced; an existing one is not written over without `--force`;
  bad `--layout`.

`--payloads` and `--layout layers`:

- Payloads on the micro engine (opened unloaded: no meshes; loaded: all) and on the Caliper configurations
  (20 shared part files); round trip through USD_Reader; `--payloads` without `--layout parts` is refused.
- Layers on the PMI file (muting `views` and `pmi`), the Caliper configurations (4 layers including `looks`) and a
  `.usdz` (CATIA Aquo bottle); `usdchecker` on each; round trip of a configuration through USD_Reader.

## Defaults and limits

- Never overwrites without `--force`; writes `<name>.partial.<ext>` and renames it after a successful save.
- Points are written as 32-bit floats (USD `point3f`): coordinates far from the origin lose precision (about 1e-7
  relative). The bounding-box check allows for it.
- PMI is written as its tessellation (what the CAD system displays), not as semantic GD&T; the semantic type,
  sub-type and text are kept as `hoops:pmi:*` attributes. Screen-aligned (flat-to-screen, fixed-size) markups are
  drawn in model space at their placement.
- PMI text uses the fonts HOOPS Exchange finds on the machine. A glyph it cannot draw (often a space, or a CAD
  font that is not installed) is replaced by an estimated advance; a text with no drawable glyph is reported and
  only its string is kept.
- The `view` variant set is written for single-configuration files only; with several configurations the cameras
  are written in each configuration but the per-view PMI filtering is not.
- Nodes that carry PMI are not instanced (their markups are written in place); a PMI set attached both to an
  occurrence and to its part is written once.
- An occurrence's own unit (`m_dUnit`) is informational: locations and geometry are already in the model-file unit
  of each load. Configurations loaded in another unit than the first are scaled into the stage unit.
- Not written yet: wireframes and curves of the B-rep, textures and UVs, vertex colours, exact B-rep (`NurbsPatch`),
  semantic PMI, section views and exploded views.
- Faces that use the reserved polyface layouts are skipped and reported.
