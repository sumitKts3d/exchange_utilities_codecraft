# USD_Reader — OpenUSD to PRC converter (HOOPS Exchange 2026.8)

Reads a USD stage (`.usd`, `.usda`, `.usdc`, `.usdz`) with the OpenUSD toolkit and writes a PRC file built with the
HOOPS Exchange public API. After writing, it reads the PRC back with HOOPS Exchange and checks that the occurrence
count, triangle count and world bounding box match the USD stage.

```
USD_Reader <input.usd|usda|usdc|usdz> <output.prc> [options]
```

| Option | Meaning |
| --- | --- |
| `--force` | overwrite an existing `<output.prc>` |
| `--variant "<path>{set=sel}"` | select a variant (repeatable), e.g. `--variant "/World/Bolt{size=large}"`; default: the authored selections |
| `--time <t>` | time code to read (default: earliest sample, else default values) |
| `--skip-hidden` | drop invisible prims (default: converted as hidden occurrences) |
| `--include-proxy`, `--include-guide` | also convert `proxy` / `guide` purpose prims (default: skipped) |
| `--keep-up-axis` | keep a Y-up stage Y-up (default: rotated to Z-up) |
| `--segments <n>` | facets around the axis of Sphere/Cylinder/Cone/Capsule (multiple of 4, default 32) |
| `--dump-tree` | print the product structure read back from the PRC (names, hidden, colours, attributes, triangles) |
| `--quiet` / `--verbose` | fewer messages / also print the HOOPS Exchange reader and writer messages |

Exit codes: `0` ok, `1` usage (bad option, output exists), `2` input (missing file, unreadable stage, unknown
variant, no geometry), `3` unsupported input/output format, `4` PRC creation/export failed, `5` verification of the
written PRC failed.

## What is converted

| USD | PRC |
| --- | --- |
| stage `metersPerUnit`, `upAxis` | model-file unit in mm (`metersPerUnit × 1000`); root occurrence named after `defaultPrim`, with a +90° X rotation for Y-up stages |
| `Xform`, `Scope`, typeless prims | product occurrence: name, local transform (`resetXformStack` honoured), attributes |
| `Mesh` | part → `A3DRiPolyBrepModel` → `A3DTess3D`; polygons fan-triangulated, holes removed, `leftHanded` winding flipped, authored normals (vertex / faceVarying / uniform / constant) or computed ones (smooth for subdivision meshes, flat for `none`) |
| `Cube`, `Sphere`, `Cylinder`, `Cylinder_1`, `Cone`, `Capsule`, `Capsule_1`, `Plane` | tessellated meshes (exact extents) |
| `displayColor` / `displayOpacity` (inherited constant primvars too) | constant → style on the mesh, uniform → per-face styles, vertex/faceVarying → vertex colours (RGBA) |
| bound `UsdPreviewSurface`, MaterialX `standard_surface` / `open_pbr_surface` | PRC materials (diffuse, specular, emissive, shininess from roughness, transparency); `GeomSubset` material bindings → per-face materials; a `UsdPrimvarReader` of `displayColor` uses the prim's colours; a texture uses its `fallback` colour |
| native instancing (`instanceable`) | one shared PRC prototype per USD prototype, one occurrence per instance (nested instancing kept) |
| `PointInstancer` | one PRC prototype per prototype prim, one occurrence per instance (positions, orientations, scales, prototype transform); `invisibleIds` → hidden, inactive ids dropped |
| variant sets | the composed selection; `--variant` overrides it in the session layer (source files are not touched) |
| `visibility = invisible` | hidden occurrence (or dropped with `--skip-hidden`) |
| prim path, type, `kind`, `purpose`, variant selections, custom attributes, `userProperties:*` | string attributes on the occurrence (attribute title `USD`) |

Reported but not converted: `BasisCurves`, `NurbsCurves`, `NurbsPatch`, `Points`, `TetMesh`, lights, cameras.

## Shared code

The OpenUSD build wiring (OpenUSD folder, runtime copy) and the conventions shared with USD_Writer (matrices, units,
looks, names, attributes, PRC tessellation decoding and tree dump) live in `../usd_common`. A prim's `displayName`
(USD_Writer stores the original CAD name there) becomes the PRC occurrence name.

## Build

Requires HOOPS Exchange 2026.8.0 with a valid `include/hoops_license.h` (a missing or placeholder key builds but
stops at startup with error -34), CMake ≥ 3.21, Visual Studio with the C++ workload, and an OpenUSD binary package
with headers, `.lib` files and `cmake/pxrTargets.cmake` (tested with NVIDIA's
`usd.py312.windows-x86_64.usdview.release-v25.08`). The build copies the OpenUSD DLLs, TBB, Python and the `usd/` plug-in resources
next to the executable, so no `PATH` change is needed. Build `Release` (the package ships release binaries only).

Build from the repository root as described in the [top-level README](../README.md#build): the repository is
cloned as `<HOOPS Exchange>/tools/plugins`, so HOOPS Exchange is found without any path setting.

```
cmake -S . -B build
cmake --build build --config Release --target USD_Reader
ctest --test-dir build -C Release -R "^USD_Reader\." --output-on-failure
```

OpenUSD is taken from `<HOOPS Exchange>/bin/usd`; if that folder does not exist, from the folder given by
`-DUSD_ROOT=<OpenUSD folder>` (or the `USD_ROOT` environment variable). Configure fails with an explanation if neither exists
(`-DHOOPS_PLUGINS_BUILD_USD=OFF` builds the QIF plugins only).

## Tests (`tests.cmake`, test data in `testdata/`)

Each case converts a file, re-imports the PRC and checks counts and bounding box ("Check: OK"); the expected
bounding boxes were also computed independently with OpenUSD's `UsdGeomBBoxCache`.

- `cube_mesh`, `cube_yup`, `cube_yup_keep` — mesh, translation, units, Y-up → Z-up
- `transforms` — translate + rotate, translate + non-uniform scale
- `primitives`, `primitives_coarse`, `usdc` (made with `usdcat`) — all intrinsic primitives, 32 and 8 segments
- `hierarchy`, `hierarchy_skip_hidden`, `hierarchy_with_proxy_guide` — names, kind/custom attributes, per-face and
  vertex colours, holes, left-handed, `resetXformStack`, invisible, guide/proxy, unsupported prim reported
- `materials` — UsdPreviewSurface, GeomSubset materials with opacity, primvar reader, texture fallback, MaterialX
- `instancing`, `instancing_skip_hidden` — shared and nested native instancing, invisible instance
- `pointinstancer` — two prototypes, orientations, scales, `invisibleIds`
- `variant_default`, `variant_large` — authored selection and `--variant`
- `usdz` — a `.usdz` package shipped with the OpenUSD package
- error paths: usage, missing input, bad input/output extension, malformed `.usda`, empty stage, existing output,
  unknown option, bad/unknown variant, bad `--segments`; non-ASCII input and output paths

## Defaults and limits

- Never overwrites without `--force`; writes `<output>.partial.prc` and renames it only after a successful export.
- Polygons are fan-triangulated: concave polygons with more than 4 vertices may be triangulated incorrectly.
- Subdivision surfaces are converted as their control cage (not subdivided).
- Textures are not converted (the texture's `fallback` colour is used); UVs are not written.
- Primvars inherited from a native instance into its prototype, and material bindings authored on instance prims,
  are not applied inside the shared prototype.
- `Capsule_1` with different top/bottom radii joins the two hemispheres with a straight band (extents are exact).
- Skinned meshes (UsdSkel) are converted in their authored (rest) pose; animation is read at one time code.
