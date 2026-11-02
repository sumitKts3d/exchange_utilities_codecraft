# QIF_Writer — CAD / PRC to QIF 3.0 (HOOPS Exchange 2026.8)

Writes any file HOOPS Exchange reads (PRC, STEP AP242, CATIA V5, NX, Creo, SolidWorks, JT, …) as a QIF 3.0 document,
with the HOOPS Exchange public API only. It is the writer counterpart of `../QIF_Reader` (that code is not used or
changed: the two tools only share the QIF conventions, and QIF_Reader is used by the tests as an independent reader).

```
QIF_Writer <input CAD file> <output.qif> [options]
```

| Option | Meaning |
| --- | --- |
| `--force` | overwrite an existing output |
| `--skip-hidden` | do not write hidden bodies (default: written with `hidden="true"`) |
| `--no-pmi` | geometry, structure and appearance only |
| `--max-gap <d>` | self-check limit for a p-curve end to vertex gap, in model units (default 1e-4 of the model size) |
| `--quiet` / `--verbose` | fewer messages / also print the HOOPS Exchange messages and unresolved PMI links |

Exit codes: `0` ok, `1` usage (bad option, output exists), `2` input (missing, unreadable), `3` nothing to write
(no exact B-rep), `4` write failed, `5` self-check failed. The file is written as `<name>.partial.qif` and renamed
after success; Windows `wmain` + UTF-8 inside (non-ASCII paths work).

## Mapping

| PRC (HOOPS Exchange) | QIF 3.0 |
| --- | --- |
| model-file unit | `FileUnits/PrimaryUnits/LinearUnit` (factor to metre); PMI angles in degree |
| product occurrences | one `Part` (PartSet + RootPart); every B-rep instance becomes a `Body` placed in model space (occurrence locations, representation-item coordinate systems and B-rep context scale applied; mirrors turn the faces) |
| exact B-rep (`A3DRiBrepModel`) | `GeometrySet` (Points, `Nurbs13` edge curves, `Nurbs12` p-curves, `Nurbs23` surfaces) + `TopologySet` (Vertex, Edge, Loop, Face, Shell, Body). The B-rep is adapted to NURBS with `A3DCopyAndAdaptBrepModel` (same topology; p-curves and 3D curves are computed when the source lacks them, faces split at seams only as a last resort) |
| loop / co-edge orientation | outer loop first, loops counter-clockwise in UV, p-curves follow the co-edge, co-edges of one edge share one QIF `Edge` (`EdgeOriented turned`) |
| colours, visibility, names, attributes | `Face`/`Body` `color`, `Body hidden`, `label`, `Attributes` (`AttributeStr`, "title:key") |
| GD&T (`A3DMarkupGDT`, first FCF row) | `<Type>CharacteristicDefinition` (ToleranceValue, DatumReferenceFrameId, MaterialCondition, ZoneShape, FreeState) + `<Type>CharacteristicNominal` (Name, FeatureNominalIds) |
| dimensions (`A3DMarkupDimension`) | `Diameter` / `Radius` / `Spherical*` / `Length` / `Angle` / `Chamfer` / `DistanceBetween` characteristics: TargetValue (decimal places from the accuracy), Tolerance Max/Min |
| datums, datum targets | `DatumDefinitions` / `DatumTargetDefinitions`; feature control frame datums as `DatumReferenceFrames` (simple and compound datums, material modifiers, precedence) |
| notes and other markups (roughness, welding, balloons, …) | `Note` in `NoteSet` (text, linked faces / edges) |
| markup links to faces / edges | `OtherNonShapeFeatureNominal` with `EntityInternalIds` (characteristics, datums) or `EntityInternalIds` on the note |
| markup tessellation | `VisualizationSet/PMIDisplay`: polylines, filled triangles and text as glyph outlines, in the markup's best-fit plane (`AnnotationViewSet`), `Reference` to the semantic entity |
| views with a camera | `CameraSet` (ViewPlaneOrigin, orientation quaternion (w, x, y, z), height, near/far) + `SavedViewSet` (visible annotations, active view) |

## Self-check

After writing, every co-edge p-curve is evaluated on its NURBS surface (with the QIF control-point order and the
written coordinates) and must meet the co-edge's vertices — within `--max-gap`, or within 1.5 × the tolerance of a
tolerant edge. A failure exits with 5.

## Build and test

Needs HOOPS Exchange 2026.8.0 with a valid `include/hoops_license.h`, CMake ≥ 3.21, Visual Studio with the C++
workload, C++17. No other dependency. The tests use CAD files from `<HOOPS Exchange>/samples/data`.

Build from the repository root as described in the [top-level README](../README.md#build): the repository is
cloned as `<HOOPS Exchange>/tools/plugins`, so HOOPS Exchange is found without any path setting.

```
cmake -S . -B build
cmake --build build --config Release --target QIF_Writer QIF_Reader
ctest --test-dir build -C Release -R "^QIF_Writer\." --output-on-failure
```

The round-trip tests also need `QIF_Reader` (build both targets). Tests (`tests.cmake`):

- writes from PRC, STEP, a PRC assembly (with and without hidden bodies), a CATIA V5 assembly, PRC PMI and views;
- CAD → QIF → PRC round trips through QIF_Reader (`roundtrip.cmake`): face count, exact area and volume must come back;
- QIF (NIST models) → PRC (QIF_Reader) → QIF (QIF_Writer) → PRC (QIF_Reader): geometry round trip, and the semantic PMI
  read back must equal QIF_Reader's independent expectation (`../QIF_Reader/compare_pmi.cmake`, `expected_pmi/*.txt`);
- error paths: usage, missing input, bad extension, existing output, unknown option, model without exact B-rep,
  non-ASCII paths.

## Known limits (v1)

- All surfaces and curves are written as NURBS (exact for planes, cylinders, cones, spheres and tori, which are
  rational NURBS). Analytic QIF surfaces (`Plane23`, `Cylinder23`, …) and shape features (Plane / Cylinder feature
  nominals with their parameters) are the next step: CMM software prefers them.
- Assemblies are flattened into one part (QIF_Reader reads one part); `AssemblySet` / `ComponentSet` are not written.
- Tessellated-only (mesh) bodies are not written: QIF holds B-rep geometry.
- GD&T: only the first row of a feature control frame is semantic (composite rows stay in the drawing); projected
  zones, tolerance per unit and indicators are not written.
- PMI drawings are written as graphics (polylines, areas, text outlines), not as QIF texts, leaders and frames.
