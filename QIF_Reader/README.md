# QIF_Reader — QIF 3.0 to PRC converter (HOOPS Exchange public API)

Reads a QIF 3.0 file with a small built-in reader and rebuilds it as a PRC model entity by entity with the HOOPS Exchange
creation API: exact B-rep, appearance and structure, graphical PMI, views and semantic PMI. The PRC is written with
`A3DAsmModelFileExportToPrcFile`, then re-imported and checked. HOOPS Exchange's own (beta) QIF reader is not used,
except as an optional cross-check (`--compare-native`).

## Files

| File | Content |
| --- | --- |
| `QIF_Reader.cpp` | Command line, B-rep creation (geometry + topology), checks, export, re-import, statistics |
| `prc_appearance.hpp` | Colours, visibility, layer index, names / QIF ids / attributes, wire bodies, coordinate systems |
| `prc_pmi.hpp` | Graphical PMI (markup tessellation), links to faces / edges / wire curves, views and cameras |
| `prc_semantic.hpp` | Semantic PMI: GD&T feature control frames, dimensions, datums, datum targets, notes |
| `qif_model.hpp`, `qif_xml.hpp` | QIF reader and exact evaluators (no HOOPS dependency) |
| `tests.cmake`, `compare_pmi.cmake`, `gen_expected_pmi.py` | CTest suite; semantic expectation generated independently from the QIF XML |
| `testdata/` | synthetic box, malformed file, six NIST QIF 3.0 models (qif-community, Boost licence), `expected_pmi/*.txt` |

## Build, run, test

Needs HOOPS Exchange 2026.8.0 (`hoops_license.h` with a valid key), CMake 3.21+ with Visual Studio (C++ workload), C++17.
Build from the repository root as described in the [top-level README](../README.md#build): the repository is
cloned as `<HOOPS Exchange>/tools/plugins`, so HOOPS Exchange is found without any path setting.

```bat
cmake -S . -B build
cmake --build build --config Release --target QIF_Reader
ctest --test-dir build -C Release -R "^QIF_Reader\." --output-on-failure

build\bin\Release\QIF_Reader.exe part.qif part.prc
build\bin\Release\QIF_Reader.exe part.qif part.prc --force --max-dev 1e-6 --dump-pmi
```

| Option | Effect |
| --- | --- |
| `--force` | overwrite the output |
| `--skip-hidden` | drop bodies marked `hidden="true"` (default: converted and hidden) |
| `--no-pmi` | geometry, structure and appearance only |
| `--no-semantic` | graphical PMI only |
| `--max-dev <d>` | exit 5 if a PRC curve/surface deviates from the QIF formula by more than *d* model units |
| `--compare-native`, `--native-tol <r>` | compare with HOOPS Exchange's beta QIF reader (faces; area/volume when measurable) |
| `--dump-pmi` | print the semantic PMI read back from the written PRC |
| `--quiet` | warnings, errors and summary only |

Defaults: output must be `.prc`; written to `<out>.partial.prc` and renamed after success; never overwritten without
`--force`. Exit codes: 0 ok, 1 usage / output exists, 2 input / parse / SDK start-up, 3 unsupported or nothing to convert,
4 export or re-import failed, 5 verification failed. Windows `wmain` + UTF-8 inside: non-ASCII paths work.

## What is converted

| QIF | PRC |
| --- | --- |
| GeometrySet + TopologySet (bodies with faces) | one `A3DRiBrepModel` per body; analytic surfaces stay analytic, QIF parameterisation kept (p-curves unchanged) |
| Bodies without faces (sketch / construction lines) | `A3DRiSet` of `A3DRiCurve` (one per edge), coloured |
| `hidden="true"` | graphics without `kA3DGraphicsShow` |
| Body / face colours | body style = dominant face colour; other face colours as entity references on the product occurrence (`A3DMiscReferenceOnTopology`) |
| Labels, ids, Part attributes, Header (File, Description, ModelNumber), body attributes | names, persistent ids, `A3DMiscAttribute` |
| LayerSet | layer index in graphics + attribute "QIF layer" |
| CoordinateSystemSet | `A3DRiCoordinateSystem` |
| FileUnits | model-file unit (mm based) |
| Features / FeatureNominals | attribute "QIF feature" on every face of the feature |
| VisualizationSet / PMIDisplay | tessellated markup on its annotation plane: texts, Y14.5 symbols (`{FLATNESS}`, `{POSITION}`, ...) drawn as vector strokes because viewer fonts lack those glyphs, frames, leaders, arrow / dot / datum heads, witness lines, polylines, filled areas; QIF colour |
| PMI → geometry | linked items to faces / edges (`A3DMiscReferenceOnTopology`) or wire curves, via FeatureNominal EntityInternalIds |
| Characteristics (GD&T) | `A3DMarkupGDT`: type, tolerance, diametral / spherical zone, material condition, free state, statistical, datum reference frame with modifiers |
| Characteristics (sizes) | `A3DMarkupDimension`: type, nominal, ± tolerance, diameter symbol, suffix |
| DatumDefinition / DatumTarget | `A3DMarkupDatum` (target flag) |
| Note | `A3DMarkupText` + attribute "QIF text" |
| AnnotationViewSet | annotation views with their plane and markups |
| SavedViewSet + CameraSet | views with orthographic camera and the visible PMI; ActiveView = default view |

## Test results (HOOPS Exchange 2026.8.0, Windows, Visual Studio)

- Geometry: all faces of the six NIST models converted; every PRC curve/surface within 1e-13 of the QIF formula;
  re-imported PRC is an exactly measurable closed solid; box area 2200 mm², volume 6000 mm³.
- PMI: every PMIDisplay → markup (23/30/22/64/50/64), every geometry-related one linked (5 general notes have no geometry),
  all annotation planes and saved views (4/3/7/9/17/7 views).
- Semantic: 233 GD&T / dimension / datum markups read back from the PRC are identical to the expectation computed from
  the QIF XML (`semantic_*` tests); features on 30/32/12/123/103/116 faces.
- Native beta reader (same faces, but its B-rep is not measurable: `A3D_LOOP_BAD_ORIENTATION_DATA`, Aggregate13 not
  implemented, topology set failure on nist_ctc_05).

## Known limits

- Assemblies (`AssemblySet`, `ComponentSet`) and body `Transform`: not converted (warning); all bodies go into one part.
  None of the sample files has them.
- Not supported (face / curve skipped with a warning): ArcCircular12, ArcConic12, parabola / hyperbola, Polyline12/13,
  Offset23, Ruled23, Extrude23, meshes / point clouds, aggregates mixing arcs with other curves, trimmed NURBS inside
  aggregates. (`CPsBinary` and segment + NURBS aggregates are supported; aggregates of those are joined into one NURBS.)
  Revolution23 surfaces are converted to `A3DSurfRevolution` (QIF u = generatrix parameter, v = angle; the PRC
  parameterisation is calibrated with `A3DSurfEvaluate`); Spline12/13 polynomial curves become exact B-splines.
- Tolerant models: edge / vertex tolerances are set to 1.5 × the measured vertex-to-curve gap (minimum 1e-6).
- Note lines: `A3DMarkupTextCreate`/`A3DMarkupTextSet` in 2026.8 keep no lines (read back as 0); the text is in the markup
  tessellation and in the attribute "QIF text".
- GD&T: second composite segments, projected zones, per-unit-area tolerances and indicators are not in the semantic
  definition (they stay in the graphical PMI).
- Text size and placement are estimated from `lineHeight` and the font box; camera orientation is read as quaternion
  (w, x, y, z) — both to be confirmed visually.
- Edge colours of solid edges, QIF Plans / Results / Rules / Statistics are not converted.
