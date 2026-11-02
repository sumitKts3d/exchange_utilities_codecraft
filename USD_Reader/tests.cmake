# CTest cases for USD_Reader. Run:
#   ctest --test-dir <build dir> -C Release -R "^USD_Reader\." --output-on-failure
#
# Every conversion re-imports the PRC it wrote and checks occurrences, triangles and the world bounding box against
# the USD stage ("Check: OK"). The bounding boxes in the regular expressions below were also computed independently
# with OpenUSD's UsdGeomBBoxCache.
set(USD_DATA "${CMAKE_CURRENT_SOURCE_DIR}/testdata")
set(USD_OUT  "${CMAKE_CURRENT_BINARY_DIR}/test_out")
file(MAKE_DIRECTORY "${USD_OUT}")

# usd_case2(<case> <input> <regex> [options...]): convert, require <regex>, fail on any error line.
function(usd_case2 name input regex)
    add_test(NAME USD_Reader.${name} COMMAND USD_Reader "${input}" "${USD_OUT}/${name}.prc" --force ${ARGN})
    set_tests_properties(USD_Reader.${name} PROPERTIES PASS_REGULAR_EXPRESSION "${regex}"
                                                      FAIL_REGULAR_EXPRESSION "ERROR|VERIFY FAILED|HOOPS error")
endfunction()

# 1. Mesh, translation, units (0.001 m -> 1 mm), constant displayColor.
usd_case2(cube_mesh "${USD_DATA}/cube_mesh.usda"
    "unit 1 mm, occurrences 3, unique meshes 1, triangles 12, bbox \\(100 0 0\\)-\\(110 20 30\\).*Check: OK")
# 2. Y-up stage turned Z-up: (x, y, z) -> (x, -z, y); 0.01 m -> 10 mm. Proves the PRC matrix convention.
usd_case2(cube_yup "${USD_DATA}/cube_yup.usda"
    "unit 10 mm,.*bbox \\(100 -330 200\\)-\\(110 -300 220\\).*Check: OK")
usd_case2(cube_yup_keep "${USD_DATA}/cube_yup.usda"
    "bbox \\(100 200 300\\)-\\(110 220 330\\).*Check: OK" --keep-up-axis)
# 3. Composed xformOps: translate + rotateZ, translate + non-uniform scale.
usd_case2(transforms "${USD_DATA}/transforms.usda" "bbox \\(0 0 0\\)-\\(10 3 14\\).*Check: OK")
# 4. Intrinsic primitives, tessellated (32 segments): Cube Sphere Cylinder(axis X) Cone(axis Y) Capsule Plane Cylinder_1.
usd_case2(primitives "${USD_DATA}/primitives.usda"
    "primitives 7,.*Triangles: unique 2308, drawn 2308.*bbox \\(-1 -1 -2\\)-\\(31 1 2\\).*Check: OK")
usd_case2(primitives_coarse "${USD_DATA}/primitives.usda" "bbox \\(-1 -1 -2\\)-\\(31 1 2\\).*Check: OK" --segments 8)
# 5. Hierarchy: names, kind and custom attributes, uniform colours (per-face styles), hole faces, left-handed
#    orientation, vertex colours + opacity, resetXformStack, invisible prim (hidden), guide/proxy skipped,
#    BasisCurves reported as unsupported.
usd_case2(hierarchy "${USD_DATA}/hierarchy.usda"
    "Skipped: purpose guide=1 purpose proxy=1 unsupported BasisCurves=1.*PRC tree:.*World \\{kind=assembly\\}.*Assembly \\[xform\\] \\{kind=component\\} \\{partNumber=PN-42\\} \\{userProperties:supplier=ACME\\}.*Panel \\| mesh 4 triangles faces: color\\(0,0,1\\) color\\(1,0,0\\).*Dots \\| mesh 1 triangles vertex-colors.*Hidden \\[hidden\\].*Check: OK"
    --dump-tree)
usd_case2(hierarchy_skip_hidden "${USD_DATA}/hierarchy.usda" "hidden 0.*invisible=1.*Check: OK" --skip-hidden)
usd_case2(hierarchy_with_proxy_guide "${USD_DATA}/hierarchy.usda" "Skipped: unsupported BasisCurves=1\n.*Check: OK" --include-proxy --include-guide)
# 6. Materials: UsdPreviewSurface (metallic), GeomSubset per-face material with opacity, primvar-reader displayColor,
#    texture fallback, MaterialX standard_surface.
usd_case2(materials "${USD_DATA}/materials.usda"
    "Sheet \\| mesh 4 triangles faces: material\\(0.5,0.5,0.5\\) material\\(1,0,0\\) alpha=0.502.*Ball \\[xform\\] \\| mesh 960 triangles material\\(0,1,0\\).*Textured \\[xform\\] \\| mesh 12 triangles material\\(0,0,1\\).*Mtlx \\[xform\\] \\| mesh 12 triangles material\\(1,1,0\\).*Check: OK"
    --dump-tree)
# 7. Native instancing: 2 USD prototypes (bolt, plate with nested bolt instances) shared in PRC; Bolt3 invisible.
usd_case2(instancing "${USD_DATA}/instancing.usda"
    "native instances 6, point instances 0, prototypes 2, hidden 1.*unique meshes 3, triangles 1252, bbox \\(-5 -1 -10\\)-\\(6 25 3\\).*Check: OK")
usd_case2(instancing_skip_hidden "${USD_DATA}/instancing.usda" "triangles 1004, bbox \\(-5 -1 -10\\)-\\(6 25 3\\).*Check: OK" --skip-hidden)
# 8. PointInstancer: 2 prototypes (proto xform included), orientations, scales, invisibleIds.
usd_case2(pointinstancer "${USD_DATA}/pointinstancer.usda"
    "point instances 5, prototypes 2, hidden 1.*unique meshes 2, triangles 1956, bbox \\(-1 -2 -2\\)-\\(41 2 2\\).*Check: OK")
# 9. Variants: authored default selection, then an override.
usd_case2(variant_default "${USD_DATA}/variants.usda" "Variant /Bolt\\{size=small\\}.*bbox \\(-0.5 -0.5 -0.5\\)-\\(0.5 0.5 0.5\\).*Check: OK")
usd_case2(variant_large "${USD_DATA}/variants.usda" "Variant /Bolt\\{size=large\\}.*bbox \\(-2 -2 -2\\)-\\(2 2 2\\).*Check: OK"
    --variant "/Bolt{size=large}")

# 10. Binary crate (.usdc) made with OpenUSD's usdcat from the primitives file: same result.
if(EXISTS "${USD_ROOT}/bin/usdcat.exe" OR EXISTS "${USD_ROOT}/bin/usdcat")
    add_test(NAME USD_Reader.make_usdc COMMAND "${USD_ROOT}/bin/usdcat" "${USD_DATA}/primitives.usda" -o "${USD_OUT}/primitives.usdc")
    set_tests_properties(USD_Reader.make_usdc PROPERTIES FIXTURES_SETUP usdc ENVIRONMENT "${HOOPS_USD_TOOLS_ENV}")
    usd_case2(usdc "${USD_OUT}/primitives.usdc" "Triangles: unique 2308, drawn 2308.*bbox \\(-1 -1 -2\\)-\\(31 1 2\\).*Check: OK")
    set_tests_properties(USD_Reader.usdc PROPERTIES FIXTURES_REQUIRED usdc)
endif()

# 10b. Zipped package (.usdz): a sample shipped with the OpenUSD package (spheres with displayOpacity, a cone in a
#      kind=prop model, a mesh).
set(USD_SAMPLE_USDZ "${USD_ROOT}/tests/ctest/testDiffToolIterateUsdz1/a_crate.usdz")
if(EXISTS "${USD_SAMPLE_USDZ}")
    usd_case2(usdz "${USD_SAMPLE_USDZ}" "primitives 4,.*color\\(1,0,0\\) alpha=0.4.*prop1 \\{kind=prop\\}.*unit 10 mm, occurrences 8, unique meshes 5, triangles 2944.*Check: OK" --dump-tree)
endif()

# 11. Error paths and defaults.
add_test(NAME USD_Reader.usage COMMAND USD_Reader)
set_tests_properties(USD_Reader.usage PROPERTIES WILL_FAIL TRUE)
add_test(NAME USD_Reader.missing_input COMMAND USD_Reader "${USD_DATA}/does_not_exist.usda" "${USD_OUT}/x.prc")
set_tests_properties(USD_Reader.missing_input PROPERTIES PASS_REGULAR_EXPRESSION "input file not found")
add_test(NAME USD_Reader.bad_output_ext COMMAND USD_Reader "${USD_DATA}/cube_mesh.usda" "${USD_OUT}/x.step")
set_tests_properties(USD_Reader.bad_output_ext PROPERTIES PASS_REGULAR_EXPRESSION "unsupported output format")
file(WRITE "${USD_OUT}/not_usd.obj" "v 0 0 0\n")
add_test(NAME USD_Reader.bad_input_ext COMMAND USD_Reader "${USD_OUT}/not_usd.obj" "${USD_OUT}/x.prc")
set_tests_properties(USD_Reader.bad_input_ext PROPERTIES PASS_REGULAR_EXPRESSION "unsupported input format")
file(WRITE "${USD_OUT}/malformed.usda" "#usda 1.0\ndef Xform \"A\" {\n    double3 xformOp:translate = (1, 2\n")
add_test(NAME USD_Reader.malformed COMMAND USD_Reader "${USD_OUT}/malformed.usda" "${USD_OUT}/malformed.prc" --force)
set_tests_properties(USD_Reader.malformed PROPERTIES PASS_REGULAR_EXPRESSION "cannot open USD stage")
file(WRITE "${USD_OUT}/empty.usda" "#usda 1.0\n")
add_test(NAME USD_Reader.empty_stage COMMAND USD_Reader "${USD_OUT}/empty.usda" "${USD_OUT}/empty.prc" --force)
set_tests_properties(USD_Reader.empty_stage PROPERTIES PASS_REGULAR_EXPRESSION "no convertible geometry")
file(MAKE_DIRECTORY "${USD_OUT}/exists")
file(WRITE "${USD_OUT}/exists/existing.prc" "not a prc")
add_test(NAME USD_Reader.no_overwrite COMMAND USD_Reader "${USD_DATA}/cube_mesh.usda" "${USD_OUT}/exists/existing.prc")
set_tests_properties(USD_Reader.no_overwrite PROPERTIES PASS_REGULAR_EXPRESSION "output exists")
add_test(NAME USD_Reader.unknown_option COMMAND USD_Reader "${USD_DATA}/cube_mesh.usda" "${USD_OUT}/u.prc" --frobnicate)
set_tests_properties(USD_Reader.unknown_option PROPERTIES PASS_REGULAR_EXPRESSION "unknown option")
add_test(NAME USD_Reader.bad_variant_syntax COMMAND USD_Reader "${USD_DATA}/variants.usda" "${USD_OUT}/v.prc" --variant "/Bolt:size=large")
set_tests_properties(USD_Reader.bad_variant_syntax PROPERTIES PASS_REGULAR_EXPRESSION "--variant expects")
add_test(NAME USD_Reader.unknown_variant COMMAND USD_Reader "${USD_DATA}/variants.usda" "${USD_OUT}/v.prc" --force --variant "/Bolt{size=huge}")
set_tests_properties(USD_Reader.unknown_variant PROPERTIES PASS_REGULAR_EXPRESSION "'huge' is not a variant of /Bolt\\{size\\} \\(small, large\\)|'huge' is not a variant of /Bolt\\{size\\} \\(large, small\\)")
add_test(NAME USD_Reader.bad_segments COMMAND USD_Reader "${USD_DATA}/primitives.usda" "${USD_OUT}/s.prc" --segments 6)
set_tests_properties(USD_Reader.bad_segments PROPERTIES PASS_REGULAR_EXPRESSION "multiple of 4")

# 12. Non-ASCII input and output paths (wmain + UTF-8 inside).
set(USD_UNICODE_DIR "${USD_OUT}/unicodé_ü")
file(MAKE_DIRECTORY "${USD_UNICODE_DIR}")
file(COPY_FILE "${USD_DATA}/cube_mesh.usda" "${USD_UNICODE_DIR}/boîte.usda")
add_test(NAME USD_Reader.unicode_paths COMMAND USD_Reader "${USD_UNICODE_DIR}/boîte.usda" "${USD_UNICODE_DIR}/boîte.prc" --force)
set_tests_properties(USD_Reader.unicode_paths PROPERTIES PASS_REGULAR_EXPRESSION "bbox \\(100 0 0\\)-\\(110 20 30\\).*Check: OK"
                                                     FAIL_REGULAR_EXPRESSION "ERROR|VERIFY FAILED|HOOPS error")
