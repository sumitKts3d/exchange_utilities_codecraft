# CTest cases for USD_Writer. Run:
#   ctest --test-dir <build dir> -C Release -R "^USD_Writer\." --output-on-failure
#
# Every case re-opens the written file with OpenUSD and checks meshes, triangles, PMI, cameras, units and the world
# bounding box against the HOOPS Exchange model ("Check: OK"), per configuration for multi-configuration files.
# Round trips go through USD_Reader (built in the same project).
set(CAD  "${A3DLIBS_ROOT}/samples/data")
set(WOUT "${CMAKE_CURRENT_BINARY_DIR}/test_out")
file(MAKE_DIRECTORY "${WOUT}")
set(W_FAIL "ERROR|VERIFY FAILED|HOOPS error")

# usd_write(<case> <input> <output file name> <regex> [options...])
function(usd_write name input out regex)
    add_test(NAME USD_Writer.${name} COMMAND USD_Writer "${input}" "${WOUT}/${out}" --force ${ARGN})
    set_tests_properties(USD_Writer.${name} PROPERTIES PASS_REGULAR_EXPRESSION "${regex}" FAIL_REGULAR_EXPRESSION "${W_FAIL}"
                                                      FIXTURES_SETUP "w_${name}")
endfunction()
# usdchecker on a written file (OpenUSD's validator).
function(usd_check name file)
    if(EXISTS "${USD_ROOT}/bin/usdchecker.exe" OR EXISTS "${USD_ROOT}/bin/usdchecker")
        add_test(NAME USD_Writer.usdchecker_${name} COMMAND "${USD_ROOT}/bin/usdchecker" "${WOUT}/${file}")
        set_tests_properties(USD_Writer.usdchecker_${name} PROPERTIES FIXTURES_REQUIRED "w_${name}" ENVIRONMENT "${HOOPS_USD_TOOLS_ENV}")
    endif()
endfunction()

# 1. One format per output flavour, from several CAD formats.
usd_write(prc_usda "${CAD}/prc/helloworld.prc" helloworld_prc.usda "metersPerUnit 0.001, upAxis Z.*Check: OK")
usd_check(prc_usda helloworld_prc.usda)
usd_write(step_usdc "${CAD}/step/helloworld.stp" helloworld_step.usdc "Check: OK")
usd_check(step_usdc helloworld_step.usdc)
usd_write(jt_usdz "${CAD}/jt/Flange287.jt" flange.usdz "Check: OK")
usd_check(jt_usdz flange.usdz)
usd_write(catia_usd "${CAD}/catiaV5/CV5_Aquo_Bottle/_Aquo Bottle.CATProduct" aquo.usd "Check: OK")
usd_check(catia_usd aquo.usd)
# 2. Assembly with repeated parts: instancing on (prototypes) and off.
usd_write(assembly "${CAD}/prc/_micro engine.prc" micro_engine.usda "prototypes [1-9][0-9]*, instances [1-9].*Check: OK" --dump-tree)
usd_check(assembly micro_engine.usda)
usd_write(assembly_flat "${CAD}/prc/_micro engine.prc" micro_engine_flat.usda "prototypes 0, instances 0.*Re-opened USD: metersPerUnit [^\n]*prototypes 0[^0-9].*Re-opened USD: prims [0-9]+, instances 0[^0-9].*Check: OK" --no-instancing)
usd_write(assembly_skip_hidden "${CAD}/prc/_micro engine.prc" micro_engine_visible.usdc "Check: OK" --skip-hidden)
usd_write(lod_high "${CAD}/step/helloworld.stp" helloworld_high.usdc "tessellation high.*Check: OK" --lod high)
usd_write(solidworks "${CAD}/solidworks/SLW_Diskbrakeassembly/Assem2.SLDASM" diskbrake.usdc "Check: OK")

# 3. Round trip CAD -> USD -> PRC (USD_Reader checks its own PRC against the USD stage).
foreach(c prc_usda:helloworld_prc.usda assembly:micro_engine.usda catia_usd:aquo.usd)
    string(REPLACE ":" ";" parts "${c}")
    list(GET parts 0 src)
    list(GET parts 1 file)
    add_test(NAME USD_Writer.roundtrip_${src} COMMAND $<TARGET_FILE:USD_Reader> "${WOUT}/${file}" "${WOUT}/${src}_roundtrip.prc" --force)
    set_tests_properties(USD_Writer.roundtrip_${src} PROPERTIES FIXTURES_REQUIRED "w_${src}" PASS_REGULAR_EXPRESSION "Check: OK"
                                                               FAIL_REGULAR_EXPRESSION "${W_FAIL}")
endforeach()

# 4. Round trip USD -> PRC (USD_Reader) -> USD (USD_Writer) with USD_Reader's test files: the world box must come back.
set(RDATA "${CMAKE_CURRENT_SOURCE_DIR}/../USD_Reader/testdata")
foreach(c "instancing:bbox \\(-5 -1 -10\\)-\\(6 25 3\\)" "pointinstancer:bbox \\(-1 -2 -2\\)-\\(41 2 2\\)" "materials:bbox \\(0 -1 -1\\)-\\(15.5 1 1\\)"
          "primitives:bbox \\(-1 -1 -2\\)-\\(31 1 2\\)")
    string(FIND "${c}" ":" colon)
    string(SUBSTRING "${c}" 0 ${colon} src)
    math(EXPR colon "${colon} + 1")
    string(SUBSTRING "${c}" ${colon} -1 regex)
    add_test(NAME USD_Writer.usd_to_prc_${src} COMMAND $<TARGET_FILE:USD_Reader> "${RDATA}/${src}.usda" "${WOUT}/${src}_from_usd.prc" --force)
    set_tests_properties(USD_Writer.usd_to_prc_${src} PROPERTIES FIXTURES_SETUP "rt_${src}" PASS_REGULAR_EXPRESSION "Check: OK")
    add_test(NAME USD_Writer.prc_to_usd_${src} COMMAND USD_Writer "${WOUT}/${src}_from_usd.prc" "${WOUT}/${src}_back.usda" --force --dump-tree)
    set_tests_properties(USD_Writer.prc_to_usd_${src} PROPERTIES FIXTURES_REQUIRED "rt_${src}" FIXTURES_SETUP "w_back_${src}"
                                                                PASS_REGULAR_EXPRESSION "${regex}.*Check: OK" FAIL_REGULAR_EXPRESSION "${W_FAIL}")
    if(EXISTS "${USD_ROOT}/bin/usdchecker.exe")
        add_test(NAME USD_Writer.usdchecker_back_${src} COMMAND "${USD_ROOT}/bin/usdchecker" "${WOUT}/${src}_back.usda")
        set_tests_properties(USD_Writer.usdchecker_back_${src} PROPERTIES FIXTURES_REQUIRED "w_back_${src}" ENVIRONMENT "${HOOPS_USD_TOOLS_ENV}")
    endif()
endforeach()

# 5. v2: PMI, views and cameras.
set(PMI_RX "PMI: markups 36 \\(curves [0-9]+\\), views 6 \\(cameras 6, view variants 4\\)")
usd_write(pmi_views "${CAD}/prc/viewswithpmis.prc" views.usda "${PMI_RX}, purpose guide.*Model: .*PMI 36, cameras 6.*Check: OK")
usd_check(pmi_views views.usda)
usd_write(pmi_default "${CAD}/prc/viewswithpmis.prc" views_default.usdc
          "purpose default.*view=All of All,BracketView,BracketView_1,FaceView,FaceView_1.*pmi=Text text=\"Flatness\".*Views <Scope>.*FaceView <Camera> annotations=12.*Check: OK"
          --pmi default --dump-tree)
usd_write(pmi_none "${CAD}/prc/viewswithpmis.prc" views_none.usda "Model: meshes 1, triangles 1892, PMI 0, cameras 0.*Check: OK" --pmi none)
usd_write(pmi_catia "${CAD}/pmi/PMI_Sample/CV5_Sample.CATPart" cv5_pmi.usdc "Model: .*PMI 20, cameras 6.*Check: OK")
usd_check(pmi_catia cv5_pmi.usdc)
usd_write(pmi_prc "${CAD}/prc/pmi_Sample.prc" pmi_sample.usda "PMI: markups [1-9].*Check: OK")
usd_check(pmi_prc pmi_sample.usda)
usd_write(views_nested "${CAD}/solidworks/SLW_Diskbrakeassembly/Assem2.SLDASM" assem2_views.usda "not written: 30 \\(--all-views.*cameras 11.*Check: OK")
usd_write(views_all "${CAD}/solidworks/SLW_Diskbrakeassembly/Assem2.SLDASM" assem2_all_views.usda "Model: .*cameras 41.*Check: OK" --all-views)
# Mixed inch / mm documents in a SolidWorks assembly, with PMI.
usd_write(mixed_units "${CAD}/solidworks/SLW_Diskbrakeassembly/DiskBrakeAssembly-01.SLDASM" diskbrake01.usdc
          "metersPerUnit 0.0254.*Model: meshes 69, triangles [0-9]+, PMI 8, cameras 11.*Check: OK")
usd_check(mixed_units diskbrake01.usdc)

# 6. v2: product configurations as the variant set "configuration".
set(CALIPER "${CAD}/solidworks/SLW_Diskbrakeassembly/CaliperSubassmebly-01.SLDASM")
usd_write(configurations "${CALIPER}" caliper.usda
          "Configurations written as variants: Default, test_sub.*Configuration 'Default': meshes 46.*Configuration 'test sub': meshes 46.*Check: OK")
usd_check(configurations caliper.usda)
usd_write(configuration_named "${CALIPER}" caliper_test_sub.usdc "Model: meshes 46, triangles 131684.*Check: OK" --configuration "test sub")
usd_write(configuration_default "${CALIPER}" caliper_default.usdc "Model: meshes 46, triangles 131684.*Check: OK" --default-configuration)
add_test(NAME USD_Writer.configuration_unknown COMMAND USD_Writer "${CALIPER}" "${WOUT}/c.usda" --force --configuration nope)
set_tests_properties(USD_Writer.configuration_unknown PROPERTIES PASS_REGULAR_EXPRESSION "no configuration 'nope'")
add_test(NAME USD_Writer.configuration_none COMMAND USD_Writer "${CAD}/prc/helloworld.prc" "${WOUT}/c.usda" --force --configuration x)
set_tests_properties(USD_Writer.configuration_none PROPERTIES PASS_REGULAR_EXPRESSION "the file has no configurations")
add_test(NAME USD_Writer.bad_pmi COMMAND USD_Writer "${CAD}/prc/helloworld.prc" "${WOUT}/u.usda" --pmi everything)
set_tests_properties(USD_Writer.bad_pmi PROPERTIES PASS_REGULAR_EXPRESSION "--pmi expects")

# 7. v2 round trips through USD_Reader: PMI (guide purpose) and cameras are skipped, the geometry comes back; a
#    configuration is selected with USD_Reader's --variant.
add_test(NAME USD_Writer.roundtrip_pmi_views COMMAND $<TARGET_FILE:USD_Reader> "${WOUT}/views.usda" "${WOUT}/views_roundtrip.prc" --force)
set_tests_properties(USD_Writer.roundtrip_pmi_views PROPERTIES FIXTURES_REQUIRED "w_pmi_views"
                     PASS_REGULAR_EXPRESSION "Camera=6.*Skipped: purpose guide=1.*triangles 1892.*Check: OK" FAIL_REGULAR_EXPRESSION "${W_FAIL}")
add_test(NAME USD_Writer.roundtrip_configuration COMMAND $<TARGET_FILE:USD_Reader> "${WOUT}/caliper.usda" "${WOUT}/caliper_roundtrip.prc" --force
         --variant "/CaliperSubassmebly_01{configuration=test_sub}")
set_tests_properties(USD_Writer.roundtrip_configuration PROPERTIES FIXTURES_REQUIRED "w_configurations"
                     PASS_REGULAR_EXPRESSION "options: Default, test_sub.*unit 25.4 mm.*triangles 131684.*Check: OK" FAIL_REGULAR_EXPRESSION "${W_FAIL}")

# 8. --layout parts: one file per part / shared sub-assembly under <output stem>_parts/.
usd_write(layout_parts "${CAD}/prc/_micro engine.prc" micro_layout.usda
          "Layout: parts, 15 part files in micro_layout_parts/ \\(referenced 21 times, instanceable 8, payloads 0\\).*Layers: 16 \\(main file \\+ 15 part files\\), unresolved asset paths 0, part files opening on their own 15.*Check: OK"
          --layout parts)
usd_check(layout_parts micro_layout.usda)
# Writing again over the same output replaces the part folder.
add_test(NAME USD_Writer.layout_parts_rewrite COMMAND USD_Writer "${CAD}/prc/_micro engine.prc" "${WOUT}/micro_layout.usda" --force --layout parts)
set_tests_properties(USD_Writer.layout_parts_rewrite PROPERTIES FIXTURES_REQUIRED "w_layout_parts" PASS_REGULAR_EXPRESSION "Layers: 16 .*Check: OK"
                     FAIL_REGULAR_EXPRESSION "${W_FAIL}")
usd_write(layout_parts_flat "${CAD}/prc/_micro engine.prc" micro_layout_flat.usdc "instanceable 0, payloads 0\\).*Check: OK" --layout parts --no-instancing)
usd_write(layout_parts_configurations "${CALIPER}" caliper_layout.usdc
          "Layout: parts, 20 part files in caliper_layout_parts/.*unresolved asset paths 0.*Configuration 'Default': meshes 46.*Configuration 'test sub': meshes 46.*Check: OK"
          --layout parts)
usd_check(layout_parts_configurations caliper_layout.usdc)
usd_write(layout_parts_usdz "${CAD}/catiaV5/CV5_Aquo_Bottle/_Aquo Bottle.CATProduct" aquo_layout.usdz
          "Layout: parts, 6 part files.*Layers: 7 .*unresolved asset paths 0.*PMI 71, cameras 6.*Check: OK" --layout parts)
usd_check(layout_parts_usdz aquo_layout.usdz)
add_test(NAME USD_Writer.roundtrip_layout_parts COMMAND $<TARGET_FILE:USD_Reader> "${WOUT}/micro_layout.usda" "${WOUT}/micro_layout_roundtrip.prc" --force)
set_tests_properties(USD_Writer.roundtrip_layout_parts PROPERTIES FIXTURES_REQUIRED "w_layout_parts" PASS_REGULAR_EXPRESSION "triangles 48828.*Check: OK"
                     FAIL_REGULAR_EXPRESSION "${W_FAIL}")
# The part folder is never replaced if it holds anything else than USD files, nor written over without --force.
file(MAKE_DIRECTORY "${WOUT}/guard_parts")
file(WRITE "${WOUT}/guard_parts/notes.txt" "not a part file\n")
add_test(NAME USD_Writer.layout_parts_foreign_folder COMMAND USD_Writer "${CAD}/prc/helloworld.prc" "${WOUT}/guard.usda" --force --layout parts)
set_tests_properties(USD_Writer.layout_parts_foreign_folder PROPERTIES PASS_REGULAR_EXPRESSION "holds other files than USD part files")
file(MAKE_DIRECTORY "${WOUT}/exists/existing2_parts")
add_test(NAME USD_Writer.layout_parts_no_overwrite COMMAND USD_Writer "${CAD}/prc/helloworld.prc" "${WOUT}/exists/existing2.usda" --layout parts)
set_tests_properties(USD_Writer.layout_parts_no_overwrite PROPERTIES PASS_REGULAR_EXPRESSION "output exists: .*existing2_parts")
add_test(NAME USD_Writer.bad_layout COMMAND USD_Writer "${CAD}/prc/helloworld.prc" "${WOUT}/u.usda" --layout everything)
set_tests_properties(USD_Writer.bad_layout PROPERTIES PASS_REGULAR_EXPRESSION "--layout expects")

# 9. --payloads (with --layout parts): the assembly loads its parts on demand.
usd_write(payloads "${CAD}/prc/_micro engine.prc" micro_payloads.usda
          "payloads 21\\).*Payloads: 21 prims; opened unloaded: meshes 0; after Load\\(\\): meshes 21, triangles 48828.*Check: OK"
          --layout parts --payloads)
usd_check(payloads micro_payloads.usda)
usd_write(payloads_configurations "${CALIPER}" caliper_payloads.usdc "20 part files.*payloads [1-9][0-9]*\\).*Configuration 'test sub': meshes 46.*Payloads: [1-9][0-9]* prims.*Check: OK"
          --layout parts --payloads)
add_test(NAME USD_Writer.roundtrip_payloads COMMAND $<TARGET_FILE:USD_Reader> "${WOUT}/micro_payloads.usda" "${WOUT}/micro_payloads_roundtrip.prc" --force)
set_tests_properties(USD_Writer.roundtrip_payloads PROPERTIES FIXTURES_REQUIRED "w_payloads" PASS_REGULAR_EXPRESSION "triangles 48828.*Check: OK"
                     FAIL_REGULAR_EXPRESSION "${W_FAIL}")
add_test(NAME USD_Writer.payloads_without_parts COMMAND USD_Writer "${CAD}/prc/helloworld.prc" "${WOUT}/u.usda" --payloads)
set_tests_properties(USD_Writer.payloads_without_parts PROPERTIES PASS_REGULAR_EXPRESSION "--payloads needs --layout parts")

# 10. --layout layers: one layer per kind of data under <output stem>_layers/; muting a layer drops exactly that data.
usd_write(layout_layers "${CAD}/prc/viewswithpmis.prc" pmi_layered.usda
          "Layout: layers, 3 layers in pmi_layered_layers/ \\(views.usda, pmi.usda, geometry.usda\\).*Layers: 4 \\(main file \\+ 3 layers\\), unresolved asset paths 0.*PMI 36, cameras 6.*Muted views.usda: meshes 1, triangles 1892, PMI 36, cameras 0.*Muted pmi.usda: meshes 1, triangles 1892, PMI 0, cameras 6.*Check: OK"
          --layout layers)
usd_check(layout_layers pmi_layered.usda)
usd_write(layout_layers_configurations "${CALIPER}" caliper_layered.usdc
          "\\(views.usdc, pmi.usdc, looks.usdc, geometry.usdc\\).*Configuration 'Default': meshes 46.*Configuration 'test sub': meshes 46.*Muted pmi.usdc: meshes 46, triangles 131684, PMI 0.*Check: OK"
          --layout layers)
usd_check(layout_layers_configurations caliper_layered.usdc)
usd_write(layout_layers_usdz "${CAD}/catiaV5/CV5_Aquo_Bottle/_Aquo Bottle.CATProduct" aquo_layered.usdz
          "Without views.usdc: .*cameras 0.*Without pmi.usdc: meshes 9, triangles 63424, PMI 0.*Check: OK" --layout layers)
usd_check(layout_layers_usdz aquo_layered.usdz)
add_test(NAME USD_Writer.roundtrip_layout_layers COMMAND $<TARGET_FILE:USD_Reader> "${WOUT}/caliper_layered.usdc" "${WOUT}/caliper_layered_roundtrip.prc" --force
         --variant "/CaliperSubassmebly_01{configuration=test_sub}")
set_tests_properties(USD_Writer.roundtrip_layout_layers PROPERTIES FIXTURES_REQUIRED "w_layout_layers_configurations"
                     PASS_REGULAR_EXPRESSION "triangles 131684.*Check: OK" FAIL_REGULAR_EXPRESSION "${W_FAIL}")

# 11. Error paths and defaults.
add_test(NAME USD_Writer.usage COMMAND USD_Writer)
set_tests_properties(USD_Writer.usage PROPERTIES WILL_FAIL TRUE)
add_test(NAME USD_Writer.missing_input COMMAND USD_Writer "${CAD}/does_not_exist.stp" "${WOUT}/x.usda")
set_tests_properties(USD_Writer.missing_input PROPERTIES PASS_REGULAR_EXPRESSION "input file not found")
add_test(NAME USD_Writer.bad_output_ext COMMAND USD_Writer "${CAD}/prc/helloworld.prc" "${WOUT}/x.obj")
set_tests_properties(USD_Writer.bad_output_ext PROPERTIES PASS_REGULAR_EXPRESSION "unsupported output format")
file(WRITE "${WOUT}/not_cad.stp" "this is not a STEP file\n")
add_test(NAME USD_Writer.unreadable_input COMMAND USD_Writer "${WOUT}/not_cad.stp" "${WOUT}/x.usda" --force)
set_tests_properties(USD_Writer.unreadable_input PROPERTIES PASS_REGULAR_EXPRESSION "A3DAsmModelFileLoadFromFile failed")
file(MAKE_DIRECTORY "${WOUT}/exists")
file(WRITE "${WOUT}/exists/existing.usda" "#usda 1.0\n")
add_test(NAME USD_Writer.no_overwrite COMMAND USD_Writer "${CAD}/prc/helloworld.prc" "${WOUT}/exists/existing.usda")
set_tests_properties(USD_Writer.no_overwrite PROPERTIES PASS_REGULAR_EXPRESSION "output exists")
add_test(NAME USD_Writer.unknown_option COMMAND USD_Writer "${CAD}/prc/helloworld.prc" "${WOUT}/u.usda" --frobnicate)
set_tests_properties(USD_Writer.unknown_option PROPERTIES PASS_REGULAR_EXPRESSION "unknown option")
add_test(NAME USD_Writer.bad_lod COMMAND USD_Writer "${CAD}/prc/helloworld.prc" "${WOUT}/u.usda" --lod ultra)
set_tests_properties(USD_Writer.bad_lod PROPERTIES PASS_REGULAR_EXPRESSION "--lod expects")
# Non-ASCII input and output paths.
set(W_UNICODE_DIR "${WOUT}/unicodé_ü")
file(MAKE_DIRECTORY "${W_UNICODE_DIR}")
file(COPY_FILE "${CAD}/prc/helloworld.prc" "${W_UNICODE_DIR}/modèle.prc")
add_test(NAME USD_Writer.unicode_paths COMMAND USD_Writer "${W_UNICODE_DIR}/modèle.prc" "${W_UNICODE_DIR}/modèle.usda" --force)
set_tests_properties(USD_Writer.unicode_paths PROPERTIES PASS_REGULAR_EXPRESSION "Check: OK" FAIL_REGULAR_EXPRESSION "${W_FAIL}")
