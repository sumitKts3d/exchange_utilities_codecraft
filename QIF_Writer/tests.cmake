# CTest cases for QIF_Writer. Run:
#   ctest --test-dir <build dir> -C Release -R "^QIF_Writer\." --output-on-failure
#
# The QIF files are checked by reading them back with ../QIF_Reader (an independent QIF reader built in the same
# workspace): faces, exact area and volume must come back (roundtrip.cmake), and the semantic PMI of the NIST models
# must survive QIF -> PRC -> QIF -> PRC unchanged (QIF_Reader's compare_pmi.cmake against its expected_pmi files).
set(CAD  "${A3DLIBS_ROOT}/samples/data")
set(QOUT "${CMAKE_CURRENT_BINARY_DIR}/test_out")
set(QIF_DATA "${CMAKE_CURRENT_SOURCE_DIR}/../QIF_Reader/testdata")
file(MAKE_DIRECTORY "${QOUT}")
set(Q_FAIL "ERROR|VERIFY FAILED")

# 1. Writes from several CAD formats (self-check: every p-curve meets its vertices).
function(qif_write name input regex)
    add_test(NAME QIF_Writer.${name} COMMAND QIF_Writer "${input}" "${QOUT}/${name}.qif" --force ${ARGN})
    set_tests_properties(QIF_Writer.${name} PROPERTIES PASS_REGULAR_EXPRESSION "${regex}" FAIL_REGULAR_EXPRESSION "${Q_FAIL}")
endfunction()
qif_write(prc_helloworld "${CAD}/prc/helloworld.prc" "Check: OK")
qif_write(step_helloworld "${CAD}/step/helloworld.stp" "Check: OK")
qif_write(step_flange "${CAD}/step/Flange287.stp" "Check: OK")
qif_write(assembly "${CAD}/prc/_micro engine.prc" "bodies [1-9][0-9]*.*Check: OK")
qif_write(assembly_skip_hidden "${CAD}/prc/_micro engine.prc" "Check: OK" --skip-hidden)
qif_write(pmi_views "${CAD}/prc/viewswithpmis.prc" "displays [1-9][0-9]*.*saved views [1-9].*Check: OK")
qif_write(no_pmi "${CAD}/prc/viewswithpmis.prc" "Check: OK" --no-pmi)
set_tests_properties(QIF_Writer.no_pmi PROPERTIES FAIL_REGULAR_EXPRESSION "${Q_FAIL}|Written PMI")
qif_write(catia_assembly "${CAD}/catiaV5/CV5_Aquo_Bottle/_Aquo Bottle.CATProduct" "Check: OK")

if(TARGET QIF_Reader)
    # 2. CAD -> QIF -> PRC: faces, exact area and volume come back.
    function(qif_roundtrip name input)
        add_test(NAME QIF_Writer.roundtrip_${name}
                 COMMAND ${CMAKE_COMMAND} -DWRITER=$<TARGET_FILE:QIF_Writer> -DREADER=$<TARGET_FILE:QIF_Reader> "-DIN=${input}"
                         "-DQIF=${QOUT}/rt_${name}.qif" "-DPRC=${QOUT}/rt_${name}.prc" -P ${CMAKE_CURRENT_SOURCE_DIR}/roundtrip.cmake)
        set_tests_properties(QIF_Writer.roundtrip_${name} PROPERTIES PASS_REGULAR_EXPRESSION "round trip OK")
    endfunction()
    qif_roundtrip(helloworld "${CAD}/prc/helloworld.prc")
    qif_roundtrip(step_helloworld "${CAD}/step/helloworld.stp")
    qif_roundtrip(step_flange "${CAD}/step/Flange287.stp")
    qif_roundtrip(catia_assembly "${CAD}/catiaV5/CV5_Aquo_Bottle/_Aquo Bottle.CATProduct")
    qif_roundtrip(micro_engine "${CAD}/prc/_micro engine.prc")

    # 3. QIF (NIST) -> PRC (QIF_Reader) -> QIF (QIF_Writer) -> PRC (QIF_Reader): geometry and semantic PMI unchanged.
    foreach(m nist_ctc_01_asme1_ap242 nist_ctc_03_asme1_ap242 nist_ctc_05_asme1_ap242
              nist_ftc_06_asme1_ap242 nist_ftc_08_asme1_ap242-1 nist_ftc_09_asme1_ap242)
        string(REPLACE " " "_" t "${m}")
        add_test(NAME QIF_Writer.from_qif_${t} COMMAND QIF_Reader "${QIF_DATA}/${m}.qif" "${QOUT}/${t}_src.prc" --force --quiet)
        set_tests_properties(QIF_Writer.from_qif_${t} PROPERTIES FIXTURES_SETUP "src_${t}")
        qif_roundtrip(${t} "${QOUT}/${t}_src.prc")
        set_tests_properties(QIF_Writer.roundtrip_${t} PROPERTIES FIXTURES_REQUIRED "src_${t}" FIXTURES_SETUP "rt_${t}")
        add_test(NAME QIF_Writer.semantic_${t}
                 COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:QIF_Reader> "-DIN=${QOUT}/rt_${t}.qif" "-DOUT=${QOUT}/${t}_sem.prc"
                         "-DEXPECTED=${QIF_DATA}/expected_pmi/${m}.txt" -P ${CMAKE_CURRENT_SOURCE_DIR}/../QIF_Reader/compare_pmi.cmake)
        set_tests_properties(QIF_Writer.semantic_${t} PROPERTIES FIXTURES_REQUIRED "rt_${t}" PASS_REGULAR_EXPRESSION "semantic PMI identical")
    endforeach()
endif()

# 4. Error paths and defaults.
# A model with tessellation only (no exact B-rep) cannot be written as QIF B-rep: exit 3 with an explanation.
add_test(NAME QIF_Writer.mesh_only COMMAND QIF_Writer "${CAD}/prc/pmi_Sample.prc" "${QOUT}/mesh_only.qif" --force)
set_tests_properties(QIF_Writer.mesh_only PROPERTIES PASS_REGULAR_EXPRESSION "no exact B-rep body to write")
add_test(NAME QIF_Writer.usage COMMAND QIF_Writer)
set_tests_properties(QIF_Writer.usage PROPERTIES WILL_FAIL TRUE)
add_test(NAME QIF_Writer.missing_input COMMAND QIF_Writer "${CAD}/prc/does_not_exist.prc" "${QOUT}/x.qif")
set_tests_properties(QIF_Writer.missing_input PROPERTIES PASS_REGULAR_EXPRESSION "input file not found")
add_test(NAME QIF_Writer.bad_extension COMMAND QIF_Writer "${CAD}/prc/helloworld.prc" "${QOUT}/x.step")
set_tests_properties(QIF_Writer.bad_extension PROPERTIES PASS_REGULAR_EXPRESSION "unsupported output format")
file(MAKE_DIRECTORY "${QOUT}/exists")
file(WRITE "${QOUT}/exists/existing.qif" "not a qif")
add_test(NAME QIF_Writer.no_overwrite COMMAND QIF_Writer "${CAD}/prc/helloworld.prc" "${QOUT}/exists/existing.qif")
set_tests_properties(QIF_Writer.no_overwrite PROPERTIES PASS_REGULAR_EXPRESSION "output exists")
add_test(NAME QIF_Writer.unknown_option COMMAND QIF_Writer "${CAD}/prc/helloworld.prc" "${QOUT}/u.qif" --frobnicate)
set_tests_properties(QIF_Writer.unknown_option PROPERTIES PASS_REGULAR_EXPRESSION "unknown option")
set(Q_UNICODE "${QOUT}/unicodé_ü")
file(MAKE_DIRECTORY "${Q_UNICODE}")
file(COPY_FILE "${CAD}/prc/helloworld.prc" "${Q_UNICODE}/bonjour_monde.prc")
add_test(NAME QIF_Writer.unicode_paths COMMAND QIF_Writer "${Q_UNICODE}/bonjour_monde.prc" "${Q_UNICODE}/sortie_é.qif" --force)
set_tests_properties(QIF_Writer.unicode_paths PROPERTIES PASS_REGULAR_EXPRESSION "Check: OK" FAIL_REGULAR_EXPRESSION "${Q_FAIL}")
