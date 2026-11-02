# CTest cases for QIF_Reader. Run:
#   ctest --test-dir <build dir> -C Release -R "^QIF_Reader\." --output-on-failure
set(QIF_DATA "${CMAKE_CURRENT_SOURCE_DIR}/testdata")
set(QIF_OUT  "${CMAKE_CURRENT_BINARY_DIR}/test_out")
file(MAKE_DIRECTORY "${QIF_OUT}")

# 1. Synthetic box (6 planes, 12 segment edges): exact area/volume after re-import.
add_test(NAME QIF_Reader.box COMMAND QIF_Reader "${QIF_DATA}/box_10x20x30.qif" "${QIF_OUT}/box.prc" --force --max-dev 1e-9)
set_tests_properties(QIF_Reader.box PROPERTIES
    PASS_REGULAR_EXPRESSION "Re-imported PRC: +B-reps 1, faces 6, area 2200 mm2, volume 6000 mm3")

# 2. NIST QIF 3.0 models (planes, cylinders, cones, spheres, tori, NURBS; segment/arc/ellipse/NURBS/aggregate curves).
#    --max-dev: every PRC surface / curve is evaluated through HOOPS Exchange and compared with the QIF definition.
foreach(m nist_ctc_01_asme1_ap242 nist_ctc_03_asme1_ap242 nist_ctc_05_asme1_ap242
          nist_ftc_06_asme1_ap242 nist_ftc_08_asme1_ap242-1 nist_ftc_09_asme1_ap242)
    add_test(NAME QIF_Reader.${m} COMMAND QIF_Reader "${QIF_DATA}/${m}.qif" "${QIF_OUT}/${m}.prc" --force --max-dev 1e-6)
    # All six NIST parts are closed solids: the re-imported PRC must be measured exactly with a non-zero volume.
    set_tests_properties(QIF_Reader.${m} PROPERTIES PASS_REGULAR_EXPRESSION "Re-imported PRC: +B-reps [0-9]+, faces [0-9]+, area [0-9.e+]+ mm2, volume [1-9][0-9.e+]* mm3(,|\n)")
    # Same file, cross-checked against HOOPS Exchange's own (beta) QIF reader: faces, area, volume.
    add_test(NAME QIF_Reader.native_${m} COMMAND QIF_Reader "${QIF_DATA}/${m}.qif" "${QIF_OUT}/${m}_vs_native.prc" --force --quiet --compare-native --skip-hidden)  # the native beta reader ignores hidden bodies
endforeach()

# 3. Error paths and defaults.
add_test(NAME QIF_Reader.usage COMMAND QIF_Reader)
set_tests_properties(QIF_Reader.usage PROPERTIES WILL_FAIL TRUE)
add_test(NAME QIF_Reader.missing_input COMMAND QIF_Reader "${QIF_DATA}/does_not_exist.qif" "${QIF_OUT}/x.prc")
set_tests_properties(QIF_Reader.missing_input PROPERTIES PASS_REGULAR_EXPRESSION "input file not found")
add_test(NAME QIF_Reader.bad_extension COMMAND QIF_Reader "${QIF_DATA}/box_10x20x30.qif" "${QIF_OUT}/box.step")
set_tests_properties(QIF_Reader.bad_extension PROPERTIES PASS_REGULAR_EXPRESSION "unsupported output format")
add_test(NAME QIF_Reader.malformed_xml COMMAND QIF_Reader "${QIF_DATA}/malformed.qif" "${QIF_OUT}/malformed.prc" --force)
set_tests_properties(QIF_Reader.malformed_xml PROPERTIES PASS_REGULAR_EXPRESSION "cannot read QIF: XML: mismatched end tag")
file(MAKE_DIRECTORY "${QIF_OUT}/exists")
file(WRITE "${QIF_OUT}/exists/existing.prc" "not a prc")
add_test(NAME QIF_Reader.no_overwrite COMMAND QIF_Reader "${QIF_DATA}/box_10x20x30.qif" "${QIF_OUT}/exists/existing.prc")
set_tests_properties(QIF_Reader.no_overwrite PROPERTIES PASS_REGULAR_EXPRESSION "output exists")
add_test(NAME QIF_Reader.unknown_option COMMAND QIF_Reader "${QIF_DATA}/box_10x20x30.qif" "${QIF_OUT}/u.prc" --frobnicate)
set_tests_properties(QIF_Reader.unknown_option PROPERTIES PASS_REGULAR_EXPRESSION "unknown option")

# 4. Non-ASCII input and output paths (wmain + UTF-8 inside).
set(QIF_UNICODE_DIR "${QIF_OUT}/unicodé_ü")
file(MAKE_DIRECTORY "${QIF_UNICODE_DIR}")
file(COPY_FILE "${QIF_DATA}/box_10x20x30.qif" "${QIF_UNICODE_DIR}/boîte_10x20x30.qif")
add_test(NAME QIF_Reader.unicode_paths COMMAND QIF_Reader "${QIF_UNICODE_DIR}/boîte_10x20x30.qif" "${QIF_UNICODE_DIR}/boîte.prc" --force)
set_tests_properties(QIF_Reader.unicode_paths PROPERTIES PASS_REGULAR_EXPRESSION "faces 6, area 2200 mm2, volume 6000 mm3")

# Success-path tests must not print an error, whatever their regular expression matched.
set_tests_properties(QIF_Reader.box QIF_Reader.unicode_paths PROPERTIES FAIL_REGULAR_EXPRESSION "ERROR|VERIFY FAILED")
foreach(m nist_ctc_01_asme1_ap242 nist_ctc_03_asme1_ap242 nist_ctc_05_asme1_ap242
          nist_ftc_06_asme1_ap242 nist_ftc_08_asme1_ap242-1 nist_ftc_09_asme1_ap242)
    set_tests_properties(QIF_Reader.${m} PROPERTIES FAIL_REGULAR_EXPRESSION "ERROR|VERIFY FAILED")
endforeach()

# 5. PMI and views: every QIF PMIDisplay becomes a markup; annotation planes + saved views become PRC views.
#    Expected counts come from the QIF files (PMIDisplay elements, annotation planes in use + SavedView elements).
#    markups:linked:views. "linked" = PMIDisplays whose QIF entity references geometry (general notes do not).
#    5th field: faces that carry QIF feature attributes (faces referenced by a FeatureNominal).
set(PMI_EXPECT "nist_ctc_01_asme1_ap242:23:23:4:30" "nist_ctc_03_asme1_ap242:30:30:3:32" "nist_ctc_05_asme1_ap242:22:20:7:12"
               "nist_ftc_06_asme1_ap242:64:62:9:123" "nist_ftc_08_asme1_ap242-1:50:50:17:103" "nist_ftc_09_asme1_ap242:64:63:7:116")
foreach(e ${PMI_EXPECT})
    string(REPLACE ":" ";" parts "${e}")
    list(GET parts 0 m)
    list(GET parts 1 nmk)
    list(GET parts 2 nln)
    list(GET parts 3 nvw)
    list(GET parts 4 nff)
    add_test(NAME QIF_Reader.pmi_${m} COMMAND QIF_Reader "${QIF_DATA}/${m}.qif" "${QIF_OUT}/${m}_pmi.prc" --force --quiet)
    # re-imported PRC: all markups present, every geometry-related one linked, all views present
    set_tests_properties(QIF_Reader.pmi_${m} PROPERTIES
        PASS_REGULAR_EXPRESSION "${nff} faces with attributes, markups ${nmk} \\(${nln} linked, ${nmk} semantic\\), views ${nvw}"
        FAIL_REGULAR_EXPRESSION "ERROR|VERIFY FAILED|markups have no text| [1-9][0-9]* unresolved links|WARNING: semantic")
endforeach()
add_test(NAME QIF_Reader.no_pmi COMMAND QIF_Reader "${QIF_DATA}/nist_ctc_01_asme1_ap242.qif" "${QIF_OUT}/ctc01_nopmi.prc" --force --no-pmi)
set_tests_properties(QIF_Reader.no_pmi PROPERTIES FAIL_REGULAR_EXPRESSION "markups|ERROR")

# 6. Graphical-only PMI: same markups, none semantic.
add_test(NAME QIF_Reader.no_semantic COMMAND QIF_Reader "${QIF_DATA}/nist_ftc_06_asme1_ap242.qif" "${QIF_OUT}/ftc06_nosem.prc" --force --quiet --no-semantic)
set_tests_properties(QIF_Reader.no_semantic PROPERTIES PASS_REGULAR_EXPRESSION "attributes, markups 64 \\(62 linked, 0 semantic\\)" FAIL_REGULAR_EXPRESSION "ERROR")

# 7. Semantic PMI content: GD&T type, tolerance, zone, material modifier, datum reference frame; dimension type, value,
#    tolerances, symbol; datum labels - read back from the written PRC and compared with an expectation computed
#    independently from the QIF XML by gen_expected_pmi.py (testdata/expected_pmi/*.txt).
foreach(m nist_ctc_01_asme1_ap242 nist_ctc_03_asme1_ap242 nist_ctc_05_asme1_ap242
          nist_ftc_06_asme1_ap242 nist_ftc_08_asme1_ap242-1 nist_ftc_09_asme1_ap242)
    add_test(NAME QIF_Reader.semantic_${m}
             COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:QIF_Reader> -DIN=${QIF_DATA}/${m}.qif
                     -DOUT=${QIF_OUT}/${m}_semantic.prc -DEXPECTED=${QIF_DATA}/expected_pmi/${m}.txt
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/compare_pmi.cmake)
endforeach()
