# QIF_Reader - QIF 3.0 to PRC converter (HOOPS Exchange public API).
# Built from the top-level CMakeLists.txt of this repository (see ../README.md).
add_executable(QIF_Reader)
target_include_directories(QIF_Reader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(QIF_Reader PRIVATE HOOPSExchangePlugin HOOPSExchangeSampleAppTools)
set_target_properties(QIF_Reader PROPERTIES FOLDER "plugins")

set(_srcs
    QIF_Reader.cpp
    prc_appearance.hpp
    prc_pmi.hpp
    prc_semantic.hpp
    qif_model.hpp
    qif_xml.hpp)

target_sources(QIF_Reader PRIVATE ${_srcs})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${_srcs})

# Tests: CTest cases live in tests.cmake (names start with "QIF_Reader.").
# They carry the same labels as the samples' tests (see samples/tests/README.md in HOOPS Exchange).
set_property(DIRECTORY PROPERTY LABELS "sample;exchange")
include("${CMAKE_CURRENT_SOURCE_DIR}/tests.cmake")
