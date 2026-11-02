# QIF_Writer - CAD / PRC to QIF 3.0 writer (HOOPS Exchange public API only).
# Built from the top-level CMakeLists.txt of this repository (see ../README.md).
add_executable(QIF_Writer)
target_include_directories(QIF_Writer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(QIF_Writer PRIVATE HOOPSExchangePlugin HOOPSExchangeSampleAppTools)
set_target_properties(QIF_Writer PROPERTIES FOLDER "plugins")
if(MSVC)
    target_compile_definitions(QIF_Writer PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    target_compile_options(QIF_Writer PRIVATE /bigobj)
endif()

set(_srcs
    QIF_Writer.cpp
    qif_out.hpp
    qif_pmi.hpp
    qif_document.hpp)

target_sources(QIF_Writer PRIVATE ${_srcs})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${_srcs})

# Tests: CTest cases live in tests.cmake (names start with "QIF_Writer.").
# They carry the same labels as the samples' tests (see samples/tests/README.md in HOOPS Exchange).
set_property(DIRECTORY PROPERTY LABELS "sample;exchange")
include("${CMAKE_CURRENT_SOURCE_DIR}/tests.cmake")
