# USD_Writer - CAD/PRC to USD writer (HOOPS Exchange + OpenUSD).
# Built from the top-level CMakeLists.txt of this repository (see ../README.md).

# OpenUSD wiring shared with USD_Reader (OpenUSD folder, imported pxr targets, runtime copy): ../usd_common/OpenUSD.cmake
include("${CMAKE_CURRENT_SOURCE_DIR}/../usd_common/OpenUSD.cmake")

add_executable(USD_Writer)
target_include_directories(USD_Writer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(USD_Writer PRIVATE HOOPSExchangePlugin HOOPSExchangeSampleAppTools)
hoops_usd_setup(USD_Writer usd usdGeom usdShade usdUtils pcp sdf tf vt gf ar)
set_target_properties(USD_Writer PROPERTIES FOLDER "plugins")

set(_srcs
    USD_Writer.cpp)

target_sources(USD_Writer PRIVATE ${_srcs})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${_srcs})

# Tests: CTest cases live in tests.cmake (names start with "USD_Writer.").
# They carry the same labels as the samples' tests (see samples/tests/README.md in HOOPS Exchange).
set_property(DIRECTORY PROPERTY LABELS "sample;exchange")
include("${CMAKE_CURRENT_SOURCE_DIR}/tests.cmake")
