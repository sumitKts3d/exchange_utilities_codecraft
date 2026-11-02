# OpenUSD wiring shared by USD_Reader (USD -> PRC) and USD_Writer (CAD/PRC -> USD).
#
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../usd_common/OpenUSD.cmake)
#   hoops_usd_setup(<target> [<pxr library> ...])     # e.g. usd usdGeom usdShade usdUtils
#
# OpenUSD folder: <HOOPS Exchange>/bin/usd if it exists, else the folder in USD_ROOT (see OpenUSDRoot.cmake). It is an
# OpenUSD binary package with include/pxr, lib/*.lib and cmake/pxrTargets.cmake (tested: NVIDIA
# usd.py312.windows-x86_64.usdview.release-v25.08). Release/RelWithDebInfo binaries only: build the apps in Release.
# The top-level CMakeLists.txt resolves it once (HOOPS_USD_ROOT); USD_ROOT is set below to the resolved folder for this
# directory, and tests.cmake files use it to find usdcat / usdchecker.
include("${CMAKE_CURRENT_LIST_DIR}/OpenUSDRoot.cmake")
if(NOT HOOPS_USD_ROOT)
    hoops_usd_resolve_root(HOOPS_USD_ROOT)
endif()
if(NOT HOOPS_USD_ROOT)
    message(FATAL_ERROR "OpenUSD not found: put it in '${A3DLIBS_ROOT}/bin/usd' or set USD_ROOT (need include/pxr/pxr.h and cmake/pxrTargets.cmake)")
endif()
set(USD_ROOT "${HOOPS_USD_ROOT}")
set(HOOPS_USD_COMMON_DIR "${CMAKE_CURRENT_LIST_DIR}")

# Imported targets are directory-scoped: each app directory that includes this file gets its own set.
# pxrTargets.cmake refers to TBB::tbb and Python3::Python; the package ships both, so describe them here.
if(NOT TARGET TBB::tbb)
    add_library(TBB::tbb INTERFACE IMPORTED)
    set_target_properties(TBB::tbb PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${USD_ROOT}/include"
                                               INTERFACE_LINK_LIBRARIES "${USD_ROOT}/lib/tbb.lib")
endif()
if(NOT TARGET Python3::Python)
    file(GLOB _usd_pylib "${USD_ROOT}/python/libs/python3[0-9]*.lib")
    add_library(Python3::Python INTERFACE IMPORTED)
    set_target_properties(Python3::Python PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${USD_ROOT}/python/include"
                                                     INTERFACE_LINK_LIBRARIES "${_usd_pylib}"
                                                     INTERFACE_LINK_DIRECTORIES "${USD_ROOT}/python/libs")
endif()
if(NOT TARGET usdGeom)
    # The package only has RelWithDebInfo binaries (release CRT): use them for Release and MinSizeRel too.
    set(CMAKE_MAP_IMPORTED_CONFIG_RELEASE RelWithDebInfo Release)
    set(CMAKE_MAP_IMPORTED_CONFIG_MINSIZEREL RelWithDebInfo Release)
    include("${USD_ROOT}/cmake/pxrTargets.cmake")
endif()

function(hoops_usd_setup target)
    set(_libs ${ARGN})
    if(NOT _libs)
        set(_libs usd usdGeom usdShade sdf tf vt gf)
    endif()
    target_link_libraries(${target} PRIVATE ${_libs})
    target_include_directories(${target} PRIVATE "${HOOPS_USD_COMMON_DIR}")
    if(MSVC)
        target_compile_definitions(${target} PRIVATE NOMINMAX _USE_MATH_DEFINES)
        target_compile_options(${target} PRIVATE /utf-8 /bigobj /wd4244 /wd4305 /wd4267 /wd4003)
    endif()
    # Runtime: OpenUSD DLLs, TBB, Python and the USD plug-in resources (usd/) next to the executable, so the program
    # runs without changing PATH. USD finds its plug-ins in <folder of usd_plug.dll>/usd.
    file(GLOB _usd_runtime_dlls "${USD_ROOT}/lib/*.dll" "${USD_ROOT}/bin/tbb*.dll" "${USD_ROOT}/python/python3*.dll")
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_usd_runtime_dlls} "$<TARGET_FILE_DIR:${target}>"
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${USD_ROOT}/lib/usd" "$<TARGET_FILE_DIR:${target}>/usd"
        COMMENT "Copying the OpenUSD runtime next to ${target}")
endfunction()

# OpenUSD command-line tools (usdcat, usdchecker) for tests: they need these folders on PATH, and usdchecker needs the
# package's Python modules (lib/python, pip-packages). Use: set_tests_properties(... ENVIRONMENT "${HOOPS_USD_TOOLS_ENV}")
string(REPLACE ";" "\;" HOOPS_USD_TOOLS_PATH "${USD_ROOT}/lib;${USD_ROOT}/bin;${USD_ROOT}/python;$ENV{PATH}")
set(HOOPS_USD_TOOLS_ENV "PATH=${HOOPS_USD_TOOLS_PATH}"
                        "PYTHONPATH=${USD_ROOT}/lib/python\;${USD_ROOT}/pip-packages"
                        "PYTHONHOME=${USD_ROOT}/python")
