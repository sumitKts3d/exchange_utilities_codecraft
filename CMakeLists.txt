################################################################################
# Top CMakeLists.txt for the HOOPS Exchange plugins (CodeCraft utilities)
#
# Same structure as the top CMakeLists.txt of the HOOPS Exchange samples:
# find_package(Exchange), one interface target that carries the common options,
# the SampleTools helper libraries and one add_subdirectory per application.
#
# Expected location of this folder:   <HOOPS Exchange>/tools/plugins
#
# Variables:
# - A3DLIBS_ROOT: HOOPS Exchange installation folder (contains include/, bin/
#       and samples/). Default: the environment variable A3DLIBS_ROOT, else
#       two folders above this one (<HOOPS Exchange>/tools/plugins/../..).
# - USD_ROOT: OpenUSD installation, used only when <HOOPS Exchange>/bin/usd
#       does not exist (see usd_common/OpenUSDRoot.cmake).
# - HOOPS_PLUGINS_BUILD_USD: OFF skips USD_Reader and USD_Writer (no OpenUSD
#       needed).
#
# Defines:
# - HOOPSExchangePlugin: Interface library defining all common properties for
#       the plugins. Including linker and compiler options.
################################################################################
cmake_minimum_required(VERSION 3.21.0)

project(HOOPSExchangePlugins)

################################################################################
# Locate HOOPS Exchange
################################################################################
if(NOT A3DLIBS_ROOT)
    if(DEFINED ENV{A3DLIBS_ROOT} AND NOT "$ENV{A3DLIBS_ROOT}" STREQUAL "")
        set(A3DLIBS_ROOT "$ENV{A3DLIBS_ROOT}")
    else()
        get_filename_component(A3DLIBS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
    endif()
endif()
file(TO_CMAKE_PATH "${A3DLIBS_ROOT}" A3DLIBS_ROOT)

if(NOT EXISTS "${A3DLIBS_ROOT}/include/A3DSDK.h"
   OR NOT EXISTS "${A3DLIBS_ROOT}/samples/cmake/FindExchange.cmake")
    message(FATAL_ERROR
        "HOOPS Exchange not found in A3DLIBS_ROOT='${A3DLIBS_ROOT}' "
        "(need include/A3DSDK.h and samples/cmake/FindExchange.cmake).\n"
        "Clone this repository to <HOOPS Exchange>/tools/plugins, or configure with -DA3DLIBS_ROOT=<HOOPS Exchange folder>.")
endif()
message(STATUS "A3DLIBS_ROOT: ${A3DLIBS_ROOT}")

# FindExchange.cmake and its helpers are the ones shipped with the samples.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${A3DLIBS_ROOT}/samples/cmake")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${A3DLIBS_ROOT}/samples/cmake")
find_package(Exchange REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_library(HOOPSExchangePlugin INTERFACE)

################################################################################
# The following options MUST be used when making project using HOOPS Exchange
# (same as the samples)
################################################################################

# Link to A3DLIBS interface, that defines the correct include headers folder.
target_link_libraries(HOOPSExchangePlugin INTERFACE A3DLIBS::Interface)

# Set Visual Studio compilers to Unicode for string encoding.
if(MSVC)
    target_compile_definitions(HOOPSExchangePlugin INTERFACE UNICODE _UNICODE)
endif()

# HOOPS Exchange loader (A3DSDKLoader.h) needs access to dynamic library load
# functions, such as LoadLibrary on windows or dlopen() on UNIX platforms.
target_link_libraries(HOOPSExchangePlugin INTERFACE ${CMAKE_DL_LIBS})

if(UNIX AND NOT APPLE)
    target_link_libraries(HOOPSExchangePlugin INTERFACE stdc++ pthread)
    target_link_options(HOOPSExchangePlugin INTERFACE "LINKER:--allow-shlib-undefined")
endif()

################################################################################
# Options that are specific to the plugins
################################################################################

# The plugins use std::filesystem and structured code written for C++17
# (the samples use C++14).
target_compile_features(HOOPSExchangePlugin INTERFACE cxx_std_17)

if(MSVC)
    # Same warning level as the samples; UTF-8 sources (accents in comments and tests).
    target_compile_options(HOOPSExchangePlugin INTERFACE /W4 /utf-8)
else()
    target_compile_options(HOOPSExchangePlugin INTERFACE -Wno-write-strings)
endif()

# Where the loader finds HOOPS Exchange, and the samples data used by the tests.
set(SAMPLES_DATA_DIRECTORY "${A3DLIBS_ROOT}/samples/data")
file(TO_NATIVE_PATH "${SAMPLES_DATA_DIRECTORY}" SAMPLES_DATA_DIRECTORY)
target_compile_definitions(HOOPSExchangePlugin
    INTERFACE
        "HOOPS_BINARY_DIRECTORY=R\"(${Exchange_HOOPS_BINARY_DIRECTORY})\""
        "SAMPLES_DATA_DIRECTORY=R\"(${SAMPLES_DATA_DIRECTORY})\"")

# A general-purpose dependency for sample projects, shipped with HOOPS Exchange.
# See <HOOPS Exchange>/samples/SampleTools/README.md for more information.
add_subdirectory("${A3DLIBS_ROOT}/samples/SampleTools" "${CMAKE_BINARY_DIR}/SampleTools")

################################################################################
# OpenUSD (only for USD_Reader and USD_Writer)
################################################################################
option(HOOPS_PLUGINS_BUILD_USD "Build the OpenUSD based plugins (USD_Reader, USD_Writer)" ON)
set(USD_ROOT "" CACHE PATH "OpenUSD install folder, used when <HOOPS Exchange>/bin/usd does not exist")

if(HOOPS_PLUGINS_BUILD_USD)
    include("${CMAKE_CURRENT_LIST_DIR}/usd_common/OpenUSDRoot.cmake")
    hoops_usd_resolve_root(HOOPS_USD_ROOT)
    if(NOT HOOPS_USD_ROOT)
        message(FATAL_ERROR
            "OpenUSD not found. Looked in '${A3DLIBS_ROOT}/bin/usd', then in USD_ROOT='${USD_ROOT}'.\n"
            "Put an OpenUSD binary package (it has include/pxr/pxr.h and cmake/pxrTargets.cmake) in "
            "<HOOPS Exchange>/bin/usd, or set -DUSD_ROOT=<OpenUSD folder>, "
            "or configure with -DHOOPS_PLUGINS_BUILD_USD=OFF to build the QIF plugins only.")
    endif()
    message(STATUS "OpenUSD: ${HOOPS_USD_ROOT}")
endif()

################################################################################
# This block defines the list of subfolders containing the plugins
################################################################################
enable_testing()
include(CTest)

# QIF_Reader comes before QIF_Writer: the writer's tests read its files back with the reader.
add_subdirectory(QIF_Reader)
add_subdirectory(QIF_Writer)

if(HOOPS_PLUGINS_BUILD_USD)
    # USD_Reader comes before USD_Writer: the writer's round-trip tests use the reader.
    add_subdirectory(USD_Reader)
    add_subdirectory(USD_Writer)
endif()
