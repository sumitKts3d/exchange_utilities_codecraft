# Locates the OpenUSD installation used by USD_Reader and USD_Writer.
#
#   hoops_usd_resolve_root(<out variable>)
#
# Search order:
#   1. <HOOPS Exchange>/bin/usd                (A3DLIBS_ROOT/bin/usd)
#   2. the folder in the USD_ROOT variable     (-DUSD_ROOT=... or environment variable USD_ROOT)
# A folder qualifies when it has include/pxr/pxr.h and cmake/pxrTargets.cmake.
# The result is empty when nothing qualifies.
function(hoops_usd_resolve_root out)
    set(_candidates "${A3DLIBS_ROOT}/bin/usd")
    if(USD_ROOT)
        list(APPEND _candidates "${USD_ROOT}")
    endif()
    if(DEFINED ENV{USD_ROOT} AND NOT "$ENV{USD_ROOT}" STREQUAL "")
        list(APPEND _candidates "$ENV{USD_ROOT}")
    endif()
    foreach(_c IN LISTS _candidates)
        file(TO_CMAKE_PATH "${_c}" _c)
        if(EXISTS "${_c}/include/pxr/pxr.h" AND EXISTS "${_c}/cmake/pxrTargets.cmake")
            set(${out} "${_c}" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    set(${out} "" PARENT_SCOPE)
endfunction()
