#!/usr/bin/env bash
# Source this before running astra_camera when the Orbbec dependencies were
# unpacked into this workspace instead of installed with apt.

_orbbec_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CMAKE_PREFIX_PATH="${_orbbec_ws}/local_debs/opt/ros/humble:${_orbbec_ws}/local_debs/usr:${CMAKE_PREFIX_PATH}"
export AMENT_PREFIX_PATH="${_orbbec_ws}/local_debs/opt/ros/humble:${AMENT_PREFIX_PATH}"
export PKG_CONFIG_PATH="${_orbbec_ws}/local_debs/usr/lib/x86_64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH}"
export PKG_CONFIG_SYSROOT_DIR="${_orbbec_ws}/local_debs"
export CPLUS_INCLUDE_PATH="${_orbbec_ws}/local_debs/usr/include:${CPLUS_INCLUDE_PATH}"
export C_INCLUDE_PATH="${_orbbec_ws}/local_debs/usr/include:${C_INCLUDE_PATH}"
export LIBRARY_PATH="${_orbbec_ws}/local_debs/usr/lib/x86_64-linux-gnu:${LIBRARY_PATH}"
export LD_LIBRARY_PATH="${_orbbec_ws}/local_debs/opt/ros/humble/lib:${_orbbec_ws}/local_debs/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"
unset _orbbec_ws
