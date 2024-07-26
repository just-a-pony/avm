#
# Copyright (c) 2021, Alliance for Open Media. All rights reserved
#
# This source code is subject to the terms of the BSD 3-Clause Clear License and
# the Alliance for Open Media Patent License 1.0. If the BSD 3-Clause Clear
# License was not distributed with this source code in the LICENSE file, you can
# obtain it at aomedia.org/license/software-license/bsd-3-c-c/.  If the Alliance
# for Open Media Patent License 1.0 was not distributed with this source code in
# the PATENTS file, you can obtain it at aomedia.org/license/patent-license/.
#
if(AOM_BUILD_CMAKE_TOOLCHAINS_PPC_LINUX_GCC_CMAKE_)
  return()
endif() # AOM_BUILD_CMAKE_TOOLCHAINS_PPC_LINUX_GCC_CMAKE_
set(AOM_BUILD_CMAKE_TOOLCHAINS_PPC_LINUX_GCC_CMAKE_ 1)

set(CMAKE_SYSTEM_NAME "Linux")

if("${CROSS}" STREQUAL "")

  # Default the cross compiler prefix to something known to work.
  set(CROSS powerpc64le-unknown-linux-gnu-)
endif()

set(CMAKE_C_COMPILER ${CROSS}gcc)
set(CMAKE_CXX_COMPILER ${CROSS}g++)
set(CMAKE_ASM_COMPILER ${CROSS}as)
set(CMAKE_SYSTEM_PROCESSOR "ppc")

set(CONFIG_RUNTIME_CPU_DETECT
    0
    CACHE STRING "")
