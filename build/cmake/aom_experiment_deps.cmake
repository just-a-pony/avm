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
if(AOM_BUILD_CMAKE_AOM_EXPERIMENT_DEPS_CMAKE_)
  return()
endif() # AOM_BUILD_CMAKE_AOM_EXPERIMENT_DEPS_CMAKE_
set(AOM_BUILD_CMAKE_AOM_EXPERIMENT_DEPS_CMAKE_ 1)

# Adjusts CONFIG_* CMake variables to address conflicts between active AV1
# experiments.
macro(fix_experiment_configs)

  if(CONFIG_ANALYZER)
    change_config_and_warn(CONFIG_INSPECTION 1 CONFIG_ANALYZER)
  endif()

  if(CONFIG_DIST_8X8 AND CONFIG_MULTITHREAD)
    change_config_and_warn(CONFIG_DIST_8X8 0 CONFIG_MULTITHREAD)
  endif()

  # CONFIG_OPTFLOW_REFINEMENT requires CONFIG_NEW_INTER_MODES. If
  # CONFIG_NEW_INTER_MODES is off, we also turn off CONFIG_OPTFLOW_REFINEMENT.
  if(NOT CONFIG_NEW_INTER_MODES AND CONFIG_OPTFLOW_REFINEMENT)
    change_config_and_warn(CONFIG_OPTFLOW_REFINEMENT 0 !CONFIG_NEW_INTER_MODES)
  endif()

  # CONFIG_IST_FIX_B076 requires CONFIG_IST. If CONFIG_IST is off, we also turn
  # off CONFIG_IST_FIX_B076.
  if(NOT CONFIG_IST AND CONFIG_IST_FIX_B076)
    change_config_and_warn(CONFIG_IST_FIX_B076 0 !CONFIG_IST)
  endif()
  # CONFIG_ADAPTIVE_MVD requires CONFIG_NEW_INTER_MODES. If
  # CONFIG_NEW_INTER_MODES is off, we also turn off CONFIG_ADAPTIVE_MVD.
  if(NOT CONFIG_NEW_INTER_MODES AND CONFIG_ADAPTIVE_MVD)
    change_config_and_warn(CONFIG_ADAPTIVE_MVD 0 !CONFIG_NEW_INTER_MODES)
  endif()

  # CONFIG_JOINT_MVD requires CONFIG_NEW_INTER_MODES. If CONFIG_NEW_INTER_MODES
  # is off, we also turn off CONFIG_JOINT_MVD.
  if(NOT CONFIG_NEW_INTER_MODES AND CONFIG_JOINT_MVD)
    change_config_and_warn(CONFIG_JOINT_MVD 0 !CONFIG_NEW_INTER_MODES)
  endif()

  # CONFIG_THROUGHPUT_ANALYSIS requires CONFIG_ACCOUNTING. If CONFIG_ACCOUNTING
  # is off, we also turn off CONFIG_THROUGHPUT_ANALYSIS.
  if(NOT CONFIG_ACCOUNTING AND CONFIG_THROUGHPUT_ANALYSIS)
    change_config_and_warn(CONFIG_THROUGHPUT_ANALYSIS 0 !CONFIG_ACCOUNTING)
  endif()

  # CONFIG_IST_FIX_B098 requires CONFIG_IST. If CONFIG_IST is off, we also turn
  # off CONFIG_IST_FIX_B098.
  if(NOT CONFIG_IST AND CONFIG_IST_FIX_B098)
    change_config_and_warn(CONFIG_IST_FIX_B098 0 !CONFIG_IST)
  endif()

  # CONFIG_CCSO_EXT is dependent on CONFIG_CCSO. If CONFIG_CCSO is off,
  # CONFIG_CCSO_EXT needs to be turned off.
  if(NOT CONFIG_CCSO_EXT AND CONFIG_CCSO)
    change_config_and_warn(CONFIG_CCSO_EXT 0 !CONFIG_CCSO)
  endif()

endmacro()
