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

  if(CONFIG_EXTRACT_PROTO)
    change_config_and_warn(CONFIG_ACCOUNTING 1 CONFIG_EXTRACT_PROTO)
    change_config_and_warn(CONFIG_INSPECTION 1 CONFIG_EXTRACT_PROTO)
  endif()

  if(CONFIG_DIST_8X8 AND CONFIG_MULTITHREAD)
    change_config_and_warn(CONFIG_DIST_8X8 0 CONFIG_MULTITHREAD)
  endif()

  if(CONFIG_ERP_TFLITE)
    change_config_and_warn(CONFIG_TENSORFLOW_LITE 1 CONFIG_EXT_RECUR_PARTITIONS)
  endif()

  # CONFIG_THROUGHPUT_ANALYSIS requires CONFIG_ACCOUNTING. If CONFIG_ACCOUNTING
  # is off, we also turn off CONFIG_THROUGHPUT_ANALYSIS.
  if(NOT CONFIG_ACCOUNTING AND CONFIG_THROUGHPUT_ANALYSIS)
    change_config_and_warn(CONFIG_THROUGHPUT_ANALYSIS 0 !CONFIG_ACCOUNTING)
  endif()

  # CONFIG_CCSO_EXT is dependent on CONFIG_CCSO. If CONFIG_CCSO is off,
  # CONFIG_CCSO_EXT needs to be turned off.
  if(NOT CONFIG_CCSO AND CONFIG_CCSO_EXT)
    change_config_and_warn(CONFIG_CCSO_EXT 0 !CONFIG_CCSO)
  endif()

  # CONFIG_OPTFLOW_ON_TIP is dependent on CONFIG_OPTFLOW_REFINEMENT. If
  # CONFIG_OPTFLOW_REFINEMENT is off, CONFIG_OPTFLOW_ON_TIP needs to be turned
  # off.
  if(NOT CONFIG_OPTFLOW_REFINEMENT AND CONFIG_OPTFLOW_ON_TIP)
    change_config_and_warn(CONFIG_OPTFLOW_ON_TIP 0 !CONFIG_OPTFLOW_REFINEMENT)
  endif()

  # CONFIG_EXPLICIT_BAWP is dependent on CONFIG_BAWP. If CONFIG_BAWP is off,
  # CONFIG_EXPLICIT_BAWP needs to be turned off.
  if(NOT CONFIG_BAWP AND CONFIG_EXPLICIT_BAWP)
    change_config_and_warn(CONFIG_EXPLICIT_BAWP 0 !CONFIG_BAWP)
  endif()

  # Tools which depend on CONFIG_EXTENDED_WARP_PREDICTION
  if(NOT CONFIG_EXTENDED_WARP_PREDICTION)
    change_config_and_warn(CONFIG_EXT_WARP_FILTER 0
                           !CONFIG_EXTENDED_WARP_PREDICTION)
    change_config_and_warn(CONFIG_INTERINTRA_IMPROVEMENT 0
                           !CONFIG_EXTENDED_WARP_PREDICTION)
    change_config_and_warn(CONFIG_COMPOUND_WARP_CAUSAL 0
                           !CONFIG_EXTENDED_WARP_PREDICTION)
    change_config_and_warn(CONFIG_AFFINE_REFINEMENT 0
                           !CONFIG_EXTENDED_WARP_PREDICTION)
  endif()

  # CONFIG_PARA_BD_REDUCE is dependent on CONFIG_ENTROPY_PARA.
  if(NOT CONFIG_ENTROPY_PARA AND CONFIG_PARA_BD_REDUCE)
    change_config_and_warn(CONFIG_PARA_BD_REDUCE 0 !CONFIG_ENTROPY_PARA)
  endif()

  # CONFIG_BLOCK_256 is dependent on CONFIG_EXT_RECUR_PARTITIONS.
  if(NOT CONFIG_EXT_RECUR_PARTITIONS AND CONFIG_BLOCK_256)
    change_config_and_warn(CONFIG_BLOCK_256 0 !CONFIG_EXT_RECUR_PARTITIONS)
  endif()

  # CONFIG_ENABLE_MHCCP is dependent on CONFIG_EXT_RECUR_PARTITIONS.
  if(NOT CONFIG_EXT_RECUR_PARTITIONS AND CONFIG_ENABLE_MHCCP)
    change_config_and_warn(CONFIG_ENABLE_MHCCP 0 !CONFIG_EXT_RECUR_PARTITIONS)
  endif()

  # CONFIG_IMPROVEIDTX_CTXS depends on CONFIG_IMPROVEIDTX_RDPH This is because
  # IDTX contexts are trained after scan and RDOQ changes advising retraining
  # contexts if CONFIG_IMPROVEIDTX_RDPH=0 and vice versa
  if(NOT CONFIG_IMPROVEIDTX_CTXS AND CONFIG_IMPROVEIDTX_RDPH)
    change_config_and_warn(CONFIG_IMPROVEIDTX_RDPH 0 !CONFIG_IMPROVEIDTX_CTXS)
  endif()

  # CONFIG_FLEX_PARTITION is dependent on CONFIG_EXT_RECUR_PARTITIONS.
  if(NOT CONFIG_EXT_RECUR_PARTITIONS AND CONFIG_FLEX_PARTITION)
    change_config_and_warn(CONFIG_FLEX_PARTITION 0 !CONFIG_EXT_RECUR_PARTITIONS)
  endif()

  # CONFIG_CB1TO4_SPLIT is dependent on CONFIG_FLEX_PARTITION.
  if(NOT CONFIG_FLEX_PARTITION AND CONFIG_CB1TO4_SPLIT)
    change_config_and_warn(CONFIG_CB1TO4_SPLIT 0 !CONFIG_FLEX_PARTITION)
  endif()

  # CONFIG_BAWP_CHROMA depends on CONFIG_BAWP
  if(NOT CONFIG_BAWP AND CONFIG_BAWP_CHROMA)
    change_config_and_warn(CONFIG_BAWP_CHROMA 0 !CONFIG_BAWP)
  endif()

  # CONFIG_IST_ANY_SET is dependent on CONFIG_IST_SET_FLAG. If
  # CONFIG_IST_SET_FLAG is off, CONFIG_IST_ANY_SET needs to be turned off.
  if(NOT CONFIG_IST_SET_FLAG AND CONFIG_IST_ANY_SET)
    change_config_and_warn(CONFIG_IST_ANY_SET 0 !CONFIG_IST_SET_FLAG)
  endif()

  # CONFIG_UV_CFL depends on CONFIG_AIMC
  if(NOT CONFIG_AIMC AND CONFIG_UV_CFL)
    change_config_and_warn(CONFIG_UV_CFL 0 !CONFIG_AIMC)
  endif()

  if(CONFIG_TEMP_LR)
    change_config_and_warn(CONFIG_COMBINE_PC_NS_WIENER 1 CONFIG_TEMP_LR)
  endif()

  # CONFIG_D072_SKIP_MODE_IMPROVE is dependent on CONFIG_SKIP_MODE_ENHANCEMENT
  # If CONFIG_SKIP_MODE_ENHANCEMENT is off, CONFIG_D072_SKIP_MODE_IMPROVE needs
  # to be turned off.
  if(NOT CONFIG_SKIP_MODE_ENHANCEMENT AND CONFIG_D072_SKIP_MODE_IMPROVE)
    change_config_and_warn(CONFIG_D072_SKIP_MODE_IMPROVE 0
                           !CONFIG_SKIP_MODE_ENHANCEMENT)
  endif()

  if(NOT CONFIG_ALLOW_SAME_REF_COMPOUND AND CONFIG_IMPROVED_SAME_REF_COMPOUND)
    change_config_and_warn(CONFIG_IMPROVED_SAME_REF_COMPOUND 0
                           !CONFIG_ALLOW_SAME_REF_COMPOUND)
  endif()

  # CONFIG_REFINED_MVS_IN_TMVP depends on CONFIG_OPTFLOW_REFINEMENT
  if(NOT CONFIG_OPTFLOW_REFINEMENT AND CONFIG_REFINED_MVS_IN_TMVP)
    change_config_and_warn(CONFIG_REFINED_MVS_IN_TMVP 0
                           !CONFIG_OPTFLOW_REFINEMENT)
  endif()

  # CONFIG_AFFINE_REFINEMENT depends on CONFIG_OPTFLOW_REFINEMENT
  if(NOT CONFIG_OPTFLOW_REFINEMENT AND CONFIG_AFFINE_REFINEMENT)
    change_config_and_warn(CONFIG_AFFINE_REFINEMENT 0
                           !CONFIG_OPTFLOW_REFINEMENT)
  endif()
  # CONFIG_AFFINE_REFINEMENT_SB depends on CONFIG_AFFINE_REFINEMENT
  if(NOT CONFIG_AFFINE_REFINEMENT AND CONFIG_AFFINE_REFINEMENT_SB)
    change_config_and_warn(CONFIG_AFFINE_REFINEMENT_SB 0
                           !CONFIG_AFFINE_REFINEMENT)
  endif()

  # CONFIG_MVP_SIMPLIFY depends on CONFIG_MVP_IMPROVEMENT
  if(NOT CONFIG_MVP_IMPROVEMENT AND CONFIG_MVP_SIMPLIFY)
    change_config_and_warn(CONFIG_MVP_SIMPLIFY 0 !CONFIG_MVP_IMPROVEMENT)
  endif()

  # CONFIG_TMVP_IMPROVE depends on CONFIG_MVP_IMPROVEMENT
  if(NOT CONFIG_MVP_IMPROVEMENT AND CONFIG_TMVP_IMPROVE)
    change_config_and_warn(CONFIG_TMVP_IMPROVE 0 !CONFIG_MVP_IMPROVEMENT)
  endif()

  # CONFIG_CCSO_SIGFIX depends on CONFIG_CCSO_BO_ONLY_OPTION
  if(NOT CONFIG_CCSO_BO_ONLY_OPTION AND CONFIG_CCSO_SIGFIX)
    change_config_and_warn(CONFIG_CCSO_SIGFIX 0 !CONFIG_CCSO_BO_ONLY_OPTION)
  endif()

  if(CONFIG_ML_PART_SPLIT)
    change_config_and_warn(CONFIG_TENSORFLOW_LITE 1 CONFIG_ML_PART_SPLIT)
  endif()

  # CONFIG_DRL_WRL_SIMPLIFY depends on CONFIG_MVP_IMPROVEMENT
  if(NOT CONFIG_MVP_IMPROVEMENT AND CONFIG_DRL_WRL_SIMPLIFY)
    change_config_and_warn(CONFIG_DRL_WRL_SIMPLIFY 0 !CONFIG_MVP_IMPROVEMENT)
  endif()

  # CONFIG_MV_TRAJECTORY depends on CONFIG_TMVP_MEM_OPT
  if(NOT CONFIG_TMVP_MEM_OPT AND CONFIG_MV_TRAJECTORY)
    change_config_and_warn(CONFIG_MV_TRAJECTORY 0 !CONFIG_TMVP_MEM_OPT)
  endif()
endmacro()
