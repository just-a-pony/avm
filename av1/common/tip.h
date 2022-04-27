/*
 * Copyright (c) 2021, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 3-Clause Clear License
 * and the Alliance for Open Media Patent License 1.0. If the BSD 3-Clause
 * Clear License was not distributed with this source code in the LICENSE file,
 * you can obtain it at aomedia.org/license/software-license/bsd-3-c-c/.
 * If the Alliance for Open Media Patent License 1.0 was not distributed with
 * this source code in the PATENTS file, you can obtain it at
 * aomedia.org/license/patent-license/.
 */

#ifndef AOM_AV1_COMMON_TIP_H_
#define AOM_AV1_COMMON_TIP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "av1/common/av1_common_int.h"
#include "av1/common/mv.h"
#include "av1/common/mvref_common.h"
#include "av1/common/reconinter.h"

// Derive temporal motion field from one closest forward and one closet backward
// reference frames, then fill the hole
void av1_setup_tip_motion_field(AV1_COMMON *cm, int check_tip_threshold);

// Generate the whole TIP frame with the temporal motion field
void av1_setup_tip_frame(AV1_COMMON *cm, MACROBLOCKD *xd, uint8_t **mc_buf,
                         CONV_BUF_TYPE *tmp_conv_dst,
                         CalcSubpelParamsFunc calc_subpel_params_func);

// Generate TIP reference block if current block is coded as TIP mode
void av1_setup_tip_on_the_fly(AV1_COMMON *cm, MACROBLOCKD *xd,
                              int blk_row_start, int blk_col_start,
                              int blk_row_end, int blk_col_end, int mvs_stride,
                              uint8_t **mc_buf, CONV_BUF_TYPE *tmp_conv_dst,
                              CalcSubpelParamsFunc calc_subpel_params_func);

// Derive TMVP from closest forward and closet backward reference frames
void av1_derive_tip_nearest_ref_frames_motion_projection(AV1_COMMON *cm);

// Save TMVP motion fields for future frame's TMVP if current frame is coded
// as TIP_FRAME_AS_OUTPUT
void av1_copy_tip_frame_tmvp_mvs(const AV1_COMMON *const cm);

// Compute scale factor for temporal scaling
static AOM_INLINE int tip_derive_scale_factor(int num, int den) {
  den = AOMMIN(den, MAX_FRAME_DISTANCE);
  num = num > 0 ? AOMMIN(num, MAX_FRAME_DISTANCE)
                : AOMMAX(num, -MAX_FRAME_DISTANCE);
  return num * div_mult[den];
}

// Temporal scaling the motion vector
static AOM_INLINE void tip_get_mv_projection(MV *output, MV ref,
                                             int scale_factor) {
  const int mv_row = ROUND_POWER_OF_TWO_SIGNED(ref.row * scale_factor, 14);
  const int mv_col = ROUND_POWER_OF_TWO_SIGNED(ref.col * scale_factor, 14);
  const int clamp_max = MV_UPP - 1;
  const int clamp_min = MV_LOW + 1;
  output->row = (int16_t)clamp(mv_row, clamp_min, clamp_max);
  output->col = (int16_t)clamp(mv_col, clamp_min, clamp_max);
}

// Convert motion vector to fullpel and clamp the fullpel mv if it is outside
// of frame boundary
static AOM_INLINE FULLPEL_MV clamp_tip_fullmv(const AV1_COMMON *const cm,
                                              const MV *mv, const int blk_row,
                                              const int blk_col) {
  const int y_width = cm->tip_ref.ref_frame_buffer[0]->buf.y_width;
  const int y_height = cm->tip_ref.ref_frame_buffer[0]->buf.y_height;

  FULLPEL_MV fullmv;
  fullmv = get_fullmv_from_mv(mv);
  if (fullmv.row + ((blk_row + 1) << TMVP_MI_SZ_LOG2) > y_height) {
    fullmv.row = y_height - ((blk_row + 1) << TMVP_MI_SZ_LOG2);
  } else if (fullmv.row + (blk_row << TMVP_MI_SZ_LOG2) < 0) {
    fullmv.row = -(blk_row << TMVP_MI_SZ_LOG2);
  }

  if (fullmv.col + ((blk_col + 1) << TMVP_MI_SZ_LOG2) > y_width) {
    fullmv.col = y_width - ((blk_col + 1) << TMVP_MI_SZ_LOG2);
  } else if (fullmv.col + (blk_col << TMVP_MI_SZ_LOG2) < 0) {
    fullmv.col = -(blk_col << TMVP_MI_SZ_LOG2);
  }

  return fullmv;
}

// SMVP candidate which is derived from TIP mode
static AOM_INLINE void clamp_tip_smvp_refmv(const AV1_COMMON *const cm, MV *mv,
                                            const int blk_row,
                                            const int blk_col) {
  const int y_width = cm->tip_ref.ref_frame_buffer[0]->buf.y_width;
  const int y_height = cm->tip_ref.ref_frame_buffer[0]->buf.y_height;

  FULLPEL_MV fullmv;
  fullmv = get_fullmv_from_mv(mv);
  const int row_offset = fullmv.row + (blk_row << MI_SIZE_LOG2);
  if (row_offset > y_height) {
    fullmv.row = y_height - (blk_row << MI_SIZE_LOG2);
  } else if (row_offset < 0) {
    fullmv.row = -(blk_row << MI_SIZE_LOG2);
  }

  const int col_offset = fullmv.col + (blk_col << MI_SIZE_LOG2);
  if (col_offset > y_width) {
    fullmv.col = y_width - (blk_col << MI_SIZE_LOG2);
  } else if (col_offset < 0) {
    fullmv.col = -(blk_col << MI_SIZE_LOG2);
  }

  *mv = get_mv_from_fullmv(&fullmv);
}

// Clamp MV to UMV border based on its distance to left/right/top/bottom edge
static AOM_INLINE MV tip_clamp_mv_to_umv_border_sb(
    InterPredParams *const inter_pred_params, const MV *src_mv, int bw, int bh,
    int ss_x, int ss_y) {
  // If the MV points so far into the UMV border that no visible pixels
  // are used for reconstruction, the subpel part of the MV can be
  // discarded and the MV limited to 16 pixels with equivalent results.
  const int spel_left = (AOM_INTERP_EXTEND + bw) << SUBPEL_BITS;
  const int spel_right = spel_left - SUBPEL_SHIFTS;
  const int spel_top = (AOM_INTERP_EXTEND + bh) << SUBPEL_BITS;
  const int spel_bottom = spel_top - SUBPEL_SHIFTS;
  MV clamped_mv = { (int16_t)(src_mv->row * (1 << (1 - ss_y))),
                    (int16_t)(src_mv->col * (1 << (1 - ss_x))) };
  assert(ss_x <= 1);
  assert(ss_y <= 1);
  const SubpelMvLimits mv_limits = {
    inter_pred_params->dist_to_left_edge * (1 << (1 - ss_x)) - spel_left,
    inter_pred_params->dist_to_right_edge * (1 << (1 - ss_x)) + spel_right,
    inter_pred_params->dist_to_top_edge * (1 << (1 - ss_y)) - spel_top,
    inter_pred_params->dist_to_bottom_edge * (1 << (1 - ss_y)) + spel_bottom
  };

  clamp_mv(&clamped_mv, &mv_limits);

  return clamped_mv;
}

// Set TIP mv fullpel range constraint
static INLINE void av1_set_tip_mv_range(
    FullMvLimits *mv_limits, const MACROBLOCKD *const xd, BLOCK_SIZE bsize,
    const CommonModeInfoParams *const mi_params) {
  const int mi_row = xd->mi_row;
  const int mi_col = xd->mi_col;
  const int mi_width = mi_size_wide[bsize];
  const int mi_height = mi_size_high[bsize];
  const int mi_rows = mi_params->mi_rows;
  const int mi_cols = mi_params->mi_cols;
  const int tmvp_mv = (TIP_MV_SEARCH_RANGE << TMVP_MI_SZ_LOG2);

  int col_min = -tmvp_mv;
  int row_min = -tmvp_mv;
  int col_max = tmvp_mv;
  int row_max = tmvp_mv;

  mv_limits->col_min = AOMMAX(col_min, -(mi_col * MI_SIZE));
  mv_limits->col_max = AOMMIN(col_max, (mi_cols - mi_col - mi_width) * MI_SIZE);
  mv_limits->row_min = AOMMAX(row_min, -(mi_row * MI_SIZE));
  mv_limits->row_max =
      AOMMIN(row_max, (mi_rows - mi_row - mi_height) * MI_SIZE);
}

// Set TIP mv subpel range constraint
static INLINE void av1_set_tip_subpel_mv_range(
    SubpelMvLimits *subpel_limits, const MACROBLOCKD *const xd,
    BLOCK_SIZE bsize, const CommonModeInfoParams *const mi_params) {
  const int mi_row = xd->mi_row;
  const int mi_col = xd->mi_col;
  const int mi_width = mi_size_wide[bsize];
  const int mi_height = mi_size_high[bsize];
  const int mi_rows = mi_params->mi_rows;
  const int mi_cols = mi_params->mi_cols;
  const int tmvp_mv = GET_MV_SUBPEL(TIP_MV_SEARCH_RANGE << TMVP_MI_SZ_LOG2);

  const int col_min = -tmvp_mv;
  const int row_min = -tmvp_mv;
  const int col_max = tmvp_mv;
  const int row_max = tmvp_mv;

  subpel_limits->col_min = AOMMAX(col_min, GET_MV_SUBPEL(-(mi_col * MI_SIZE)));
  subpel_limits->col_max =
      AOMMIN(col_max, GET_MV_SUBPEL((mi_cols - mi_col - mi_width) * MI_SIZE));
  subpel_limits->row_min = AOMMAX(row_min, GET_MV_SUBPEL(-(mi_row * MI_SIZE)));
  subpel_limits->row_max =
      AOMMIN(row_max, GET_MV_SUBPEL((mi_rows - mi_row - mi_height) * MI_SIZE));
}

// Clamp refmv for TIP nearmv mode
static INLINE int tip_clamp_and_check_mv(int_mv *out_mv, int_mv in_mv,
                                         BLOCK_SIZE bsize, const AV1_COMMON *cm,
                                         const MACROBLOCKD *const xd) {
  *out_mv = in_mv;
  lower_mv_precision(&out_mv->as_mv, cm->features.allow_high_precision_mv,
                     cm->features.cur_frame_force_integer_mv);
  SubpelMvLimits subpel_mv_limits;
  av1_set_tip_subpel_mv_range(&subpel_mv_limits, xd, bsize, &cm->mi_params);
  clamp_mv(&out_mv->as_mv, &subpel_mv_limits);
  return av1_is_subpelmv_in_range(&subpel_mv_limits, out_mv->as_mv);
}

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AOM_AV1_COMMON_TIP_H_
