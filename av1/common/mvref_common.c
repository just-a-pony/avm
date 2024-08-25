/*
 * Copyright (c) 2021, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 3-Clause Clear License
 * and the Alliance for Open Media Patent License 1.0. If the BSD 3-Clause Clear
 * License was not distributed with this source code in the LICENSE file, you
 * can obtain it at aomedia.org/license/software-license/bsd-3-c-c/.  If the
 * Alliance for Open Media Patent License 1.0 was not distributed with this
 * source code in the PATENTS file, you can obtain it at
 * aomedia.org/license/patent-license/.
 */

#include <stdlib.h>

#include "av1/common/mv.h"
#include "av1/common/mvref_common.h"
#include "av1/common/tip.h"
#include "av1/common/warped_motion.h"

#if CONFIG_MVP_IMPROVEMENT
typedef struct single_mv_candidate {
  int_mv mv;
  MV_REFERENCE_FRAME ref_frame;
} SINGLE_MV_CANDIDATE;
#endif  // CONFIG_MVP_IMPROVEMENT

#if CONFIG_MVP_SIMPLIFY
#define TMVP_SEARCH_COUNT 5
#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
#define SMVP_COL_SEARCH_COUNT 2
#else
#define SMVP_COL_SEARCH_COUNT 3
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
typedef struct mvp_unit_status {
  int is_available;
  int row_offset;
  int col_offset;
} MVP_UNIT_STATUS;
#endif  // CONFIG_MVP_SIMPLIFY

#if CONFIG_TMVP_MEM_OPT
#define TIP_MFMV_STACK_SIZE 3  // The limit for original TMVP w/ TIP.
#define MFMV_STACK_SIZE 4      // The total limit of motion field candidates.
#else
#define MFMV_STACK_SIZE 3
#endif  // CONFIG_TMVP_MEM_OPT

#if CONFIG_TMVP_MEM_OPT
// Check and make sure that the MVs are stored to the correct slots.
static INLINE void check_frame_mv_slot(const AV1_COMMON *const cm, MV_REF *mv) {
  if (mv->ref_frame[0] != NONE_FRAME && mv->ref_frame[1] == NONE_FRAME) {
    mv->ref_frame[1] = mv->ref_frame[0];
    mv->mv[1] = mv->mv[0];
  } else if (mv->ref_frame[0] == NONE_FRAME && mv->ref_frame[1] != NONE_FRAME) {
    mv->ref_frame[0] = mv->ref_frame[1];
    mv->mv[0] = mv->mv[1];
  } else if (mv->ref_frame[0] != NONE_FRAME && mv->ref_frame[1] != NONE_FRAME) {
    int ref_display_order[2] = {
      get_ref_frame_buf(cm, mv->ref_frame[0])->display_order_hint,
      get_ref_frame_buf(cm, mv->ref_frame[1])->display_order_hint
    };
    int cur_display_order = cm->cur_frame->display_order_hint;

    bool to_switch = false;
    if (ref_display_order[0] < cur_display_order &&
        ref_display_order[1] < cur_display_order) {
      if (ref_display_order[0] < ref_display_order[1]) {
        to_switch = true;
      }
    } else if (ref_display_order[0] > cur_display_order &&
               ref_display_order[1] > cur_display_order) {
      if (ref_display_order[1] > ref_display_order[0]) {
        to_switch = true;
      }
    } else if (ref_display_order[0] > cur_display_order &&
               ref_display_order[1] < cur_display_order) {
      to_switch = true;
    }

    if (to_switch) {
      int tmp_ref = mv->ref_frame[0];
      int_mv tmp_mv = mv->mv[0];

      mv->ref_frame[0] = mv->ref_frame[1];
      mv->mv[0] = mv->mv[1];

      mv->ref_frame[1] = tmp_ref;
      mv->mv[1] = tmp_mv;
    }
  }
}
#endif  // CONFIG_TMVP_MEM_OPT

#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
#define ADJACENT_SMVP_WEIGHT 1
#define OTHER_SMVP_WEIGHT 0
#define TMVP_WEIGHT 1
#define HIGH_PRIORITY_TMVP_WEIGHT 2
#else
#define ADJACENT_SMVP_WEIGHT 2
#define OTHER_SMVP_WEIGHT 0
#define TMVP_WEIGHT 2
#define HIGH_PRIORITY_TMVP_WEIGHT 6
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY

#if CONFIG_REFINED_MVS_IN_TMVP
#define OPFL_MVS_CLAMPED 0
// Overwrite the MVs in TMVP list by optical flow refined MVs (for TIP frame
// mode)
void av1_copy_frame_refined_mvs_tip_frame_mode(const AV1_COMMON *const cm,
                                               const MACROBLOCKD *xd,
                                               const MB_MODE_INFO *const mi,
                                               int mi_row, int mi_col,
                                               int x_inside_boundary,
                                               int y_inside_boundary) {
  const int frame_mvs_stride =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  const int cur_tpl_row = (mi_row >> TMVP_SHIFT_BITS);
  const int cur_tpl_col = (mi_col >> TMVP_SHIFT_BITS);
  const int offset = cur_tpl_row * frame_mvs_stride + cur_tpl_col;
  MV_REF *frame_mvs = cm->cur_frame->mvs + offset;
  x_inside_boundary = ROUND_POWER_OF_TWO(x_inside_boundary, TMVP_SHIFT_BITS);
  y_inside_boundary = ROUND_POWER_OF_TWO(y_inside_boundary, TMVP_SHIFT_BITS);
  int bw = block_size_wide[mi->sb_type[xd->tree_type == CHROMA_PART]];
  int bh = block_size_high[mi->sb_type[xd->tree_type == CHROMA_PART]];
  const bool is_opfl_mode = opfl_allowed_for_cur_block(cm,
#if CONFIG_COMPOUND_4XN
                                                       xd,
#endif  // CONFIG_COMPOUND_4XN
                                                       mi);
  int n = opfl_get_subblock_size(bw, bh, AOM_PLANE_Y
#if CONFIG_OPTFLOW_ON_TIP
                                 ,
                                 1
#endif  // CONFIG_OPTFLOW_ON_TIP
  );
#if CONFIG_AFFINE_REFINEMENT_SB
  int sb_idx = 0;
  int sub_bw = AOMMIN(AFFINE_MAX_UNIT, bw);
  int sub_bh = AOMMIN(AFFINE_MAX_UNIT, bh);
  int wms_stride = bw / sub_bw;
#endif  // CONFIG_AFFINE_REFINEMENT_SB

  // Pointers to hold optical flow MV offsets.
  int *vx0 = xd->opfl_vxy_bufs;
  int *vx1 = xd->opfl_vxy_bufs + (N_OF_OFFSETS * 1);
  int *vy0 = xd->opfl_vxy_bufs + (N_OF_OFFSETS * 2);
  int *vy1 = xd->opfl_vxy_bufs + (N_OF_OFFSETS * 3);

  int apply_sub_block_refinemv =
      mi->refinemv_flag &&
#if CONFIG_AFFINE_REFINEMENT
      (is_damr_allowed_with_refinemv(mi->mode) ||
       mi->comp_refine_type < COMP_AFFINE_REFINE_START) &&
#endif  // CONFIG_AFFINE_REFINEMENT
      !is_tip_ref_frame(mi->ref_frame[0]);

  for (int h = 0; h < y_inside_boundary; h++) {
    MV_REF *mv = frame_mvs;
    for (int w = 0; w < x_inside_boundary; w++) {
#if CONFIG_AFFINE_REFINEMENT_SB
      sb_idx = ((h * 8) / sub_bh) * wms_stride + (w * 8) / sub_bw;
#endif  // CONFIG_AFFINE_REFINEMENT_SB
      for (int idx = 0; idx < 2; ++idx) {
        MV_REFERENCE_FRAME ref_frame = mi->ref_frame[idx];
        if (!is_inter_ref_frame(ref_frame) || is_tip_ref_frame(ref_frame))
          continue;

        int_mv refined_mv;
#if CONFIG_AFFINE_REFINEMENT
        if (is_opfl_mode && xd->use_affine_opfl &&
            mi->comp_refine_type >= COMP_AFFINE_REFINE_START
#if CONFIG_REFINEMV
            && (is_damr_allowed_with_refinemv(mi->mode) ||
                !apply_sub_block_refinemv)
#endif  // CONFIG_REFINEMV
        ) {
          // Apply offsets based on the affine parameters
          const int32_t src_x = mi_col * MI_SIZE + w * 8 + 4;
          const int32_t src_y = mi_row * MI_SIZE + h * 8 + 4;
#if CONFIG_AFFINE_REFINEMENT_SB
          const int32_t dst_x =
              xd->wm_params_sb[2 * sb_idx + idx].wmmat[2] * src_x +
              xd->wm_params_sb[2 * sb_idx + idx].wmmat[3] * src_y +
              xd->wm_params_sb[2 * sb_idx + idx].wmmat[0];
          const int32_t dst_y =
              xd->wm_params_sb[2 * sb_idx + idx].wmmat[4] * src_x +
              xd->wm_params_sb[2 * sb_idx + idx].wmmat[5] * src_y +
              xd->wm_params_sb[2 * sb_idx + idx].wmmat[1];
#else
          const int32_t dst_x = mi->wm_params[idx].wmmat[2] * src_x +
                                mi->wm_params[idx].wmmat[3] * src_y +
                                mi->wm_params[idx].wmmat[0];
          const int32_t dst_y = mi->wm_params[idx].wmmat[4] * src_x +
                                mi->wm_params[idx].wmmat[5] * src_y +
                                mi->wm_params[idx].wmmat[1];
#endif  // CONFIG_AFFINE_REFINEMENT_SB
          const int32_t submv_x_hp = dst_x - (src_x << WARPEDMODEL_PREC_BITS);
          const int32_t submv_y_hp = dst_y - (src_y << WARPEDMODEL_PREC_BITS);
          const int mv_offset_y =
              ROUND_POWER_OF_TWO_SIGNED(submv_y_hp, WARPEDMODEL_PREC_BITS - 3);
          const int mv_offset_x =
              ROUND_POWER_OF_TWO_SIGNED(submv_x_hp, WARPEDMODEL_PREC_BITS - 3);
          refined_mv.as_mv.row = mv_offset_y;
          refined_mv.as_mv.col = mv_offset_x;
        } else {
#endif  // CONFIG_AFFINE_REFINEMENT
#if CONFIG_REFINEMV
          // Refined MVs are stored per 4x4 in refinemv_subinfo, but h and
          // w for TMVP are per 8x8, so (h<<1) and (w<<1) are used here.
          if (apply_sub_block_refinemv)
            refined_mv.as_mv =
                xd->refinemv_subinfo[(h << 1) * MAX_MIB_SIZE + (w << 1)]
                    .refinemv[idx]
                    .as_mv;
          else
#endif  // CONFIG_REFINEMV
            refined_mv.as_mv = mi->mv[idx].as_mv;
#if CONFIG_AFFINE_REFINEMENT
        }
#endif  // CONFIG_AFFINE_REFINEMENT
        if (is_opfl_mode) {
          int *vy = idx ? vy1 : vy0;
          int *vx = idx ? vx1 : vx0;
          if (n == 4) {
            // Since TMVP is stored per 8x8 unit, for refined MV with 4x4
            // subblock, take the average of 4 refined MVs
            refined_mv.as_mv.row += ROUND_POWER_OF_TWO_SIGNED(
                vy[0] + vy[1] + vy[2] + vy[3], 2 + MV_REFINE_PREC_BITS - 3);
            refined_mv.as_mv.col += ROUND_POWER_OF_TWO_SIGNED(
                vx[0] + vx[1] + vx[2] + vx[3], 2 + MV_REFINE_PREC_BITS - 3);
          } else {
            int sbmv_stride = bw >> 3;
            refined_mv.as_mv.row += ROUND_POWER_OF_TWO_SIGNED(
                vy[h * sbmv_stride + w], MV_REFINE_PREC_BITS - 3);
            refined_mv.as_mv.col += ROUND_POWER_OF_TWO_SIGNED(
                vx[h * sbmv_stride + w], MV_REFINE_PREC_BITS - 3);
          }
        }
#if OPFL_MVS_CLAMPED
        refined_mv.as_mv.row =
            clamp(refined_mv.as_mv.row, -REFMVS_LIMIT, REFMVS_LIMIT);
        refined_mv.as_mv.col =
            clamp(refined_mv.as_mv.col, -REFMVS_LIMIT, REFMVS_LIMIT);
#else
        if ((abs(refined_mv.as_mv.row) > REFMVS_LIMIT) ||
            (abs(refined_mv.as_mv.col) > REFMVS_LIMIT))
          continue;
#endif
        mv->ref_frame[idx] = ref_frame;
        mv->mv[idx].as_int = refined_mv.as_int;
      }
#if CONFIG_TMVP_MEM_OPT
      check_frame_mv_slot(cm, mv);
#endif  // CONFIG_TMVP_MEM_OPT
      mv++;
    }
    frame_mvs += frame_mvs_stride;
  }
}
#endif  // CONFIG_REFINED_MVS_IN_TMVP

// Copy the MVs into the TMVP list (for TIP frame mode)
void av1_copy_frame_mvs_tip_frame_mode(const AV1_COMMON *const cm,
                                       const MACROBLOCKD *const xd,
                                       const MB_MODE_INFO *const mi, int mi_row,
                                       int mi_col, int x_inside_boundary,
                                       int y_inside_boundary) {
  const int frame_mvs_stride =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  const int cur_tpl_row = (mi_row >> TMVP_SHIFT_BITS);
  const int cur_tpl_col = (mi_col >> TMVP_SHIFT_BITS);
  const int offset = cur_tpl_row * frame_mvs_stride + cur_tpl_col;
  MV_REF *frame_mvs = cm->cur_frame->mvs + offset;
  const TIP *tip_ref = &cm->tip_ref;
  x_inside_boundary = ROUND_POWER_OF_TWO(x_inside_boundary, TMVP_SHIFT_BITS);
  y_inside_boundary = ROUND_POWER_OF_TWO(y_inside_boundary, TMVP_SHIFT_BITS);

#if CONFIG_WEDGE_TMVP
  const uint8_t *decisions = NULL;
  const BLOCK_SIZE bsize = mi->sb_type[xd->tree_type == CHROMA_PART];
  const int bw = block_size_wide[bsize];
  const bool is_wedge = is_inter_ref_frame(mi->ref_frame[0]) &&
                        is_inter_ref_frame(mi->ref_frame[1]) &&
                        !is_tip_ref_frame(mi->ref_frame[0]) &&
                        !is_tip_ref_frame(mi->ref_frame[1]) &&
                        mi->interinter_comp.type == COMPOUND_WEDGE;
  if (is_wedge) {
    decisions = av1_get_contiguous_soft_mask_decision(
        mi->interinter_comp.wedge_index, mi->interinter_comp.wedge_sign, bsize);
  }
#else
  (void)xd;
#endif  // CONFIG_WEDGE_TMVP

  for (int h = 0; h < y_inside_boundary; h++) {
    MV_REF *mv = frame_mvs;
    for (int w = 0; w < x_inside_boundary; w++) {
      for (int idx = 0; idx < 2; ++idx) {
        mv->ref_frame[idx] = NONE_FRAME;
        mv->mv[idx].as_int = 0;
      }

      for (int idx = 0; idx < 2; ++idx) {
        MV_REFERENCE_FRAME ref_frame = mi->ref_frame[idx];
        if (is_inter_ref_frame(ref_frame) && !is_tip_ref_frame(ref_frame)) {
          if ((abs(mi->mv[idx].as_mv.row) > REFMVS_LIMIT) ||
              (abs(mi->mv[idx].as_mv.col) > REFMVS_LIMIT))
            continue;

#if CONFIG_WEDGE_TMVP
          if (is_wedge) {
            const int this_decision =
                decisions[h * TMVP_MI_SIZE * bw + w * TMVP_MI_SIZE];

            if (this_decision == 0 && idx == 1) continue;
            if (this_decision == 1 && idx == 0) continue;
          }
#endif  // CONFIG_WEDGE_TMVP
          mv->ref_frame[idx] = ref_frame;
          mv->mv[idx].as_int = mi->mv[idx].as_int;
        } else if (is_tip_ref_frame(ref_frame)) {
          int_mv this_mv[2] = { { 0 } };
          const MV *blk_mv = &mi->mv[idx].as_mv;
#if !CONFIG_MF_HOLE_FILL_SIMPLIFY
          const bool is_mfmv_valid =
#endif  // !CONFIG_MF_HOLE_FILL_SIMPLIFY
              get_tip_mv(cm, blk_mv, cur_tpl_col + w, cur_tpl_row + h, this_mv);
#if !CONFIG_MF_HOLE_FILL_SIMPLIFY
          if (is_mfmv_valid) {
#endif  // !CONFIG_MF_HOLE_FILL_SIMPLIFY
            if ((abs(this_mv[0].as_mv.row) <= REFMVS_LIMIT) &&
                (abs(this_mv[0].as_mv.col) <= REFMVS_LIMIT)) {
              mv->ref_frame[0] = tip_ref->ref_frame[0];
              mv->mv[0].as_int = this_mv[0].as_int;
            }

            if ((abs(this_mv[1].as_mv.row) <= REFMVS_LIMIT) &&
                (abs(this_mv[1].as_mv.col) <= REFMVS_LIMIT)) {
              mv->ref_frame[1] = tip_ref->ref_frame[1];
              mv->mv[1].as_int = this_mv[1].as_int;
            }
#if !CONFIG_MF_HOLE_FILL_SIMPLIFY
          }
#endif  // !CONFIG_MF_HOLE_FILL_SIMPLIFY
          break;
        }
      }
#if CONFIG_TMVP_MEM_OPT
      check_frame_mv_slot(cm, mv);
#endif  // CONFIG_TMVP_MEM_OPT
      mv++;
    }
    frame_mvs += frame_mvs_stride;
  }
}

#if CONFIG_REFINED_MVS_IN_TMVP
// Overwrite the MVs in TMVP list by optical flow refined MVs
void av1_copy_frame_refined_mvs(const AV1_COMMON *const cm,
                                const MACROBLOCKD *xd,
                                const MB_MODE_INFO *const mi, int mi_row,
                                int mi_col, int x_inside_boundary,
                                int y_inside_boundary) {
  if (cm->seq_params.enable_tip && cm->features.tip_frame_mode) {
    av1_copy_frame_refined_mvs_tip_frame_mode(
        cm, xd, mi, mi_row, mi_col, x_inside_boundary, y_inside_boundary);
    return;
  }

  const int frame_mvs_stride = ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, 1);
  MV_REF *frame_mvs =
      cm->cur_frame->mvs + (mi_row >> 1) * frame_mvs_stride + (mi_col >> 1);
  x_inside_boundary = ROUND_POWER_OF_TWO(x_inside_boundary, 1);
  y_inside_boundary = ROUND_POWER_OF_TWO(y_inside_boundary, 1);
  int bw = block_size_wide[mi->sb_type[xd->tree_type == CHROMA_PART]];
  int bh = block_size_high[mi->sb_type[xd->tree_type == CHROMA_PART]];
  const bool is_opfl_mode = opfl_allowed_for_cur_block(cm,
#if CONFIG_COMPOUND_4XN
                                                       xd,
#endif  // CONFIG_COMPOUND_4XN

                                                       mi);
  int n = opfl_get_subblock_size(bw, bh, AOM_PLANE_Y
#if CONFIG_OPTFLOW_ON_TIP
                                 ,
                                 1
#endif  // CONFIG_OPTFLOW_ON_TIP
  );
  int w, h;
#if CONFIG_AFFINE_REFINEMENT_SB
  int sb_idx = 0;
  int sub_bw = AOMMIN(AFFINE_MAX_UNIT, bw);
  int sub_bh = AOMMIN(AFFINE_MAX_UNIT, bh);
  int wms_stride = bw / sub_bw;
#endif  // CONFIG_AFFINE_REFINEMENT_SB

  // Pointers to hold optical flow MV offsets.
  int *vx0 = xd->opfl_vxy_bufs;
  int *vx1 = xd->opfl_vxy_bufs + (N_OF_OFFSETS * 1);
  int *vy0 = xd->opfl_vxy_bufs + (N_OF_OFFSETS * 2);
  int *vy1 = xd->opfl_vxy_bufs + (N_OF_OFFSETS * 3);

  int apply_sub_block_refinemv =
      mi->refinemv_flag &&
#if CONFIG_AFFINE_REFINEMENT
      (is_damr_allowed_with_refinemv(mi->mode) ||
       mi->comp_refine_type < COMP_AFFINE_REFINE_START) &&
#endif  // CONFIG_AFFINE_REFINEMENT
      !is_tip_ref_frame(mi->ref_frame[0]);

  for (h = 0; h < y_inside_boundary; h++) {
    MV_REF *mv = frame_mvs;
    for (w = 0; w < x_inside_boundary; w++) {
#if CONFIG_AFFINE_REFINEMENT_SB
      sb_idx = ((h * 8) / sub_bh) * wms_stride + (w * 8) / sub_bw;
#endif  // CONFIG_AFFINE_REFINEMENT_SB
      for (int idx = 0; idx < 2; ++idx) {
        MV_REFERENCE_FRAME ref_frame = mi->ref_frame[idx];
        if (is_inter_ref_frame(ref_frame)) {
          int8_t ref_idx = cm->ref_frame_side[ref_frame];
          if (ref_idx) continue;
          int_mv refined_mv;
#if CONFIG_AFFINE_REFINEMENT
          if (is_opfl_mode && xd->use_affine_opfl &&
              mi->comp_refine_type >= COMP_AFFINE_REFINE_START
#if CONFIG_REFINEMV
              && (is_damr_allowed_with_refinemv(mi->mode) ||
                  !apply_sub_block_refinemv)
#endif  // CONFIG_REFINEMV
          ) {
            // Apply offsets based on the affine parameters
            const int32_t src_x = mi_col * MI_SIZE + w * 8 + 4;
            const int32_t src_y = mi_row * MI_SIZE + h * 8 + 4;
#if CONFIG_AFFINE_REFINEMENT_SB
            const int32_t dst_x =
                xd->wm_params_sb[2 * sb_idx + idx].wmmat[2] * src_x +
                xd->wm_params_sb[2 * sb_idx + idx].wmmat[3] * src_y +
                xd->wm_params_sb[2 * sb_idx + idx].wmmat[0];
            const int32_t dst_y =
                xd->wm_params_sb[2 * sb_idx + idx].wmmat[4] * src_x +
                xd->wm_params_sb[2 * sb_idx + idx].wmmat[5] * src_y +
                xd->wm_params_sb[2 * sb_idx + idx].wmmat[1];
#else
            const int32_t dst_x = mi->wm_params[idx].wmmat[2] * src_x +
                                  mi->wm_params[idx].wmmat[3] * src_y +
                                  mi->wm_params[idx].wmmat[0];
            const int32_t dst_y = mi->wm_params[idx].wmmat[4] * src_x +
                                  mi->wm_params[idx].wmmat[5] * src_y +
                                  mi->wm_params[idx].wmmat[1];
#endif  // CONFIG_AFFINE_REFINEMENT_SB
            const int32_t submv_x_hp = dst_x - (src_x << WARPEDMODEL_PREC_BITS);
            const int32_t submv_y_hp = dst_y - (src_y << WARPEDMODEL_PREC_BITS);
            const int mv_offset_y = ROUND_POWER_OF_TWO_SIGNED(
                submv_y_hp, WARPEDMODEL_PREC_BITS - 3);
            const int mv_offset_x = ROUND_POWER_OF_TWO_SIGNED(
                submv_x_hp, WARPEDMODEL_PREC_BITS - 3);
            refined_mv.as_mv.row = mv_offset_y;
            refined_mv.as_mv.col = mv_offset_x;
          } else {
#endif  // CONFIG_AFFINE_REFINEMENT
#if CONFIG_REFINEMV
            // Refined MVs are stored per 4x4 in refinemv_subinfo, but h and
            // w for TMVP are per 8x8, so (h<<1) and (w<<1) are used here.
            if (apply_sub_block_refinemv)
              refined_mv.as_mv =
                  xd->refinemv_subinfo[(h << 1) * MAX_MIB_SIZE + (w << 1)]
                      .refinemv[idx]
                      .as_mv;
            else
#endif  // CONFIG_REFINEMV
              refined_mv.as_mv = mi->mv[idx].as_mv;
#if CONFIG_AFFINE_REFINEMENT
          }
#endif  // CONFIG_AFFINE_REFINEMENT
          if (is_opfl_mode) {
            int *vy = idx ? vy1 : vy0;
            int *vx = idx ? vx1 : vx0;
            if (n == 4) {
              // Since TMVP is stored per 8x8 unit, for refined MV with 4x4
              // subblock, take the average of 4 refined MVs
              refined_mv.as_mv.row += ROUND_POWER_OF_TWO_SIGNED(
                  vy[0] + vy[1] + vy[2] + vy[3], 2 + MV_REFINE_PREC_BITS - 3);
              refined_mv.as_mv.col += ROUND_POWER_OF_TWO_SIGNED(
                  vx[0] + vx[1] + vx[2] + vx[3], 2 + MV_REFINE_PREC_BITS - 3);
            } else {
              int sbmv_stride = bw >> 3;
              refined_mv.as_mv.row += ROUND_POWER_OF_TWO_SIGNED(
                  vy[h * sbmv_stride + w], MV_REFINE_PREC_BITS - 3);
              refined_mv.as_mv.col += ROUND_POWER_OF_TWO_SIGNED(
                  vx[h * sbmv_stride + w], MV_REFINE_PREC_BITS - 3);
            }
          }
#if OPFL_MVS_CLAMPED
          refined_mv.as_mv.row =
              clamp(refined_mv.as_mv.row, -REFMVS_LIMIT, REFMVS_LIMIT);
          refined_mv.as_mv.col =
              clamp(refined_mv.as_mv.col, -REFMVS_LIMIT, REFMVS_LIMIT);
#else
          if ((abs(refined_mv.as_mv.row) > REFMVS_LIMIT) ||
              (abs(refined_mv.as_mv.col) > REFMVS_LIMIT))
            continue;
#endif
          mv->ref_frame[idx] = ref_frame;
          mv->mv[idx].as_int = refined_mv.as_int;
        }
      }
#if CONFIG_TMVP_MEM_OPT
      check_frame_mv_slot(cm, mv);
#endif  // CONFIG_TMVP_MEM_OPT
      mv++;
    }
    frame_mvs += frame_mvs_stride;
  }
}
#endif  // CONFIG_REFINED_MVS_IN_TMVP

// Copy the MVs into the TMVP list
void av1_copy_frame_mvs(const AV1_COMMON *const cm, const MACROBLOCKD *const xd,
                        const MB_MODE_INFO *const mi, int mi_row, int mi_col,
                        int x_inside_boundary, int y_inside_boundary) {
  if (cm->seq_params.enable_tip && cm->features.tip_frame_mode) {
    av1_copy_frame_mvs_tip_frame_mode(cm, xd, mi, mi_row, mi_col,
                                      x_inside_boundary, y_inside_boundary);
    return;
  }

  const int frame_mvs_stride = ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, 1);
  MV_REF *frame_mvs =
      cm->cur_frame->mvs + (mi_row >> 1) * frame_mvs_stride + (mi_col >> 1);
  x_inside_boundary = ROUND_POWER_OF_TWO(x_inside_boundary, 1);
  y_inside_boundary = ROUND_POWER_OF_TWO(y_inside_boundary, 1);
  int w, h;

#if CONFIG_WEDGE_TMVP
  const uint8_t *decisions = NULL;
  const BLOCK_SIZE bsize = mi->sb_type[xd->tree_type == CHROMA_PART];
  const int bw = block_size_wide[bsize];
  const bool is_wedge = is_inter_ref_frame(mi->ref_frame[0]) &&
                        is_inter_ref_frame(mi->ref_frame[1]) &&
                        !is_tip_ref_frame(mi->ref_frame[0]) &&
                        !is_tip_ref_frame(mi->ref_frame[1]) &&
                        mi->interinter_comp.type == COMPOUND_WEDGE;
  if (is_wedge) {
    decisions = av1_get_contiguous_soft_mask_decision(
        mi->interinter_comp.wedge_index, mi->interinter_comp.wedge_sign, bsize);
  }
#endif  // CONFIG_WEDGE_TMVP

  for (h = 0; h < y_inside_boundary; h++) {
    MV_REF *mv = frame_mvs;
    for (w = 0; w < x_inside_boundary; w++) {
      mv->ref_frame[0] = NONE_FRAME;
      mv->ref_frame[1] = NONE_FRAME;
      mv->mv[0].as_int = 0;
      mv->mv[1].as_int = 0;

#if CONFIG_MVP_IMPROVEMENT
      if (is_inter_ref_frame(mi->ref_frame[0]) &&
          mi->ref_frame[1] == NONE_FRAME) {
        if ((abs(mi->mv[0].as_mv.row) <= REFMVS_LIMIT) &&
            (abs(mi->mv[0].as_mv.col) <= REFMVS_LIMIT)) {
          mv->ref_frame[0] = mi->ref_frame[0];
          mv->mv[0].as_int = mi->mv[0].as_int;
        }
      } else {
#endif  // CONFIG_MVP_IMPROVEMENT
        for (int idx = 0; idx < 2; ++idx) {
          MV_REFERENCE_FRAME ref_frame = mi->ref_frame[idx];
          if (is_inter_ref_frame(ref_frame)) {
            int8_t ref_idx = cm->ref_frame_side[ref_frame];
            if (ref_idx) continue;
            if ((abs(mi->mv[idx].as_mv.row) > REFMVS_LIMIT) ||
                (abs(mi->mv[idx].as_mv.col) > REFMVS_LIMIT))
              continue;
#if CONFIG_WEDGE_TMVP
            if (is_wedge) {
              const int this_decision =
                  decisions[h * TMVP_MI_SIZE * bw + w * TMVP_MI_SIZE];

              if (this_decision == 0 && idx == 1) continue;
              if (this_decision == 1 && idx == 0) continue;
            }
#endif  // CONFIG_WEDGE_TMVP
            mv->ref_frame[0] = ref_frame;
            mv->mv[0].as_int = mi->mv[idx].as_int;
          }
        }
#if CONFIG_MVP_IMPROVEMENT
      }
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_TMVP_MEM_OPT
      check_frame_mv_slot(cm, mv);
#endif  // CONFIG_TMVP_MEM_OPT
      mv++;
    }
    frame_mvs += frame_mvs_stride;
  }
}

#if CONFIG_MVP_IMPROVEMENT
// Fetch MVP candidates from derived SMVP into MVP candidate list
// when there is no enough MVP candidates.
static AOM_INLINE void fill_mvp_from_derived_smvp(
    const MV_REFERENCE_FRAME rf[2], CANDIDATE_MV *ref_mv_stack,
    uint16_t *ref_mv_weight, uint8_t *refmv_count,
    CANDIDATE_MV *derived_mv_stack, uint8_t derived_mv_count,
#if CONFIG_SKIP_MODE_ENHANCEMENT
    const MB_MODE_INFO *mbmi, MV_REFERENCE_FRAME *ref_frame_idx0,
    MV_REFERENCE_FRAME *ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
    const int max_ref_mv_count) {
  int index = 0;
  int derived_idx = 0;

#if CONFIG_D072_SKIP_MODE_IMPROVE
  if (mbmi->skip_mode) return;
#endif  // CONFIG_D072_SKIP_MODE_IMPROVE

  if (rf[1] == NONE_FRAME) {
#if CONFIG_SKIP_MODE_ENHANCEMENT
    assert(!mbmi->skip_mode);
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT

    for (derived_idx = 0; derived_idx < derived_mv_count; ++derived_idx) {
      for (index = 0; index < *refmv_count; ++index) {
        if (ref_mv_stack[index].this_mv.as_int ==
            derived_mv_stack[derived_idx].this_mv.as_int) {
          break;
        }
      }

      // Add a new item to the list.
      if (index == *refmv_count && *refmv_count < max_ref_mv_count) {
        ref_mv_stack[index].this_mv = derived_mv_stack[derived_idx].this_mv;
#if CONFIG_EXTENDED_WARP_PREDICTION
        ref_mv_stack[index].row_offset = OFFSET_NONSPATIAL;
        ref_mv_stack[index].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
        ref_mv_stack[index].cwp_idx = derived_mv_stack[derived_idx].cwp_idx;
        ref_mv_weight[index] = REF_CAT_LEVEL;
        ++(*refmv_count);
      }
    }
  } else {
    for (derived_idx = 0; derived_idx < derived_mv_count; ++derived_idx) {
      for (index = 0; index < *refmv_count; ++index) {
        if ((ref_mv_stack[index].this_mv.as_int ==
             derived_mv_stack[derived_idx].this_mv.as_int) &&
            (ref_mv_stack[index].comp_mv.as_int ==
             derived_mv_stack[derived_idx].comp_mv.as_int)) {
#if CONFIG_SKIP_MODE_ENHANCEMENT
          if (!mbmi->skip_mode || (ref_frame_idx0[index] == rf[0] &&
                                   ref_frame_idx1[index] == rf[1]))
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
            break;
        }
      }

      // Add a new item to the list.
      if (index == *refmv_count && *refmv_count < max_ref_mv_count) {
        ref_mv_stack[index].this_mv = derived_mv_stack[derived_idx].this_mv;
        ref_mv_stack[index].comp_mv = derived_mv_stack[derived_idx].comp_mv;
#if CONFIG_EXTENDED_WARP_PREDICTION
        ref_mv_stack[index].row_offset = OFFSET_NONSPATIAL;
        ref_mv_stack[index].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
        ref_mv_stack[index].cwp_idx = derived_mv_stack[derived_idx].cwp_idx;
#if CONFIG_SKIP_MODE_ENHANCEMENT
        if (mbmi->skip_mode) {
          ref_frame_idx0[index] = rf[0];
          ref_frame_idx1[index] = rf[1];
        }
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
        ref_mv_weight[index] = REF_CAT_LEVEL;
        ++(*refmv_count);
      }
    }
  }
}
#endif  // CONFIG_MVP_IMPROVEMENT

static AOM_INLINE void derive_ref_mv_candidate_from_tip_mode(
    const AV1_COMMON *cm, int mi_row, int mi_col, int mi_row_cand,
    int mi_col_cand, const MB_MODE_INFO *const candidate, uint8_t *refmv_count,
    uint8_t *ref_match_count, uint8_t *newmv_count, CANDIDATE_MV *ref_mv_stack,
    uint16_t *ref_mv_weight, uint16_t weight) {
  int index = 0;

  const int cand_tpl_row = (mi_row_cand >> TMVP_SHIFT_BITS);
  const int cand_tpl_col = (mi_col_cand >> TMVP_SHIFT_BITS);
#if CONFIG_C071_SUBBLK_WARPMV
  int_mv cand_mv = candidate->mv[0];
#else
  int_mv cand_mv = get_block_mv(candidate, 0);
#endif  // CONFIG_C071_SUBBLK_WARPMV
  int_mv ref_mv[2];
#if !CONFIG_MF_HOLE_FILL_SIMPLIFY
  const bool is_mfmv_valid =
#endif  // !CONFIG_MF_HOLE_FILL_SIMPLIFY
      get_tip_mv(cm, &cand_mv.as_mv, cand_tpl_col, cand_tpl_row, ref_mv);
#if !CONFIG_MF_HOLE_FILL_SIMPLIFY
  if (!is_mfmv_valid) return;
#endif  // !CONFIG_MF_HOLE_FILL_SIMPLIFY

  clamp_tip_smvp_refmv(cm, &ref_mv[0].as_mv, mi_row, mi_col);
  clamp_tip_smvp_refmv(cm, &ref_mv[1].as_mv, mi_row, mi_col);

  for (index = 0; index < *refmv_count; ++index) {
    if ((ref_mv_stack[index].this_mv.as_int == ref_mv[0].as_int) &&
        (ref_mv_stack[index].comp_mv.as_int == ref_mv[1].as_int)) {
      ref_mv_weight[index] += weight;
      break;
    }
  }

  // Add a new item to the list.
  if (index == *refmv_count && index < MAX_REF_MV_STACK_SIZE) {
    ref_mv_stack[index].this_mv = ref_mv[0];
    ref_mv_stack[index].comp_mv = ref_mv[1];
    ref_mv_weight[index] = weight;
#if CONFIG_EXTENDED_WARP_PREDICTION
    ref_mv_stack[index].row_offset = OFFSET_NONSPATIAL;
    ref_mv_stack[index].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
    ref_mv_stack[index].cwp_idx = candidate->cwp_idx;
    ++(*refmv_count);
  }
  if (have_newmv_in_inter_mode(candidate->mode)) ++*newmv_count;
  ++*ref_match_count;
}

#if CONFIG_C076_INTER_MOD_CTX
// add neighbor info to inter mode contexts
static AOM_INLINE void add_ref_mv_candidate_ctx(
    const MB_MODE_INFO *const candidate, uint8_t *ref_match_count,
    uint8_t *newmv_count, const AV1_COMMON *cm, const MV_REFERENCE_FRAME rf[2],
    const MB_MODE_INFO *mbmi) {
  if (!is_inter_block(candidate, SHARED_PART)) return;
  const TIP *tip_ref = &cm->tip_ref;
  if (mbmi->skip_mode) return;
  if (rf[1] == NONE_FRAME) {
    // single reference frame
    for (int ref = 0; ref < 2; ++ref) {
      if (candidate->ref_frame[ref] == rf[0]) {
        if (have_newmv_in_inter_mode(candidate->mode)) ++*newmv_count;
        ++*ref_match_count;
      }
    }
  } else {
    if (is_tip_ref_frame(candidate->ref_frame[0]) &&
        candidate->ref_frame[1] == NONE_FRAME &&
        rf[0] == tip_ref->ref_frame[0] && rf[1] == tip_ref->ref_frame[1] &&
        cm->features.tip_frame_mode) {
      if (have_newmv_in_inter_mode(candidate->mode)) ++*newmv_count;
      ++*ref_match_count;
    } else {
      // compound reference frame
      if (candidate->ref_frame[0] == rf[0] &&
          candidate->ref_frame[1] == rf[1]) {
        if (have_newmv_in_inter_mode(candidate->mode)) ++*newmv_count;
        ++*ref_match_count;
      }
    }
  }
}
#endif  // CONFIG_C076_INTER_MOD_CTX

static AOM_INLINE void add_ref_mv_candidate(
#if !CONFIG_MVP_IMPROVEMENT
    const AV1_COMMON *cm,
#endif  // !CONFIG_MVP_IMPROVEMENT
    int mi_row, int mi_col, int mi_row_cand, int mi_col_cand,
    const MB_MODE_INFO *const candidate,
#if CONFIG_C071_SUBBLK_WARPMV
    const SUBMB_INFO *const submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
    const MV_REFERENCE_FRAME rf[2], uint8_t *refmv_count,
    uint8_t *ref_match_count, uint8_t *newmv_count, CANDIDATE_MV *ref_mv_stack,
    uint16_t *ref_mv_weight, int_mv *gm_mv_candidates,
    const WarpedMotionParams *gm_params,
#if CONFIG_SKIP_MODE_ENHANCEMENT
    const MB_MODE_INFO *mbmi,
    MV_REFERENCE_FRAME ref_frame_idx0[MAX_REF_MV_STACK_SIZE],
    MV_REFERENCE_FRAME ref_frame_idx1[MAX_REF_MV_STACK_SIZE],
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
    const AV1_COMMON *cm, int add_more_mvs, SINGLE_MV_CANDIDATE *single_mv,
    uint8_t *single_mv_count, CANDIDATE_MV *derived_mv_stack,
    uint16_t *derived_mv_weight, uint8_t *derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_IBC_SR_EXT
    uint8_t is_intrabc,
#endif  // CONFIG_IBC_SR_EXT
#if CONFIG_EXTENDED_WARP_PREDICTION
    int row_offset, int col_offset,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
    uint16_t weight, const MvSubpelPrecision precision) {
#if CONFIG_C071_SUBBLK_WARPMV
  (void)precision;
#endif  // CONFIG_C071_SUBBLK_WARPMV
  if (!is_inter_block(candidate, SHARED_PART)) return;

#if CONFIG_IBC_SR_EXT
  if (is_intrabc != is_intrabc_block(candidate, SHARED_PART)) return;
#endif  // CONFIG_IBC_SR_EXT

#if !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  assert(weight % 2 == 0);
#endif  // !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  int index, ref;

  const TIP *tip_ref = &cm->tip_ref;

#if CONFIG_SKIP_MODE_ENHANCEMENT
  if (mbmi->skip_mode) {
    if (!is_tip_ref_frame(candidate->ref_frame[0]) &&
        !is_tip_ref_frame(candidate->ref_frame[1]) &&
        (is_inter_ref_frame(candidate->ref_frame[0]) &&
         is_inter_ref_frame(candidate->ref_frame[1]))) {
      int_mv this_refmv[2];
      for (ref = 0; ref < 2; ++ref) {
        if (is_global_mv_block(candidate, gm_params[rf[ref]].wmtype))
          this_refmv[ref] = gm_mv_candidates[ref];
        else
          this_refmv[ref] = get_block_mv(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                         submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                         ref);
      }

      for (index = 0; index < *refmv_count; ++index) {
        if ((ref_mv_stack[index].this_mv.as_int == this_refmv[0].as_int) &&
            (ref_mv_stack[index].comp_mv.as_int == this_refmv[1].as_int) &&
            (ref_frame_idx0[index] == candidate->ref_frame[0]) &&
            (ref_frame_idx1[index] == candidate->ref_frame[1])) {
          ref_mv_weight[index] += weight;
          break;
        }
      }

      // Add a new item to the list.
      if (index == *refmv_count && *refmv_count < MAX_REF_MV_STACK_SIZE) {
        ref_mv_stack[index].this_mv = this_refmv[0];
        ref_mv_stack[index].comp_mv = this_refmv[1];
        ref_frame_idx0[index] = candidate->ref_frame[0];
        ref_frame_idx1[index] = candidate->ref_frame[1];
        ref_mv_stack[index].cwp_idx = candidate->cwp_idx;
        ref_mv_weight[index] = weight;
        ++(*refmv_count);
      }
    }
#if CONFIG_D072_SKIP_MODE_IMPROVE
    else if (is_inter_ref_frame(candidate->ref_frame[0]) &&
             candidate->ref_frame[1] == NONE_FRAME &&
             !is_tip_ref_frame(candidate->ref_frame[0])) {
      int_mv this_refmv;
      if (is_global_mv_block(candidate,
                             gm_params[candidate->ref_frame[0]].wmtype)) {
        return;
      } else {
        this_refmv = get_block_mv(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                  submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                  0);
      }

      for (index = 0; index < *refmv_count; ++index) {
        if (ref_mv_stack[index].this_mv.as_int == this_refmv.as_int &&
            ref_mv_stack[index].comp_mv.as_int == INVALID_MV &&
            ref_frame_idx0[index] == candidate->ref_frame[0] &&
            ref_frame_idx1[index] == candidate->ref_frame[1]) {
          ref_mv_weight[index] += weight;
          break;
        }
      }

      // Add a new item to the list.
      if (index == *refmv_count && *refmv_count < MAX_REF_MV_STACK_SIZE) {
        ref_mv_stack[index].this_mv = this_refmv;
        ref_mv_stack[index].comp_mv.as_int = INVALID_MV;
        ref_frame_idx0[index] = candidate->ref_frame[0];
        ref_frame_idx1[index] = candidate->ref_frame[1];
        ref_mv_weight[index] = weight;
        ++(*refmv_count);
      }
    }
#endif  // CONFIG_D072_SKIP_MODE_IMPROVE
    return;
  }
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT

  if (rf[1] == NONE_FRAME) {
#if CONFIG_SKIP_MODE_ENHANCEMENT
    assert(!mbmi->skip_mode);
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT

    // single reference frame
    for (ref = 0; ref < 2; ++ref) {
      if (candidate->ref_frame[ref] == rf[0]) {
        int_mv this_refmv;
        if (is_tip_ref_frame(rf[0])) {
          this_refmv = get_block_mv(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                    submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                    ref);
        } else {
          const int is_gm_block =
              is_global_mv_block(candidate, gm_params[rf[0]].wmtype);
          this_refmv = is_gm_block ? gm_mv_candidates[0]
                                   : get_block_mv(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                                  submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                                  ref);
        }

        for (index = 0; index < *refmv_count; ++index) {
          if (ref_mv_stack[index].this_mv.as_int == this_refmv.as_int) {
            ref_mv_weight[index] += weight;
            break;
          }
        }

        // Add a new item to the list.
        if (index == *refmv_count && *refmv_count < MAX_REF_MV_STACK_SIZE) {
          ref_mv_stack[index].this_mv = this_refmv;
#if CONFIG_EXTENDED_WARP_PREDICTION
          ref_mv_stack[index].row_offset = row_offset;
          ref_mv_stack[index].col_offset = col_offset;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
          ref_mv_stack[index].cwp_idx = candidate->cwp_idx;
          ref_mv_weight[index] = weight;
          ++(*refmv_count);
        }
        if (have_newmv_in_inter_mode(candidate->mode)) ++*newmv_count;
        ++*ref_match_count;
      }
#if CONFIG_MVP_IMPROVEMENT
      else if (add_more_mvs && is_inter_ref_frame(candidate->ref_frame[ref]) &&
#if CONFIG_IBC_SR_EXT
               rf[0] != INTRA_FRAME &&
#endif  // CONFIG_IBC_SR_EXT
               !is_tip_ref_frame(rf[0]) &&
               !is_tip_ref_frame(candidate->ref_frame[ref]) &&
               cm->seq_params.order_hint_info.enable_order_hint) {
        const int is_gm_block = is_global_mv_block(
            candidate, gm_params[candidate->ref_frame[ref]].wmtype);
        const int_mv cand_refmv = is_gm_block ? gm_mv_candidates[0]
                                              : get_block_mv(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                                             submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                                             ref);
#if CONFIG_MV_TRAJECTORY
        const int frame_mvs_stride =
            ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);

        int cand_ref_mv_row;
        int cand_ref_mv_col;
        int valid;
        if (cand_refmv.as_int == 0) {
          valid = 1;
          cand_ref_mv_row = mi_row_cand >> 1;
          cand_ref_mv_col = mi_col_cand >> 1;
        } else {
          valid = get_block_position(cm, &cand_ref_mv_row, &cand_ref_mv_col,
                                     mi_row_cand >> 1, mi_col_cand >> 1,
                                     cand_refmv.as_mv, 0);
        }
        int traj_id = valid
                          ? cm->blk_id_map[candidate->ref_frame[ref]]
                                          [cand_ref_mv_row * frame_mvs_stride +
                                           cand_ref_mv_col]
                          : -1;
        if (traj_id >= 0 && cm->features.allow_ref_frame_mvs &&
            cm->id_offset_map[rf[0]][traj_id].as_int != INVALID_MV &&
            cm->id_offset_map[candidate->ref_frame[ref]][traj_id].as_int !=
                INVALID_MV) {
          int_mv mv_traj_cand_ref =
              cm->id_offset_map[candidate->ref_frame[ref]][traj_id];
          int_mv mv_traj_cur_ref = cm->id_offset_map[rf[0]][traj_id];

          const int clamp_max = MV_UPP - 1;
          const int clamp_min = MV_LOW + 1;
          int_mv derived_mv;
          derived_mv.as_mv.row = clamp(
              cand_refmv.as_mv.row +
                  (mv_traj_cur_ref.as_mv.row - mv_traj_cand_ref.as_mv.row),
              clamp_min, clamp_max);
          derived_mv.as_mv.col = clamp(
              cand_refmv.as_mv.col +
                  (mv_traj_cur_ref.as_mv.col - mv_traj_cand_ref.as_mv.col),
              clamp_min, clamp_max);

#if !CONFIG_C071_SUBBLK_WARPMV
          lower_mv_precision(&derived_mv.as_mv, precision);
#endif  // !CONFIG_C071_SUBBLK_WARPMV

          for (index = 0; index < *derived_mv_count; ++index) {
            if (derived_mv_stack[index].this_mv.as_int == derived_mv.as_int) {
              derived_mv_weight[index] += weight;
              break;
            }
          }
          // Add a new item to the list.
          if (index == *derived_mv_count &&
              *derived_mv_count < MAX_REF_MV_STACK_SIZE) {
            derived_mv_stack[index].this_mv = derived_mv;
            derived_mv_weight[index] = weight;
            derived_mv_stack[index].cwp_idx = candidate->cwp_idx;
            ++(*derived_mv_count);
          }
        } else {
#endif  // CONFIG_MV_TRAJECTORY
          const int cur_blk_ref_side = cm->ref_frame_side[rf[0]];
          const int cand_blk_ref_side =
              cm->ref_frame_side[candidate->ref_frame[ref]];

          const int same_side =
              (cur_blk_ref_side > 0 && cand_blk_ref_side > 0) ||
              (cur_blk_ref_side == 0 && cand_blk_ref_side == 0);

          if (same_side) {
            const int cur_to_ref_dist = cm->ref_frame_relative_dist[rf[0]];
            const int cand_to_ref_dist =
                cm->ref_frame_relative_dist[candidate->ref_frame[ref]];

            int_mv this_refmv;
            get_mv_projection(&this_refmv.as_mv, cand_refmv.as_mv,
                              cur_to_ref_dist, cand_to_ref_dist);
#if !CONFIG_C071_SUBBLK_WARPMV
            lower_mv_precision(&this_refmv.as_mv, precision);
#endif  // !CONFIG_C071_SUBBLK_WARPMV

            for (index = 0; index < *derived_mv_count; ++index) {
              if (derived_mv_stack[index].this_mv.as_int == this_refmv.as_int) {
                derived_mv_weight[index] += weight;
                break;
              }
            }

            // Add a new item to the list.
            if (index == *derived_mv_count &&
                *derived_mv_count < MAX_REF_MV_STACK_SIZE) {
              derived_mv_stack[index].this_mv = this_refmv;
              derived_mv_weight[index] = weight;
              derived_mv_stack[index].cwp_idx = candidate->cwp_idx;
              ++(*derived_mv_count);
            }
          }
#if CONFIG_MV_TRAJECTORY
        }
#endif  // CONFIG_MV_TRAJECTORY
      }
#endif  // CONFIG_MVP_IMPROVEMENT
    }
  } else {
    if (is_tip_ref_frame(candidate->ref_frame[0]) &&
        candidate->ref_frame[1] == NONE_FRAME &&
        rf[0] == tip_ref->ref_frame[0] && rf[1] == tip_ref->ref_frame[1] &&
        cm->features.tip_frame_mode) {
      derive_ref_mv_candidate_from_tip_mode(
          cm, mi_row, mi_col, mi_row_cand, mi_col_cand, candidate, refmv_count,
          ref_match_count, newmv_count, ref_mv_stack, ref_mv_weight, weight);
    } else {
      // compound reference frame
      if (candidate->ref_frame[0] == rf[0] &&
          candidate->ref_frame[1] == rf[1]) {
#if CONFIG_SKIP_MODE_ENHANCEMENT
        if (mbmi->skip_mode) return;
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT

        int_mv this_refmv[2];

        for (ref = 0; ref < 2; ++ref) {
          if (is_global_mv_block(candidate, gm_params[rf[ref]].wmtype))
            this_refmv[ref] = gm_mv_candidates[ref];
          else
            this_refmv[ref] = get_block_mv(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                           submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                           ref);
        }

        for (index = 0; index < *refmv_count; ++index) {
          if ((ref_mv_stack[index].this_mv.as_int == this_refmv[0].as_int) &&
              (ref_mv_stack[index].comp_mv.as_int == this_refmv[1].as_int)) {
            ref_mv_weight[index] += weight;
            break;
          }
        }

        // Add a new item to the list.
        if (index == *refmv_count && *refmv_count < MAX_REF_MV_STACK_SIZE) {
          ref_mv_stack[index].this_mv = this_refmv[0];
          ref_mv_stack[index].comp_mv = this_refmv[1];
          ref_mv_weight[index] = weight;
#if CONFIG_EXTENDED_WARP_PREDICTION
          ref_mv_stack[index].row_offset = OFFSET_NONSPATIAL;
          ref_mv_stack[index].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
          ref_mv_stack[index].cwp_idx = candidate->cwp_idx;
          ++(*refmv_count);
        }
        if (have_newmv_in_inter_mode(candidate->mode)) ++*newmv_count;
        ++*ref_match_count;
      }
#if CONFIG_MVP_IMPROVEMENT
      else if (add_more_mvs) {
#if CONFIG_MV_TRAJECTORY
        if (cm->seq_params.order_hint_info.enable_order_hint &&
            cm->features.allow_ref_frame_mvs && rf[0] != rf[1] &&
            is_inter_ref_frame(rf[0]) && is_inter_ref_frame(rf[1])) {
          const int frame_mvs_stride =
              ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);

          for (ref = 0; ref < 2; ref++) {
            if (!is_inter_ref_frame(candidate->ref_frame[ref]) ||
                is_tip_ref_frame(candidate->ref_frame[ref]))
              continue;
            const int is_gm_block = is_global_mv_block(
                candidate, gm_params[candidate->ref_frame[ref]].wmtype);
            const int_mv cand_refmv = is_gm_block ? gm_mv_candidates[ref]
                                                  : get_block_mv(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                                                 submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                                                 ref);
            int cand_ref_mv_row;
            int cand_ref_mv_col;
            int valid;
            if (cand_refmv.as_int == 0) {
              valid = 1;
              cand_ref_mv_row = (mi_row_cand >> 1);
              cand_ref_mv_col = (mi_col_cand >> 1);
            } else {
              valid = get_block_position(cm, &cand_ref_mv_row, &cand_ref_mv_col,
                                         mi_row_cand >> 1, mi_col_cand >> 1,
                                         cand_refmv.as_mv, 0);
            }
            int traj_id =
                valid ? cm->blk_id_map[candidate->ref_frame[ref]]
                                      [cand_ref_mv_row * frame_mvs_stride +
                                       cand_ref_mv_col]
                      : -1;
            if (traj_id >= 0) {
              int_mv mv_traj_cand_ref =
                  cm->id_offset_map[candidate->ref_frame[ref]][traj_id];
              int_mv mv_traj_cur_ref0 = cm->id_offset_map[rf[0]][traj_id];
              int_mv mv_traj_cur_ref1 = cm->id_offset_map[rf[1]][traj_id];

              if (mv_traj_cand_ref.as_int == INVALID_MV ||
                  mv_traj_cur_ref0.as_int == INVALID_MV ||
                  mv_traj_cur_ref1.as_int == INVALID_MV) {
                continue;
              }

              int_mv this_refmv[2];
              const int clamp_max = MV_UPP - 1;
              const int clamp_min = MV_LOW + 1;
              this_refmv[0].as_mv.row = clamp(
                  cand_refmv.as_mv.row +
                      (mv_traj_cur_ref0.as_mv.row - mv_traj_cand_ref.as_mv.row),
                  clamp_min, clamp_max);
              this_refmv[0].as_mv.col = clamp(
                  cand_refmv.as_mv.col +
                      (mv_traj_cur_ref0.as_mv.col - mv_traj_cand_ref.as_mv.col),
                  clamp_min, clamp_max);
              this_refmv[1].as_mv.row = clamp(
                  cand_refmv.as_mv.row +
                      (mv_traj_cur_ref1.as_mv.row - mv_traj_cand_ref.as_mv.row),
                  clamp_min, clamp_max);
              this_refmv[1].as_mv.col = clamp(
                  cand_refmv.as_mv.col +
                      (mv_traj_cur_ref1.as_mv.col - mv_traj_cand_ref.as_mv.col),
                  clamp_min, clamp_max);

              for (index = 0; index < *derived_mv_count; ++index) {
                if ((derived_mv_stack[index].this_mv.as_int ==
                     this_refmv[0].as_int) &&
                    (derived_mv_stack[index].comp_mv.as_int ==
                     this_refmv[1].as_int)) {
                  derived_mv_weight[index] += weight;
                  break;
                }
              }

              // Add a new item to the list.
              if (index == *derived_mv_count &&
                  *derived_mv_count < MAX_REF_MV_STACK_SIZE) {
                derived_mv_stack[index].this_mv = this_refmv[0];
                derived_mv_stack[index].comp_mv = this_refmv[1];
                derived_mv_weight[index] = weight;
                derived_mv_stack[index].cwp_idx = candidate->cwp_idx;
                ++(*derived_mv_count);
              }
            }
          }
        }
#endif  // CONFIG_MV_TRAJECTORY

        // Compound reference frame, but only have one reference frame
        // is the same as the reference frame of the neighboring block
        int candidate_ref_idx0 = -1;
        int candidate_ref_idx1 = -1;
        int which_cand_ref = -1;
        if (candidate->ref_frame[0] == rf[0] ||
            candidate->ref_frame[1] == rf[0]) {
          candidate_ref_idx0 = 0;
          candidate_ref_idx1 = 1;
          which_cand_ref = (candidate->ref_frame[0] == rf[0]) ? 0 : 1;
        } else if (candidate->ref_frame[0] == rf[1] ||
                   candidate->ref_frame[1] == rf[1]) {
          candidate_ref_idx0 = 1;
          candidate_ref_idx1 = 0;
          which_cand_ref = (candidate->ref_frame[0] == rf[1]) ? 0 : 1;
        }

        if (candidate_ref_idx0 != -1 && candidate_ref_idx1 != -1) {
          int_mv this_refmv[2];
          const int is_gm_block = is_global_mv_block(
              candidate, gm_params[rf[candidate_ref_idx0]].wmtype);
          this_refmv[candidate_ref_idx0] =
              is_gm_block ? gm_mv_candidates[candidate_ref_idx0]
                          : get_block_mv(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                         submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                         which_cand_ref);

          int cand_idx = 0;
          for (cand_idx = 0; cand_idx < *single_mv_count; ++cand_idx) {
            if (single_mv[cand_idx].ref_frame == rf[candidate_ref_idx1]) {
              this_refmv[candidate_ref_idx1].as_int =
                  single_mv[cand_idx].mv.as_int;
              break;
            }
          }

          // Add a new item to the list.
          if (cand_idx < *single_mv_count) {
            for (index = 0; index < *derived_mv_count; ++index) {
              if ((derived_mv_stack[index].this_mv.as_int ==
                   this_refmv[0].as_int) &&
                  (derived_mv_stack[index].comp_mv.as_int ==
                   this_refmv[1].as_int)) {
                derived_mv_weight[index] += weight;
                break;
              }
            }

            // Add a new item to the list.
            if (index == *derived_mv_count &&
                *derived_mv_count < MAX_REF_MV_STACK_SIZE) {
              derived_mv_stack[index].this_mv = this_refmv[0];
              derived_mv_stack[index].comp_mv = this_refmv[1];
              derived_mv_weight[index] = weight;
              derived_mv_stack[index].cwp_idx = candidate->cwp_idx;
              ++(*derived_mv_count);
            }
          }

          // Add the candidate to single MV stack
          for (cand_idx = 0; cand_idx < *single_mv_count; ++cand_idx) {
            if (single_mv[cand_idx].ref_frame == rf[candidate_ref_idx0] &&
                (single_mv[cand_idx].mv.as_int ==
                 this_refmv[candidate_ref_idx0].as_int)) {
              break;
            }
          }

          if (cand_idx == *single_mv_count &&
              *single_mv_count < MAX_REF_MV_STACK_SIZE) {
            single_mv[cand_idx].mv.as_int =
                this_refmv[candidate_ref_idx0].as_int;
            single_mv[cand_idx].ref_frame = rf[candidate_ref_idx0];
            ++(*single_mv_count);
          }
        }
      }
#endif  // CONFIG_MVP_IMPROVEMENT
    }
  }
}

#if CONFIG_EXTENDED_WARP_PREDICTION
// Check if the candidate block has valid warp parameters
// Return 1 if the candidate warp parameters are valid
static INLINE uint8_t is_valid_warp_parameters(
    const AV1_COMMON *cm, const MB_MODE_INFO *neighbor_mbmi,
    const int ref_frame, WarpedMotionParams *neighbor_params) {
  (void)cm;
#if CONFIG_COMPOUND_WARP_CAUSAL
  if (is_warp_mode(neighbor_mbmi->motion_mode)) {
    for (int ref_idx = 0;
         ref_idx < 1 + is_inter_compound_mode(neighbor_mbmi->mode); ref_idx++) {
      int is_same_ref = (neighbor_mbmi->ref_frame[ref_idx] == ref_frame);
      if (is_same_ref && !neighbor_mbmi->wm_params[ref_idx].invalid &&
          neighbor_params) {
        *neighbor_params = neighbor_mbmi->wm_params[ref_idx];
        return 1;
      }
    }
  }
#else
  int is_same_ref = (neighbor_mbmi->ref_frame[0] == ref_frame);
  if (is_same_ref && is_warp_mode(neighbor_mbmi->motion_mode) &&
      !neighbor_mbmi->wm_params[0].invalid && neighbor_params) {
    *neighbor_params = neighbor_mbmi->wm_params[0];
    return 1;
  }
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
  return 0;
}

// Insert the candidate warp parameters to the WRL
void insert_neighbor_warp_candidate(
    WARP_CANDIDATE warp_candidates[MAX_WARP_REF_CANDIDATES],
    const WarpedMotionParams *neigh_params, uint8_t curr_num_of_candidates,
    const WarpProjectionType proj_type) {
  if (neigh_params)
    warp_candidates[curr_num_of_candidates].wm_params = *neigh_params;
  warp_candidates[curr_num_of_candidates].proj_type = proj_type;
}

// Check if the candidate warp parameters are already in the list or not.
static int is_this_param_already_in_list(
    const uint8_t curr_num_of_candidates,
    WARP_CANDIDATE warp_candidates[MAX_WARP_REF_CANDIDATES],
    WarpedMotionParams neigh_params) {
  for (int i = 0; i < curr_num_of_candidates; i++) {
    int same_param =
        (neigh_params.wmmat[2] == warp_candidates[i].wm_params.wmmat[2]);
    same_param &=
        (neigh_params.wmmat[3] == warp_candidates[i].wm_params.wmmat[3]);
    same_param &=
        (neigh_params.wmmat[4] == warp_candidates[i].wm_params.wmmat[4]);
    same_param &=
        (neigh_params.wmmat[5] == warp_candidates[i].wm_params.wmmat[5]);
#if CONFIG_EXTENDED_WARP_PREDICTION
    // The translational part is used for WARPMV mode
    same_param &=
        (neigh_params.wmmat[0] == warp_candidates[i].wm_params.wmmat[0]);
    same_param &=
        (neigh_params.wmmat[1] == warp_candidates[i].wm_params.wmmat[1]);
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
    if (same_param) return 1;
  }

  return 0;
}

void check_this_warp_candidate(
    const AV1_COMMON *cm, const MB_MODE_INFO *const neighbor_mbmi,
    WARP_CANDIDATE warp_candidates[MAX_WARP_REF_CANDIDATES],
    const int ref_frame, const int max_num_of_candidates,
    uint8_t *curr_num_of_candidates, const WarpProjectionType proj_type) {
  if (!is_inter_block(neighbor_mbmi, SHARED_PART)) return;
#if CONFIG_IBC_SR_EXT
  if (is_intrabc_block(neighbor_mbmi, SHARED_PART)) return;
#endif  // CONFIG_IBC_SR_EXT

  WarpedMotionParams neigh_params;
  if (*curr_num_of_candidates < max_num_of_candidates &&
      is_valid_warp_parameters(cm, neighbor_mbmi, ref_frame, &neigh_params)) {
    if (!is_this_param_already_in_list(*curr_num_of_candidates, warp_candidates,
                                       neigh_params)) {
      insert_neighbor_warp_candidate(warp_candidates, &neigh_params,
                                     *curr_num_of_candidates, proj_type);
      ++(*curr_num_of_candidates);
    }
  }
}
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

// when CONFIG_MVP_IMPROVEMENT is ture, scan_row_mbmi does not called
#if !CONFIG_MVP_IMPROVEMENT
static AOM_INLINE void scan_row_mbmi(
    const AV1_COMMON *cm, const MACROBLOCKD *xd, int mi_row, int mi_col,
    const MV_REFERENCE_FRAME rf[2], int row_offset, CANDIDATE_MV *ref_mv_stack,
    uint16_t *ref_mv_weight, uint8_t *refmv_count, uint8_t *ref_match_count,
    uint8_t *newmv_count, int_mv *gm_mv_candidates, int max_row_offset,
#if CONFIG_SKIP_MODE_ENHANCEMENT
    MV_REFERENCE_FRAME *ref_frame_idx0, MV_REFERENCE_FRAME *ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
    int add_more_mvs, SINGLE_MV_CANDIDATE *single_mv, uint8_t *single_mv_count,
    CANDIDATE_MV *derived_mv_stack, uint16_t *derived_mv_weight,
    uint8_t *derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
    WARP_CANDIDATE warp_param_stack[MAX_WARP_REF_CANDIDATES],
    int max_num_of_warp_candidates, uint8_t *valid_num_warp_candidates,
    MV_REFERENCE_FRAME ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

    int *processed_rows) {
  int end_mi = AOMMIN(xd->width, cm->mi_params.mi_cols - mi_col);
  end_mi = AOMMIN(end_mi, mi_size_wide[BLOCK_64X64]);
  const int width_8x8 = mi_size_wide[BLOCK_8X8];
  const int width_16x16 = mi_size_wide[BLOCK_16X16];
  MvSubpelPrecision precision = cm->features.fr_mv_precision;
  int col_offset = 0;
  // TODO(jingning): Revisit this part after cb4x4 is stable.
  if (abs(row_offset) > 1) {
    col_offset = 1;
    if ((mi_col & 0x01) && xd->width < width_8x8) --col_offset;
  }
  const int use_step_16 = (xd->width >= 16);
  MB_MODE_INFO **const candidate_mi0 = xd->mi + row_offset * xd->mi_stride;
#if CONFIG_C071_SUBBLK_WARPMV
  SUBMB_INFO **const submi_mi0 = xd->submi + row_offset * xd->mi_stride;
#endif  // CONFIG_C071_SUBBLK_WARPMV
  const int plane_type = (xd->tree_type == CHROMA_PART);
  for (int i = 0; i < end_mi;) {
#if CONFIG_EXT_RECUR_PARTITIONS
    if (xd->mi_col + col_offset + i >= cm->mi_params.mi_cols) break;
    const int sb_mi_size = mi_size_wide[cm->sb_size];
    const int mask_row = mi_row & (sb_mi_size - 1);
    const int mask_col = mi_col & (sb_mi_size - 1);
    const int ref_mask_row = mask_row + row_offset;
    const int ref_mask_col = mask_col + col_offset + i;
    if (ref_mask_row >= 0) {
      if (ref_mask_col >= sb_mi_size) break;

      const int ref_offset =
          ref_mask_row * xd->is_mi_coded_stride + ref_mask_col;
      if (!xd->is_mi_coded[0][ref_offset]) break;
    }
#endif  // CONFIG_EXT_RECUR_PARTITIONS
    const MB_MODE_INFO *const candidate = candidate_mi0[col_offset + i];
    assert(candidate != NULL);
#if CONFIG_C071_SUBBLK_WARPMV
    const SUBMB_INFO *const submi = submi_mi0[col_offset + i];
#endif  // CONFIG_C071_SUBBLK_WARPMV
    const int candidate_bsize = candidate->sb_type[plane_type];
    const int n4_w = mi_size_wide[candidate_bsize];
    int len = AOMMIN(xd->width, n4_w);
    if (use_step_16)
      len = AOMMAX(width_16x16, len);
    else if (abs(row_offset) > 1)
      len = AOMMAX(len, width_8x8);

#if CONFIG_MVP_IMPROVEMENT
    // Don't add weight to row_offset < -1 which is in the outer area
    uint16_t weight =
        row_offset < -1 ? OTHER_SMVP_WEIGHT : ADJACENT_SMVP_WEIGHT;
#else
    uint16_t weight = 2;
#endif
    if (xd->width >= width_8x8 && xd->width <= n4_w) {
      uint16_t inc = AOMMIN(-max_row_offset + row_offset + 1,
                            mi_size_high[candidate_bsize]);
#if !CONFIG_MVP_IMPROVEMENT
      // Obtain range used in weight calculation.
      weight = AOMMAX(weight, inc);
#endif
      // Update processed rows.
      *processed_rows = inc - row_offset - 1;
    }

    const int cand_mi_row = xd->mi_row + row_offset;
    const int cand_mi_col = xd->mi_col + col_offset + i;

#if CONFIG_EXTENDED_WARP_PREDICTION
    if (warp_param_stack && valid_num_warp_candidates &&
        max_num_of_warp_candidates) {
      check_this_warp_candidate(cm, candidate, warp_param_stack, ref_frame,
                                max_num_of_warp_candidates,
                                valid_num_warp_candidates, PROJ_SPATIAL);
    }
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

    add_ref_mv_candidate(
#if !CONFIG_MVP_IMPROVEMENT
        cm,
#endif  // !CONFIG_MVP_IMPROVEMENT
        mi_row, mi_col, cand_mi_row, cand_mi_col, candidate,
#if CONFIG_C071_SUBBLK_WARPMV
        submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
        rf, refmv_count, ref_match_count, newmv_count, ref_mv_stack,
        ref_mv_weight, gm_mv_candidates, cm->global_motion,
#if CONFIG_SKIP_MODE_ENHANCEMENT
        xd->mi[0], ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
        cm, add_more_mvs, single_mv, single_mv_count, derived_mv_stack,
        derived_mv_weight, derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_IBC_SR_EXT
        xd->mi[0]->use_intrabc[xd->tree_type == CHROMA_PART],
#endif  // CONFIG_IBC_SR_EXT
#if CONFIG_EXTENDED_WARP_PREDICTION
        row_offset, col_offset + i,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
        len * weight, precision);

    i += len;
  }
}
#endif  // !CONFIG_MVP_IMPROVEMENT

#if CONFIG_MVP_IMPROVEMENT
// update processed_cols variable, when scan_col_mbmi() is not used for adjacent
// neigbhors
static AOM_INLINE void update_processed_cols(const MACROBLOCKD *xd, int mi_row,
                                             int mi_col, int row_offset,
                                             int col_offset, int max_col_offset,
                                             int *processed_cols) {
  const TileInfo *const tile = &xd->tile;
  const POSITION mi_pos = { row_offset, col_offset };
  if (is_inside(tile, mi_col, mi_row, &mi_pos)) {
    const MB_MODE_INFO *const candidate =
        xd->mi[row_offset * xd->mi_stride + col_offset];
    const int n8_h_8 = mi_size_high[BLOCK_8X8];
    const int candidate_bsize =
        candidate->sb_type[xd->tree_type == CHROMA_PART];
    const int n4_h = mi_size_high[candidate_bsize];
    if (xd->height >= n8_h_8 && xd->height <= n4_h) {
      const int inc = AOMMIN(-max_col_offset + col_offset + 1,
                             mi_size_wide[candidate_bsize]);
      // Update processed cols.
      *processed_cols = inc - col_offset - 1;
    }
  }
}
#endif  // CONFIG_MVP_IMPROVEMENT

#if !CONFIG_MVP_SIMPLIFY
static AOM_INLINE void scan_col_mbmi(
    const AV1_COMMON *cm, const MACROBLOCKD *xd, int mi_row, int mi_col,
    const MV_REFERENCE_FRAME rf[2], int col_offset, CANDIDATE_MV *ref_mv_stack,
    uint16_t *ref_mv_weight, uint8_t *refmv_count, uint8_t *ref_match_count,
    uint8_t *newmv_count, int_mv *gm_mv_candidates, int max_col_offset,
#if CONFIG_SKIP_MODE_ENHANCEMENT
    MV_REFERENCE_FRAME *ref_frame_idx0, MV_REFERENCE_FRAME *ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
    int add_more_mvs, SINGLE_MV_CANDIDATE *single_mv, uint8_t *single_mv_count,
    CANDIDATE_MV *derived_mv_stack, uint16_t *derived_mv_weight,
    uint8_t *derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
    WARP_CANDIDATE warp_param_stack[MAX_WARP_REF_CANDIDATES],
    int max_num_of_warp_candidates, uint8_t *valid_num_warp_candidates,
    MV_REFERENCE_FRAME ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
    int *processed_cols) {
  int end_mi = AOMMIN(xd->height, cm->mi_params.mi_rows - mi_row);
  end_mi = AOMMIN(end_mi, mi_size_high[BLOCK_64X64]);
  const int n8_h_8 = mi_size_high[BLOCK_8X8];
  const int n8_h_16 = mi_size_high[BLOCK_16X16];
  int i;
  MvSubpelPrecision precision = cm->features.fr_mv_precision;
  int row_offset = 0;
  if (abs(col_offset) > 1) {
    row_offset = 1;
    if ((mi_row & 0x01) && xd->height < n8_h_8) --row_offset;
  }
  const int use_step_16 = (xd->height >= 16);

  for (i = 0; i < end_mi;) {
#if CONFIG_EXT_RECUR_PARTITIONS
    if (xd->mi_row + row_offset + i >= cm->mi_params.mi_rows) break;
    const int sb_mi_size = mi_size_wide[cm->sb_size];
    const int mask_row = mi_row & (sb_mi_size - 1);
    const int mask_col = mi_col & (sb_mi_size - 1);
    const int ref_mask_row = mask_row + row_offset + i;
    const int ref_mask_col = mask_col + col_offset;
    if (ref_mask_col >= 0) {
      if (ref_mask_row >= sb_mi_size) break;
      const int ref_offset =
          ref_mask_row * xd->is_mi_coded_stride + ref_mask_col;
      if (!xd->is_mi_coded[0][ref_offset]) break;
    }
#endif  // CONFIG_EXT_RECUR_PARTITIONS
    const MB_MODE_INFO *const candidate =
        xd->mi[(row_offset + i) * xd->mi_stride + col_offset];
    assert(candidate != NULL);
#if CONFIG_C071_SUBBLK_WARPMV
    const SUBMB_INFO *const submi =
        xd->submi[(row_offset + i) * xd->mi_stride + col_offset];
#endif  // CONFIG_C071_SUBBLK_WARPMV
    const int candidate_bsize =
        candidate->sb_type[xd->tree_type == CHROMA_PART];
    const int n4_h = mi_size_high[candidate_bsize];
    int len = AOMMIN(xd->height, n4_h);
    if (use_step_16)
      len = AOMMAX(n8_h_16, len);
    else if (abs(col_offset) > 1)
      len = AOMMAX(len, n8_h_8);

#if CONFIG_MVP_IMPROVEMENT
    // Don't add weight to col_offset < -1 which is in the outer area
    uint16_t weight =
        col_offset < -1 ? OTHER_SMVP_WEIGHT : ADJACENT_SMVP_WEIGHT;
#else
    int weight = 2;
#endif
    if (xd->height >= n8_h_8 && xd->height <= n4_h) {
      int inc = AOMMIN(-max_col_offset + col_offset + 1,
                       mi_size_wide[candidate_bsize]);
#if !CONFIG_MVP_IMPROVEMENT
      // Obtain range used in weight calculation.
      weight = AOMMAX(weight, inc);
#endif
      // Update processed cols.
      *processed_cols = inc - col_offset - 1;
    }

    const int cand_mi_row = xd->mi_row + row_offset + i;
    const int cand_mi_col = xd->mi_col + col_offset;

#if CONFIG_EXTENDED_WARP_PREDICTION
    if (warp_param_stack && valid_num_warp_candidates &&
        max_num_of_warp_candidates && (col_offset == -1)) {
      check_this_warp_candidate(cm, candidate, warp_param_stack, ref_frame,
                                max_num_of_warp_candidates,
                                valid_num_warp_candidates, PROJ_SPATIAL);
    }
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

    add_ref_mv_candidate(
#if !CONFIG_MVP_IMPROVEMENT
        cm,
#endif  // !CONFIG_MVP_IMPROVEMENT
        mi_row, mi_col, cand_mi_row, cand_mi_col, candidate,
#if CONFIG_C071_SUBBLK_WARPMV
        submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
        rf, refmv_count, ref_match_count, newmv_count, ref_mv_stack,
        ref_mv_weight, gm_mv_candidates, cm->global_motion,
#if CONFIG_SKIP_MODE_ENHANCEMENT
        xd->mi[0], ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
        cm, add_more_mvs, single_mv, single_mv_count, derived_mv_stack,
        derived_mv_weight, derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_IBC_SR_EXT
        xd->mi[0]->use_intrabc[xd->tree_type == CHROMA_PART],
#endif  // CONFIG_IBC_SR_EXT
#if CONFIG_EXTENDED_WARP_PREDICTION
        row_offset + i, col_offset,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
        len * weight, precision

    );
    i += len;
  }
}
#endif  // !CONFIG_MVP_SIMPLIFY

#if CONFIG_C076_INTER_MOD_CTX
static AOM_INLINE void scan_blk_mbmi_ctx(
    const AV1_COMMON *cm, const MACROBLOCKD *xd, const int mi_row,
    const int mi_col, const MV_REFERENCE_FRAME rf[2], int row_offset,
    int col_offset, uint8_t *ref_match_count, uint8_t *newmv_count) {
  const TileInfo *const tile = &xd->tile;
  POSITION mi_pos;

  mi_pos.row = row_offset;
  mi_pos.col = col_offset;
  if (is_inside(tile, mi_col, mi_row, &mi_pos)) {
    const MB_MODE_INFO *const candidate =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    add_ref_mv_candidate_ctx(candidate, ref_match_count, newmv_count, cm, rf,
                             xd->mi[0]);
  }
}
#endif  // CONFIG_C076_INTER_MOD_CTX

static AOM_INLINE void scan_blk_mbmi(
    const AV1_COMMON *cm, const MACROBLOCKD *xd, const int mi_row,
    const int mi_col, const MV_REFERENCE_FRAME rf[2], int row_offset,
    int col_offset, CANDIDATE_MV *ref_mv_stack, uint16_t *ref_mv_weight,
    uint8_t *ref_match_count, uint8_t *newmv_count, int_mv *gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
    MV_REFERENCE_FRAME *ref_frame_idx0, MV_REFERENCE_FRAME *ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
    int add_more_mvs, SINGLE_MV_CANDIDATE *single_mv, uint8_t *single_mv_count,
    CANDIDATE_MV *derived_mv_stack, uint16_t *derived_mv_weight,
    uint8_t *derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
    WARP_CANDIDATE warp_param_stack[MAX_WARP_REF_CANDIDATES],
    int max_num_of_warp_candidates, uint8_t *valid_num_warp_candidates,
    MV_REFERENCE_FRAME ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
    uint8_t *refmv_count) {

#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  if (*refmv_count >= MAX_REF_MV_STACK_SIZE) return;
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY

  const TileInfo *const tile = &xd->tile;
  POSITION mi_pos;

  mi_pos.row = row_offset;
  mi_pos.col = col_offset;
  MvSubpelPrecision precision = cm->features.fr_mv_precision;

  if (is_inside(tile, mi_col, mi_row, &mi_pos)) {
    const MB_MODE_INFO *const candidate =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
#if CONFIG_C071_SUBBLK_WARPMV
    const SUBMB_INFO *const submi =
        xd->submi[mi_pos.row * xd->mi_stride + mi_pos.col];
#endif  // CONFIG_C071_SUBBLK_WARPMV
#if !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
    const int len = mi_size_wide[BLOCK_8X8];
#endif  // !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY

#if CONFIG_MVP_IMPROVEMENT
    uint16_t weight = ADJACENT_SMVP_WEIGHT;
    // Don't add weight to (-1,-1) which is in the outer area
    if (row_offset == -1 && col_offset == -1) {
      weight = OTHER_SMVP_WEIGHT;
    }
#if CONFIG_MVP_SIMPLIFY
    // Don't add weight to col_offset < -1 which is in the outer area
    if (col_offset < -1) {
      weight = OTHER_SMVP_WEIGHT;
    }
#endif  // CONFIG_MVP_SIMPLIFY
#endif  // CONFIG_MVP_IMPROVEMENT

    const int cand_mi_row = xd->mi_row + mi_pos.row;
    const int cand_mi_col = xd->mi_col + mi_pos.col;

#if CONFIG_EXTENDED_WARP_PREDICTION
    if (warp_param_stack && valid_num_warp_candidates &&
        max_num_of_warp_candidates) {
      check_this_warp_candidate(cm, candidate, warp_param_stack, ref_frame,
                                max_num_of_warp_candidates,
                                valid_num_warp_candidates, PROJ_SPATIAL);
    }
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

    add_ref_mv_candidate(
#if !CONFIG_MVP_IMPROVEMENT
        cm,
#endif  // !CONFIG_MVP_IMPROVEMENT
        mi_row, mi_col, cand_mi_row, cand_mi_col, candidate,
#if CONFIG_C071_SUBBLK_WARPMV
        submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
        rf, refmv_count, ref_match_count, newmv_count, ref_mv_stack,
        ref_mv_weight, gm_mv_candidates, cm->global_motion,
#if CONFIG_SKIP_MODE_ENHANCEMENT
        xd->mi[0], ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
        cm, add_more_mvs, single_mv, single_mv_count, derived_mv_stack,
        derived_mv_weight, derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_IBC_SR_EXT
        xd->mi[0]->use_intrabc[xd->tree_type == CHROMA_PART],
#endif  // CONFIG_IBC_SR_EXT
#if CONFIG_EXTENDED_WARP_PREDICTION
        row_offset, col_offset,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
        weight
#else
#if CONFIG_MVP_IMPROVEMENT
        weight * len
#else
        2 * len
#endif
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
        ,
        precision);
  }  // Analyze a single 8x8 block motion information.
}

#if CONFIG_EXT_RECUR_PARTITIONS
static int has_top_right(const AV1_COMMON *cm, const MACROBLOCKD *xd,
                         int mi_row, int mi_col, int n4_w) {
  const int sb_mi_size = mi_size_wide[cm->sb_size];
  const int mask_row = mi_row & (sb_mi_size - 1);
  const int mask_col = mi_col & (sb_mi_size - 1);

  if (n4_w > mi_size_wide[BLOCK_64X64]) return 0;

  const int tr_mask_row = mask_row - 1;
  const int tr_mask_col = mask_col + n4_w;
  int has_tr;

  if (tr_mask_row < 0) {
    // The top-right block is in a superblock above the current sb row. If it is
    // in the current tile or a previously coded one, it has been coded.
    // Otherwise later the tile boundary checker will figure out whether it is
    // available.
    has_tr = 1;
  } else if (tr_mask_col >= sb_mi_size) {
    // The top-right block is in the superblock on the right side, therefore it
    // is not coded yet.
    has_tr = 0;
  } else {
    // For a general case, we use is_mi_coded array for the current superblock
    // to figure out the availability.
    const int tr_offset = tr_mask_row * xd->is_mi_coded_stride + tr_mask_col;

    has_tr = xd->is_mi_coded[av1_get_sdp_idx(xd->tree_type)][tr_offset];
  }

  return has_tr;
}

#if CONFIG_MVP_IMPROVEMENT || CONFIG_EXTENDED_WARP_PREDICTION
static int has_bottom_left(const AV1_COMMON *cm, const MACROBLOCKD *xd,
                           int mi_row, int mi_col, int n4_h) {
  const int sb_mi_size = mi_size_wide[cm->sb_size];
  const int mask_row = mi_row & (sb_mi_size - 1);
  const int mask_col = mi_col & (sb_mi_size - 1);

  if (n4_h > mi_size_high[BLOCK_64X64]) return 0;

  const int bl_mask_row = mask_row + n4_h;
  const int bl_mask_col = mask_col - 1;

  if (bl_mask_row >= sb_mi_size) {
    // If the bottom right block is in the superblock row below, then it's not
    // ready yet
    // TODO(chiyotsai): Take care of tile boundary
    return 0;
  } else if (bl_mask_col < 0) {
    // The bottom-left block is in a superblock left of the current sb and it is
    // in the same sb row.  If it in the same tile, then it has been coded.
    // Otherwise, boundary check will figure out when it's available
    return 1;
  } else {
    // For a general case, we use is_mi_coded array for the current superblock
    // to figure out the availability.
    const int bl_offset = bl_mask_row * xd->is_mi_coded_stride + bl_mask_col;

    return xd->is_mi_coded[av1_get_sdp_idx(xd->tree_type)][bl_offset];
  }
}
#endif  // CONFIG_MVP_IMPROVEMENT || CONFIG_EXTENDED_WARP_PREDICTION
#else
static int has_top_right(const AV1_COMMON *cm, const MACROBLOCKD *xd,
                         int mi_row, int mi_col, int bs) {
  const int sb_mi_size = mi_size_wide[cm->sb_size];
  const int mask_row = mi_row & (sb_mi_size - 1);
  const int mask_col = mi_col & (sb_mi_size - 1);

#if !CONFIG_MVP_IMPROVEMENT
  if (bs > mi_size_wide[BLOCK_64X64]) return 0;
#endif  // !CONFIG_MVP_IMPROVEMENT

  // In a split partition all apart from the bottom right has a top right
  int has_tr = !((mask_row & bs) && (mask_col & bs));

  // bs > 0 and bs is a power of 2
  assert(bs > 0 && !(bs & (bs - 1)));

  // For each 4x4 group of blocks, when the bottom right is decoded the blocks
  // to the right have not been decoded therefore the bottom right does
  // not have a top right
  while (bs < sb_mi_size) {
    if (mask_col & bs) {
      if ((mask_col & (2 * bs)) && (mask_row & (2 * bs))) {
        has_tr = 0;
        break;
      }
    } else {
      break;
    }
    bs <<= 1;
  }

  // In a VERTICAL or VERTICAL_4 partition, all partition before the last one
  // always have a top right (as the block above will have been decoded).
  if (xd->width < xd->height) {
    if (!xd->is_last_vertical_rect) has_tr = 1;
  }

  // In a HORIZONTAL or HORIZONTAL_4 partition, partitions after the first one
  // never have a top right (as the block to the right won't have been decoded).
  if (xd->width > xd->height) {
    if (!xd->is_first_horizontal_rect) has_tr = 0;
  }

  // The bottom left square of a Vertical A (in the old format) does
  // not have a top right as it is decoded before the right hand
  // rectangle of the partition
  if (xd->mi[0]->partition == PARTITION_VERT_A) {
    if (xd->width == xd->height)
      if (mask_row & bs) has_tr = 0;
  }

  return has_tr;
}

#if CONFIG_MVP_IMPROVEMENT
static int has_bottom_left(const AV1_COMMON *cm, const MACROBLOCKD *xd,
                           int mi_row, int mi_col, int bs) {
  const int sb_mi_size = mi_size_wide[cm->sb_size];
  const int mask_row = mi_row & (sb_mi_size - 1);
  const int mask_col = mi_col & (sb_mi_size - 1);

  // In a split partition, only top left subblock has a bottom right
  int has_bl = !((mask_row & bs) || (mask_col & bs));

  // bs lareger than 64x64 or equals to sb_size case not allowed
  if (bs > mi_size_wide[BLOCK_64X64]) has_bl = 0;
  if (bs == mi_size_wide[cm->sb_size]) has_bl = 0;

  // bs > 0 and bs is a power of 2
  assert(bs > 0 && !(bs & (bs - 1)));

  // For each 4x4 group of blocks, when the tob left is decoded the blocks
  // to the left have been decoded therefore the top left does
  // have a bottom left
  while (bs < sb_mi_size) {
    if (!(mask_col & bs)) {
      if (2 * bs == sb_mi_size) break;
      if (!(mask_col & (2 * bs)) && !(mask_row & (2 * bs))) {
        has_bl = 1;
        break;
      }
    } else {
      break;
    }
    bs <<= 1;
  }

  // In a VERTICAL or VERTICAL_4 partition, all partition after the first one
  // never have a bottom left (as the block to the left won't have been
  // decoded).
  if (xd->width < xd->height) {
    if (!xd->is_first_vertical_rect) has_bl = 0;
  }

  // In a HORIZONTAL or HORIZONTAL_4 partition, partitions before the last one
  // always have a bottom left (as the block above will have been decoded).
  if (xd->width > xd->height) {
    if (!xd->is_last_horizontal_rect) has_bl = 1;
  }

  // The bottom left square of a Vertical B (in the old format) does
  // have a bottom left as it is decoded after the left hand
  // rectangle of the partition
  if (xd->mi[0]->partition == PARTITION_VERT_B) {
    if (xd->width == xd->height)
      if (!(mask_row & bs)) has_bl = 1;
  }

  return has_bl;
}
#endif  // CONFIG_MVP_IMPROVEMENT
#endif  // CONFIG_EXT_RECUR_PARTITIONS

#if !CONFIG_MVP_IMPROVEMENT
static int check_sb_border(const int mi_row, const int mi_col,
                           const int row_offset, const int col_offset) {
  const int sb_mi_size = mi_size_wide[BLOCK_64X64];
  const int row = mi_row & (sb_mi_size - 1);
  const int col = mi_col & (sb_mi_size - 1);

  if (row + row_offset < 0 || row + row_offset >= sb_mi_size ||
      col + col_offset < 0 || col + col_offset >= sb_mi_size)
    return 0;

  return 1;
}
#endif  // !CONFIG_MVP_IMPROVEMENT

static int add_tpl_ref_mv(const AV1_COMMON *cm, const MACROBLOCKD *xd,
                          int mi_row, int mi_col, MV_REFERENCE_FRAME ref_frame,
                          int blk_row, int blk_col
#if !CONFIG_C076_INTER_MOD_CTX
                          ,
                          int_mv *gm_mv_candidates
#endif  // !CONFIG_C076_INTER_MOD_CTX
                          ,
                          uint8_t *const refmv_count,
#if CONFIG_MVP_IMPROVEMENT
                          int *added_tmvp_cnt,
#endif  // CONFIG_MVP_IMPROVEMENT
                          CANDIDATE_MV ref_mv_stack[MAX_REF_MV_STACK_SIZE],
                          uint16_t ref_mv_weight[MAX_REF_MV_STACK_SIZE]
#if CONFIG_SKIP_MODE_ENHANCEMENT
                          ,
                          MV_REFERENCE_FRAME *ref_frame_idx0,
                          MV_REFERENCE_FRAME *ref_frame_idx1
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if !CONFIG_C076_INTER_MOD_CTX
                          ,
                          int16_t *mode_context
#endif  // !CONFIG_C076_INTER_MOD_CTX
) {
#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  if (*refmv_count >= MAX_REF_MV_STACK_SIZE) return 0;
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY

  POSITION mi_pos;
  mi_pos.row = (mi_row & 0x01) ? blk_row : blk_row + 1;
  mi_pos.col = (mi_col & 0x01) ? blk_col : blk_col + 1;

  if (!is_inside(&xd->tile, mi_col, mi_row, &mi_pos)) return 0;

  const int tpl_row = ((mi_row + mi_pos.row) >> TMVP_SHIFT_BITS);
  const int tpl_col = ((mi_col + mi_pos.col) >> TMVP_SHIFT_BITS);
  const int tpl_stride =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  const TPL_MV_REF *prev_frame_mvs =
      cm->tpl_mvs + tpl_row * tpl_stride + tpl_col;

#if !CONFIG_MV_TRAJECTORY
  if (prev_frame_mvs->mfmv0.as_int == INVALID_MV) return 0;
#endif  // !CONFIG_MV_TRAJECTORY

  MV_REFERENCE_FRAME rf[2];
  av1_set_ref_frame(rf, ref_frame);

  if (is_tip_ref_frame(rf[0])) {
    return 0;
  }

#if CONFIG_MV_TRAJECTORY
  const int tpl_mv_offset = tpl_row * tpl_stride + tpl_col;
  bool linear_available = prev_frame_mvs->mfmv0.as_int != INVALID_MV;
  bool mvtj_available[2] = { true, true };
  if (rf[0] == NONE_FRAME || rf[0] >= cm->ref_frames_info.num_total_refs ||
      cm->id_offset_map[rf[0]][tpl_mv_offset].as_int == INVALID_MV) {
    mvtj_available[0] = false;
  }
  if (rf[1] == NONE_FRAME || rf[1] >= cm->ref_frames_info.num_total_refs ||
      cm->id_offset_map[rf[1]][tpl_mv_offset].as_int == INVALID_MV) {
    mvtj_available[1] = false;
  }
  bool mvtj_ok = rf[1] == NONE_FRAME ? mvtj_available[0]
                                     : mvtj_available[0] && mvtj_available[1];
  if (!linear_available && !mvtj_ok) return 0;
#endif  // CONFIG_MV_TRAJECTORY

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int cur_frame_index = cm->cur_frame->display_order_hint;
#else
  const int cur_frame_index = cm->cur_frame->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const RefCntBuffer *const buf_0 = get_ref_frame_buf(cm, rf[0]);
#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int frame0_index = buf_0->display_order_hint;
#else
  const int frame0_index = buf_0->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int cur_offset_0 = get_relative_dist(&cm->seq_params.order_hint_info,
                                             cur_frame_index, frame0_index);
  int idx;
#if !CONFIG_C071_SUBBLK_WARPMV
  const MvSubpelPrecision fr_mv_precision = cm->features.fr_mv_precision;
#endif  // !CONFIG_C071_SUBBLK_WARPMV

  int_mv this_refmv;
#if CONFIG_MV_TRAJECTORY
  if (mvtj_available[0]) {
    this_refmv = cm->id_offset_map[rf[0]][tpl_mv_offset];
  } else {
    assert(linear_available);
    get_mv_projection(&this_refmv.as_mv, prev_frame_mvs->mfmv0.as_mv,
                      cur_offset_0, prev_frame_mvs->ref_frame_offset);
  }
#else
  get_mv_projection(&this_refmv.as_mv, prev_frame_mvs->mfmv0.as_mv,
                    cur_offset_0, prev_frame_mvs->ref_frame_offset);
#endif  // CONFIG_MV_TRAJECTORY
#if !CONFIG_C071_SUBBLK_WARPMV
  lower_mv_precision(&this_refmv.as_mv, fr_mv_precision);
#endif  // !CONFIG_C071_SUBBLK_WARPMV

  uint16_t weight = TMVP_WEIGHT;

  if (rf[1] == NONE_FRAME) {
#if CONFIG_TMVP_IMPROVE
    if (abs(cur_offset_0) <= 2) {
      weight = HIGH_PRIORITY_TMVP_WEIGHT;
    }
#endif  // CONFIG_TMVP_IMPROVE

#if !CONFIG_C076_INTER_MOD_CTX
    if (blk_row == 0 && blk_col == 0) {
      if (abs(this_refmv.as_mv.row - gm_mv_candidates[0].as_mv.row) >= 16 ||
          abs(this_refmv.as_mv.col - gm_mv_candidates[0].as_mv.col) >= 16)
        mode_context[ref_frame] |= (1 << GLOBALMV_OFFSET);
    }
#endif  // !CONFIG_C076_INTER_MOD_CTX

#if CONFIG_SKIP_MODE_ENHANCEMENT
    assert(!xd->mi[0]->skip_mode);
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT

    for (idx = 0; idx < *refmv_count; ++idx)
      if (this_refmv.as_int == ref_mv_stack[idx].this_mv.as_int) break;

    if (idx < *refmv_count) ref_mv_weight[idx] += weight;

    if (idx == *refmv_count && *refmv_count < MAX_REF_MV_STACK_SIZE) {
      ref_mv_stack[idx].this_mv.as_int = this_refmv.as_int;
#if CONFIG_EXTENDED_WARP_PREDICTION
      ref_mv_stack[idx].row_offset = OFFSET_NONSPATIAL;
      ref_mv_stack[idx].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
      ref_mv_stack[idx].cwp_idx = CWP_EQUAL;
      ref_mv_weight[idx] = weight;
      ++(*refmv_count);
#if CONFIG_MVP_IMPROVEMENT
      ++(*added_tmvp_cnt);
#endif  // CONFIG_MVP_IMPROVEMENT
    }
  } else {
    // Process compound inter mode
    int_mv comp_refmv;
#if CONFIG_MV_TRAJECTORY
    if (mvtj_available[1]) {
      comp_refmv = cm->id_offset_map[rf[1]][tpl_mv_offset];
    } else {
      assert(linear_available);
      const RefCntBuffer *const buf_1 = get_ref_frame_buf(cm, rf[1]);
#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
      const int frame1_index = buf_1->display_order_hint;
#else
      const int frame1_index = buf_1->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
      const int cur_offset_1 = get_relative_dist(
          &cm->seq_params.order_hint_info, cur_frame_index, frame1_index);
      get_mv_projection(&comp_refmv.as_mv, prev_frame_mvs->mfmv0.as_mv,
                        cur_offset_1, prev_frame_mvs->ref_frame_offset);
    }
#else
    const RefCntBuffer *const buf_1 = get_ref_frame_buf(cm, rf[1]);
#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    const int frame1_index = buf_1->display_order_hint;
#else
    const int frame1_index = buf_1->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    const int cur_offset_1 = get_relative_dist(&cm->seq_params.order_hint_info,
                                               cur_frame_index, frame1_index);
    get_mv_projection(&comp_refmv.as_mv, prev_frame_mvs->mfmv0.as_mv,
                      cur_offset_1, prev_frame_mvs->ref_frame_offset);
#endif  // CONFIG_MV_TRAJECTORY
#if !CONFIG_C071_SUBBLK_WARPMV
    lower_mv_precision(&comp_refmv.as_mv, fr_mv_precision);
#endif  // !CONFIG_C071_SUBBLK_WARPMV

#if !CONFIG_C076_INTER_MOD_CTX
    if (blk_row == 0 && blk_col == 0) {
      if (abs(this_refmv.as_mv.row - gm_mv_candidates[0].as_mv.row) >= 16 ||
          abs(this_refmv.as_mv.col - gm_mv_candidates[0].as_mv.col) >= 16 ||
          abs(comp_refmv.as_mv.row - gm_mv_candidates[1].as_mv.row) >= 16 ||
          abs(comp_refmv.as_mv.col - gm_mv_candidates[1].as_mv.col) >= 16)
        mode_context[ref_frame] |= (1 << GLOBALMV_OFFSET);
    }
#endif  // !CONFIG_C076_INTER_MOD_CTX

#if CONFIG_SKIP_MODE_ENHANCEMENT
    if (xd->mi[0]->skip_mode) {
      for (idx = 0; idx < *refmv_count; ++idx) {
        if (this_refmv.as_int == ref_mv_stack[idx].this_mv.as_int &&
            comp_refmv.as_int == ref_mv_stack[idx].comp_mv.as_int &&
            ref_frame_idx0[idx] == rf[0] && ref_frame_idx1[idx] == rf[1])
          break;
      }

      if (idx < *refmv_count) ref_mv_weight[idx] += weight;

      if (idx == *refmv_count && *refmv_count < MAX_REF_MV_STACK_SIZE) {
        ref_mv_stack[idx].this_mv.as_int = this_refmv.as_int;
        ref_mv_stack[idx].comp_mv.as_int = comp_refmv.as_int;
        ref_mv_stack[idx].cwp_idx = CWP_EQUAL;
        ref_frame_idx0[idx] = rf[0];
        ref_frame_idx1[idx] = rf[1];
        ref_mv_weight[idx] = weight;
        ++(*refmv_count);
#if CONFIG_MVP_IMPROVEMENT
        ++(*added_tmvp_cnt);
#endif  // CONFIG_MVP_IMPROVEMENT
      }
    } else {
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
      for (idx = 0; idx < *refmv_count; ++idx) {
        if (this_refmv.as_int == ref_mv_stack[idx].this_mv.as_int &&
            comp_refmv.as_int == ref_mv_stack[idx].comp_mv.as_int)
          break;
      }

      if (idx < *refmv_count) ref_mv_weight[idx] += weight;

      if (idx == *refmv_count && *refmv_count < MAX_REF_MV_STACK_SIZE) {
        ref_mv_stack[idx].this_mv.as_int = this_refmv.as_int;
        ref_mv_stack[idx].comp_mv.as_int = comp_refmv.as_int;

#if CONFIG_EXTENDED_WARP_PREDICTION
        ref_mv_stack[idx].row_offset = OFFSET_NONSPATIAL;
        ref_mv_stack[idx].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
        ref_mv_stack[idx].cwp_idx = CWP_EQUAL;
        ref_mv_weight[idx] = weight;
        ++(*refmv_count);
#if CONFIG_MVP_IMPROVEMENT
        ++(*added_tmvp_cnt);
#endif  // CONFIG_MVP_IMPROVEMENT
      }
#if CONFIG_SKIP_MODE_ENHANCEMENT
    }
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
  }

  return 1;
}

#if !CONFIG_MVP_SIMPLIFY
static AOM_INLINE void process_compound_ref_mv_candidate(
    const MB_MODE_INFO *const candidate,
#if CONFIG_C071_SUBBLK_WARPMV
    const SUBMB_INFO *const submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
    const AV1_COMMON *const cm, const MV_REFERENCE_FRAME *const rf,
    int_mv ref_id[2][2], int ref_id_count[2], int_mv ref_diff[2][2],
    int ref_diff_count[2]) {
  for (int rf_idx = 0; rf_idx < 2; ++rf_idx) {
    MV_REFERENCE_FRAME can_rf = candidate->ref_frame[rf_idx];

    for (int cmp_idx = 0; cmp_idx < 2; ++cmp_idx) {
      if (can_rf == rf[cmp_idx] && ref_id_count[cmp_idx] < 2) {
        ref_id[cmp_idx][ref_id_count[cmp_idx]] =
#if CONFIG_C071_SUBBLK_WARPMV
            is_warp_mode(candidate->motion_mode) ? submi->mv[rf_idx] :
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                                 candidate->mv[rf_idx];
        ++ref_id_count[cmp_idx];
      } else if (is_inter_ref_frame(can_rf) && !is_tip_ref_frame(can_rf) &&
                 ref_diff_count[cmp_idx] < 2) {
        int_mv this_mv =
#if CONFIG_C071_SUBBLK_WARPMV
            is_warp_mode(candidate->motion_mode) ? submi->mv[rf_idx] :
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                                 candidate->mv[rf_idx];
        if (cm->ref_frame_sign_bias[can_rf] !=
            cm->ref_frame_sign_bias[rf[cmp_idx]]) {
          this_mv.as_mv.row = -this_mv.as_mv.row;
          this_mv.as_mv.col = -this_mv.as_mv.col;
        }
        ref_diff[cmp_idx][ref_diff_count[cmp_idx]] = this_mv;
        ++ref_diff_count[cmp_idx];
      }
    }
  }
}

static AOM_INLINE void process_single_ref_mv_candidate(
    const MB_MODE_INFO *const candidate,
#if CONFIG_C071_SUBBLK_WARPMV
    const SUBMB_INFO *const submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
    const AV1_COMMON *const cm, MV_REFERENCE_FRAME ref_frame,
    uint8_t *const refmv_count,
    CANDIDATE_MV ref_mv_stack[MAX_REF_MV_STACK_SIZE],
    uint16_t ref_mv_weight[MAX_REF_MV_STACK_SIZE]) {

  if (is_tip_ref_frame(ref_frame)) return;

  for (int rf_idx = 0; rf_idx < 2; ++rf_idx) {
    if (is_inter_ref_frame(candidate->ref_frame[rf_idx]) &&
        !is_tip_ref_frame(candidate->ref_frame[rf_idx])) {
      int_mv this_mv =
#if CONFIG_C071_SUBBLK_WARPMV
          is_warp_mode(candidate->motion_mode) ? submi->mv[rf_idx] :
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                               candidate->mv[rf_idx];
      if (cm->ref_frame_sign_bias[candidate->ref_frame[rf_idx]] !=
          cm->ref_frame_sign_bias[ref_frame]) {
        this_mv.as_mv.row = -this_mv.as_mv.row;
        this_mv.as_mv.col = -this_mv.as_mv.col;
      }
      int stack_idx;
      for (stack_idx = 0; stack_idx < *refmv_count; ++stack_idx) {
        const int_mv stack_mv = ref_mv_stack[stack_idx].this_mv;
        if (this_mv.as_int == stack_mv.as_int) break;
      }

      if (stack_idx == *refmv_count) {
        ref_mv_stack[stack_idx].this_mv = this_mv;
#if CONFIG_EXTENDED_WARP_PREDICTION
        ref_mv_stack[stack_idx].row_offset = OFFSET_NONSPATIAL;
        ref_mv_stack[stack_idx].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
        ref_mv_stack[stack_idx].cwp_idx = candidate->cwp_idx;

        // TODO(jingning): Set an arbitrary small number here. The weight
        // doesn't matter as long as it is properly initialized.
#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
        ref_mv_weight[stack_idx] = 0;
#else
        ref_mv_weight[stack_idx] = 2;
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
        ++(*refmv_count);
        if (*refmv_count >= MAX_MV_REF_CANDIDATES) return;
      }
    }
  }
}
#endif  // !CONFIG_MVP_SIMPLIFY

static AOM_INLINE bool check_rmb_cand(
    CANDIDATE_MV cand_mv, CANDIDATE_MV *ref_mv_stack, uint16_t *ref_mv_weight,
    uint8_t *refmv_count, int is_comp, int mi_row, int mi_col, int block_width,
    int block_height, int frame_width, int frame_height) {
  // Check if the MV candidate is already existing in the ref MV stack.
  for (int i = 0; i < *refmv_count; ++i) {
    if (ref_mv_stack[i].this_mv.as_int == cand_mv.this_mv.as_int &&
        (!is_comp ||
         ref_mv_stack[i].comp_mv.as_int == cand_mv.comp_mv.as_int)) {
      return false;
    }
  }

  // Check if the MV candidate is pointing to ref block inside frame boundary.
  for (int i = 0; i < 1 + is_comp; ++i) {
    const int mv_row =
        (i ? cand_mv.comp_mv.as_mv.row : cand_mv.this_mv.as_mv.row) / 8;
    const int mv_col =
        (i ? cand_mv.comp_mv.as_mv.col : cand_mv.this_mv.as_mv.col) / 8;
    const int ref_x = mi_col * MI_SIZE + mv_col;
    const int ref_y = mi_row * MI_SIZE + mv_row;
    if (ref_x <= -block_width || ref_y <= -block_height ||
        ref_x >= frame_width || ref_y >= frame_height) {
      return false;
    }
  }

  ref_mv_stack[*refmv_count] = cand_mv;
  ref_mv_weight[*refmv_count] = REF_CAT_LEVEL;
#if CONFIG_EXTENDED_WARP_PREDICTION
  ref_mv_stack[*refmv_count].row_offset = OFFSET_NONSPATIAL;
  ref_mv_stack[*refmv_count].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
  ref_mv_stack[*refmv_count].cwp_idx = cand_mv.cwp_idx;
  ++*refmv_count;

  return true;
}

#if CONFIG_IBC_BV_IMPROVEMENT
// Add a BV candidate to ref MV stack without duplicate check
static AOM_INLINE bool add_to_ref_bv_list(CANDIDATE_MV cand_mv,
                                          CANDIDATE_MV *ref_mv_stack,
                                          uint16_t *ref_mv_weight,
                                          uint8_t *refmv_count) {
  ref_mv_stack[*refmv_count] = cand_mv;
  ref_mv_weight[*refmv_count] = REF_CAT_LEVEL;
  ref_mv_stack[*refmv_count].cwp_idx = cand_mv.cwp_idx;
  ++*refmv_count;

  return true;
}
#endif  // CONFIG_IBC_BV_IMPROVEMENT

static AOM_INLINE void setup_ref_mv_list(
    const AV1_COMMON *cm, const MACROBLOCKD *xd, MV_REFERENCE_FRAME ref_frame,
    uint8_t *const refmv_count,
    CANDIDATE_MV ref_mv_stack[MAX_REF_MV_STACK_SIZE],
    uint16_t ref_mv_weight[MAX_REF_MV_STACK_SIZE],
#if CONFIG_SKIP_MODE_ENHANCEMENT
    MV_REFERENCE_FRAME *ref_frame_idx0, MV_REFERENCE_FRAME *ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
    int_mv mv_ref_list[MAX_MV_REF_CANDIDATES], int_mv *gm_mv_candidates,
    int mi_row, int mi_col
#if !CONFIG_C076_INTER_MOD_CTX
    ,
    int16_t *mode_context
#endif  //! CONFIG_C076_INTER_MOD_CTX
#if CONFIG_EXTENDED_WARP_PREDICTION
    ,
    WARP_CANDIDATE warp_param_stack[MAX_WARP_REF_CANDIDATES],
    int max_num_of_warp_candidates, uint8_t *valid_num_warp_candidates
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

) {
#if CONFIG_EXT_RECUR_PARTITIONS
  const int has_tr = has_top_right(cm, xd, mi_row, mi_col, xd->width);
#if CONFIG_MVP_IMPROVEMENT
  const int has_bl = has_bottom_left(cm, xd, mi_row, mi_col, xd->height);
#endif  // CONFIG_MVP_IMPROVEMENT
#else
  const int bs = AOMMAX(xd->width, xd->height);
  const int has_tr = has_top_right(cm, xd, mi_row, mi_col, bs);
#if CONFIG_MVP_IMPROVEMENT
  const int has_bl = has_bottom_left(cm, xd, mi_row, mi_col, bs);
#endif  // CONFIG_MVP_IMPROVEMENT
#endif  // CONFIG_EXT_RECUR_PARTITIONS
  MV_REFERENCE_FRAME rf[2];

  const TileInfo *const tile = &xd->tile;
#if CONFIG_MVP_SIMPLIFY
  int max_col_offset = 0;
#else
  int max_row_offset = 0, max_col_offset = 0;
  const int row_adj = (xd->height < mi_size_high[BLOCK_8X8]) && (mi_row & 0x01);
#endif  // CONFIG_MVP_SIMPLIFY
  const int col_adj = (xd->width < mi_size_wide[BLOCK_8X8]) && (mi_col & 0x01);
  // when CONFIG_MVP_IMPROVEMENT is true, processed_rows does not needed
#if !CONFIG_MVP_IMPROVEMENT
  int processed_rows = 0;
#endif  // !CONFIG_MVP_IMPROVEMENT
  int processed_cols = 0;

  av1_set_ref_frame(rf, ref_frame);
#if !CONFIG_C076_INTER_MOD_CTX
  mode_context[ref_frame] = 0;
#endif  //! CONFIG_C076_INTER_MOD_CTX
  *refmv_count = 0;

#if CONFIG_MVP_SIMPLIFY
  /*
   * The constuction of the DRL after CWG-E021 was adopted:
   *
   * 1) Adjacent SMVP search; up to 9 blocks.
   * 2) TMVP search; up to 5 blocks.
   * 3) Non-adjacent SMVP search; up to 3 blocks in 2nd or 3rd column
   * 4) Find the adjacent SMVP candidate with the maximum weight, then switch
   *    it to the first place of the DRL.
   * 4) RefMV bank search.
   * 5) Derived SMVP search.
   * 6) Global MV is added if there is space in DRL.
   *
   */
#endif  // CONFIG_MVP_SIMPLIFY

#if CONFIG_EXTENDED_WARP_PREDICTION
  for (int k = 0; k < MAX_REF_MV_STACK_SIZE; k++) {
    ref_mv_stack[k].row_offset = OFFSET_NONSPATIAL;
    ref_mv_stack[k].col_offset = OFFSET_NONSPATIAL;
    ref_mv_stack[k].cwp_idx = CWP_EQUAL;
  }

  // derive a warp model from the 3 corner MVs
  if (warp_param_stack && valid_num_warp_candidates &&
      *valid_num_warp_candidates < max_num_of_warp_candidates) {
    int mvs_32[2 * 3];
    int pts[2 * 3];
    int np = 0;
    WarpedMotionParams cand_warp_param = default_warp_params;
    const int valid_points =
        generate_points_from_corners(xd, pts, mvs_32, &np, ref_frame);
    const int valid_model =
        get_model_from_corner_mvs(&cand_warp_param, pts, valid_points, mvs_32,
                                  xd->mi[0]->sb_type[PLANE_TYPE_Y]);
    if (valid_model && !cand_warp_param.invalid &&
        !is_this_param_already_in_list(*valid_num_warp_candidates,
                                       warp_param_stack, cand_warp_param)) {
      insert_neighbor_warp_candidate(warp_param_stack, &cand_warp_param,
                                     *valid_num_warp_candidates, PROJ_SPATIAL);
      (*valid_num_warp_candidates)++;
    }
  }
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

#if !CONFIG_MVP_SIMPLIFY
  // Find valid maximum row/col offset.
  if (xd->up_available) {
#if CONFIG_MVP_IMPROVEMENT
    max_row_offset = -(MVREF_ROWS << 1) + row_adj;
#else
    max_row_offset = -(MVREF_ROW_COLS << 1) + row_adj;
#endif  // CONFIG_MVP_IMPROVEMENT

    if (xd->height < mi_size_high[BLOCK_8X8])
      max_row_offset = -(2 << 1) + row_adj;

    max_row_offset = find_valid_row_offset(tile, mi_row, max_row_offset);
  }
#endif  // !CONFIG_MVP_SIMPLIFY

  if (xd->left_available) {
#if CONFIG_MVP_IMPROVEMENT
    max_col_offset = -(MVREF_COLS << 1) + col_adj;
#else
    max_col_offset = -(MVREF_ROW_COLS << 1) + col_adj;
#endif  // CONFIG_MVP_IMPROVEMENT

    if (xd->width < mi_size_wide[BLOCK_8X8])
      max_col_offset = -(2 << 1) + col_adj;

    max_col_offset = find_valid_col_offset(tile, mi_col, max_col_offset);
  }

  uint8_t col_match_count = 0;
  uint8_t row_match_count = 0;
  uint8_t newmv_count = 0;

#if CONFIG_MVP_IMPROVEMENT
  SINGLE_MV_CANDIDATE single_mv[MAX_REF_MV_STACK_SIZE];
  uint8_t single_mv_count = 0;
  CANDIDATE_MV derived_mv_stack[MAX_REF_MV_STACK_SIZE];
  uint16_t derived_mv_weight[MAX_REF_MV_STACK_SIZE];
  uint8_t derived_mv_count = 0;
#endif  // CONFIG_MVP_IMPROVEMENT

#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  const int width_at_least_two = xd->up_available ? (xd->width > 1) : 0;
  const int height_at_least_two = xd->left_available ? (xd->height > 1) : 0;
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY

#if CONFIG_MVP_IMPROVEMENT
  if (xd->left_available) {
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, (xd->height - 1), -1,
                  ref_mv_stack, ref_mv_weight, &col_match_count, &newmv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
    update_processed_cols(xd, mi_row, mi_col, (xd->height - 1), -1,
                          max_col_offset, &processed_cols);
  }
  if (xd->up_available) {
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, -1, (xd->width - 1), ref_mv_stack,
                  ref_mv_weight, &row_match_count, &newmv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
  }
#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  if (height_at_least_two) {
#else
  if (xd->left_available) {
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, 0, -1, ref_mv_stack,
                  ref_mv_weight, &col_match_count, &newmv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
    update_processed_cols(xd, mi_row, mi_col, 0, -1, max_col_offset,
                          &processed_cols);
  }
#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  if (width_at_least_two) {
#else
  if (xd->up_available) {
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, -1, 0, ref_mv_stack,
                  ref_mv_weight, &row_match_count, &newmv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
  }
  if (has_bl) {
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, xd->height, -1, ref_mv_stack,
                  ref_mv_weight, &col_match_count, &newmv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
  }
  if (has_tr) {
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, -1, xd->width, ref_mv_stack,
                  ref_mv_weight, &row_match_count, &newmv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
  }

#if !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  if (xd->up_available && xd->left_available) {
    uint8_t dummy_ref_match_count = 0;
    uint8_t dummy_new_mv_count = 0;
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, -1, -1, ref_mv_stack,
                  ref_mv_weight, &dummy_ref_match_count, &dummy_new_mv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
  }

  if (xd->left_available) {
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, (xd->height >> 1), -1,
                  ref_mv_stack, ref_mv_weight, &col_match_count, &newmv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
    update_processed_cols(xd, mi_row, mi_col, (xd->height >> 1), -1,
                          max_col_offset, &processed_cols);
  }

  if (xd->up_available) {
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, -1, (xd->width >> 1),
                  ref_mv_stack, ref_mv_weight, &row_match_count, &newmv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
  }
#endif  // !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
#else
  // Scan the first above row mode info. row_offset = -1;
  if (abs(max_row_offset) >= 1)
    scan_row_mbmi(cm, xd, mi_row, mi_col, rf, -1, ref_mv_stack, ref_mv_weight,
                  refmv_count, &row_match_count, &newmv_count, gm_mv_candidates,
                  max_row_offset,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  &processed_rows);

  // Scan the first left column mode info. col_offset = -1;
  if (abs(max_col_offset) >= 1)
    scan_col_mbmi(cm, xd, mi_row, mi_col, rf, -1, ref_mv_stack, ref_mv_weight,
                  refmv_count, &col_match_count, &newmv_count, gm_mv_candidates,
                  max_col_offset,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  &processed_cols);

  // Check top-right boundary
  if (has_tr)
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, -1, xd->width, ref_mv_stack,
                  ref_mv_weight, &row_match_count, &newmv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
#endif  // CONFIG_MVP_IMPROVEMENT

#if !CONFIG_C076_INTER_MOD_CTX
  const uint8_t nearest_match = (row_match_count > 0) + (col_match_count > 0);
#endif  //! CONFIG_C076_INTER_MOD_CTX
#if !CONFIG_MVP_IMPROVEMENT || !CONFIG_TMVP_IMPROVE
  const uint8_t nearest_refmv_count = *refmv_count;

#if !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  // TODO(yunqing): for comp_search, do it for all 3 cases.
  for (int idx = 0; idx < nearest_refmv_count; ++idx)
    ref_mv_weight[idx] += REF_CAT_LEVEL;
#endif  // !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
#endif  // !CONFIG_MVP_IMPROVEMENT || !CONFIG_TMVP_IMPROVE

#if CONFIG_IBC_SR_EXT
  if (cm->features.allow_ref_frame_mvs &&
#if CONFIG_D072_SKIP_MODE_IMPROVE
      (xd->mi[0]->skip_mode || rf[0] != rf[1]) &&
#endif  // CONFIG_D072_SKIP_MODE_IMPROVE
      !xd->mi[0]->use_intrabc[xd->tree_type == CHROMA_PART]) {
#else
  if (cm->features.allow_ref_frame_mvs
#if CONFIG_D072_SKIP_MODE_IMPROVE
      && (xd->mi[0]->skip_mode || rf[0] != rf[1])
#endif  // CONFIG_D072_SKIP_MODE_IMPROVE
  ) {
#endif  // CONFIG_IBC_SR_EXT
#if !CONFIG_C076_INTER_MOD_CTX
    int is_available = 0;
#endif  //! CONFIG_C076_INTER_MOD_CTX
#if !CONFIG_MVP_IMPROVEMENT
    const int voffset = AOMMAX(mi_size_high[BLOCK_8X8], xd->height);
    const int hoffset = AOMMAX(mi_size_wide[BLOCK_8X8], xd->width);
#endif  // !CONFIG_MVP_IMPROVEMENT
    const int blk_row_end = AOMMIN(xd->height, mi_size_high[BLOCK_64X64]);
    const int blk_col_end = AOMMIN(xd->width, mi_size_wide[BLOCK_64X64]);
#if !CONFIG_MVP_IMPROVEMENT
    const int tpl_sample_pos[3][2] = {
      { voffset, -2 },
      { voffset, hoffset },
      { voffset - 2, hoffset },
    };
    const int allow_extension = (xd->height >= mi_size_high[BLOCK_8X8]) &&
                                (xd->height < mi_size_high[BLOCK_64X64]) &&
                                (xd->width >= mi_size_wide[BLOCK_8X8]) &&
                                (xd->width < mi_size_wide[BLOCK_64X64]);
#endif  // !CONFIG_MVP_IMPROVEMENT

    const int step_h = (xd->height >= mi_size_high[BLOCK_64X64])
                           ? mi_size_high[BLOCK_16X16]
                           : mi_size_high[BLOCK_8X8];
    const int step_w = (xd->width >= mi_size_wide[BLOCK_64X64])
                           ? mi_size_wide[BLOCK_16X16]
                           : mi_size_wide[BLOCK_8X8];

#if CONFIG_MVP_IMPROVEMENT
    int added_tmvp_cnt = 0;
#endif  // CONFIG_MVP_IMPROVEMENT

#if CONFIG_MVP_SIMPLIFY
    const MVP_UNIT_STATUS tmvp_units_status[TMVP_SEARCH_COUNT] = {
      { (blk_row_end - step_h > 0 && blk_col_end - step_w > 0),
        blk_row_end - step_h, blk_col_end - step_w },
      { (blk_row_end - step_h > 0 && blk_col_end - step_w == 0),
        blk_row_end - step_h, 0 },
      { (blk_row_end - step_h > 1 && blk_col_end - step_w > 1),
        blk_row_end >> 1, blk_col_end >> 1 },
      { (blk_row_end - step_h == 0 && blk_col_end - step_w > 0), 0,
        blk_col_end - step_w },
      { (blk_row_end - step_h >= 0 && blk_col_end - step_w >= 0), 0, 0 },
    };

    for (int iter = 0; iter < TMVP_SEARCH_COUNT; ++iter) {
      if (added_tmvp_cnt) break;
      if (tmvp_units_status[iter].is_available) {
        add_tpl_ref_mv(cm, xd, mi_row, mi_col, ref_frame,
                       tmvp_units_status[iter].row_offset,
                       tmvp_units_status[iter].col_offset
#if !CONFIG_C076_INTER_MOD_CTX
                       ,
                       gm_mv_candidates
#endif  //! CONFIG_C076_INTER_MOD_CTX
                       ,
                       refmv_count,
#if CONFIG_MVP_IMPROVEMENT
                       &added_tmvp_cnt,
#endif  // CONFIG_MVP_IMPROVEMENT
                       ref_mv_stack, ref_mv_weight
#if CONFIG_SKIP_MODE_ENHANCEMENT
                       ,
                       ref_frame_idx0, ref_frame_idx1
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if !CONFIG_C076_INTER_MOD_CTX
                       ,
                       mode_context
#endif  // !CONFIG_C076_INTER_MOD_CTX
        );
      }
    }
#else
#if CONFIG_MVP_IMPROVEMENT
    // Use reversed horizontal scan order to check TMVP candidates
    for (int blk_row = blk_row_end - step_h; blk_row >= 0; blk_row -= step_h) {
      for (int blk_col = blk_col_end - step_w; blk_col >= 0;
           blk_col -= step_w) {
        if (added_tmvp_cnt) break;
#else
    for (int blk_row = 0; blk_row < blk_row_end; blk_row += step_h) {
      for (int blk_col = 0; blk_col < blk_col_end; blk_col += step_w) {
#endif  // CONFIG_MVP_IMPROVEMENT
#if !CONFIG_C076_INTER_MOD_CTX
        int ret =
#endif  //! CONFIG_C076_INTER_MOD_CTX
            add_tpl_ref_mv(cm, xd, mi_row, mi_col, ref_frame, blk_row, blk_col
#if !CONFIG_C076_INTER_MOD_CTX
                           ,
                           gm_mv_candidates
#endif  //! CONFIG_C076_INTER_MOD_CTX
                           ,
                           refmv_count,
#if CONFIG_MVP_IMPROVEMENT
                           &added_tmvp_cnt,
#endif  // CONFIG_MVP_IMPROVEMENT
                           ref_mv_stack, ref_mv_weight
#if CONFIG_SKIP_MODE_ENHANCEMENT
                           ,
                           ref_frame_idx0, ref_frame_idx1
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if !CONFIG_C076_INTER_MOD_CTX
                           ,
                           mode_context
#endif  // !CONFIG_C076_INTER_MOD_CTX
            );
#if !CONFIG_C076_INTER_MOD_CTX
#if CONFIG_MVP_IMPROVEMENT
        if (added_tmvp_cnt) is_available = ret;
#else
        if (blk_row == 0 && blk_col == 0) is_available = ret;
#endif  // CONFIG_MVP_IMPROVEMENT
#endif  //! CONFIG_C076_INTER_MOD_CTX
      }
    }

#if !CONFIG_C076_INTER_MOD_CTX
    if (is_available == 0) mode_context[ref_frame] |= (1 << GLOBALMV_OFFSET);
#endif  //! CONFIG_C076_INTER_MOD_CTX
#if !CONFIG_MVP_IMPROVEMENT
    for (int i = 0; i < 3 && allow_extension; ++i) {
      const int blk_row = tpl_sample_pos[i][0];
      const int blk_col = tpl_sample_pos[i][1];

      if (!check_sb_border(mi_row, mi_col, blk_row, blk_col)) continue;
      add_tpl_ref_mv(cm, xd, mi_row, mi_col, ref_frame, blk_row, blk_col,
#if !CONFIG_C076_INTER_MOD_CTX
                     gm_mv_candidates,
#endif  //! CONFIG_C076_INTER_MOD_CTX
                     refmv_count, ref_mv_stack, ref_mv_weight,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                     ref_frame_idx0,
                     ref_frame_idx1
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if !CONFIG_C076_INTER_MOD_CTX
                         mode_context
#endif  //! CONFIG_C076_INTER_MOD_CTX
      );
    }
#endif  // !CONFIG_MVP_IMPROVEMENT
#endif  // CONFIG_MVP_SIMPLIFY
  }

#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  if (xd->up_available && xd->left_available) {
    uint8_t dummy_ref_match_count = 0;
    uint8_t dummy_new_mv_count = 0;
    scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, -1, -1, ref_mv_stack,
                  ref_mv_weight, &dummy_ref_match_count, &dummy_new_mv_count,
                  gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                  ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                  1, single_mv, &single_mv_count, derived_mv_stack,
                  derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                  warp_param_stack, max_num_of_warp_candidates,
                  valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                  refmv_count);
  }
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY

#if CONFIG_TMVP_IMPROVE
  const uint8_t nearest_refmv_count = *refmv_count;

#if !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
  // TODO(yunqing): for comp_search, do it for all 3 cases.
  for (int idx = 0; idx < nearest_refmv_count; ++idx) {
    ref_mv_weight[idx] += REF_CAT_LEVEL;
  }
#endif  // !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
#endif  // CONFIG_TMVP_IMPROVE

#if !CONFIG_MVP_SIMPLIFY
  uint8_t dummy_newmv_count = 0;
#endif  // !CONFIG_MVP_SIMPLIFY

#if !CONFIG_MVP_IMPROVEMENT
  // Scan the second outer area.
  scan_blk_mbmi(cm, xd, mi_row, mi_col, rf, -1, -1, ref_mv_stack, ref_mv_weight,
                &row_match_count, &dummy_newmv_count, gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_MVP_IMPROVEMENT
                0, single_mv, &single_mv_count, derived_mv_stack,
                derived_mv_weight, &derived_mv_count,
#endif  // CONFIG_MVP_IMPROVEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                warp_param_stack, max_num_of_warp_candidates,
                valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                refmv_count);
#endif  // !CONFIG_MVP_IMPROVEMENT

#if CONFIG_MVP_SIMPLIFY
  if (xd->left_available) {
    for (int idx = 2; idx <= MVREF_COLS; ++idx) {
      const int col_offset = -(idx << 1) + 1 + col_adj;
      const MVP_UNIT_STATUS col_units_status[SMVP_COL_SEARCH_COUNT] = {
        { 1, (xd->height - 1), col_offset },
        { xd->height > 1, 0, col_offset },
#if !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
        { xd->height > 2, (xd->height >> 1), col_offset },
#endif  // !CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
      };
      if (abs(col_offset) <= abs(max_col_offset) &&
          abs(col_offset) > processed_cols) {
        for (int unit_idx = 0; unit_idx < SMVP_COL_SEARCH_COUNT; unit_idx++) {
          if (col_units_status[unit_idx].is_available) {
            scan_blk_mbmi(cm, xd, mi_row, mi_col, rf,
                          col_units_status[unit_idx].row_offset,
                          col_units_status[unit_idx].col_offset, ref_mv_stack,
                          ref_mv_weight, &col_match_count, &newmv_count,
                          gm_mv_candidates,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                          ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
                          1, single_mv, &single_mv_count, derived_mv_stack,
                          derived_mv_weight, &derived_mv_count,
#if CONFIG_EXTENDED_WARP_PREDICTION
                          warp_param_stack, max_num_of_warp_candidates,
                          valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                          refmv_count);
            update_processed_cols(xd, mi_row, mi_col,
                                  col_units_status[unit_idx].row_offset,
                                  col_units_status[unit_idx].col_offset,
                                  max_col_offset, &processed_cols);
          }
        }
      }
    }
  }
#else
#if CONFIG_MVP_IMPROVEMENT
  for (int idx = 2; idx <= MVREF_COLS; ++idx) {
    const int col_offset = -(idx << 1) + 1 + col_adj;
    if (abs(col_offset) <= abs(max_col_offset) &&
        abs(col_offset) > processed_cols) {
      scan_col_mbmi(cm, xd, mi_row, mi_col, rf, col_offset, ref_mv_stack,
                    ref_mv_weight, refmv_count, &col_match_count,
                    &dummy_newmv_count, gm_mv_candidates, max_col_offset,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                    ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
                    0, single_mv, &single_mv_count, derived_mv_stack,
                    derived_mv_weight, &derived_mv_count,
#if CONFIG_EXTENDED_WARP_PREDICTION
                    warp_param_stack, max_num_of_warp_candidates,
                    valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                    &processed_cols);
    }
  }
#else
  for (int idx = 2; idx <= MVREF_ROW_COLS; ++idx) {
    const int row_offset = -(idx << 1) + 1 + row_adj;
    const int col_offset = -(idx << 1) + 1 + col_adj;

    if (abs(row_offset) <= abs(max_row_offset) &&
        abs(row_offset) > processed_rows)
      scan_row_mbmi(cm, xd, mi_row, mi_col, rf, row_offset, ref_mv_stack,
                    ref_mv_weight, refmv_count, &row_match_count,
                    &dummy_newmv_count, gm_mv_candidates, max_row_offset,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                    ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                    warp_param_stack, max_num_of_warp_candidates,
                    valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

                    &processed_rows);

    if (abs(col_offset) <= abs(max_col_offset) &&
        abs(col_offset) > processed_cols)
      scan_col_mbmi(cm, xd, mi_row, mi_col, rf, col_offset, ref_mv_stack,
                    ref_mv_weight, refmv_count, &col_match_count,
                    &dummy_newmv_count, gm_mv_candidates, max_col_offset,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                    ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_EXTENDED_WARP_PREDICTION
                    warp_param_stack, max_num_of_warp_candidates,
                    valid_num_warp_candidates, ref_frame,
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
                    &processed_cols);
  }
#endif  // CONFIG_MVP_IMPROVEMENT
#endif  // CONFIG_MVP_SIMPLIFY

#if !CONFIG_C076_INTER_MOD_CTX
#if CONFIG_MVP_IMPROVEMENT
  // These contexts are independent of the outer area search
  int new_ctx = 2 * nearest_match + (newmv_count > 0);
  int ref_ctx = 2 * nearest_match + (newmv_count < 3);
  mode_context[ref_frame] |= new_ctx;
  mode_context[ref_frame] |= (ref_ctx << REFMV_OFFSET);
#else
  const uint8_t ref_match_count = (row_match_count > 0) + (col_match_count > 0);

  switch (nearest_match) {
    case 0:
      if (ref_match_count >= 1) mode_context[ref_frame] |= 1;
      if (ref_match_count == 1)
        mode_context[ref_frame] |= (1 << REFMV_OFFSET);
      else if (ref_match_count >= 2)
        mode_context[ref_frame] |= (2 << REFMV_OFFSET);
      break;
    case 1:
      mode_context[ref_frame] |= (newmv_count > 0) ? 2 : 3;
      if (ref_match_count == 1)
        mode_context[ref_frame] |= (3 << REFMV_OFFSET);
      else if (ref_match_count >= 2)
        mode_context[ref_frame] |= (4 << REFMV_OFFSET);
      break;
    case 2:
    default:
      if (newmv_count >= 1)
        mode_context[ref_frame] |= 4;
      else
        mode_context[ref_frame] |= 5;

      mode_context[ref_frame] |= (5 << REFMV_OFFSET);
      break;
  }
#endif
#endif  //! CONFIG_C076_INTER_MOD_CTX

#if CONFIG_MVP_SIMPLIFY
  if (nearest_refmv_count > 1) {
    int max_weight = ref_mv_weight[0];
    int max_weight_idx = 0;
    for (int idx = 1; idx < nearest_refmv_count; ++idx) {
      if (ref_mv_weight[idx] > max_weight) {
        max_weight = ref_mv_weight[idx];
        max_weight_idx = idx;
      }
    }

    if (max_weight_idx != 0) {
      const CANDIDATE_MV tmp_mv = ref_mv_stack[0];
      const uint16_t tmp_ref_mv_weight = ref_mv_weight[0];
      ref_mv_stack[0] = ref_mv_stack[max_weight_idx];
      ref_mv_stack[max_weight_idx] = tmp_mv;
      ref_mv_weight[0] = ref_mv_weight[max_weight_idx];
      ref_mv_weight[max_weight_idx] = tmp_ref_mv_weight;
#if CONFIG_SKIP_MODE_ENHANCEMENT
      if (xd->mi[0]->skip_mode) {
        const MV_REFERENCE_FRAME temp_ref0 = ref_frame_idx0[0];
        const MV_REFERENCE_FRAME temp_ref1 = ref_frame_idx1[0];

        ref_frame_idx0[0] = ref_frame_idx0[max_weight_idx];
        ref_frame_idx0[max_weight_idx] = temp_ref0;
        ref_frame_idx1[0] = ref_frame_idx1[max_weight_idx];
        ref_frame_idx1[max_weight_idx] = temp_ref1;
      }
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
    }
  }
#else
  // Rank the likelihood and assign nearest and near mvs.
  int len = nearest_refmv_count;
  while (len > 0) {
    int nr_len = 0;
    for (int idx = 1; idx < len; ++idx) {
      if (ref_mv_weight[idx - 1] < ref_mv_weight[idx]) {
        const CANDIDATE_MV tmp_mv = ref_mv_stack[idx - 1];
        const uint16_t tmp_ref_mv_weight = ref_mv_weight[idx - 1];
        ref_mv_stack[idx - 1] = ref_mv_stack[idx];
        ref_mv_stack[idx] = tmp_mv;
        ref_mv_weight[idx - 1] = ref_mv_weight[idx];
        ref_mv_weight[idx] = tmp_ref_mv_weight;
#if CONFIG_SKIP_MODE_ENHANCEMENT
        if (xd->mi[0]->skip_mode) {
          const MV_REFERENCE_FRAME temp_ref0 = ref_frame_idx0[idx - 1];
          const MV_REFERENCE_FRAME temp_ref1 = ref_frame_idx1[idx - 1];

          ref_frame_idx0[idx - 1] = ref_frame_idx0[idx];
          ref_frame_idx0[idx] = temp_ref0;
          ref_frame_idx1[idx - 1] = ref_frame_idx1[idx];
          ref_frame_idx1[idx] = temp_ref1;
        }
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
        nr_len = idx;
      }
    }
    len = nr_len;
  }

#if !CONFIG_MVP_IMPROVEMENT
  len = *refmv_count;
  while (len > nearest_refmv_count) {
    int nr_len = nearest_refmv_count;
    for (int idx = nearest_refmv_count + 1; idx < len; ++idx) {
      if (ref_mv_weight[idx - 1] < ref_mv_weight[idx]) {
        const CANDIDATE_MV tmp_mv = ref_mv_stack[idx - 1];
        const uint16_t tmp_ref_mv_weight = ref_mv_weight[idx - 1];
        ref_mv_stack[idx - 1] = ref_mv_stack[idx];
        ref_mv_stack[idx] = tmp_mv;
        ref_mv_weight[idx - 1] = ref_mv_weight[idx];
        ref_mv_weight[idx] = tmp_ref_mv_weight;
        nr_len = idx;
      }
    }
    len = nr_len;
  }
#endif
#endif  // CONFIG_MVP_SIMPLIFY

#if CONFIG_MVP_IMPROVEMENT
  if (cm->seq_params.enable_refmvbank) {
    const int ref_mv_limit =
#if CONFIG_IBC_BV_IMPROVEMENT && CONFIG_IBC_MAX_DRL
        xd->mi[0]->use_intrabc[xd->tree_type == CHROMA_PART]
            ? AOMMIN(cm->features.max_bvp_drl_bits + 1, MAX_REF_BV_STACK_SIZE)
            :
#endif  // CONFIG_IBC_BV_IMPROVEMENT && CONFIG_IBC_MAX_DRL
            AOMMIN(cm->features.max_drl_bits + 1, MAX_REF_MV_STACK_SIZE);
    // If open slots are available, fetch reference MVs from the ref mv banks.
    if (*refmv_count < ref_mv_limit
#if !CONFIG_IBC_BV_IMPROVEMENT
        && ref_frame != INTRA_FRAME
#endif  // CONFIG_IBC_BV_IMPROVEMENT
    ) {
      const REF_MV_BANK *ref_mv_bank = &xd->ref_mv_bank;
      const CANDIDATE_MV *queue = ref_mv_bank->rmb_buffer[ref_frame];
      const int count = ref_mv_bank->rmb_count[ref_frame];
      const int start_idx = ref_mv_bank->rmb_start_idx[ref_frame];
      const int is_comp = is_inter_ref_frame(rf[1]);
      const int block_width = xd->width * MI_SIZE;
      const int block_height = xd->height * MI_SIZE;

      for (int idx_bank = 0; idx_bank < count && *refmv_count < ref_mv_limit;
           ++idx_bank) {
        const int idx = (start_idx + count - 1 - idx_bank) % REF_MV_BANK_SIZE;
        const CANDIDATE_MV cand_mv = queue[idx];
#if CONFIG_SKIP_MODE_ENHANCEMENT
        bool rmb_candi_exist =
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
            check_rmb_cand(cand_mv, ref_mv_stack, ref_mv_weight, refmv_count,
                           is_comp, xd->mi_row, xd->mi_col, block_width,
                           block_height, cm->width, cm->height);
#if CONFIG_SKIP_MODE_ENHANCEMENT
        if (xd->mi[0]->skip_mode && rmb_candi_exist) {
          ref_frame_idx0[*refmv_count - 1] = rf[0];
          ref_frame_idx1[*refmv_count - 1] = rf[1];
        }
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
      }
    }
  }
#endif  // CONFIG_MVP_IMPROVEMENT

#if CONFIG_MVP_IMPROVEMENT
  const int max_ref_mv_count =
#if CONFIG_IBC_BV_IMPROVEMENT && CONFIG_IBC_MAX_DRL
      xd->mi[0]->use_intrabc[xd->tree_type == CHROMA_PART]
          ? AOMMIN(cm->features.max_bvp_drl_bits + 1, MAX_REF_BV_STACK_SIZE)
          :
#endif  // CONFIG_IBC_BV_IMPROVEMENT && CONFIG_IBC_MAX_DRL
          AOMMIN(cm->features.max_drl_bits + 1, MAX_REF_MV_STACK_SIZE);

#if CONFIG_SKIP_MODE_ENHANCEMENT
  if (xd->mi[0]->skip_mode) derived_mv_count = 0;
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT

  if (*refmv_count < max_ref_mv_count && derived_mv_count > 0) {
    fill_mvp_from_derived_smvp(rf, ref_mv_stack, ref_mv_weight, refmv_count,
                               derived_mv_stack, derived_mv_count,
#if CONFIG_SKIP_MODE_ENHANCEMENT
                               xd->mi[0], ref_frame_idx0, ref_frame_idx1,
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
                               max_ref_mv_count);
  }
#endif  // CONFIG_MVP_IMPROVEMENT

#if CONFIG_MVP_SIMPLIFY
  for (int idx = 0; idx < *refmv_count; ++idx) {
    clamp_mv_ref(&ref_mv_stack[idx].this_mv.as_mv, xd->width << MI_SIZE_LOG2,
                 xd->height << MI_SIZE_LOG2, xd);
    if (rf[1] > NONE_FRAME) {
      clamp_mv_ref(&ref_mv_stack[idx].comp_mv.as_mv, xd->width << MI_SIZE_LOG2,
                   xd->height << MI_SIZE_LOG2, xd);
    }
  }

  if (rf[1] == NONE_FRAME && mv_ref_list != NULL) {
    for (int idx = *refmv_count; idx < MAX_MV_REF_CANDIDATES; ++idx) {
      mv_ref_list[idx].as_int = gm_mv_candidates[0].as_int;
    }

    for (int idx = 0; idx < AOMMIN(MAX_MV_REF_CANDIDATES, *refmv_count);
         ++idx) {
      mv_ref_list[idx].as_int = ref_mv_stack[idx].this_mv.as_int;
    }
  }

  // If there is extra space in the stack, copy the GLOBALMV vector into it.
  // This also guarantees the existence of at least one vector to search.
  if (*refmv_count < MAX_REF_MV_STACK_SIZE
#if CONFIG_IBC_BV_IMPROVEMENT
      && !xd->mi[0]->use_intrabc[xd->tree_type == CHROMA_PART]
#endif  // CONFIG_IBC_BV_IMPROVEMENT
  ) {
    int idx = 0;
    for (idx = 0; idx < *refmv_count; ++idx) {
      if ((ref_mv_stack[idx].this_mv.as_int == gm_mv_candidates[0].as_int) &&
          (rf[1] == NONE_FRAME ||
           (rf[1] > NONE_FRAME &&
            ref_mv_stack[idx].comp_mv.as_int == gm_mv_candidates[1].as_int))) {
        break;
      }
    }

    // Add a new item to the list.
    if (idx == *refmv_count) {
      ref_mv_stack[idx].this_mv.as_int = gm_mv_candidates[0].as_int;
      ref_mv_stack[idx].comp_mv.as_int = gm_mv_candidates[1].as_int;
#if CONFIG_EXTENDED_WARP_PREDICTION
      ref_mv_stack[idx].row_offset = OFFSET_NONSPATIAL;
      ref_mv_stack[idx].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
      ref_mv_stack[idx].cwp_idx = CWP_EQUAL;
#if CONFIG_SKIP_MODE_ENHANCEMENT
      if (xd->mi[0]->skip_mode) {
        ref_frame_idx0[idx] = rf[0];
        ref_frame_idx1[idx] = rf[1];
      }
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
      ref_mv_weight[idx] = REF_CAT_LEVEL;
      ++(*refmv_count);
    }
  }
#else  // CONFIG_MVP_SIMPLIFY
  int mi_width = AOMMIN(mi_size_wide[BLOCK_64X64], xd->width);
  mi_width = AOMMIN(mi_width, cm->mi_params.mi_cols - mi_col);
  int mi_height = AOMMIN(mi_size_high[BLOCK_64X64], xd->height);
  mi_height = AOMMIN(mi_height, cm->mi_params.mi_rows - mi_row);
  const int mi_size = AOMMIN(mi_width, mi_height);
  if (rf[1] > NONE_FRAME) {
    // TODO(jingning, yunqing): Refactor and consolidate the compound and
    // single reference frame modes. Reduce unnecessary redundancy.
    if (*refmv_count < MAX_MV_REF_CANDIDATES) {
      int_mv ref_id[2][2], ref_diff[2][2];
      int ref_id_count[2] = { 0 }, ref_diff_count[2] = { 0 };

      for (int idx = 0; abs(max_row_offset) >= 1 && idx < mi_size;) {
        const MB_MODE_INFO *const candidate = xd->mi[-xd->mi_stride + idx];
#if CONFIG_C071_SUBBLK_WARPMV
        const SUBMB_INFO *const submi = xd->submi[-xd->mi_stride + idx];
#endif  // CONFIG_C071_SUBBLK_WARPMV
        process_compound_ref_mv_candidate(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                          submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                          cm, rf, ref_id, ref_id_count,
                                          ref_diff, ref_diff_count);
        idx += mi_size_wide[candidate->sb_type[PLANE_TYPE_Y]];
      }

      for (int idx = 0; abs(max_col_offset) >= 1 && idx < mi_size;) {
        const MB_MODE_INFO *const candidate = xd->mi[idx * xd->mi_stride - 1];
#if CONFIG_C071_SUBBLK_WARPMV
        const SUBMB_INFO *const submi = xd->submi[idx * xd->mi_stride - 1];
#endif  // CONFIG_C071_SUBBLK_WARPMV
        process_compound_ref_mv_candidate(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                          submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                          cm, rf, ref_id, ref_id_count,
                                          ref_diff, ref_diff_count);
        idx += mi_size_high[candidate->sb_type[PLANE_TYPE_Y]];
      }

      // Build up the compound mv predictor
      int_mv comp_list[MAX_MV_REF_CANDIDATES][2];

      for (int idx = 0; idx < 2; ++idx) {
        int comp_idx = 0;
        for (int list_idx = 0;
             list_idx < ref_id_count[idx] && comp_idx < MAX_MV_REF_CANDIDATES;
             ++list_idx, ++comp_idx)
          comp_list[comp_idx][idx] = ref_id[idx][list_idx];
        for (int list_idx = 0;
             list_idx < ref_diff_count[idx] && comp_idx < MAX_MV_REF_CANDIDATES;
             ++list_idx, ++comp_idx)
          comp_list[comp_idx][idx] = ref_diff[idx][list_idx];
        for (; comp_idx < MAX_MV_REF_CANDIDATES; ++comp_idx)
          comp_list[comp_idx][idx] = gm_mv_candidates[idx];
      }

      if (*refmv_count) {
        assert(*refmv_count == 1);
        if (comp_list[0][0].as_int == ref_mv_stack[0].this_mv.as_int &&
            comp_list[0][1].as_int == ref_mv_stack[0].comp_mv.as_int) {
          ref_mv_stack[*refmv_count].this_mv = comp_list[1][0];
          ref_mv_stack[*refmv_count].comp_mv = comp_list[1][1];
#if CONFIG_EXTENDED_WARP_PREDICTION
          ref_mv_stack[*refmv_count].row_offset = OFFSET_NONSPATIAL;
          ref_mv_stack[*refmv_count].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
          ref_mv_stack[*refmv_count].cwp_idx = CWP_EQUAL;
        } else {
          ref_mv_stack[*refmv_count].this_mv = comp_list[0][0];
          ref_mv_stack[*refmv_count].comp_mv = comp_list[0][1];
#if CONFIG_EXTENDED_WARP_PREDICTION
          ref_mv_stack[*refmv_count].row_offset = OFFSET_NONSPATIAL;
          ref_mv_stack[*refmv_count].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
          ref_mv_stack[*refmv_count].cwp_idx = CWP_EQUAL;
        }
#if CONFIG_SKIP_MODE_ENHANCEMENT
        if (xd->mi[0]->skip_mode) {
          ref_frame_idx0[*refmv_count] = rf[0];
          ref_frame_idx1[*refmv_count] = rf[1];
        }
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
        ref_mv_weight[*refmv_count] = 0;
#else
        ref_mv_weight[*refmv_count] = 2;
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
        ++*refmv_count;
      } else {
        for (int idx = 0; idx < MAX_MV_REF_CANDIDATES; ++idx) {
          ref_mv_stack[*refmv_count].this_mv = comp_list[idx][0];
          ref_mv_stack[*refmv_count].comp_mv = comp_list[idx][1];
#if CONFIG_EXTENDED_WARP_PREDICTION
          ref_mv_stack[*refmv_count].row_offset = OFFSET_NONSPATIAL;
          ref_mv_stack[*refmv_count].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
          ref_mv_stack[*refmv_count].cwp_idx = CWP_EQUAL;
#if CONFIG_SKIP_MODE_ENHANCEMENT
          if (xd->mi[0]->skip_mode) {
            ref_frame_idx0[*refmv_count] = rf[0];
            ref_frame_idx1[*refmv_count] = rf[1];
          }
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
          ref_mv_weight[*refmv_count] = 0;
#else
          ref_mv_weight[*refmv_count] = 2;
#endif  // CONFIG_CWG_E099_DRL_WRL_SIMPLIFY
          ++*refmv_count;
        }
      }
    }

    assert(*refmv_count >= MAX_MV_REF_CANDIDATES);

    for (int idx = 0; idx < *refmv_count; ++idx) {
      clamp_mv_ref(&ref_mv_stack[idx].this_mv.as_mv, xd->width << MI_SIZE_LOG2,
                   xd->height << MI_SIZE_LOG2, xd);
      clamp_mv_ref(&ref_mv_stack[idx].comp_mv.as_mv, xd->width << MI_SIZE_LOG2,
                   xd->height << MI_SIZE_LOG2, xd);
    }
  } else {
    // Handle single reference frame extension
#if CONFIG_SKIP_MODE_ENHANCEMENT
    assert(!xd->mi[0]->skip_mode);
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
#if CONFIG_IBC_SR_EXT
    if (!xd->mi[0]->use_intrabc[xd->tree_type == CHROMA_PART]) {
#endif  // CONFIG_IBC_SR_EXT
      for (int idx = 0; abs(max_row_offset) >= 1 && idx < mi_size &&
                        *refmv_count < MAX_MV_REF_CANDIDATES;) {
        const MB_MODE_INFO *const candidate = xd->mi[-xd->mi_stride + idx];
#if CONFIG_C071_SUBBLK_WARPMV
        const SUBMB_INFO *const submi = xd->submi[-xd->mi_stride + idx];
#endif  // CONFIG_C071_SUBBLK_WARPMV
        process_single_ref_mv_candidate(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                        submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                        cm, ref_frame, refmv_count,
                                        ref_mv_stack, ref_mv_weight);
        idx += mi_size_wide[candidate->sb_type[PLANE_TYPE_Y]];
      }

      for (int idx = 0; abs(max_col_offset) >= 1 && idx < mi_size &&
                        *refmv_count < MAX_MV_REF_CANDIDATES;) {
        const MB_MODE_INFO *const candidate = xd->mi[idx * xd->mi_stride - 1];
#if CONFIG_C071_SUBBLK_WARPMV
        const SUBMB_INFO *const submi = xd->submi[idx * xd->mi_stride - 1];
#endif  // CONFIG_C071_SUBBLK_WARPMV
        process_single_ref_mv_candidate(candidate,
#if CONFIG_C071_SUBBLK_WARPMV
                                        submi,
#endif  // CONFIG_C071_SUBBLK_WARPMV
                                        cm, ref_frame, refmv_count,
                                        ref_mv_stack, ref_mv_weight);
        idx += mi_size_high[candidate->sb_type[PLANE_TYPE_Y]];
      }
#if CONFIG_IBC_SR_EXT
    }
#endif  // CONFIG_IBC_SR_EXT

    for (int idx = 0; idx < *refmv_count; ++idx) {
      clamp_mv_ref(&ref_mv_stack[idx].this_mv.as_mv, xd->width << MI_SIZE_LOG2,
                   xd->height << MI_SIZE_LOG2, xd);
    }

    if (mv_ref_list != NULL) {
      for (int idx = *refmv_count; idx < MAX_MV_REF_CANDIDATES; ++idx)
        mv_ref_list[idx].as_int = gm_mv_candidates[0].as_int;

      for (int idx = 0; idx < AOMMIN(MAX_MV_REF_CANDIDATES, *refmv_count);
           ++idx) {
        mv_ref_list[idx].as_int = ref_mv_stack[idx].this_mv.as_int;
      }
    }
    // If there is extra space in the stack, copy the GLOBALMV vector into it.
    // This also guarantees the existence of at least one vector to search.
    if (*refmv_count < MAX_REF_MV_STACK_SIZE
#if CONFIG_IBC_BV_IMPROVEMENT
        && !xd->mi[0]->use_intrabc[xd->tree_type == CHROMA_PART]
#endif  // CONFIG_IBC_BV_IMPROVEMENT
    ) {
      int stack_idx;
      for (stack_idx = 0; stack_idx < *refmv_count; ++stack_idx) {
        const int_mv stack_mv = ref_mv_stack[stack_idx].this_mv;
        if (gm_mv_candidates[0].as_int == stack_mv.as_int) break;
      }
      if (stack_idx == *refmv_count) {
        ref_mv_stack[*refmv_count].this_mv.as_int = gm_mv_candidates[0].as_int;
        ref_mv_stack[*refmv_count].comp_mv.as_int = gm_mv_candidates[1].as_int;
#if CONFIG_EXTENDED_WARP_PREDICTION
        ref_mv_stack[*refmv_count].row_offset = OFFSET_NONSPATIAL;
        ref_mv_stack[*refmv_count].col_offset = OFFSET_NONSPATIAL;
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
        ref_mv_stack[*refmv_count].cwp_idx = CWP_EQUAL;
        ref_mv_weight[*refmv_count] = REF_CAT_LEVEL;
        (*refmv_count)++;
      }
    }
  }
#endif  // CONFIG_MVP_SIMPLIFY

#if !CONFIG_MVP_IMPROVEMENT
  if (!cm->seq_params.enable_refmvbank) return;
  const int ref_mv_limit =
      AOMMIN(cm->features.max_drl_bits + 1, MAX_REF_MV_STACK_SIZE);
  // If open slots are available, fetch reference MVs from the ref mv banks.
  if (*refmv_count < ref_mv_limit
#if !CONFIG_IBC_BV_IMPROVEMENT
      && ref_frame != INTRA_FRAME
#endif  // CONFIG_IBC_BV_IMPROVEMENT
  ) {
    const REF_MV_BANK *ref_mv_bank = xd->ref_mv_bank_pt;
    const CANDIDATE_MV *queue = ref_mv_bank->rmb_buffer[ref_frame];
    const int count = ref_mv_bank->rmb_count[ref_frame];
    const int start_idx = ref_mv_bank->rmb_start_idx[ref_frame];
    const int is_comp = is_inter_ref_frame(rf[1]);
    const int block_width = xd->width * MI_SIZE;
    const int block_height = xd->height * MI_SIZE;

    for (int idx_bank = 0; idx_bank < count && *refmv_count < ref_mv_limit;
         ++idx_bank) {
      const int idx = (start_idx + count - 1 - idx_bank) % REF_MV_BANK_SIZE;
      const CANDIDATE_MV cand_mv = queue[idx];
#if CONFIG_SKIP_MODE_ENHANCEMENT
      bool rmb_candi_exist =
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
          check_rmb_cand(cand_mv, ref_mv_stack, ref_mv_weight, refmv_count,
                         is_comp, xd->mi_row, xd->mi_col, block_width,
                         block_height, cm->width, cm->height);
#if CONFIG_SKIP_MODE_ENHANCEMENT
      if (xd->mi[0]->skip_mode && rmb_candi_exist) {
        ref_frame_idx0[*refmv_count - 1] = rf[0];
        ref_frame_idx1[*refmv_count - 1] = rf[1];
      }
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
    }
  }
#endif  // !CONFIG_MVP_IMPROVEMENT

#if CONFIG_EXTENDED_WARP_PREDICTION
  if (warp_param_stack && valid_num_warp_candidates &&
      *valid_num_warp_candidates < max_num_of_warp_candidates) {
    // Insert warp parameters from the bank
#if WARP_CU_BANK
    const WARP_PARAM_BANK *warp_param_bank = &xd->warp_param_bank;
#else
    const WARP_PARAM_BANK *warp_param_bank = xd->warp_param_bank_pt;
#endif  // WARP_CU_BANK
    const WarpedMotionParams *queue = warp_param_bank->wpb_buffer[ref_frame];
    const int count = warp_param_bank->wpb_count[ref_frame];
    const int start_idx = warp_param_bank->wpb_start_idx[ref_frame];

    for (int idx_bank = 0; idx_bank < count && *valid_num_warp_candidates <
                                                   max_num_of_warp_candidates;
         ++idx_bank) {
      const int idx = (start_idx + count - 1 - idx_bank) % WARP_PARAM_BANK_SIZE;
      const WarpedMotionParams cand_warp_param = queue[idx];

      if (!cand_warp_param.invalid &&
          !is_this_param_already_in_list(*valid_num_warp_candidates,
                                         warp_param_stack, cand_warp_param)) {
        insert_neighbor_warp_candidate(warp_param_stack, &cand_warp_param,
                                       *valid_num_warp_candidates,
                                       PROJ_PARAM_BANK);
        (*valid_num_warp_candidates)++;
      }
    }

    // Insert Global motion of the current
    if (*valid_num_warp_candidates < max_num_of_warp_candidates) {
      if (!xd->global_motion[ref_frame].invalid &&
          !is_this_param_already_in_list(*valid_num_warp_candidates,
                                         warp_param_stack,
                                         xd->global_motion[ref_frame])) {
        insert_neighbor_warp_candidate(
            warp_param_stack, &xd->global_motion[ref_frame],
            *valid_num_warp_candidates, PROJ_GLOBAL_MOTION);
        (*valid_num_warp_candidates)++;
      }
    }

    // Filled with default values( currently all params are zeros)
    int max_num_of_default_allowed = AOMMIN(2, max_num_of_warp_candidates);
    int current_number_of_defaults = 0;
    int tmp_curr_num = *valid_num_warp_candidates;
    for (int cand_num = tmp_curr_num;
         (cand_num < max_num_of_warp_candidates) &&
         (current_number_of_defaults < max_num_of_default_allowed);
         cand_num++) {
      warp_param_stack[cand_num].wm_params = default_warp_params;
      warp_param_stack[cand_num].proj_type = PROJ_DEFAULT;
      (*valid_num_warp_candidates)++;
      current_number_of_defaults++;
    }
  }

#endif  // CONFIG_EXTENDED_WARP_PREDICTION

#if CONFIG_IBC_BV_IMPROVEMENT
  // If there are open slots in reference BV candidate list
  // fetch reference BVs from the default BVPs
  if (xd->mi[0]->use_intrabc[xd->tree_type == CHROMA_PART]) {
    const int w = xd->width * MI_SIZE;
    const int h = xd->height * MI_SIZE;
    const int sb_width = block_size_wide[cm->sb_size];
    const int sb_height = block_size_high[cm->sb_size];
    const int default_ref_bv_list[MAX_REF_BV_STACK_SIZE][2] = {
      { 0, -sb_height },
      { -sb_width - INTRABC_DELAY_PIXELS, 0 },
      { 0, -h },
      { -w, 0 },
    };
#if CONFIG_IBC_BV_IMPROVEMENT && CONFIG_IBC_MAX_DRL
    const int max_bvp_size = cm->features.max_bvp_drl_bits + 1;
    for (int i = 0; i < max_bvp_size; ++i) {
      if (*refmv_count >= max_bvp_size) break;
#else
    for (int i = 0; i < MAX_REF_BV_STACK_SIZE; ++i) {
      if (*refmv_count >= MAX_REF_BV_STACK_SIZE) break;
#endif  // CONFIG_IBC_BV_IMPROVEMENT && CONFIG_IBC_MAX_DRL
      CANDIDATE_MV tmp_mv;
      tmp_mv.this_mv.as_mv.col =
          (int16_t)GET_MV_SUBPEL(default_ref_bv_list[i][0]);
      tmp_mv.this_mv.as_mv.row =
          (int16_t)GET_MV_SUBPEL(default_ref_bv_list[i][1]);
      tmp_mv.comp_mv.as_int = 0;
      add_to_ref_bv_list(tmp_mv, ref_mv_stack, ref_mv_weight, refmv_count);
    }
  }
#endif  // CONFIG_IBC_BV_IMPROVEMENT
}

#if CONFIG_D072_SKIP_MODE_IMPROVE
void get_skip_mode_ref_offsets(const AV1_COMMON *cm, int ref_order_hint[2]) {
  const SkipModeInfo *const skip_mode_info = &cm->current_frame.skip_mode_info;
  ref_order_hint[0] = ref_order_hint[1] = 0;
  if (!skip_mode_info->skip_mode_allowed) return;

  const RefCntBuffer *const buf_0 =
      get_ref_frame_buf(cm, skip_mode_info->ref_frame_idx_0);
  const RefCntBuffer *const buf_1 =
      get_ref_frame_buf(cm, skip_mode_info->ref_frame_idx_1);
  assert(buf_0 != NULL && buf_1 != NULL);

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  ref_order_hint[0] = buf_0->display_order_hint;
  ref_order_hint[1] = buf_1->display_order_hint;
#else
  ref_order_hint[0] = buf_0->order_hint;
  ref_order_hint[1] = buf_1->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
}
#endif  // CONFIG_D072_SKIP_MODE_IMPROVE

#if CONFIG_EXTENDED_WARP_PREDICTION
// Initialize the warp parameter list
void av1_initialize_warp_wrl_list(
    WARP_CANDIDATE warp_param_stack[][MAX_WARP_REF_CANDIDATES],
    uint8_t valid_num_warp_candidates[INTER_REFS_PER_FRAME]) {
  for (int ref_frame = 0; ref_frame < INTER_REFS_PER_FRAME; ref_frame++) {
    for (int warp_idx = 0; warp_idx < MAX_WARP_REF_CANDIDATES; warp_idx++) {
      warp_param_stack[ref_frame][warp_idx].wm_params.invalid = 1;
    }
    valid_num_warp_candidates[ref_frame] = 0;
  }
}
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

#if CONFIG_C076_INTER_MOD_CTX
void av1_find_mode_ctx(const AV1_COMMON *cm, const MACROBLOCKD *xd,
                       int16_t *mode_context, MV_REFERENCE_FRAME ref_frame) {
  const int mi_row = xd->mi_row;
  const int mi_col = xd->mi_col;
  MV_REFERENCE_FRAME rf[2];

  av1_set_ref_frame(rf, ref_frame);
  mode_context[ref_frame] = 0;

  uint8_t col_match_count = 0;
  uint8_t row_match_count = 0;
  uint8_t newmv_count = 0;

  if (xd->left_available) {
    scan_blk_mbmi_ctx(cm, xd, mi_row, mi_col, rf, (xd->height - 1), -1,
                      &col_match_count, &newmv_count);
  }
  if (xd->up_available) {
    scan_blk_mbmi_ctx(cm, xd, mi_row, mi_col, rf, -1, (xd->width - 1),
                      &row_match_count, &newmv_count);
  }
  if (xd->left_available) {
    scan_blk_mbmi_ctx(cm, xd, mi_row, mi_col, rf, 0, -1, &col_match_count,
                      &newmv_count);
  }
  if (xd->up_available) {
    scan_blk_mbmi_ctx(cm, xd, mi_row, mi_col, rf, -1, 0, &row_match_count,
                      &newmv_count);
  }

  const uint8_t nearest_match = (row_match_count > 0) + (col_match_count > 0);

  // These contexts are independent of the outer area search
  int new_ctx = 2 * nearest_match + (newmv_count > 0);
  int ref_ctx = 2 * nearest_match + (newmv_count < 3);
  mode_context[ref_frame] |= new_ctx;
  mode_context[ref_frame] |= (ref_ctx << REFMV_OFFSET);
}
#endif  // CONFIG_C076_INTER_MOD_CTX

void av1_find_mv_refs(
    const AV1_COMMON *cm, const MACROBLOCKD *xd, MB_MODE_INFO *mi,
    MV_REFERENCE_FRAME ref_frame, uint8_t ref_mv_count[MODE_CTX_REF_FRAMES],
    CANDIDATE_MV ref_mv_stack[][MAX_REF_MV_STACK_SIZE],
    uint16_t ref_mv_weight[][MAX_REF_MV_STACK_SIZE],
    int_mv mv_ref_list[][MAX_MV_REF_CANDIDATES], int_mv *global_mvs
#if !CONFIG_C076_INTER_MOD_CTX
    ,
    int16_t *mode_context
#endif  // !CONFIG_C076_INTER_MOD_CTX
#if CONFIG_EXTENDED_WARP_PREDICTION
    ,
    WARP_CANDIDATE warp_param_stack[][MAX_WARP_REF_CANDIDATES],
    int max_num_of_warp_candidates,
    uint8_t valid_num_warp_candidates[INTER_REFS_PER_FRAME]
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
) {
  const int mi_row = xd->mi_row;
  const int mi_col = xd->mi_col;
  int_mv gm_mv[2];

  if (ref_frame == INTRA_FRAME) {
    gm_mv[0].as_int = gm_mv[1].as_int = 0;
    if (global_mvs != NULL) {
      global_mvs[ref_frame].as_int = INVALID_MV;
    }
  } else if (is_tip_ref_frame(ref_frame)) {
    gm_mv[0].as_int = gm_mv[1].as_int = 0;
  } else {
    const BLOCK_SIZE bsize = mi->sb_type[PLANE_TYPE_Y];
    const int fr_mv_precision = cm->features.fr_mv_precision;
    if (ref_frame < INTER_REFS_PER_FRAME) {
      gm_mv[0] = get_warp_motion_vector(xd, &cm->global_motion[ref_frame],
                                        fr_mv_precision, bsize, mi_col, mi_row);
      gm_mv[1].as_int = 0;
      if (global_mvs != NULL) global_mvs[ref_frame] = gm_mv[0];
    } else {
      MV_REFERENCE_FRAME rf[2];
      av1_set_ref_frame(rf, ref_frame);
      gm_mv[0] = get_warp_motion_vector(xd, &cm->global_motion[rf[0]],
                                        fr_mv_precision, bsize, mi_col, mi_row);
      gm_mv[1] = get_warp_motion_vector(xd, &cm->global_motion[rf[1]],
                                        fr_mv_precision, bsize, mi_col, mi_row);
    }
  }

#if CONFIG_EXTENDED_WARP_PREDICTION
  bool derive_wrl = (warp_param_stack && valid_num_warp_candidates &&
                     max_num_of_warp_candidates);
  derive_wrl &= (ref_frame < INTER_REFS_PER_FRAME);
#if CONFIG_SEP_COMP_DRL
  if (has_second_drl(mi)) derive_wrl = 0;
#endif  // CONFIG_SEP_COMP_DRL

  derive_wrl &= is_motion_variation_allowed_bsize(mi->sb_type[PLANE_TYPE_Y],
                                                  mi_row, mi_col);
  if (derive_wrl && valid_num_warp_candidates) {
    valid_num_warp_candidates[ref_frame] =
        0;  // initialize the number of valid candidates to 0 at the beginning
  }
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

#if CONFIG_SKIP_MODE_ENHANCEMENT
  if (mi->skip_mode) {
    SKIP_MODE_MVP_LIST *skip_list =
        (SKIP_MODE_MVP_LIST *)&(xd->skip_mvp_candidate_list);
    setup_ref_mv_list(
        cm, xd, ref_frame, &(skip_list->ref_mv_count), skip_list->ref_mv_stack,
        skip_list->weight, skip_list->ref_frame0, skip_list->ref_frame1,
        mv_ref_list ? mv_ref_list[ref_frame] : NULL, gm_mv, mi_row, mi_col
#if !CONFIG_C076_INTER_MOD_CTX
        ,
        skip_list->mode_context
#endif  // !CONFIG_C076_INTER_MOD_CTX
#if CONFIG_EXTENDED_WARP_PREDICTION
        ,
        NULL, 0, NULL
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
    );
  } else {
#if CONFIG_SEP_COMP_DRL
    MV_REFERENCE_FRAME rf[2];
    av1_set_ref_frame(rf, ref_frame);
    if (!has_second_drl(mi))
      rf[0] = ref_frame;
    else {
      const BLOCK_SIZE bsize = mi->sb_type[PLANE_TYPE_Y];
      const int fr_mv_precision = cm->features.fr_mv_precision;
      gm_mv[0] = get_warp_motion_vector(xd, &cm->global_motion[rf[0]],
                                        fr_mv_precision, bsize, mi_col, mi_row);
      gm_mv[1].as_int = 0;
    }
    setup_ref_mv_list(cm, xd, rf[0], &ref_mv_count[rf[0]], ref_mv_stack[rf[0]],
                      ref_mv_weight[rf[0]], NULL, NULL,
                      mv_ref_list ? mv_ref_list[rf[0]] : NULL, gm_mv, mi_row,
                      mi_col
#if !CONFIG_C076_INTER_MOD_CTX
                      ,
                      mode_context
#endif  //! CONFIG_C076_INTER_MOD_CTX
#if CONFIG_EXTENDED_WARP_PREDICTION
                      ,
                      derive_wrl ? warp_param_stack[rf[0]] : NULL,
                      derive_wrl ? max_num_of_warp_candidates : 0,
                      derive_wrl ? &valid_num_warp_candidates[rf[0]] : NULL
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
    );

    if (has_second_drl(mi)) {
      assert(rf[0] == mi->ref_frame[0]);
      assert(rf[1] == mi->ref_frame[1]);
      const BLOCK_SIZE bsize = mi->sb_type[PLANE_TYPE_Y];
      const int fr_mv_precision = cm->features.fr_mv_precision;
      gm_mv[0] = get_warp_motion_vector(xd, &cm->global_motion[rf[1]],
                                        fr_mv_precision, bsize, mi_col, mi_row);
      gm_mv[1].as_int = 0;

      setup_ref_mv_list(cm, xd, rf[1], &ref_mv_count[rf[1]],
                        ref_mv_stack[rf[1]], ref_mv_weight[rf[1]], NULL, NULL,
                        mv_ref_list ? mv_ref_list[rf[1]] : NULL, gm_mv, mi_row,
                        mi_col
#if !CONFIG_C076_INTER_MOD_CTX
                        ,
                        mode_context
#endif  //! CONFIG_C076_INTER_MOD_CTX
#if CONFIG_EXTENDED_WARP_PREDICTION
                        ,
                        derive_wrl ? warp_param_stack[rf[1]] : NULL,
                        derive_wrl ? max_num_of_warp_candidates : 0,
                        derive_wrl ? &valid_num_warp_candidates[rf[1]] : NULL
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
      );
    }
#if CONFIG_EXTENDED_WARP_PREDICTION
    if (derive_wrl) assert(rf[0] == ref_frame);
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
#else
    setup_ref_mv_list(cm, xd, ref_frame, &ref_mv_count[ref_frame],
                      ref_mv_stack[ref_frame], ref_mv_weight[ref_frame], NULL,
                      NULL, mv_ref_list ? mv_ref_list[ref_frame] : NULL, gm_mv,
                      mi_row, mi_col
#if !CONFIG_C076_INTER_MOD_CTX
                      ,
                      mode_context
#endif  //! CONFIG_C076_INTER_MOD_CTX
#if CONFIG_EXTENDED_WARP_PREDICTION
                      ,
                      derive_wrl ? warp_param_stack[ref_frame] : NULL,
                      derive_wrl ? max_num_of_warp_candidates : 0,
                      derive_wrl ? &valid_num_warp_candidates[ref_frame] : NULL
#endif  // CONFIG_EXTENDED_WARP_PREDICTION

    );
#endif  // CONFIG_SEP_COMP_DRL
  }
#else
  setup_ref_mv_list(cm, xd, ref_frame, &ref_mv_count[ref_frame],
                    ref_mv_stack[ref_frame], ref_mv_weight[ref_frame],
                    mv_ref_list ? mv_ref_list[ref_frame] : NULL, gm_mv, mi_row,
                    mi_col
#if !CONFIG_C076_INTER_MOD_CTX
                    ,
                    mode_context
#endif  //! CONFIG_C076_INTER_MOD_CTX
#if CONFIG_EXTENDED_WARP_PREDICTION
                    ,
                    derive_wrl ? warp_param_stack[ref_frame] : NULL,
                    derive_wrl ? max_num_of_warp_candidates : 0,
                    derive_wrl ? &valid_num_warp_candidates[ref_frame] : NULL
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
  );
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
}

void av1_find_best_ref_mvs(int_mv *mvlist, int_mv *nearest_mv, int_mv *near_mv,
                           MvSubpelPrecision precision) {
  int i;
  // Make sure all the candidates are properly clamped etc
  for (i = 0; i < MAX_MV_REF_CANDIDATES; ++i) {
    lower_mv_precision(&mvlist[i].as_mv, precision);
  }
  *nearest_mv = mvlist[0];
  *near_mv = mvlist[1];
}

void av1_setup_frame_buf_refs(AV1_COMMON *cm) {
  cm->cur_frame->order_hint = cm->current_frame.order_hint;
  cm->cur_frame->display_order_hint = cm->current_frame.display_order_hint;
  cm->cur_frame->absolute_poc = cm->current_frame.absolute_poc;
  cm->cur_frame->pyramid_level = cm->current_frame.pyramid_level;

  MV_REFERENCE_FRAME ref_frame;
  for (ref_frame = 0; ref_frame < INTER_REFS_PER_FRAME; ++ref_frame) {
    const RefCntBuffer *const buf = get_ref_frame_buf(cm, ref_frame);
    if (buf != NULL && ref_frame < cm->ref_frames_info.num_total_refs) {
      cm->cur_frame->ref_order_hints[ref_frame] = buf->order_hint;
      cm->cur_frame->ref_display_order_hint[ref_frame] =
          buf->display_order_hint;
    } else {
      cm->cur_frame->ref_order_hints[ref_frame] = -1;
      cm->cur_frame->ref_display_order_hint[ref_frame] = -1;
    }
  }
}

void av1_setup_frame_sign_bias(AV1_COMMON *cm) {
  memset(&cm->ref_frame_sign_bias, 0, sizeof(cm->ref_frame_sign_bias));
  for (int ref_frame = 0; ref_frame < cm->ref_frames_info.num_future_refs;
       ++ref_frame) {
    const int index = cm->ref_frames_info.future_refs[ref_frame];
    cm->ref_frame_sign_bias[index] = 1;
  }
}

#if CONFIG_MF_IMPROVEMENT
// Get the temporal distance of start_frame to its closest ref frame
// that has interpolation property relative to current frame. Interpolation
// means start_frame and its ref frame are on two sides of current frame
static INLINE int get_dist_to_closest_interp_ref(const AV1_COMMON *const cm,
                                                 MV_REFERENCE_FRAME start_frame,
                                                 const int find_forward_ref) {
  if (start_frame == -1) return INT_MAX;
  const OrderHintInfo *const order_hint_info = &cm->seq_params.order_hint_info;

  const RefCntBuffer *const start_frame_buf =
      get_ref_frame_buf(cm, start_frame);

  if (!is_ref_motion_field_eligible(cm, start_frame_buf)) return INT_MAX;

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int start_frame_order_hint = start_frame_buf->display_order_hint;
  const int cur_order_hint = cm->cur_frame->display_order_hint;
#else
  const int start_frame_order_hint = start_frame_buf->order_hint;
  const int cur_order_hint = cm->cur_frame->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  int abs_closest_ref_offset = INT_MAX;
#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int *const ref_order_hints =
      &start_frame_buf->ref_display_order_hint[0];
#else
  const int *const ref_order_hints = &start_frame_buf->ref_order_hints[0];
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  for (MV_REFERENCE_FRAME ref = 0; ref < INTER_REFS_PER_FRAME; ++ref) {
    if (ref_order_hints[ref] != -1) {
      const int start_to_ref_offset = get_relative_dist(
          order_hint_info, start_frame_order_hint, ref_order_hints[ref]);
      const int cur_to_ref_offset = get_relative_dist(
          order_hint_info, cur_order_hint, ref_order_hints[ref]);
      const int abs_start_to_ref_offset = abs(start_to_ref_offset);
      const int is_two_sides =
          (start_to_ref_offset > 0 && cur_to_ref_offset > 0 &&
           find_forward_ref == 1) ||
          (start_to_ref_offset < 0 && cur_to_ref_offset < 0 &&
           find_forward_ref == 0);
      if (is_two_sides && abs_start_to_ref_offset < abs_closest_ref_offset) {
        abs_closest_ref_offset = abs_start_to_ref_offset;
      }
    }
  }

  return abs_closest_ref_offset;
}
#endif  // CONFIG_MF_IMPROVEMENT

// Check if a reference frame is an overlay frame (i.e., has the same
// order_hint as the current frame).
static INLINE int is_ref_overlay(const AV1_COMMON *const cm, int ref) {
  const OrderHintInfo *const order_hint_info = &cm->seq_params.order_hint_info;
  if (!order_hint_info->enable_order_hint) return -1;
  const RefCntBuffer *const buf = get_ref_frame_buf(cm, ref);
  if (buf == NULL) return -1;
#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int ref_order_hint = buf->display_order_hint;
  for (int r = 0; r < INTER_REFS_PER_FRAME; ++r) {
    if (buf->ref_display_order_hint[r] == -1) continue;
    const int ref_ref_order_hint = buf->ref_display_order_hint[r];
    if (get_relative_dist(order_hint_info, ref_order_hint,
                          ref_ref_order_hint) == 0)
      return 1;
  }
#else
  const int ref_order_hint = buf->order_hint;
  for (int r = 0; r < INTER_REFS_PER_FRAME; ++r) {
    if (buf->ref_order_hints[r] == -1) continue;
    const int ref_ref_order_hint = buf->ref_order_hints[r];
    if (get_relative_dist(order_hint_info, ref_order_hint,
                          ref_ref_order_hint) == 0)
      return 1;
  }
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  return 0;
}

#if CONFIG_TMVP_MEM_OPT
// Performs a topological sort on the reference frames so they are sorted by
// their dependency.
static void recur_topo_sort_refs(const AV1_COMMON *cm, const bool *is_overlay,
                                 int *rf_stack, int *visited, int *stack_count,
                                 int rf) {
  visited[rf] = 1;
  const RefCntBuffer *const buf = get_ref_frame_buf(cm, rf);
  for (int i = 0; i < INTER_REFS_PER_FRAME; i++) {
    if (buf->ref_display_order_hint[i] < 0) continue;
    int found_rf = -1;
    for (int j = 0; j < cm->ref_frames_info.num_total_refs; j++) {
      if ((int)get_ref_frame_buf(cm, j)->display_order_hint ==
          buf->ref_display_order_hint[i]) {
        if (is_overlay[j]) continue;
        found_rf = j;
        break;
      }
    }
    if (found_rf == -1) continue;
    if (visited[found_rf] == 0) {
      recur_topo_sort_refs(cm, is_overlay, rf_stack, visited, stack_count,
                           found_rf);
    }
  }
  rf_stack[*stack_count] = rf;
  *stack_count = *stack_count + 1;
}

// Whether a reference frame buffer had a reference frame in the future.
static int has_future_ref(const AV1_COMMON *cm, int rf) {
  if (rf < 0) return 0;
  RefCntBuffer *buf = get_ref_frame_buf(cm, rf);
  for (int i = 0; i < INTER_REFS_PER_FRAME; i++) {
    if (buf->ref_display_order_hint[i] >= 0 &&
        buf->ref_display_order_hint[i] > (int)buf->display_order_hint) {
      return 1;
    }
  }
  return 0;
}

// Whether a reference frame buffer had a reference frame in the past.
static int has_past_ref(const AV1_COMMON *cm, int rf) {
  if (rf < 0) return 0;
  RefCntBuffer *buf = get_ref_frame_buf(cm, rf);
  for (int i = 0; i < INTER_REFS_PER_FRAME; i++) {
    if (buf->ref_display_order_hint[i] >= 0 &&
        buf->ref_display_order_hint[i] < (int)buf->display_order_hint) {
      return 1;
    }
  }
  return 0;
}

// Struct to store the reference frame motion field candidates.
struct ProcessRefTMVP {
  int type;  // 0 means with side, 1 means with target frame.
  int side;  // 0 means right to left, 1 means left to right.
  int start_frame;
  int target_frame;
};

// Add a reference frame motion field candidate, if we have not reached the
// maximum allowed.
static void check_and_add_process_ref(const AV1_COMMON *cm, int max_check,
                                      int type, int start_frame,
                                      int target_frame, int side,
                                      int checked_ref[INTER_REFS_PER_FRAME][2],
                                      int *checked_count,
                                      struct ProcessRefTMVP *process_ref,
                                      int *process_count) {
  if (get_ref_frame_buf(cm, start_frame) == NULL ||
      get_ref_frame_buf(cm, start_frame)->frame_type != INTER_FRAME)
    return;
  if (!checked_ref[start_frame][side] && *checked_count < max_check) {
    checked_ref[start_frame][side] = 1;
    (*checked_count)++;
  }
  if (checked_ref[start_frame][side]) {
    process_ref[*process_count].type = type;
    process_ref[*process_count].start_frame = start_frame;
    process_ref[*process_count].side = side;
    process_ref[*process_count].target_frame = target_frame;
    (*process_count)++;
  }
}

#if CONFIG_MV_TRAJECTORY
// Check if the mv intersects with exisiting trajectories, and if yes, update
// the trajectories.
static INLINE void check_traj_intersect(AV1_COMMON *cm,
                                        MV_REFERENCE_FRAME start_frame,
                                        MV_REFERENCE_FRAME end_frame,
                                        const MV *mv, int start_row,
                                        int start_col, int mvs_cols) {
  const int start_offset = start_row * mvs_cols + start_col;
  assert(start_row % cm->tmvp_sample_step == 0);
  assert(start_col % cm->tmvp_sample_step == 0);
  // Check starting point
  if (cm->blk_id_map[start_frame][start_offset] >= 0) {
    if (end_frame != NONE_FRAME) {
      int traj_id = cm->blk_id_map[start_frame][start_offset];
      int traj_row = traj_id / mvs_cols;
      int traj_col = traj_id % mvs_cols;
      assert(traj_row % cm->tmvp_sample_step == 0);
      assert(traj_col % cm->tmvp_sample_step == 0);
      if (cm->id_offset_map[end_frame][traj_id].as_int == INVALID_MV) {
        // Update trajectory mv
        cm->id_offset_map[end_frame][traj_id].as_mv.row =
            cm->id_offset_map[start_frame][traj_id].as_mv.row + mv->row;
        cm->id_offset_map[end_frame][traj_id].as_mv.col =
            cm->id_offset_map[start_frame][traj_id].as_mv.col + mv->col;
        // Update reverse mapping
        int end_row = 0, end_col = 0;
        int pos_valid;
        if (cm->id_offset_map[end_frame][traj_id].as_int == 0) {
          pos_valid = 1;
          end_row = traj_row;
          end_col = traj_col;
        } else {
          pos_valid = get_block_position(
              cm, &end_row, &end_col, traj_row, traj_col,
              cm->id_offset_map[end_frame][traj_id].as_mv, 0);
          end_row = (end_row / cm->tmvp_sample_step) * cm->tmvp_sample_step;
          end_col = (end_col / cm->tmvp_sample_step) * cm->tmvp_sample_step;
        }
        if (pos_valid) {
          cm->blk_id_map[end_frame][end_row * mvs_cols + end_col] = traj_id;
        }
      }
    }
    return;
  }

  // Check ending point
  if (end_frame != NONE_FRAME) {
    int end_row = 0, end_col = 0;
    int pos_valid;
    if (mv->col == 0 && mv->row == 0) {
      pos_valid = 1;
      end_row = start_row;
      end_col = start_col;
    } else {
      pos_valid = get_block_position(cm, &end_row, &end_col, start_row,
                                     start_col, *mv, 0);
      end_row = (end_row / cm->tmvp_sample_step) * cm->tmvp_sample_step;
      end_col = (end_col / cm->tmvp_sample_step) * cm->tmvp_sample_step;
    }
    const int end_offset = end_row * mvs_cols + end_col;
    if (pos_valid && cm->blk_id_map[end_frame][end_offset] >= 0) {
      int traj_id = cm->blk_id_map[end_frame][end_offset];
      int traj_row = traj_id / mvs_cols;
      int traj_col = traj_id % mvs_cols;
      assert(traj_row % cm->tmvp_sample_step == 0);
      assert(traj_col % cm->tmvp_sample_step == 0);
      if (cm->id_offset_map[start_frame][traj_id].as_int == INVALID_MV) {
        // Update trajectory mv
        cm->id_offset_map[start_frame][traj_id].as_mv.row =
            cm->id_offset_map[end_frame][traj_id].as_mv.row - mv->row;
        cm->id_offset_map[start_frame][traj_id].as_mv.col =
            cm->id_offset_map[end_frame][traj_id].as_mv.col - mv->col;
        // Update reverse mapping
        int new_start_row = 0, new_start_col = 0;
        int new_pos_valid;
        if (cm->id_offset_map[start_frame][traj_id].as_int == 0) {
          new_pos_valid = 1;
          new_start_row = traj_row;
          new_start_col = traj_col;
        } else {
          new_pos_valid = get_block_position(
              cm, &new_start_row, &new_start_col, traj_row, traj_col,
              cm->id_offset_map[start_frame][traj_id].as_mv, 0);
          new_start_row =
              (new_start_row / cm->tmvp_sample_step) * cm->tmvp_sample_step;
          new_start_col =
              (new_start_col / cm->tmvp_sample_step) * cm->tmvp_sample_step;
        }
        if (new_pos_valid) {
          cm->blk_id_map[start_frame]
                        [new_start_row * mvs_cols + new_start_col] = traj_id;
        }
      }

      return;
    }
  }
}
#endif  // CONFIG_MV_TRAJECTORY

// Calculate the projected motion field from the TMVP mvs that points from
// start_frame to target_frame.
static int motion_field_projection_start_target(
    AV1_COMMON *cm, MV_REFERENCE_FRAME start_frame,
    MV_REFERENCE_FRAME target_frame) {
  const int cur_order_hint = cm->cur_frame->display_order_hint;
  int start_order_hint = get_ref_frame_buf(cm, start_frame)->display_order_hint;
  int target_order_hint =
      get_ref_frame_buf(cm, target_frame)->display_order_hint;

  OrderHintInfo *order_hint_info = &cm->seq_params.order_hint_info;

  int ref_frame_offset =
      get_relative_dist(order_hint_info, start_order_hint, target_order_hint);

  const RefCntBuffer *const start_frame_buf =
      get_ref_frame_buf(cm, start_frame);
  if (!is_ref_motion_field_eligible(cm, start_frame_buf)) return 0;

  assert(start_frame_buf->width == cm->width &&
         start_frame_buf->height == cm->height);

  const int *const ref_order_hints = start_frame_buf->ref_display_order_hint;

  int start_to_current_frame_offset =
      get_relative_dist(order_hint_info, start_order_hint, cur_order_hint);

  const int is_backward = ref_frame_offset < 0;
  int mv_idx = is_backward ? 1 : 0;
  if (is_backward) {
    ref_frame_offset = -ref_frame_offset;
    start_to_current_frame_offset = -start_to_current_frame_offset;
  }

  const int temporal_scale_factor =
      tip_derive_scale_factor(start_to_current_frame_offset, ref_frame_offset);
#if CONFIG_MV_TRAJECTORY
  const int ref_temporal_scale_factor = tip_derive_scale_factor(
      -ref_frame_offset + start_to_current_frame_offset, -ref_frame_offset);
#endif  // CONFIG_MV_TRAJECTORY

  TPL_MV_REF *tpl_mvs_base = cm->tpl_mvs;
  const MV_REF *mv_ref_base = start_frame_buf->mvs;
  const int mvs_rows =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_rows, TMVP_SHIFT_BITS);
  const int mvs_cols =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  const int mvs_stride = mvs_cols;
  for (int blk_row = 0; blk_row < mvs_rows; blk_row += cm->tmvp_sample_step) {
    for (int blk_col = 0; blk_col < mvs_cols; blk_col += cm->tmvp_sample_step) {
      const MV_REF *mv_ref = &mv_ref_base[blk_row * mvs_stride + blk_col];
      if (is_inter_ref_frame(mv_ref->ref_frame[mv_idx])) {
        const int ref_frame_order_hint =
            ref_order_hints[mv_ref->ref_frame[mv_idx]];
        if (ref_frame_order_hint == target_order_hint) {
#if CONFIG_MV_TRAJECTORY
          check_traj_intersect(cm, start_frame, target_frame,
                               &mv_ref->mv[mv_idx].as_mv, blk_row, blk_col,
                               mvs_cols);
#endif  // CONFIG_MV_TRAJECTORY
          MV ref_mv = mv_ref->mv[mv_idx].as_mv;
          int_mv this_mv;
          int mi_r = 0;
          int mi_c = 0;
          tip_get_mv_projection(&this_mv.as_mv, ref_mv, temporal_scale_factor);
          int pos_valid;
          if (this_mv.as_int == 0) {
            pos_valid = 1;
            mi_r = blk_row;
            mi_c = blk_col;
          } else {
            pos_valid = get_block_position(cm, &mi_r, &mi_c, blk_row, blk_col,
                                           this_mv.as_mv, 0);
            mi_r = (mi_r / cm->tmvp_sample_step) * cm->tmvp_sample_step;
            mi_c = (mi_c / cm->tmvp_sample_step) * cm->tmvp_sample_step;
          }

          if (pos_valid) {
            const int mi_offset = mi_r * mvs_stride + mi_c;
            if (tpl_mvs_base[mi_offset].mfmv0.as_int == INVALID_MV) {
#if CONFIG_MV_TRAJECTORY
              int traj_id = mi_offset;

              cm->id_offset_map[start_frame][traj_id].as_mv.row =
                  -this_mv.as_mv.row;
              cm->id_offset_map[start_frame][traj_id].as_mv.col =
                  -this_mv.as_mv.col;
              cm->blk_id_map[start_frame][blk_row * mvs_cols + blk_col] =
                  traj_id;

              MV target_frame_mv;
              tip_get_mv_projection(&target_frame_mv, ref_mv,
                                    ref_temporal_scale_factor);
              cm->id_offset_map[target_frame][traj_id].as_mv = target_frame_mv;
              int target_row = 0, target_col = 0;
              int target_pos_valid;
              if (ref_mv.row == 0 && ref_mv.col == 0) {
                target_pos_valid = 1;
                target_row = blk_row;
                target_col = blk_col;
              } else {
                target_pos_valid = get_block_position(
                    cm, &target_row, &target_col, blk_row, blk_col, ref_mv, 0);
                target_row =
                    (target_row / cm->tmvp_sample_step) * cm->tmvp_sample_step;
                target_col =
                    (target_col / cm->tmvp_sample_step) * cm->tmvp_sample_step;
              }

              if (target_pos_valid) {
                cm->blk_id_map[target_frame]
                              [target_row * mvs_cols + target_col] = traj_id;
              }
#endif  // CONFIG_MV_TRAJECTORY

              if (is_backward) {
                ref_mv.row = -ref_mv.row;
                ref_mv.col = -ref_mv.col;
              }
              tpl_mvs_base[mi_offset].mfmv0.as_mv.row = ref_mv.row;
              tpl_mvs_base[mi_offset].mfmv0.as_mv.col = ref_mv.col;
              tpl_mvs_base[mi_offset].ref_frame_offset = ref_frame_offset;
            }
          }
        }
      }
    }
  }
  return 1;
}

// Calculate the projected motion field from the TMVP mvs that points from
// start_frame to one side.
static int motion_field_projection_side(AV1_COMMON *cm,
                                        MV_REFERENCE_FRAME start_frame,
                                        int side_idx) {
  TPL_MV_REF *tpl_mvs_base = cm->tpl_mvs;
  int ref_offset[INTER_REFS_PER_FRAME] = { 0 };

  const RefCntBuffer *const start_frame_buf =
      get_ref_frame_buf(cm, start_frame);
  if (!is_ref_motion_field_eligible(cm, start_frame_buf)) return 0;

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int start_frame_order_hint = start_frame_buf->display_order_hint;
  const int cur_order_hint = cm->cur_frame->display_order_hint;
#else
  const int start_frame_order_hint = start_frame_buf->order_hint;
  const int cur_order_hint = cm->cur_frame->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  int start_to_current_frame_offset = get_relative_dist(
      &cm->seq_params.order_hint_info, start_frame_order_hint, cur_order_hint);

  if (abs(start_to_current_frame_offset) > MAX_FRAME_DISTANCE) {
    return 0;
  }

  int temporal_scale_factor[REF_FRAMES] = { 0 };
#if CONFIG_MV_TRAJECTORY
  int ref_temporal_scale_factor[REF_FRAMES] = { 0 };
#endif  // CONFIG_MV_TRAJECTORY
  int ref_abs_offset[REF_FRAMES] = { 0 };

  assert(start_frame_buf->width == cm->width &&
         start_frame_buf->height == cm->height);

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int *const ref_order_hints =
      &start_frame_buf->ref_display_order_hint[0];
#else
  const int *const ref_order_hints = &start_frame_buf->ref_order_hints[0];
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  for (MV_REFERENCE_FRAME rf = 0; rf < INTER_REFS_PER_FRAME; ++rf) {
    if (ref_order_hints[rf] != -1) {
      ref_offset[rf] =
          get_relative_dist(&cm->seq_params.order_hint_info,
                            start_frame_order_hint, ref_order_hints[rf]);
      ref_abs_offset[rf] = abs(ref_offset[rf]);
      temporal_scale_factor[rf] = tip_derive_scale_factor(
          start_to_current_frame_offset, ref_offset[rf]);
#if CONFIG_MV_TRAJECTORY
      int ref_to_current_frame_offset =
          -ref_offset[rf] + start_to_current_frame_offset;
      ref_temporal_scale_factor[rf] =
          tip_derive_scale_factor(ref_to_current_frame_offset, -ref_offset[rf]);
#endif  // CONFIG_MV_TRAJECTORY
    }
  }

#if CONFIG_MV_TRAJECTORY
  int start_ref_map[INTER_REFS_PER_FRAME];
  for (int k = 0; k < INTER_REFS_PER_FRAME; k++) {
    start_ref_map[k] = NONE_FRAME;
    for (int rf = 0; rf < cm->ref_frames_info.num_total_refs; rf++) {
      if ((int)get_ref_frame_buf(cm, rf)->display_order_hint ==
          start_frame_buf->ref_display_order_hint[k]) {
        start_ref_map[k] = rf;
        break;
      }
    }
  }
#endif  // CONFIG_MV_TRAJECTORY

  MV_REF *mv_ref_base = start_frame_buf->mvs;
  const int mvs_rows =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_rows, TMVP_SHIFT_BITS);
  const int mvs_cols =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  const int mvs_stride = mvs_cols;
  for (int blk_row = 0; blk_row < mvs_rows; blk_row += cm->tmvp_sample_step) {
    for (int blk_col = 0; blk_col < mvs_cols; blk_col += cm->tmvp_sample_step) {
      MV_REF *mv_ref = &mv_ref_base[blk_row * mvs_cols + blk_col];
      const MV_REFERENCE_FRAME ref_frame = mv_ref->ref_frame[side_idx];
      if (is_inter_ref_frame(ref_frame)) {
#if CONFIG_MV_TRAJECTORY
        MV_REFERENCE_FRAME end_frame = start_ref_map[ref_frame];
        check_traj_intersect(cm, start_frame, end_frame,
                             &mv_ref->mv[side_idx].as_mv, blk_row, blk_col,
                             mvs_cols);
#endif  // CONFIG_MV_TRAJECTORY

        int ref_frame_offset = ref_offset[ref_frame];
        int pos_valid = ref_abs_offset[ref_frame] <= MAX_FRAME_DISTANCE;
        if ((side_idx == 0 && ref_frame_offset < 0) ||
            (side_idx == 1 && ref_frame_offset > 0)) {
          pos_valid = 0;
        }
        if (pos_valid) {
          MV ref_mv = mv_ref->mv[side_idx].as_mv;
          int_mv this_mv;
          int mi_r = blk_row;
          int mi_c = blk_col;
          if (mv_ref->mv[side_idx].as_int != 0) {
            tip_get_mv_projection(&this_mv.as_mv, ref_mv,
                                  temporal_scale_factor[ref_frame]);
            pos_valid = get_block_position(cm, &mi_r, &mi_c, blk_row, blk_col,
                                           this_mv.as_mv, 0);
            mi_r = (mi_r / cm->tmvp_sample_step) * cm->tmvp_sample_step;
            mi_c = (mi_c / cm->tmvp_sample_step) * cm->tmvp_sample_step;
          } else {
            this_mv.as_int = 0;
          }
          if (pos_valid) {
            const int mi_offset = mi_r * mvs_stride + mi_c;
            if (tpl_mvs_base[mi_offset].mfmv0.as_int == INVALID_MV) {
#if CONFIG_MV_TRAJECTORY
              int traj_id = mi_offset;

              cm->id_offset_map[start_frame][traj_id].as_mv.row =
                  -this_mv.as_mv.row;
              cm->id_offset_map[start_frame][traj_id].as_mv.col =
                  -this_mv.as_mv.col;
              cm->blk_id_map[start_frame][blk_row * mvs_cols + blk_col] =
                  traj_id;

              if (end_frame != NONE_FRAME) {
                MV end_frame_mv;
                tip_get_mv_projection(&end_frame_mv, ref_mv,
                                      ref_temporal_scale_factor[ref_frame]);
                cm->id_offset_map[end_frame][traj_id].as_mv = end_frame_mv;
                int end_row = 0, end_col = 0;
                int end_pos_valid;
                if (ref_mv.row == 0 && ref_mv.col == 0) {
                  end_pos_valid = 1;
                  end_row = blk_row;
                  end_col = blk_col;
                } else {
                  end_pos_valid = get_block_position(
                      cm, &end_row, &end_col, blk_row, blk_col, ref_mv, 0);
                  end_row =
                      (end_row / cm->tmvp_sample_step) * cm->tmvp_sample_step;
                  end_col =
                      (end_col / cm->tmvp_sample_step) * cm->tmvp_sample_step;
                }

                if (end_pos_valid) {
                  cm->blk_id_map[end_frame][end_row * mvs_cols + end_col] =
                      traj_id;
                }
              }
#endif  // CONFIG_MV_TRAJECTORY

              if (side_idx == 1) {
                ref_mv.row = -ref_mv.row;
                ref_mv.col = -ref_mv.col;
                ref_frame_offset = ref_abs_offset[ref_frame];
              }
              tpl_mvs_base[mi_offset].mfmv0.as_mv.row = ref_mv.row;
              tpl_mvs_base[mi_offset].mfmv0.as_mv.col = ref_mv.col;
              tpl_mvs_base[mi_offset].ref_frame_offset = ref_frame_offset;
            }
          }
        }
      }
    }
  }
  return 1;
}

// Whether to do interpolation for sampled TMVP mvs.
#define DO_AVG_FILL 1
// Interpolate the sampled tpl_mvs.
void av1_fill_tpl_mvs_sample_gap(AV1_COMMON *cm) {
  if (cm->tmvp_sample_step != 2) {
    return;
  }
  const int mvs_rows =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_rows, TMVP_SHIFT_BITS);
  const int mvs_cols =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);

  for (int r = 0; r < mvs_rows; r += cm->tmvp_sample_step) {
    for (int c = 0; c < mvs_cols; c += cm->tmvp_sample_step) {
      int offset = r * mvs_cols + c;
      if (cm->tpl_mvs[offset].mfmv0.as_int == INVALID_MV) continue;

      int avg_offset[3] = { 0 };  // [hor, ver, diag]
      int avg[3][2] = { 0 };      // [hor, ver, diag][row, col]

      // this
      if (cm->tpl_mvs[offset].mfmv0.as_int != INVALID_MV) {
        avg_offset[0] += cm->tpl_mvs[offset].ref_frame_offset;
        avg_offset[1] += cm->tpl_mvs[offset].ref_frame_offset;
        avg_offset[2] += cm->tpl_mvs[offset].ref_frame_offset;

        avg[0][0] += cm->tpl_mvs[offset].mfmv0.as_mv.row;
        avg[1][0] += cm->tpl_mvs[offset].mfmv0.as_mv.row;
        avg[2][0] += cm->tpl_mvs[offset].mfmv0.as_mv.row;

        avg[0][1] += cm->tpl_mvs[offset].mfmv0.as_mv.col;
        avg[1][1] += cm->tpl_mvs[offset].mfmv0.as_mv.col;
        avg[2][1] += cm->tpl_mvs[offset].mfmv0.as_mv.col;
      }

      if (DO_AVG_FILL) {
        // right
        int offset_right = offset + cm->tmvp_sample_step;
        if (c + cm->tmvp_sample_step < mvs_cols &&
            cm->tpl_mvs[offset_right].mfmv0.as_int != INVALID_MV) {
          avg_offset[0] += cm->tpl_mvs[offset_right].ref_frame_offset;
          avg_offset[2] += cm->tpl_mvs[offset_right].ref_frame_offset;

          avg[0][0] += cm->tpl_mvs[offset_right].mfmv0.as_mv.row;
          avg[2][0] += cm->tpl_mvs[offset_right].mfmv0.as_mv.row;

          avg[0][1] += cm->tpl_mvs[offset_right].mfmv0.as_mv.col;
          avg[2][1] += cm->tpl_mvs[offset_right].mfmv0.as_mv.col;
        }

        // lower
        int offset_lower = offset + cm->tmvp_sample_step * mvs_cols;
        if (r + cm->tmvp_sample_step < mvs_rows &&
            cm->tpl_mvs[offset_lower].mfmv0.as_int != INVALID_MV) {
          avg_offset[1] += cm->tpl_mvs[offset_lower].ref_frame_offset;
          avg_offset[2] += cm->tpl_mvs[offset_lower].ref_frame_offset;

          avg[1][0] += cm->tpl_mvs[offset_lower].mfmv0.as_mv.row;
          avg[2][0] += cm->tpl_mvs[offset_lower].mfmv0.as_mv.row;

          avg[1][1] += cm->tpl_mvs[offset_lower].mfmv0.as_mv.col;
          avg[2][1] += cm->tpl_mvs[offset_lower].mfmv0.as_mv.col;
        }

        // lower_right
        int offset_lower_right =
            offset + cm->tmvp_sample_step * mvs_cols + cm->tmvp_sample_step;
        if (r + cm->tmvp_sample_step < mvs_rows &&
            c + cm->tmvp_sample_step < mvs_cols &&
            cm->tpl_mvs[offset_lower_right].mfmv0.as_int != INVALID_MV) {
          avg_offset[2] += cm->tpl_mvs[offset_lower_right].ref_frame_offset;

          avg[2][0] += cm->tpl_mvs[offset_lower_right].mfmv0.as_mv.row;
          avg[2][1] += cm->tpl_mvs[offset_lower_right].mfmv0.as_mv.col;
        }
      }

      if (c + 1 < mvs_cols && avg_offset[0] > 0) {
        assert(cm->tpl_mvs[offset + 1].mfmv0.as_int == INVALID_MV);
        cm->tpl_mvs[offset + 1].mfmv0.as_mv.row = DIVIDE_AND_ROUND_SIGNED(
            avg[0][0] * cm->tpl_mvs[offset].ref_frame_offset, avg_offset[0]);
        cm->tpl_mvs[offset + 1].mfmv0.as_mv.col = DIVIDE_AND_ROUND_SIGNED(
            avg[0][1] * cm->tpl_mvs[offset].ref_frame_offset, avg_offset[0]);
        cm->tpl_mvs[offset + 1].ref_frame_offset =
            cm->tpl_mvs[offset].ref_frame_offset;
      }
      if (r + 1 < mvs_rows && avg_offset[1] > 0) {
        assert(cm->tpl_mvs[offset + mvs_cols].mfmv0.as_int == INVALID_MV);
        cm->tpl_mvs[offset + mvs_cols].mfmv0.as_mv.row =
            DIVIDE_AND_ROUND_SIGNED(
                avg[1][0] * cm->tpl_mvs[offset].ref_frame_offset,
                avg_offset[1]);
        cm->tpl_mvs[offset + mvs_cols].mfmv0.as_mv.col =
            DIVIDE_AND_ROUND_SIGNED(
                avg[1][1] * cm->tpl_mvs[offset].ref_frame_offset,
                avg_offset[1]);
        cm->tpl_mvs[offset + mvs_cols].ref_frame_offset =
            cm->tpl_mvs[offset].ref_frame_offset;
      }
      if (r + 1 < mvs_rows && c + 1 < mvs_cols && avg_offset[2] > 0) {
        assert(cm->tpl_mvs[offset + mvs_cols + 1].mfmv0.as_int == INVALID_MV);
        cm->tpl_mvs[offset + mvs_cols + 1].mfmv0.as_mv.row =
            DIVIDE_AND_ROUND_SIGNED(
                avg[2][0] * cm->tpl_mvs[offset].ref_frame_offset,
                avg_offset[2]);
        cm->tpl_mvs[offset + mvs_cols + 1].mfmv0.as_mv.col =
            DIVIDE_AND_ROUND_SIGNED(
                avg[2][1] * cm->tpl_mvs[offset].ref_frame_offset,
                avg_offset[2]);
        cm->tpl_mvs[offset + mvs_cols + 1].ref_frame_offset =
            cm->tpl_mvs[offset].ref_frame_offset;
      }
    }
  }
}

#if CONFIG_MV_TRAJECTORY
// Interpolate the sampled id_offset_map.
static void fill_id_offset_sample_gap(AV1_COMMON *cm) {
  if (cm->tmvp_sample_step != 2) {
    return;
  }
  const int mvs_rows =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_rows, TMVP_SHIFT_BITS);
  const int mvs_cols =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  for (int rf = 0; rf < cm->ref_frames_info.num_total_refs; rf++) {
    for (int r = 0; r < mvs_rows; r += cm->tmvp_sample_step) {
      for (int c = 0; c < mvs_cols; c += cm->tmvp_sample_step) {
        int offset = r * mvs_cols + c;
        if (cm->id_offset_map[rf][offset].as_int == INVALID_MV) continue;

        int count[3] = { 0 };   // [hor, ver, diag]
        int avg[3][2] = { 0 };  // [hor, ver, diag][row, col]

        // this
        if (cm->id_offset_map[rf][offset].as_int != INVALID_MV) {
          count[0]++;
          count[1]++;
          count[2]++;

          avg[0][0] += cm->id_offset_map[rf][offset].as_mv.row;
          avg[1][0] += cm->id_offset_map[rf][offset].as_mv.row;
          avg[2][0] += cm->id_offset_map[rf][offset].as_mv.row;

          avg[0][1] += cm->id_offset_map[rf][offset].as_mv.col;
          avg[1][1] += cm->id_offset_map[rf][offset].as_mv.col;
          avg[2][1] += cm->id_offset_map[rf][offset].as_mv.col;
        }

        if (DO_AVG_FILL) {
          // right
          int offset_right = offset + cm->tmvp_sample_step;
          if (c + cm->tmvp_sample_step < mvs_cols &&
              cm->id_offset_map[rf][offset_right].as_int != INVALID_MV) {
            count[0]++;
            count[2]++;

            avg[0][0] += cm->id_offset_map[rf][offset_right].as_mv.row;
            avg[2][0] += cm->id_offset_map[rf][offset_right].as_mv.row;

            avg[0][1] += cm->id_offset_map[rf][offset_right].as_mv.col;
            avg[2][1] += cm->id_offset_map[rf][offset_right].as_mv.col;
          }

          // lower
          int offset_lower = offset + cm->tmvp_sample_step * mvs_cols;
          if (r + cm->tmvp_sample_step < mvs_rows &&
              cm->id_offset_map[rf][offset_lower].as_int != INVALID_MV) {
            count[1]++;
            count[2]++;

            avg[1][0] += cm->id_offset_map[rf][offset_lower].as_mv.row;
            avg[2][0] += cm->id_offset_map[rf][offset_lower].as_mv.row;

            avg[1][1] += cm->id_offset_map[rf][offset_lower].as_mv.col;
            avg[2][1] += cm->id_offset_map[rf][offset_lower].as_mv.col;
          }

          // lower_right
          int offset_lower_right =
              offset + cm->tmvp_sample_step * mvs_cols + cm->tmvp_sample_step;
          if (r + cm->tmvp_sample_step < mvs_rows &&
              c + cm->tmvp_sample_step < mvs_cols &&
              cm->id_offset_map[rf][offset_lower_right].as_int != INVALID_MV) {
            count[2]++;

            avg[2][0] += cm->id_offset_map[rf][offset_lower_right].as_mv.row;
            avg[2][1] += cm->id_offset_map[rf][offset_lower_right].as_mv.col;
          }
        }

        if (c + 1 < mvs_cols && count[0] > 0) {
          assert(cm->id_offset_map[rf][offset + 1].as_int == INVALID_MV);
          cm->id_offset_map[rf][offset + 1].as_mv.row =
              DIVIDE_AND_ROUND_SIGNED(avg[0][0], count[0]);
          cm->id_offset_map[rf][offset + 1].as_mv.col =
              DIVIDE_AND_ROUND_SIGNED(avg[0][1], count[0]);
        }
        if (r + 1 < mvs_rows && count[1] > 0) {
          assert(cm->id_offset_map[rf][offset + mvs_cols].as_int == INVALID_MV);
          cm->id_offset_map[rf][offset + mvs_cols].as_mv.row =
              DIVIDE_AND_ROUND_SIGNED(avg[1][0], count[1]);
          cm->id_offset_map[rf][offset + mvs_cols].as_mv.col =
              DIVIDE_AND_ROUND_SIGNED(avg[1][1], count[1]);
        }
        if (r + 1 < mvs_rows && c + 1 < mvs_cols && count[2] > 0) {
          assert(cm->id_offset_map[rf][offset + 1 + mvs_cols].as_int ==
                 INVALID_MV);
          cm->id_offset_map[rf][offset + mvs_cols + 1].as_mv.row =
              DIVIDE_AND_ROUND_SIGNED(avg[2][0], count[2]);
          cm->id_offset_map[rf][offset + mvs_cols + 1].as_mv.col =
              DIVIDE_AND_ROUND_SIGNED(avg[2][1], count[2]);
        }
      }
    }
  }
}

// Interpolate the sampled blk_id_map.
static void fill_block_id_sample_gap(AV1_COMMON *cm) {
  if (cm->tmvp_sample_step != 2) {
    return;
  }
  const int mvs_rows =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_rows, TMVP_SHIFT_BITS);
  const int mvs_cols =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  for (int rf = 0; rf < cm->ref_frames_info.num_total_refs; rf++) {
    for (int r = 0; r < mvs_rows; r += cm->tmvp_sample_step) {
      for (int c = 0; c < mvs_cols; c += cm->tmvp_sample_step) {
        int offset = r * mvs_cols + c;
        if (cm->blk_id_map[rf][offset] == -1) continue;

        int count[3] = { 0 };   // [hor, ver, diag]
        int avg[3][2] = { 0 };  // [hor, ver, diag][row, col]

        int this_traj_row = -1;
        int this_traj_col = -1;
        // this
        if (cm->blk_id_map[rf][offset] != -1) {
          this_traj_row = cm->blk_id_map[rf][offset] / mvs_cols;
          this_traj_col = cm->blk_id_map[rf][offset] % mvs_cols;

          count[0]++;
          count[1]++;
          count[2]++;

          avg[0][0] += this_traj_row;
          avg[1][0] += this_traj_row + 1;
          avg[2][0] += this_traj_row + 1;

          avg[0][1] += this_traj_col + 1;
          avg[1][1] += this_traj_col;
          avg[2][1] += this_traj_col + 1;
        }

        if (DO_AVG_FILL) {
          // right
          int offset_right = offset + cm->tmvp_sample_step;
          if (c + cm->tmvp_sample_step < mvs_cols &&
              cm->blk_id_map[rf][offset_right] != -1) {
            int traj_row = cm->blk_id_map[rf][offset_right] / mvs_cols;
            int traj_col = cm->blk_id_map[rf][offset_right] % mvs_cols;
            count[0]++;
            count[2]++;

            avg[0][0] += traj_row;
            avg[2][0] += traj_row + 1;

            avg[0][1] += traj_col - 1;
            avg[2][1] += traj_col - 1;
          }

          // lower
          int offset_lower = offset + cm->tmvp_sample_step * mvs_cols;
          if (r + cm->tmvp_sample_step < mvs_rows &&
              cm->blk_id_map[rf][offset_lower] != -1) {
            int traj_row = cm->blk_id_map[rf][offset_lower] / mvs_cols;
            int traj_col = cm->blk_id_map[rf][offset_lower] % mvs_cols;
            count[1]++;
            count[2]++;

            avg[1][0] += traj_row - 1;
            avg[2][0] += traj_row - 1;

            avg[1][1] += traj_col;
            avg[2][1] += traj_col + 1;
          }

          // lower_right
          int offset_lower_right =
              offset + cm->tmvp_sample_step * mvs_cols + cm->tmvp_sample_step;
          if (r + cm->tmvp_sample_step < mvs_rows &&
              c + cm->tmvp_sample_step < mvs_cols &&
              cm->blk_id_map[rf][offset_lower_right] != -1) {
            int traj_row = cm->blk_id_map[rf][offset_lower_right] / mvs_cols;
            int traj_col = cm->blk_id_map[rf][offset_lower_right] % mvs_cols;

            count[2]++;

            avg[2][0] += traj_row - 1;
            avg[2][1] += traj_col - 1;
          }
        }

        if (c + 1 < mvs_cols && count[0] > 0) {
          assert(cm->blk_id_map[rf][offset + 1] == -1);
          const int traj_row = DIVIDE_AND_ROUND_SIGNED(avg[0][0], count[0]);
          const int traj_col = DIVIDE_AND_ROUND_SIGNED(avg[0][1], count[0]);
          if (traj_row >= 0 && traj_row < mvs_rows && traj_col >= 0 &&
              traj_col < mvs_cols) {
            cm->blk_id_map[rf][offset + 1] = traj_row * mvs_cols + traj_col;
          }
        }
        if (r + 1 < mvs_rows && count[1] > 0) {
          assert(cm->blk_id_map[rf][offset + mvs_cols] == -1);
          const int traj_row = DIVIDE_AND_ROUND_SIGNED(avg[1][0], count[1]);
          const int traj_col = DIVIDE_AND_ROUND_SIGNED(avg[1][1], count[1]);
          if (traj_row >= 0 && traj_row < mvs_rows && traj_col >= 0 &&
              traj_col < mvs_cols) {
            cm->blk_id_map[rf][offset + mvs_cols] =
                traj_row * mvs_cols + traj_col;
          }
        }
        if (r + 1 < mvs_rows && c + 1 < mvs_cols && count[2] > 0) {
          assert(cm->blk_id_map[rf][offset + mvs_cols + 1] == -1);
          const int traj_row = DIVIDE_AND_ROUND_SIGNED(avg[2][0], count[2]);
          const int traj_col = DIVIDE_AND_ROUND_SIGNED(avg[2][1], count[2]);
          if (traj_row >= 0 && traj_row < mvs_rows && traj_col >= 0 &&
              traj_col < mvs_cols) {
            cm->blk_id_map[rf][offset + mvs_cols + 1] =
                traj_row * mvs_cols + traj_col;
          }
        }
      }
    }
  }
}
#endif  // CONFIG_MV_TRAJECTORY

// Calculate the average MV length in the reference frame motion field
// candidate.
void calc_and_set_avg_lengths(AV1_COMMON *cm, int ref, int side) {
  RefCntBuffer *buf = get_ref_frame_buf(cm, ref);
  const int mvs_cols =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  const int mvs_rows =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_rows, TMVP_SHIFT_BITS);
  int64_t avg_row = 0;
  int64_t avg_col = 0;
  int64_t count = 0;
  for (int r = 0; r < mvs_rows; r += 2) {
    for (int c = 0; c < mvs_cols; c += 2) {
      const MV_REF *mv_ref = &buf->mvs[r * mvs_cols + c];
      if (mv_ref->ref_frame[side] != NONE_FRAME) {
        const int dist =
            abs((int)buf->display_order_hint -
                (int)buf->ref_display_order_hint[mv_ref->ref_frame[side]]);
        avg_row += abs(mv_ref->mv[side].as_mv.row * 2 / dist);
        avg_col += abs(mv_ref->mv[side].as_mv.col * 2 / dist);
        count++;
      }
    }
  }
  buf->avg_row[side] = count == 0 ? 0 : avg_row / count;
  buf->avg_col[side] = count == 0 ? 0 : avg_col / count;
  return;
}

// Determine whether we should use sampling of TMVP mvs for the current frame.
void determine_tmvp_sample_step(AV1_COMMON *cm,
                                int checked_ref[INTER_REFS_PER_FRAME][2]) {
  cm->tmvp_sample_step = 1;
  int small_count = 0;
  int large_count = 0;
  for (int i = 0; i < INTER_REFS_PER_FRAME; i++) {
    for (int j = 0; j < 2; j++) {
      if (!checked_ref[i][j]) continue;
      const RefCntBuffer *buf = get_ref_frame_buf(cm, i);
      if (!is_ref_motion_field_eligible(cm, buf)) continue;
      calc_and_set_avg_lengths(cm, i, j);
      const int dist = abs((int)cm->cur_frame->display_order_hint -
                           (int)buf->display_order_hint);
      if (buf->avg_row[j] * dist / 16 > 8 || buf->avg_col[j] * dist / 16 > 16) {
        large_count++;
      } else {
        small_count++;
      }
    }
  }
  if (large_count > small_count * 2) {
    cm->tmvp_sample_step = 2;
  } else {
    cm->tmvp_sample_step = 1;
  }
}
#else
// Note: motion_filed_projection finds motion vectors of current frame's
// reference frame, and projects them to current frame. To make it clear,
// let's call current frame's reference frame as start frame.
// Call Start frame's reference frames as reference frames.
// Call ref_offset as frame distances between start frame and its reference
// frames.
static int motion_field_projection_bwd(AV1_COMMON *cm,
                                       MV_REFERENCE_FRAME start_frame, int dir,
                                       int overwrite_mv) {
  TPL_MV_REF *tpl_mvs_base = cm->tpl_mvs;
  int ref_offset[INTER_REFS_PER_FRAME] = { 0 };

  const RefCntBuffer *const start_frame_buf =
      get_ref_frame_buf(cm, start_frame);
  if (!is_ref_motion_field_eligible(cm, start_frame_buf)) return 0;

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int start_frame_order_hint = start_frame_buf->display_order_hint;
  const int cur_order_hint = cm->cur_frame->display_order_hint;
#else
  const int start_frame_order_hint = start_frame_buf->order_hint;
  const int cur_order_hint = cm->cur_frame->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  int start_to_current_frame_offset = get_relative_dist(
      &cm->seq_params.order_hint_info, start_frame_order_hint, cur_order_hint);

  if (abs(start_to_current_frame_offset) > MAX_FRAME_DISTANCE) {
    return 0;
  }

  if (dir == 2) start_to_current_frame_offset = -start_to_current_frame_offset;

  int temporal_scale_factor[REF_FRAMES] = { 0 };
  int ref_abs_offset[REF_FRAMES] = { 0 };
  int has_bwd_ref = 0;

  assert(start_frame_buf->width == cm->width &&
         start_frame_buf->height == cm->height);

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int *const ref_order_hints =
      &start_frame_buf->ref_display_order_hint[0];
#else
  const int *const ref_order_hints = &start_frame_buf->ref_order_hints[0];
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  for (MV_REFERENCE_FRAME rf = 0; rf < INTER_REFS_PER_FRAME; ++rf) {
    if (ref_order_hints[rf] != -1) {
      ref_offset[rf] =
          get_relative_dist(&cm->seq_params.order_hint_info,
                            start_frame_order_hint, ref_order_hints[rf]);
      has_bwd_ref |= (ref_offset[rf] < 0);
      ref_abs_offset[rf] = abs(ref_offset[rf]);
      temporal_scale_factor[rf] = tip_derive_scale_factor(
          start_to_current_frame_offset, ref_abs_offset[rf]);
    }
  }
  if (has_bwd_ref == 0) return 0;

  MV_REF *mv_ref_base = start_frame_buf->mvs;
  const int mvs_rows =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_rows, TMVP_SHIFT_BITS);
  const int mvs_cols =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  const int mvs_stride = mvs_cols;
  const int enable_compound_mv = cm->seq_params.enable_tip;
  for (int blk_row = 0; blk_row < mvs_rows; ++blk_row) {
    for (int blk_col = 0; blk_col < mvs_cols; ++blk_col) {
      MV_REF *mv_ref = &mv_ref_base[blk_row * mvs_cols + blk_col];
      for (int idx = 0; idx < 1 + enable_compound_mv; ++idx) {
        const MV_REFERENCE_FRAME ref_frame = mv_ref->ref_frame[idx];
        if (is_inter_ref_frame(ref_frame)) {
          int ref_frame_offset = ref_offset[ref_frame];
          int pos_valid = ref_frame_offset < 0 &&
                          ref_abs_offset[ref_frame] <= MAX_FRAME_DISTANCE;
          if (pos_valid) {
            MV ref_mv = mv_ref->mv[idx].as_mv;
            int_mv this_mv;
            int mi_r = blk_row;
            int mi_c = blk_col;
            if (mv_ref->mv[idx].as_int != 0) {
              tip_get_mv_projection(&this_mv.as_mv, ref_mv,
                                    temporal_scale_factor[ref_frame]);
              pos_valid = get_block_position(cm, &mi_r, &mi_c, blk_row, blk_col,
                                             this_mv.as_mv, 0);
            }
            if (pos_valid) {
              ref_mv.row = -ref_mv.row;
              ref_mv.col = -ref_mv.col;

              const int mi_offset = mi_r * mvs_stride + mi_c;
              if (overwrite_mv ||
                  tpl_mvs_base[mi_offset].mfmv0.as_int == INVALID_MV) {
                tpl_mvs_base[mi_offset].mfmv0.as_mv.row = ref_mv.row;
                tpl_mvs_base[mi_offset].mfmv0.as_mv.col = ref_mv.col;
                tpl_mvs_base[mi_offset].ref_frame_offset =
                    ref_abs_offset[ref_frame];
              }
            }
          }
        }
      }
    }
  }

  return 1;
}

static int motion_field_projection(AV1_COMMON *cm,
                                   MV_REFERENCE_FRAME start_frame, int dir,
                                   int overwrite_mv) {
  TPL_MV_REF *tpl_mvs_base = cm->tpl_mvs;
  int ref_offset[INTER_REFS_PER_FRAME] = { 0 };

  const RefCntBuffer *const start_frame_buf =
      get_ref_frame_buf(cm, start_frame);
  if (!is_ref_motion_field_eligible(cm, start_frame_buf)) return 0;

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int start_frame_order_hint = start_frame_buf->display_order_hint;
  const int cur_order_hint = cm->cur_frame->display_order_hint;
#else
  const int start_frame_order_hint = start_frame_buf->order_hint;
  const int cur_order_hint = cm->cur_frame->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  int start_to_current_frame_offset = get_relative_dist(
      &cm->seq_params.order_hint_info, start_frame_order_hint, cur_order_hint);

  if (abs(start_to_current_frame_offset) > MAX_FRAME_DISTANCE) {
    return 0;
  }

  if (dir == 2) start_to_current_frame_offset = -start_to_current_frame_offset;

  int temporal_scale_factor[REF_FRAMES] = { 0 };
  int ref_abs_offset[REF_FRAMES] = { 0 };

  assert(start_frame_buf->width == cm->width &&
         start_frame_buf->height == cm->height);

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int *const ref_order_hints =
      &start_frame_buf->ref_display_order_hint[0];
#else
  const int *const ref_order_hints = &start_frame_buf->ref_order_hints[0];
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  for (MV_REFERENCE_FRAME rf = 0; rf < INTER_REFS_PER_FRAME; ++rf) {
    if (ref_order_hints[rf] != -1) {
      ref_offset[rf] =
          get_relative_dist(&cm->seq_params.order_hint_info,
                            start_frame_order_hint, ref_order_hints[rf]);
      ref_abs_offset[rf] = abs(ref_offset[rf]);
      temporal_scale_factor[rf] = tip_derive_scale_factor(
          start_to_current_frame_offset, ref_abs_offset[rf]);
    }
  }

  MV_REF *mv_ref_base = start_frame_buf->mvs;
  const int mvs_rows =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_rows, TMVP_SHIFT_BITS);
  const int mvs_cols =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  const int mvs_stride = mvs_cols;
  const int enable_compound_mv = cm->seq_params.enable_tip;
  for (int blk_row = 0; blk_row < mvs_rows; ++blk_row) {
    for (int blk_col = 0; blk_col < mvs_cols; ++blk_col) {
      MV_REF *mv_ref = &mv_ref_base[blk_row * mvs_cols + blk_col];
      for (int idx = 0; idx < 1 + enable_compound_mv; ++idx) {
        const MV_REFERENCE_FRAME ref_frame = mv_ref->ref_frame[idx];
        if (is_inter_ref_frame(ref_frame)) {
          const int ref_frame_offset = ref_offset[ref_frame];
          int pos_valid = ref_frame_offset > 0 &&
                          ref_abs_offset[ref_frame] <= MAX_FRAME_DISTANCE;
          if (pos_valid) {
            MV ref_mv = mv_ref->mv[idx].as_mv;
            int_mv this_mv;
            int mi_r = blk_row;
            int mi_c = blk_col;
            if (mv_ref->mv[idx].as_int != 0) {
              tip_get_mv_projection(&this_mv.as_mv, ref_mv,
                                    temporal_scale_factor[ref_frame]);
              pos_valid = get_block_position(cm, &mi_r, &mi_c, blk_row, blk_col,
                                             this_mv.as_mv, dir >> 1);
            }
            if (pos_valid) {
              const int mi_offset = mi_r * mvs_stride + mi_c;
              if (overwrite_mv ||
                  tpl_mvs_base[mi_offset].mfmv0.as_int == INVALID_MV) {
                tpl_mvs_base[mi_offset].mfmv0.as_mv.row = ref_mv.row;
                tpl_mvs_base[mi_offset].mfmv0.as_mv.col = ref_mv.col;
                tpl_mvs_base[mi_offset].ref_frame_offset = ref_frame_offset;
              }
            }
          }
        }
      }
    }
  }

  return 1;
}
#endif  // CONFIG_TMVP_MEM_OPT

void av1_setup_motion_field(AV1_COMMON *cm) {
#if CONFIG_TMVP_MEM_OPT
  cm->tmvp_sample_step = 0;
#endif  // CONFIG_TMVP_MEM_OPT
  const OrderHintInfo *const order_hint_info = &cm->seq_params.order_hint_info;

  memset(cm->ref_frame_side, 0, sizeof(cm->ref_frame_side));
  if (!order_hint_info->enable_order_hint) return;

  TPL_MV_REF *tpl_mvs_base = cm->tpl_mvs;
  const int mvs_rows =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_rows, TMVP_SHIFT_BITS);
  const int mvs_cols =
      ROUND_POWER_OF_TWO(cm->mi_params.mi_cols, TMVP_SHIFT_BITS);
  int size = mvs_rows * mvs_cols;
  for (int idx = 0; idx < size; ++idx) {
    tpl_mvs_base[idx].mfmv0.as_int = INVALID_MV;
    tpl_mvs_base[idx].ref_frame_offset = 0;
  }

  const RefCntBuffer *ref_buf[INTER_REFS_PER_FRAME];

  for (int i = 0; i < INTER_REFS_PER_FRAME; i++) {
    ref_buf[i] = NULL;
    cm->ref_frame_side[i] = 0;
    cm->ref_frame_relative_dist[i] = 0;
  }
  for (int index = 0; index < cm->ref_frames_info.num_past_refs; index++) {
    const int ref_frame = cm->ref_frames_info.past_refs[index];
    cm->ref_frame_side[ref_frame] = 0;
    const RefCntBuffer *const buf = get_ref_frame_buf(cm, ref_frame);
    ref_buf[ref_frame] = buf;
#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    const int relative_dist =
        get_relative_dist(order_hint_info, buf->display_order_hint,
                          cm->cur_frame->display_order_hint);
#else
    const int relative_dist = get_relative_dist(
        order_hint_info, buf->order_hint, cm->cur_frame->order_hint);
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    cm->ref_frame_relative_dist[ref_frame] = abs(relative_dist);
  }
  for (int index = 0; index < cm->ref_frames_info.num_future_refs; index++) {
    const int ref_frame = cm->ref_frames_info.future_refs[index];
    cm->ref_frame_side[ref_frame] = 1;
    const RefCntBuffer *const buf = get_ref_frame_buf(cm, ref_frame);
    ref_buf[ref_frame] = buf;
#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    const int relative_dist =
        get_relative_dist(order_hint_info, buf->display_order_hint,
                          cm->cur_frame->display_order_hint);
#else
    const int relative_dist = get_relative_dist(
        order_hint_info, buf->order_hint, cm->cur_frame->order_hint);
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    cm->ref_frame_relative_dist[ref_frame] = abs(relative_dist);
  }
  for (int index = 0; index < cm->ref_frames_info.num_cur_refs; index++) {
    const int ref_frame = cm->ref_frames_info.cur_refs[index];
    cm->ref_frame_side[ref_frame] = -1;
    const RefCntBuffer *const buf = get_ref_frame_buf(cm, ref_frame);
    ref_buf[ref_frame] = buf;
#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    const int relative_dist =
        get_relative_dist(order_hint_info, buf->display_order_hint,
                          cm->cur_frame->display_order_hint);
#else
    const int relative_dist = get_relative_dist(
        order_hint_info, buf->order_hint, cm->cur_frame->order_hint);
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    cm->ref_frame_relative_dist[ref_frame] = abs(relative_dist);
  }

  cm->has_bwd_ref = cm->ref_frames_info.num_future_refs ? 1 : 0;

#if CONFIG_TMVP_MEM_OPT
  (void)ref_buf;
#if CONFIG_MV_TRAJECTORY
  int_mv **id_offset_map = cm->id_offset_map;
  int **blk_id_map = cm->blk_id_map;
  for (int rf = 0; rf < INTER_REFS_PER_FRAME; rf++) {
    for (int i = 0; i < mvs_rows * mvs_cols; i++) {
      id_offset_map[rf][i].as_int = INVALID_MV;
      blk_id_map[rf][i] = -1;
    }
  }
#endif  // CONFIG_MV_TRAJECTORY

  // Find the sorted map of refs.
  int sort_ref[INTER_REFS_PER_FRAME] = { 0, 1, 2, 3, 4, 5, 6 };
  int disp_order[INTER_REFS_PER_FRAME] = { 0 };

  bool is_overlay[INTER_REFS_PER_FRAME] = { false };
  for (int rf = cm->ref_frames_info.num_total_refs - 1; rf >= 0; rf--) {
    if (is_ref_overlay(cm, rf) &&
        get_ref_frame_buf(cm, rf)->frame_type != KEY_FRAME) {
      is_overlay[rf] = true;
    }
  }

  for (int rf = 0; rf < cm->ref_frames_info.num_total_refs; rf++) {
    disp_order[rf] = get_ref_frame_buf(cm, rf)->display_order_hint;
  }
  // Sort the points by x.
  for (int i = 0; i < cm->ref_frames_info.num_total_refs; i++) {
    for (int j = i + 1; j < cm->ref_frames_info.num_total_refs; j++) {
      if (disp_order[j] < disp_order[i]) {
        int16_t tmp = disp_order[i];
        disp_order[i] = disp_order[j];
        disp_order[j] = tmp;

        tmp = sort_ref[i];
        sort_ref[i] = sort_ref[j];
        sort_ref[j] = tmp;
      }
    }
  }
  int cur_disp_order = cm->cur_frame->display_order_hint;
  // The idx of rf in sort_ref that is before current frame, and closest.
  int cur_frame_sort_idx = -1;
  for (int rf_idx = 0; rf_idx < cm->ref_frames_info.num_total_refs; rf_idx++) {
    if (disp_order[rf_idx] < cur_disp_order) {
      cur_frame_sort_idx = rf_idx;
    } else {
      break;
    }
  }

  int rf_stack[INTER_REFS_PER_FRAME];
  int visited[INTER_REFS_PER_FRAME] = { 0 };
  int stack_count = 0;
  for (int rf = 0; rf < cm->ref_frames_info.num_total_refs; rf++) {
    if (visited[rf] == 0) {
      recur_topo_sort_refs(cm, is_overlay, rf_stack, visited, &stack_count, rf);
    }
  }

  if (stack_count < 2) return;

  int rf_topo_stack_idx[INTER_REFS_PER_FRAME];
  for (int rf = 0; rf < cm->ref_frames_info.num_total_refs; rf++) {
    rf_topo_stack_idx[rf] = -1;
    for (int stack_idx = 0; stack_idx < stack_count; stack_idx++) {
      if (rf_stack[stack_idx] == rf) {
        rf_topo_stack_idx[rf] = stack_idx;
        break;
      }
    }
  }

  struct ProcessRefTMVP process_ref[20];
  int process_count = 0;
  int checked_ref[INTER_REFS_PER_FRAME][2] = { 0 };
  int checked_count = 0;

  if (cm->seq_params.enable_tip) {
    if (cur_frame_sort_idx >= 0 &&
        cur_frame_sort_idx < cm->ref_frames_info.num_total_refs - 1) {
      cm->tip_ref.ref_frame[0] = sort_ref[cur_frame_sort_idx];
      cm->tip_ref.ref_frame[1] = sort_ref[cur_frame_sort_idx + 1];
      int start_frame, target_frame;
      int side;
      if (rf_topo_stack_idx[cm->tip_ref.ref_frame[0]] >
          rf_topo_stack_idx[cm->tip_ref.ref_frame[1]]) {
        start_frame = cm->tip_ref.ref_frame[0];
        target_frame = cm->tip_ref.ref_frame[1];
        side = 1;  // pointing from left to right
      } else {
        start_frame = cm->tip_ref.ref_frame[1];
        target_frame = cm->tip_ref.ref_frame[0];
        side = 0;  // pointing from right to left
      }
      check_and_add_process_ref(cm, TIP_MFMV_STACK_SIZE, 1, start_frame,
                                target_frame, side, checked_ref, &checked_count,
                                process_ref, &process_count);
    } else {
      cm->tip_ref.ref_frame[0] = NONE_FRAME;
      cm->tip_ref.ref_frame[1] = NONE_FRAME;
    }
  }

  for (int group_idx = 0; group_idx < 2; ++group_idx) {
    int past_ref_sort_idx =
        cur_frame_sort_idx >= group_idx ? cur_frame_sort_idx - group_idx : -1;
    if (past_ref_sort_idx >= 0 &&
        !has_future_ref(cm, sort_ref[past_ref_sort_idx]))
      past_ref_sort_idx = -1;

    int future_ref_sort_idx =
        cur_frame_sort_idx < cm->ref_frames_info.num_total_refs - group_idx - 1
            ? cur_frame_sort_idx + 1 + group_idx
            : -1;
    if (future_ref_sort_idx >= 0 &&
        !has_past_ref(cm, sort_ref[future_ref_sort_idx]))
      future_ref_sort_idx = -1;

    const int past_ref_to_its_ref_dist =
        past_ref_sort_idx >= 0
            ? get_dist_to_closest_interp_ref(cm, sort_ref[past_ref_sort_idx], 0)
            : -1;

    const int future_ref_to_its_ref_dist =
        future_ref_sort_idx >= 0 ? get_dist_to_closest_interp_ref(
                                       cm, sort_ref[future_ref_sort_idx], 1)
                                 : -1;

    if (future_ref_to_its_ref_dist < past_ref_to_its_ref_dist) {
      if (future_ref_sort_idx != -1) {
        check_and_add_process_ref(
            cm, TIP_MFMV_STACK_SIZE, 0, sort_ref[future_ref_sort_idx], -1, 0,
            checked_ref, &checked_count, process_ref, &process_count);
      }
      if (past_ref_sort_idx != -1) {
        check_and_add_process_ref(
            cm, TIP_MFMV_STACK_SIZE, 0, sort_ref[past_ref_sort_idx], -1, 1,
            checked_ref, &checked_count, process_ref, &process_count);
      }
    } else {
      if (past_ref_sort_idx != -1) {
        check_and_add_process_ref(
            cm, TIP_MFMV_STACK_SIZE, 0, sort_ref[past_ref_sort_idx], -1, 1,
            checked_ref, &checked_count, process_ref, &process_count);
      }
      if (future_ref_sort_idx != -1) {
        check_and_add_process_ref(
            cm, TIP_MFMV_STACK_SIZE, 0, sort_ref[future_ref_sort_idx], -1, 0,
            checked_ref, &checked_count, process_ref, &process_count);
      }
    }
  }

  if (cur_frame_sort_idx >= 0) {
    check_and_add_process_ref(cm, TIP_MFMV_STACK_SIZE, 0,
                              sort_ref[cur_frame_sort_idx], -1, 0, checked_ref,
                              &checked_count, process_ref, &process_count);
  }
  if (cur_frame_sort_idx >= 1) {
    check_and_add_process_ref(
        cm, TIP_MFMV_STACK_SIZE, 0, sort_ref[cur_frame_sort_idx - 1], -1, 0,
        checked_ref, &checked_count, process_ref, &process_count);
  }

  for (int ri = stack_count - 1; ri > 0; ri--) {
    int side;
    if (get_ref_frame_buf(cm, rf_stack[ri])->display_order_hint <
        cm->cur_frame->display_order_hint) {
      side = 1;
    } else {
      side = 0;
    }

    if (!checked_ref[rf_stack[ri]][side]) {
      check_and_add_process_ref(cm, MFMV_STACK_SIZE, 0, rf_stack[ri], -1, side,
                                checked_ref, &checked_count, process_ref,
                                &process_count);
    }

    side = !side;
    if (!checked_ref[rf_stack[ri]][side]) {
      check_and_add_process_ref(cm, MFMV_STACK_SIZE, 0, rf_stack[ri], -1, side,
                                checked_ref, &checked_count, process_ref,
                                &process_count);
    }
  }

  determine_tmvp_sample_step(cm, checked_ref);

  for (int pi = 0; pi < process_count; pi++) {
    if (process_ref[pi].type == 0) {
      motion_field_projection_side(cm, process_ref[pi].start_frame,
                                   process_ref[pi].side);
    } else {
      motion_field_projection_start_target(cm, process_ref[pi].start_frame,
                                           process_ref[pi].target_frame);
    }
  }
#if CONFIG_MV_TRAJECTORY
  fill_id_offset_sample_gap(cm);
  fill_block_id_sample_gap(cm);
#endif  // CONFIG_MV_TRAJECTORY
#else
  if (cm->seq_params.enable_tip) {
    av1_derive_tip_nearest_ref_frames_motion_projection(cm);
  }

  int n_refs_used = 0;

  // Implements a strategy where the closest references in the past
  // and future ranked lists are processed first, followed by
  // processing the second closest references up to MFMV_STACK_SIZE.
  //
  // Find two closest past and future references
  int dist[2][2] = { { INT_MAX, INT_MAX }, { INT_MAX, INT_MAX } };
  int closest_ref[2][2] = { { -1, -1 }, { -1, -1 } };
  for (int ref_frame = 0; ref_frame < cm->ref_frames_info.num_total_refs;
       ref_frame++) {
    const int dir = cm->ref_frame_side[ref_frame];
    if (dir == -1 || is_ref_overlay(cm, ref_frame) ||
        !is_ref_motion_field_eligible(cm, ref_buf[ref_frame]))
      continue;
    const int absdist = abs(cm->ref_frames_info.ref_frame_distance[ref_frame]);
    if (absdist < dist[dir][0]) {
      dist[dir][1] = dist[dir][0];
      closest_ref[dir][1] = closest_ref[dir][0];
      dist[dir][0] = absdist;
      closest_ref[dir][0] = ref_frame;
    } else if (absdist < dist[dir][1]) {
      dist[dir][1] = absdist;
      closest_ref[dir][1] = ref_frame;
    }
  }

#if CONFIG_MF_IMPROVEMENT
  // Do projection on group 0 (closest past (backward MV), closest future),
  // group 1(second closest future, second closest past (backward MV)),
  // closest past (forward MV), and then second closest past (forward MVs),
  // without overwriting the MVs.
  // The projection order of the ref frames in group 0 and group 1 depends
  // on the ref frame to its own first ref frame that has interpolation
  // property relative to current frame. Interpolation means two frames are on
  // two sides of current frame
  for (int group_idx = 0; group_idx < 2; ++group_idx) {
    const int past_ref_to_its_ref_dist =
        get_dist_to_closest_interp_ref(cm, closest_ref[0][group_idx], 0);
    const int future_ref_to_its_ref_dist =
        get_dist_to_closest_interp_ref(cm, closest_ref[1][group_idx], 1);
    if (future_ref_to_its_ref_dist < past_ref_to_its_ref_dist) {
      if (closest_ref[1][group_idx] != -1 && n_refs_used < MFMV_STACK_SIZE) {
        n_refs_used +=
            motion_field_projection(cm, closest_ref[1][group_idx], 0, 0);
      }

      if (closest_ref[0][group_idx] != -1 && n_refs_used < MFMV_STACK_SIZE) {
        n_refs_used +=
            motion_field_projection_bwd(cm, closest_ref[0][group_idx], 2, 0);
      }
    } else {
      if (closest_ref[0][group_idx] != -1 && n_refs_used < MFMV_STACK_SIZE) {
        n_refs_used +=
            motion_field_projection_bwd(cm, closest_ref[0][group_idx], 2, 0);
      }
      if (closest_ref[1][group_idx] != -1 && n_refs_used < MFMV_STACK_SIZE) {
        n_refs_used +=
            motion_field_projection(cm, closest_ref[1][group_idx], 0, 0);
      }
    }
  }

  if (closest_ref[0][0] != -1 && n_refs_used < MFMV_STACK_SIZE) {
    n_refs_used += motion_field_projection(cm, closest_ref[0][0], 2, 0);
  }

  if (closest_ref[0][1] != -1 && n_refs_used < MFMV_STACK_SIZE) {
    motion_field_projection(cm, closest_ref[0][1], 2, 0);
  }
#else
  // Do projection on closest past (backward MV), closest future, second
  // closest future, second closest past (backward MV), closest path (forward
  // MV), and then second closest past (forward MVs), without overwriting
  // the MVs.
  if (closest_ref[0][0] != -1) {
    const int ret = motion_field_projection_bwd(cm, closest_ref[0][0], 2, 0);
    n_refs_used += ret;
  }
  if (closest_ref[1][0] != -1) {
    const int ret = motion_field_projection(cm, closest_ref[1][0], 0, 0);
    n_refs_used += ret;
  }
  if (closest_ref[1][1] != -1 && n_refs_used < MFMV_STACK_SIZE) {
    const int ret = motion_field_projection(cm, closest_ref[1][1], 0, 0);
    n_refs_used += ret;
  }
  if (closest_ref[0][0] != -1 && n_refs_used < MFMV_STACK_SIZE) {
    const int ret = motion_field_projection(cm, closest_ref[0][0], 2, 0);
    n_refs_used += ret;
  }
  if (closest_ref[0][1] != -1 && n_refs_used < MFMV_STACK_SIZE) {
    const int ret = motion_field_projection_bwd(cm, closest_ref[0][1], 2, 0);
    n_refs_used += ret;
  }
  if (closest_ref[0][1] != -1 && n_refs_used < MFMV_STACK_SIZE) {
    motion_field_projection(cm, closest_ref[0][1], 2, 0);
  }
#endif  // CONFIG_MF_IMPROVEMENT
#endif  // #if CONFIG_TMVP_MEM_OPT
}

void av1_setup_ref_frame_sides(AV1_COMMON *cm) {
  const OrderHintInfo *const order_hint_info = &cm->seq_params.order_hint_info;

  memset(cm->ref_frame_side, 0, sizeof(cm->ref_frame_side));
  if (!order_hint_info->enable_order_hint) return;

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int cur_order_hint = cm->cur_frame->display_order_hint;
#else
  const int cur_order_hint = cm->cur_frame->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC

  for (int ref_frame = 0; ref_frame < cm->ref_frames_info.num_total_refs;
       ref_frame++) {
    const RefCntBuffer *const buf = get_ref_frame_buf(cm, ref_frame);
    int order_hint = 0;

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    if (buf != NULL) order_hint = buf->display_order_hint;
#else
    if (buf != NULL) order_hint = buf->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    const int relative_dist =
        get_relative_dist(order_hint_info, order_hint, cur_order_hint);
    if (relative_dist > 0) {
      cm->ref_frame_side[ref_frame] = 1;
    } else if (order_hint == cur_order_hint) {
      cm->ref_frame_side[ref_frame] = -1;
    }
    cm->ref_frame_relative_dist[ref_frame] = abs(relative_dist);
  }
}

static INLINE void record_samples(const MB_MODE_INFO *mbmi,
#if CONFIG_COMPOUND_WARP_SAMPLES
                                  int ref,
#endif  // CONFIG_COMPOUND_WARP_SAMPLES
                                  int *pts, int *pts_inref, int row_offset,
                                  int sign_r, int col_offset, int sign_c) {
  int bw = block_size_wide[mbmi->sb_type[PLANE_TYPE_Y]];
  int bh = block_size_high[mbmi->sb_type[PLANE_TYPE_Y]];
  int x = col_offset * MI_SIZE + sign_c * AOMMAX(bw, MI_SIZE) / 2 - 1;
  int y = row_offset * MI_SIZE + sign_r * AOMMAX(bh, MI_SIZE) / 2 - 1;

  pts[0] = GET_MV_SUBPEL(x);
  pts[1] = GET_MV_SUBPEL(y);
#if !CONFIG_COMPOUND_WARP_SAMPLES
  const int ref = 0;
#endif  // CONFIG_COMPOUND_WARP_SAMPLES
  pts_inref[0] = GET_MV_SUBPEL(x) + mbmi->mv[ref].as_mv.col;
  pts_inref[1] = GET_MV_SUBPEL(y) + mbmi->mv[ref].as_mv.row;
}

// Select samples according to the motion vector difference.
uint8_t av1_selectSamples(MV *mv, int *pts, int *pts_inref, int len,
                          BLOCK_SIZE bsize) {
  const int bw = block_size_wide[bsize];
  const int bh = block_size_high[bsize];
  const int thresh = clamp(AOMMAX(bw, bh), 16, 112);
  int pts_mvd[SAMPLES_ARRAY_SIZE] = { 0 };
  int i, j, k, l = len;
  uint8_t ret = 0;
  assert(len <= LEAST_SQUARES_SAMPLES_MAX);

  // Obtain the motion vector difference.
  for (i = 0; i < len; ++i) {
    pts_mvd[i] = abs(pts_inref[2 * i] - pts[2 * i] - mv->col) +
                 abs(pts_inref[2 * i + 1] - pts[2 * i + 1] - mv->row);

    if (pts_mvd[i] > thresh)
      pts_mvd[i] = -1;
    else
      ret++;
  }

  // Keep at least 1 sample.
  if (!ret) return 1;

  i = 0;
  j = l - 1;
  for (k = 0; k < l - ret; k++) {
    while (pts_mvd[i] != -1) i++;
    while (pts_mvd[j] == -1) j--;
    assert(i != j);
    if (i > j) break;

    // Replace the discarded samples;
    pts_mvd[i] = pts_mvd[j];
    pts[2 * i] = pts[2 * j];
    pts[2 * i + 1] = pts[2 * j + 1];
    pts_inref[2 * i] = pts_inref[2 * j];
    pts_inref[2 * i + 1] = pts_inref[2 * j + 1];
    i++;
    j--;
  }

  return ret;
}

// Note: Samples returned are at 1/8-pel precision
// Sample are the neighbor block center point's coordinates relative to the
// left-top pixel of current block.
uint8_t av1_findSamples(const AV1_COMMON *cm, MACROBLOCKD *xd, int *pts,
                        int *pts_inref
#if CONFIG_COMPOUND_WARP_CAUSAL
                        ,
                        int ref_idx
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
) {
  const MB_MODE_INFO *const mbmi = xd->mi[0];
#if CONFIG_COMPOUND_WARP_CAUSAL
  const int ref_frame = mbmi->ref_frame[ref_idx];
#else
  const int ref_frame = mbmi->ref_frame[0];
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
  const int up_available = xd->up_available;
  const int left_available = xd->left_available;
  int i, mi_step;
  uint8_t np = 0;
  int do_top_left = 1;
  int do_top_right = 1;
  const int mi_stride = xd->mi_stride;
  const int mi_row = xd->mi_row;
  const int mi_col = xd->mi_col;

  // scan the nearest above rows
  if (up_available) {
    const int mi_row_offset = -1;
    const MB_MODE_INFO *above_mbmi = xd->mi[mi_row_offset * mi_stride];
    uint8_t above_block_width = mi_size_wide[above_mbmi->sb_type[PLANE_TYPE_Y]];

    if (xd->width <= above_block_width) {
      const int col_offset = -mi_col % above_block_width;

      if (col_offset < 0) do_top_left = 0;
      if (col_offset + above_block_width > xd->width) do_top_right = 0;

#if CONFIG_COMPOUND_WARP_SAMPLES
      for (int ref = 0; ref < 1 + has_second_ref(above_mbmi); ++ref) {
        if (above_mbmi->ref_frame[ref] == ref_frame) {
          record_samples(above_mbmi, ref, pts, pts_inref, 0, -1, col_offset, 1);
          pts += 2;
          pts_inref += 2;
          if (++np >= LEAST_SQUARES_SAMPLES_MAX) {
            return LEAST_SQUARES_SAMPLES_MAX;
          }
        }
      }
#else
      if (above_mbmi->ref_frame[0] == ref_frame &&
          above_mbmi->ref_frame[1] == NONE_FRAME) {
        record_samples(above_mbmi, pts, pts_inref, 0, -1, col_offset, 1);
        pts += 2;
        pts_inref += 2;
        if (++np >= LEAST_SQUARES_SAMPLES_MAX) return LEAST_SQUARES_SAMPLES_MAX;
      }
#endif        // CONFIG_COMPOUND_WARP_SAMPLES
    } else {  // xd->width > above_block_width
      for (i = 0; i < AOMMIN(xd->width, cm->mi_params.mi_cols - mi_col);
           i += mi_step) {
        above_mbmi = xd->mi[i + mi_row_offset * mi_stride];
        above_block_width = mi_size_wide[above_mbmi->sb_type[PLANE_TYPE_Y]];
        mi_step = AOMMIN(xd->width, above_block_width);
#if CONFIG_COMPOUND_WARP_SAMPLES
        for (int ref = 0; ref < 1 + has_second_ref(above_mbmi); ++ref) {
          if (above_mbmi->ref_frame[ref] == ref_frame) {
            record_samples(above_mbmi, ref, pts, pts_inref, 0, -1, i, 1);
            pts += 2;
            pts_inref += 2;
            if (++np >= LEAST_SQUARES_SAMPLES_MAX)
              return LEAST_SQUARES_SAMPLES_MAX;
          }
        }
#else
        if (above_mbmi->ref_frame[0] == ref_frame &&
            above_mbmi->ref_frame[1] == NONE_FRAME) {
          record_samples(above_mbmi, pts, pts_inref, 0, -1, i, 1);
          pts += 2;
          pts_inref += 2;
          if (++np >= LEAST_SQUARES_SAMPLES_MAX) {
            return LEAST_SQUARES_SAMPLES_MAX;
          }
        }
#endif  // CONFIG_COMPOUND_WARP_SAMPLES
      }
    }
  }
  assert(np <= LEAST_SQUARES_SAMPLES_MAX);

  // scan the nearest left columns
  if (left_available) {
    const int mi_col_offset = -1;
    const MB_MODE_INFO *left_mbmi = xd->mi[mi_col_offset];
    uint8_t left_block_height = mi_size_high[left_mbmi->sb_type[PLANE_TYPE_Y]];

    if (xd->height <= left_block_height) {
      const int row_offset = -mi_row % left_block_height;

      if (row_offset < 0) do_top_left = 0;

#if CONFIG_COMPOUND_WARP_SAMPLES
      for (int ref = 0; ref < 1 + has_second_ref(left_mbmi); ++ref) {
        if (left_mbmi->ref_frame[ref] == ref_frame) {
          record_samples(left_mbmi, ref, pts, pts_inref, row_offset, 1, 0, -1);
          pts += 2;
          pts_inref += 2;
          if (++np >= LEAST_SQUARES_SAMPLES_MAX) {
            return LEAST_SQUARES_SAMPLES_MAX;
          }
        }
      }
#else
      if (left_mbmi->ref_frame[0] == ref_frame &&
          left_mbmi->ref_frame[1] == NONE_FRAME) {
        record_samples(left_mbmi, pts, pts_inref, row_offset, 1, 0, -1);
        pts += 2;
        pts_inref += 2;
        if (++np >= LEAST_SQUARES_SAMPLES_MAX) return LEAST_SQUARES_SAMPLES_MAX;
      }
#endif        // CONFIG_COMPOUND_WARP_SAMPLES
    } else {  // xd->height > left_block_height
      for (i = 0; i < AOMMIN(xd->height, cm->mi_params.mi_rows - mi_row);
           i += mi_step) {
        left_mbmi = xd->mi[mi_col_offset + i * mi_stride];
        left_block_height = mi_size_high[left_mbmi->sb_type[PLANE_TYPE_Y]];
        mi_step = AOMMIN(xd->height, left_block_height);
#if CONFIG_COMPOUND_WARP_SAMPLES
        for (int ref = 0; ref < 1 + has_second_ref(left_mbmi); ++ref) {
          if (left_mbmi->ref_frame[ref] == ref_frame) {
            record_samples(left_mbmi, ref, pts, pts_inref, i, 1, 0, -1);
            pts += 2;
            pts_inref += 2;
            if (++np >= LEAST_SQUARES_SAMPLES_MAX) {
              return LEAST_SQUARES_SAMPLES_MAX;
            }
          }
        }
#else
        if (left_mbmi->ref_frame[0] == ref_frame &&
            left_mbmi->ref_frame[1] == NONE_FRAME) {
          record_samples(left_mbmi, pts, pts_inref, i, 1, 0, -1);
          pts += 2;
          pts_inref += 2;
          if (++np >= LEAST_SQUARES_SAMPLES_MAX) {
            return LEAST_SQUARES_SAMPLES_MAX;
          }
        }
#endif  // CONFIG_COMPOUND_WARP_SAMPLES
      }
    }
  }
  assert(np <= LEAST_SQUARES_SAMPLES_MAX);

  // Top-left block
  if (do_top_left && left_available && up_available) {
    const int mi_row_offset = -1;
    const int mi_col_offset = -1;
    const MB_MODE_INFO *top_left_mbmi =
        xd->mi[mi_col_offset + mi_row_offset * mi_stride];
#if CONFIG_COMPOUND_WARP_SAMPLES
    for (int ref = 0; ref < 1 + has_second_ref(top_left_mbmi); ++ref) {
      if (top_left_mbmi->ref_frame[ref] == ref_frame) {
        record_samples(top_left_mbmi, ref, pts, pts_inref, 0, -1, 0, -1);
        pts += 2;
        pts_inref += 2;
        if (++np >= LEAST_SQUARES_SAMPLES_MAX) return LEAST_SQUARES_SAMPLES_MAX;
      }
    }
#else
    if (top_left_mbmi->ref_frame[0] == ref_frame &&
        top_left_mbmi->ref_frame[1] == NONE_FRAME) {
      record_samples(top_left_mbmi, pts, pts_inref, 0, -1, 0, -1);
      pts += 2;
      pts_inref += 2;
      if (++np >= LEAST_SQUARES_SAMPLES_MAX) return LEAST_SQUARES_SAMPLES_MAX;
    }
#endif  // CONFIG_COMPOUND_WARP_SAMPLES
  }
  assert(np <= LEAST_SQUARES_SAMPLES_MAX);

  // Top-right block
#if CONFIG_EXT_RECUR_PARTITIONS
  if (do_top_right && has_top_right(cm, xd, mi_row, mi_col, xd->width)) {
#else
  if (do_top_right &&
      has_top_right(cm, xd, mi_row, mi_col, AOMMAX(xd->width, xd->height))) {
#endif
    const POSITION top_right_block_pos = { -1, xd->width };
    const TileInfo *const tile = &xd->tile;
    if (is_inside(tile, mi_col, mi_row, &top_right_block_pos)) {
      const int mi_row_offset = -1;
      const int mi_col_offset = xd->width;
      const MB_MODE_INFO *top_right_mbmi =
          xd->mi[mi_col_offset + mi_row_offset * mi_stride];
#if CONFIG_COMPOUND_WARP_SAMPLES
      for (int ref = 0; ref < 1 + has_second_ref(top_right_mbmi); ++ref) {
        if (top_right_mbmi->ref_frame[ref] == ref_frame) {
          record_samples(top_right_mbmi, ref, pts, pts_inref, 0, -1, xd->width,
                         1);
          pts += 2;
          pts_inref += 2;
          if (++np >= LEAST_SQUARES_SAMPLES_MAX) {
            return LEAST_SQUARES_SAMPLES_MAX;
          }
        }
      }
#else
      if (top_right_mbmi->ref_frame[0] == ref_frame &&
          top_right_mbmi->ref_frame[1] == NONE_FRAME) {
        record_samples(top_right_mbmi, pts, pts_inref, 0, -1, xd->width, 1);
        if (++np >= LEAST_SQUARES_SAMPLES_MAX) return LEAST_SQUARES_SAMPLES_MAX;
      }
#endif  // CONFIG_COMPOUND_WARP_SAMPLES
    }
  }
  assert(np <= LEAST_SQUARES_SAMPLES_MAX);

  return np;
}

void av1_setup_skip_mode_allowed(AV1_COMMON *cm) {
  const OrderHintInfo *const order_hint_info = &cm->seq_params.order_hint_info;
  SkipModeInfo *const skip_mode_info = &cm->current_frame.skip_mode_info;

  skip_mode_info->skip_mode_allowed = 0;
  skip_mode_info->ref_frame_idx_0 = INVALID_IDX;
  skip_mode_info->ref_frame_idx_1 = INVALID_IDX;

  if (!order_hint_info->enable_order_hint || frame_is_intra_only(cm)
#if !CONFIG_D072_SKIP_MODE_IMPROVE
      || cm->current_frame.reference_mode == SINGLE_REFERENCE
#endif  // !CONFIG_D072_SKIP_MODE_IMPROVE
  )
    return;

#if CONFIG_ALLOW_SAME_REF_COMPOUND
  skip_mode_info->skip_mode_allowed = 1;

  if (cm->ref_frames_info.num_total_refs > 1) {
    skip_mode_info->ref_frame_idx_1 = 1;
    skip_mode_info->ref_frame_idx_0 = 0;
#if CONFIG_D072_SKIP_MODE_IMPROVE
#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    const int cur_order_hint = cm->current_frame.display_order_hint;
#else
    const int cur_order_hint = cm->current_frame.order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    int ref_offset[2];
    get_skip_mode_ref_offsets(cm, ref_offset);
    const int cur_to_ref0 = abs(get_relative_dist(
        &cm->seq_params.order_hint_info, cur_order_hint, ref_offset[0]));
    const int cur_to_ref1 = abs(get_relative_dist(
        &cm->seq_params.order_hint_info, cur_order_hint, ref_offset[1]));

    if (abs(cur_to_ref0 - cur_to_ref1) > 1) {
      skip_mode_info->ref_frame_idx_0 = 0;
      skip_mode_info->ref_frame_idx_1 = 0;
    }
#endif  // CONFIG_D072_SKIP_MODE_IMPROVE
  } else {
    skip_mode_info->ref_frame_idx_1 = 0;
    skip_mode_info->ref_frame_idx_0 = 0;
  }
#else
#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  const int cur_order_hint = cm->current_frame.display_order_hint;
#else
  const int cur_order_hint = cm->current_frame.order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
  int ref_order_hints[2] = { -1, INT_MAX };
  int ref_idx[2] = { INVALID_IDX, INVALID_IDX };

  // Identify the top ranked forward and backward references.
  for (int i = 0; i < cm->ref_frames_info.num_total_refs; ++i) {
    const RefCntBuffer *const buf = get_ref_frame_buf(cm, i);
    if (buf == NULL) continue;

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    const int ref_order_hint = buf->display_order_hint;
#else
    const int ref_order_hint = buf->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
    if (get_relative_dist(order_hint_info, ref_order_hint, cur_order_hint) <
        0) {
      // Forward reference
      if (ref_order_hints[0] == -1 ||
          get_relative_dist(order_hint_info, ref_order_hint,
                            ref_order_hints[0]) > 0) {
        ref_order_hints[0] = ref_order_hint;
        ref_idx[0] = i;
      }
    } else if (get_relative_dist(order_hint_info, ref_order_hint,
                                 cur_order_hint) > 0) {
      // Backward reference
      if (ref_order_hints[1] == INT_MAX ||
          get_relative_dist(order_hint_info, ref_order_hint,
                            ref_order_hints[1]) < 0) {
        ref_order_hints[1] = ref_order_hint;
        ref_idx[1] = i;
      }
    }
  }

  if (ref_idx[0] != INVALID_IDX && ref_idx[1] != INVALID_IDX) {
    // == Bi-directional prediction ==
    skip_mode_info->skip_mode_allowed = 1;
#if CONFIG_SKIP_MODE_ENHANCEMENT
    skip_mode_info->ref_frame_idx_0 = ref_idx[0];
    skip_mode_info->ref_frame_idx_1 = ref_idx[1];
#else
    skip_mode_info->ref_frame_idx_0 = AOMMIN(ref_idx[0], ref_idx[1]);
    skip_mode_info->ref_frame_idx_1 = AOMMAX(ref_idx[0], ref_idx[1]);
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
  } else if (ref_idx[0] != INVALID_IDX && ref_idx[1] == INVALID_IDX) {
    // == Forward prediction only ==
    // Identify the second nearest forward reference.
    ref_order_hints[1] = -1;
    for (int i = 0; i < INTER_REFS_PER_FRAME; ++i) {
      const RefCntBuffer *const buf = get_ref_frame_buf(cm, i);
      if (buf == NULL) continue;

#if CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
      const int ref_order_hint = buf->display_order_hint;
#else
      const int ref_order_hint = buf->order_hint;
#endif  // CONFIG_EXPLICIT_TEMPORAL_DIST_CALC
      if ((ref_order_hints[0] != -1 &&
           get_relative_dist(order_hint_info, ref_order_hint,
                             ref_order_hints[0]) < 0) &&
          (ref_order_hints[1] == -1 ||
           get_relative_dist(order_hint_info, ref_order_hint,
                             ref_order_hints[1]) > 0)) {
        // Second closest forward reference
        ref_order_hints[1] = ref_order_hint;
        ref_idx[1] = i;
      }
    }
    if (ref_order_hints[1] != -1) {
      skip_mode_info->skip_mode_allowed = 1;
#if CONFIG_SKIP_MODE_ENHANCEMENT
      skip_mode_info->ref_frame_idx_0 = ref_idx[0];
      skip_mode_info->ref_frame_idx_1 = ref_idx[1];
#else
      skip_mode_info->ref_frame_idx_0 = AOMMIN(ref_idx[0], ref_idx[1]);
      skip_mode_info->ref_frame_idx_1 = AOMMAX(ref_idx[0], ref_idx[1]);
#endif  // CONFIG_SKIP_MODE_ENHANCEMENT
    }
  }
#endif  // CONFIG_ALLOW_SAME_REF_COMPOUND
}

#if CONFIG_BANK_IMPROVE
#define SB_TO_RMB_UNITS_LOG2 3
#define SB_TO_RMB_UNITS (1 << SB_TO_RMB_UNITS_LOG2)
#define BANK_1ST_UNIT_UPDATE_COUNT 4
#define BANK_UNIT_MAX_ALLOWED_LEFTOVER_UPDATES 16

// Divide SB into 64 units, and each unit has one hit. Thus, in worst case,
// one SB has 64 hits as defined in MAX_RMB_SB_HITS.
// If SB size is 128x128, then each unit size is (128/8)x(128/8)=16x16
// If SB size is 256x256, then each unit size is (256/8)x(256/8)=32x32

void decide_rmb_unit_update_count(const AV1_COMMON *const cm,
                                  MACROBLOCKD *const xd,
                                  const MB_MODE_INFO *const mbmi) {
  if (!cm->seq_params.enable_refmvbank) return;
  if (xd->tree_type == CHROMA_PART) return;
  const int mi_sb_size = cm->mib_size;
  const int mi_sb_size_log2 = cm->mib_size_log2;
  const int mi_row_in_sb = xd->mi_row % mi_sb_size;
  const int mi_col_in_sb = xd->mi_col % mi_sb_size;
  const int rmb_unit_mi_size_log2 = mi_sb_size_log2 - SB_TO_RMB_UNITS_LOG2;
  const int rmb_unit_mi_size = (1 << rmb_unit_mi_size_log2);

  BLOCK_SIZE bsize = mbmi->sb_type[PLANE_TYPE_Y];
  const int mi_bw = mi_size_wide[bsize];
  const int mi_bh = mi_size_high[bsize];
  const int rmb_units_count = AOMMAX(mi_bw >> rmb_unit_mi_size_log2, 1) *
                              AOMMAX(mi_bh >> rmb_unit_mi_size_log2, 1);
  if (mi_row_in_sb == 0 && mi_col_in_sb == 0) {
    xd->ref_mv_bank.remain_hits =
        AOMMAX(rmb_units_count, BANK_1ST_UNIT_UPDATE_COUNT);
    xd->ref_mv_bank.rmb_unit_hits = 0;
  } else if (((mi_row_in_sb % rmb_unit_mi_size) == 0) &&
             ((mi_col_in_sb % rmb_unit_mi_size) == 0)) {
    xd->ref_mv_bank.remain_hits += rmb_units_count;
    xd->ref_mv_bank.rmb_unit_hits = 0;
  }
}
#endif  // CONFIG_BANK_IMPROVE

static INLINE void update_ref_mv_bank(
#if CONFIG_BANK_IMPROVE
    const AV1_COMMON *const cm, MACROBLOCKD *const xd, int from_within_sb,
#endif  // CONFIG_BANK_IMPROVE
    const MB_MODE_INFO *const mbmi, REF_MV_BANK *ref_mv_bank) {
#if CONFIG_BANK_IMPROVE
  if (from_within_sb) {
    decide_rmb_unit_update_count(cm, xd, mbmi);

    if (ref_mv_bank->remain_hits == 0 ||
        ref_mv_bank->rmb_unit_hits >= BANK_UNIT_MAX_ALLOWED_LEFTOVER_UPDATES ||
        ref_mv_bank->rmb_sb_hits >= MAX_RMB_SB_HITS) {
      return;
    }

    ref_mv_bank->remain_hits--;
    ref_mv_bank->rmb_unit_hits++;
  } else {
    // If max hits have been reached return.
    if (ref_mv_bank->rmb_sb_hits >= MAX_RMB_SB_HITS) return;
  }
#else
  // If max hits have been reached return.
  if (ref_mv_bank->rmb_sb_hits >= MAX_RMB_SB_HITS) return;
#endif  // CONFIG_BANK_IMPROVE
  // else increment count and proceed with updating.
  ++ref_mv_bank->rmb_sb_hits;

  const MV_REFERENCE_FRAME ref_frame = av1_ref_frame_type(mbmi->ref_frame);
  CANDIDATE_MV *queue = ref_mv_bank->rmb_buffer[ref_frame];
  const int is_comp = has_second_ref(mbmi);
  const int start_idx = ref_mv_bank->rmb_start_idx[ref_frame];
  const int count = ref_mv_bank->rmb_count[ref_frame];
  int found = -1;

  // Check if current MV is already existing in the buffer.
  for (int i = 0; i < count; ++i) {
    const int idx = (start_idx + i) % REF_MV_BANK_SIZE;
    if (mbmi->mv[0].as_int == queue[idx].this_mv.as_int &&
        (!is_comp || mbmi->mv[1].as_int == queue[idx].comp_mv.as_int)) {
      found = i;
      break;
    }
  }

  // If current MV is found in the buffer, move it to the end of the buffer.
  if (found >= 0) {
    const int idx = (start_idx + found) % REF_MV_BANK_SIZE;
    const CANDIDATE_MV cand = queue[idx];
    for (int i = found; i < count - 1; ++i) {
      const int idx0 = (start_idx + i) % REF_MV_BANK_SIZE;
      const int idx1 = (start_idx + i + 1) % REF_MV_BANK_SIZE;
      queue[idx0] = queue[idx1];
    }
    const int tail = (start_idx + count - 1) % REF_MV_BANK_SIZE;
    queue[tail] = cand;
    return;
  }

  // If current MV is not found in the buffer, append it to the end of the
  // buffer, and update the count and start_idx accordingly.
  const int idx = (start_idx + count) % REF_MV_BANK_SIZE;
  queue[idx].this_mv = mbmi->mv[0];
  if (is_comp) queue[idx].comp_mv = mbmi->mv[1];
  queue[idx].cwp_idx = mbmi->cwp_idx;
  if (count < REF_MV_BANK_SIZE) {
    ++ref_mv_bank->rmb_count[ref_frame];
  } else {
    ++ref_mv_bank->rmb_start_idx[ref_frame];
  }
}

void av1_update_ref_mv_bank(const AV1_COMMON *const cm, MACROBLOCKD *const xd,
#if CONFIG_BANK_IMPROVE
                            int from_within_sb,
#endif  // CONFIG_BANK_IMPROVE
                            const MB_MODE_INFO *const mbmi) {
  update_ref_mv_bank(
#if CONFIG_BANK_IMPROVE
      cm, xd, from_within_sb,
#endif  // CONFIG_BANK_IMPROVE
      mbmi, &xd->ref_mv_bank);
  (void)cm;
}

#if CONFIG_C071_SUBBLK_WARPMV
void assign_warpmv(const AV1_COMMON *cm, SUBMB_INFO **submi, BLOCK_SIZE bsize,
                   WarpedMotionParams *wm_params, int mi_row, int mi_col
#if CONFIG_COMPOUND_WARP_CAUSAL
                   ,
                   int ref
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
) {
  assert(wm_params->invalid == 0);
  const int bw = mi_size_wide[bsize];
  const int bh = mi_size_high[bsize];
  const int p_x_mis = AOMMIN(bw, cm->mi_params.mi_cols - mi_col) * MI_SIZE;
  const int p_y_mis = AOMMIN(bh, cm->mi_params.mi_rows - mi_row) * MI_SIZE;
  const int p_row = mi_row * MI_SIZE;
  const int p_col = mi_col * MI_SIZE;
  const int mi_stride = cm->mi_params.mi_stride;
  for (int i = p_row; i < p_row + p_y_mis; i += 8) {
    for (int j = p_col; j < p_col + p_x_mis; j += 8) {
      const int32_t src_x = j + 4;
      const int32_t src_y = i + 4;
      const int32_t dst_x = wm_params->wmmat[2] * src_x +
                            wm_params->wmmat[3] * src_y + wm_params->wmmat[0];
      const int32_t dst_y = wm_params->wmmat[4] * src_x +
                            wm_params->wmmat[5] * src_y + wm_params->wmmat[1];
      int32_t submv_x_hp = dst_x - (src_x << WARPEDMODEL_PREC_BITS);
      int32_t submv_y_hp = dst_y - (src_y << WARPEDMODEL_PREC_BITS);
      int mi_y = (i - p_row) / MI_SIZE;
      int mi_x = (j - p_col) / MI_SIZE;
      const int mv_row =
          ROUND_POWER_OF_TWO_SIGNED(submv_y_hp, WARPEDMODEL_PREC_BITS - 3);
      const int mv_col =
          ROUND_POWER_OF_TWO_SIGNED(submv_x_hp, WARPEDMODEL_PREC_BITS - 3);
#if CONFIG_COMPOUND_WARP_CAUSAL
      submi[mi_y * mi_stride + mi_x]->mv[ref].as_mv.row =
#else
      submi[mi_y * mi_stride + mi_x]->mv[0].as_mv.row =
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
          clamp(mv_row, MV_LOW + 1, MV_UPP - 1);
#if CONFIG_COMPOUND_WARP_CAUSAL
      submi[mi_y * mi_stride + mi_x]->mv[ref].as_mv.col =
#else
      submi[mi_y * mi_stride + mi_x]->mv[0].as_mv.col =
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
          clamp(mv_col, MV_LOW + 1, MV_UPP - 1);

      span_submv(cm, (submi + mi_y * mi_stride + mi_x), mi_row + mi_y,
                 mi_col + mi_x, BLOCK_8X8
#if CONFIG_COMPOUND_WARP_CAUSAL
                 ,
                 ref
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
      );
    }
  }
}

void span_submv(const AV1_COMMON *cm, SUBMB_INFO **submi, int mi_row,
                int mi_col, BLOCK_SIZE bsize
#if CONFIG_COMPOUND_WARP_CAUSAL
                ,
                int ref
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
) {
  const int bw = mi_size_wide[bsize];
  const int bh = mi_size_high[bsize];
  const int x_inside_boundary = AOMMIN(bw, cm->mi_params.mi_cols - mi_col);
  const int y_inside_boundary = AOMMIN(bh, cm->mi_params.mi_rows - mi_row);
  const int stride = cm->mi_params.mi_stride;
  for (int y = 0; y < y_inside_boundary; y++) {
    for (int x = 0; x < x_inside_boundary; x++) {
      if (x == 0 && y == 0) continue;
#if CONFIG_COMPOUND_WARP_CAUSAL
      submi[y * stride + x]->mv[ref] = submi[0]->mv[ref];
#else
      submi[y * stride + x]->mv[0] = submi[0]->mv[0];
      submi[y * stride + x]->mv[1] = submi[0]->mv[1];
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
    }
  }
}
#endif  // CONFIG_C071_SUBBLK_WARPMV

#if CONFIG_EXTENDED_WARP_PREDICTION

#define MAX_WARP_SB_HITS 64
// Update the warp parameter bank
//  If the warp parameters are already exist in the bank, then bank is
//  rearranged If the warp parameters are not in the bank, insert it to the
//  bank.
static INLINE void update_warp_param_bank(const MB_MODE_INFO *const mbmi,
                                          WARP_PARAM_BANK *warp_param_bank) {
#if CONFIG_COMPOUND_WARP_CAUSAL
  for (int ref_idx = 0; ref_idx < 1 + is_inter_compound_mode(mbmi->mode);
       ref_idx++) {
    if (!mbmi->wm_params[ref_idx].invalid) {
      const MV_REFERENCE_FRAME ref_frame = mbmi->ref_frame[ref_idx];
#else
  const MV_REFERENCE_FRAME ref_frame = av1_ref_frame_type(mbmi->ref_frame);
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
      WarpedMotionParams *queue = warp_param_bank->wpb_buffer[ref_frame];
      const int start_idx = warp_param_bank->wpb_start_idx[ref_frame];
      const int count = warp_param_bank->wpb_count[ref_frame];
      int found = -1;

      // If max hits have been reached return.
      if (warp_param_bank->wpb_sb_hits >= MAX_WARP_SB_HITS) return;
      // else increment count and proceed with updating.
      ++warp_param_bank->wpb_sb_hits;

      // Check if current warp parameters is already existing in the buffer.
      for (int i = 0; i < count; ++i) {
        const int idx = (start_idx + i) % WARP_PARAM_BANK_SIZE;
#if CONFIG_COMPOUND_WARP_CAUSAL
        int same_param =
            (mbmi->wm_params[ref_idx].wmmat[2] == queue[idx].wmmat[2]);
        same_param &=
            (mbmi->wm_params[ref_idx].wmmat[3] == queue[idx].wmmat[3]);

        same_param &=
            (mbmi->wm_params[ref_idx].wmmat[4] == queue[idx].wmmat[4]);
        same_param &=
            (mbmi->wm_params[ref_idx].wmmat[5] == queue[idx].wmmat[5]);

        same_param &= (mbmi->wm_params[ref_idx].wmtype == queue[idx].wmtype);
#else
    int same_param = (mbmi->wm_params[0].wmmat[2] == queue[idx].wmmat[2]);
    same_param &= (mbmi->wm_params[0].wmmat[3] == queue[idx].wmmat[3]);

    same_param &= (mbmi->wm_params[0].wmmat[4] == queue[idx].wmmat[4]);
    same_param &= (mbmi->wm_params[0].wmmat[5] == queue[idx].wmmat[5]);

    same_param &= (mbmi->wm_params[0].wmtype == queue[idx].wmtype);
#endif  // CONFIG_COMPOUND_WARP_CAUSAL

        if (same_param) {
          found = i;
          break;
        }
      }

      // If current warp parameters is found in the buffer, move it to the end
      // of the buffer.
      if (found >= 0) {
        const int idx = (start_idx + found) % WARP_PARAM_BANK_SIZE;
        const WarpedMotionParams cand = queue[idx];
        for (int i = found; i < count - 1; ++i) {
          const int idx0 = (start_idx + i) % WARP_PARAM_BANK_SIZE;
          const int idx1 = (start_idx + i + 1) % WARP_PARAM_BANK_SIZE;
          queue[idx0] = queue[idx1];
        }
        const int tail = (start_idx + count - 1) % WARP_PARAM_BANK_SIZE;
        queue[tail] = cand;
        return;
      }

      // If current warp parameter is not found in the buffer, append it to the
      // end of the buffer, and update the count and start_idx accordingly.
      const int idx = (start_idx + count) % WARP_PARAM_BANK_SIZE;
#if CONFIG_COMPOUND_WARP_CAUSAL
      queue[idx].wmtype = mbmi->wm_params[ref_idx].wmtype;
      queue[idx].wmmat[0] = mbmi->wm_params[ref_idx].wmmat[0];
      queue[idx].wmmat[1] = mbmi->wm_params[ref_idx].wmmat[1];
      queue[idx].wmmat[2] = mbmi->wm_params[ref_idx].wmmat[2];
      queue[idx].wmmat[3] = mbmi->wm_params[ref_idx].wmmat[3];
      queue[idx].wmmat[4] = mbmi->wm_params[ref_idx].wmmat[4];
      queue[idx].wmmat[5] = mbmi->wm_params[ref_idx].wmmat[5];
#else
  queue[idx].wmtype = mbmi->wm_params[0].wmtype;
  queue[idx].wmmat[0] = mbmi->wm_params[0].wmmat[0];
  queue[idx].wmmat[1] = mbmi->wm_params[0].wmmat[1];
  queue[idx].wmmat[2] = mbmi->wm_params[0].wmmat[2];
  queue[idx].wmmat[3] = mbmi->wm_params[0].wmmat[3];
  queue[idx].wmmat[4] = mbmi->wm_params[0].wmmat[4];
  queue[idx].wmmat[5] = mbmi->wm_params[0].wmmat[5];
#endif  // CONFIG_COMPOUND_WARP_CAUSAL

      if (count < WARP_PARAM_BANK_SIZE) {
        ++warp_param_bank->wpb_count[ref_frame];
      } else {
        ++warp_param_bank->wpb_start_idx[ref_frame];
      }
#if CONFIG_COMPOUND_WARP_CAUSAL
    }
  }
#endif  // CONFIG_COMPOUND_WARP_CAUSAL
}
void av1_update_warp_param_bank(const AV1_COMMON *const cm,
                                MACROBLOCKD *const xd,
                                const MB_MODE_INFO *const mbmi) {
  (void)cm;
  if (is_warp_mode(mbmi->motion_mode)) {
    update_warp_param_bank(mbmi, &xd->warp_param_bank);
  }
}

// The wrl_list is the warp reference list which is already generated in the
// av1_find_mv_refs

// If the mode is not equal to the GLOBALMV mode, wrl_list is copied to the
// warp_param_stack
void av1_find_warp_delta_base_candidates(
    const MACROBLOCKD *xd, const MB_MODE_INFO *mbmi,
    WARP_CANDIDATE warp_param_stack[MAX_WARP_REF_CANDIDATES],
    WARP_CANDIDATE wrl_list[MAX_WARP_REF_CANDIDATES], uint8_t num_wrl_cand,
    uint8_t *p_valid_num_candidates) {
  // Global MV mode insert the global motion
  if (mbmi->mode == GLOBALMV) {
    warp_param_stack[0].wm_params = xd->global_motion[mbmi->ref_frame[0]];
    warp_param_stack[0].proj_type = PROJ_GLOBAL_MOTION;
    if (p_valid_num_candidates) {
      *p_valid_num_candidates = 1;
    }
    return;
  }

  memcpy(&warp_param_stack[0], &wrl_list[0],
         num_wrl_cand * sizeof(wrl_list[0]));
  if (p_valid_num_candidates) {
    // for NEARMV mode, the maximum number of candidates is 1
    *p_valid_num_candidates = (mbmi->mode == NEARMV || mbmi->mode == AMVDNEWMV

                               )
                                  ? 1
                                  : num_wrl_cand;
  }
}

// check if the the derive MV is inside of frame boundary
// return false if the MV is outside of the frame boundary
bool is_warp_candidate_inside_of_frame(const AV1_COMMON *cm,
                                       const MACROBLOCKD *xd, int_mv cand_mv) {
  // Check if the MV candidate is pointing to ref block inside frame boundary.

  const int block_width = xd->width * MI_SIZE;
  const int block_height = xd->height * MI_SIZE;
  int frame_width = cm->width;
  int frame_height = cm->height;

  const int mv_row = (cand_mv.as_mv.row) / 8;
  const int mv_col = (cand_mv.as_mv.col) / 8;
  const int ref_x = xd->mi_col * MI_SIZE + mv_col;
  const int ref_y = xd->mi_row * MI_SIZE + mv_row;
  if (ref_x <= -block_width || ref_y <= -block_height || ref_x >= frame_width ||
      ref_y >= frame_height) {
    return false;
  }
  return true;
}

int allow_extend_nb(const AV1_COMMON *cm, const MACROBLOCKD *xd,
                    const MB_MODE_INFO *mbmi, int *p_num_of_warp_neighbors) {
  const TileInfo *const tile = &xd->tile;

#if CONFIG_EXT_RECUR_PARTITIONS
  const int has_tr = has_top_right(cm, xd, xd->mi_row, xd->mi_col, xd->width);
  const int has_bl =
      has_bottom_left(cm, xd, xd->mi_row, xd->mi_col, xd->height);
#else
  const int bs = AOMMAX(xd->width, xd->height);
  const int has_tr = has_top_right(cm, xd, xd->mi_row, xd->mi_col, bs);
  const int has_bl = has_bottom_left(cm, xd, xd->mi_row, xd->mi_col, bs);
#endif  // CONFIG_EXT_RECUR_PARTITIONS
  POSITION mi_pos;

  int allow_new_ext = 0;
  int allow_near_ext = 0;

  // counter to count number of warp neighbors
  int num_of_warp_neighbors = 0;

  // left
  mi_pos.row = xd->height - 1;
  mi_pos.col = -1;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && xd->left_available) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    if (is_inter_ref_frame(neighbor_mi->ref_frame[0]) &&
        neighbor_mi->ref_frame[0] == mbmi->ref_frame[0]) {
      allow_new_ext |= 1;
      allow_near_ext |= is_warp_mode(neighbor_mi->motion_mode);
      if (p_num_of_warp_neighbors && is_warp_mode(neighbor_mi->motion_mode))
        num_of_warp_neighbors++;
    }
  }

  // up
  mi_pos.row = -1;
  mi_pos.col = xd->width - 1;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && xd->up_available) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    if (is_inter_ref_frame(neighbor_mi->ref_frame[0]) &&
        neighbor_mi->ref_frame[0] == mbmi->ref_frame[0]) {
      allow_new_ext |= 1;
      allow_near_ext |= is_warp_mode(neighbor_mi->motion_mode);
      if (p_num_of_warp_neighbors && is_warp_mode(neighbor_mi->motion_mode))
        num_of_warp_neighbors++;
    }
  }

  // left
  mi_pos.row = 0;
  mi_pos.col = -1;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && xd->left_available) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    if (is_inter_ref_frame(neighbor_mi->ref_frame[0]) &&
        neighbor_mi->ref_frame[0] == mbmi->ref_frame[0]) {
      allow_new_ext |= 1;
      allow_near_ext |= is_warp_mode(neighbor_mi->motion_mode);
      if (p_num_of_warp_neighbors && is_warp_mode(neighbor_mi->motion_mode))
        num_of_warp_neighbors++;
    }
  }

  // up
  mi_pos.row = -1;
  mi_pos.col = 0;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && xd->up_available) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    if (is_inter_ref_frame(neighbor_mi->ref_frame[0]) &&
        neighbor_mi->ref_frame[0] == mbmi->ref_frame[0]) {
      allow_new_ext |= 1;
      allow_near_ext |= is_warp_mode(neighbor_mi->motion_mode);
      if (p_num_of_warp_neighbors && is_warp_mode(neighbor_mi->motion_mode))
        num_of_warp_neighbors++;
    }
  }

  mi_pos.row = xd->height;
  mi_pos.col = -1;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && has_bl) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    if (is_inter_ref_frame(neighbor_mi->ref_frame[0]) &&
        neighbor_mi->ref_frame[0] == mbmi->ref_frame[0]) {
      allow_new_ext |= 1;
      allow_near_ext |= is_warp_mode(neighbor_mi->motion_mode);
      if (p_num_of_warp_neighbors && is_warp_mode(neighbor_mi->motion_mode))
        num_of_warp_neighbors++;
    }
  }

  mi_pos.row = -1;
  mi_pos.col = xd->width;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && has_tr) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    if (is_inter_ref_frame(neighbor_mi->ref_frame[0]) &&
        neighbor_mi->ref_frame[0] == mbmi->ref_frame[0]) {
      allow_new_ext |= 1;
      allow_near_ext |= is_warp_mode(neighbor_mi->motion_mode);
      if (p_num_of_warp_neighbors && is_warp_mode(neighbor_mi->motion_mode))
        num_of_warp_neighbors++;
    }
  }

  mi_pos.row = -1;
  mi_pos.col = -1;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && xd->up_available &&
      xd->left_available) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    if (is_inter_ref_frame(neighbor_mi->ref_frame[0]) &&
        neighbor_mi->ref_frame[0] == mbmi->ref_frame[0]) {
      allow_new_ext |= 1;
      allow_near_ext |= is_warp_mode(neighbor_mi->motion_mode);
      if (p_num_of_warp_neighbors && is_warp_mode(neighbor_mi->motion_mode))
        num_of_warp_neighbors++;
    }
  }

  mi_pos.row = (xd->height >> 1);
  mi_pos.col = -1;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && xd->left_available) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    if (is_inter_ref_frame(neighbor_mi->ref_frame[0]) &&
        neighbor_mi->ref_frame[0] == mbmi->ref_frame[0]) {
      allow_new_ext |= 1;
      allow_near_ext |= is_warp_mode(neighbor_mi->motion_mode);
      if (p_num_of_warp_neighbors && is_warp_mode(neighbor_mi->motion_mode))
        num_of_warp_neighbors++;
    }
  }

  mi_pos.row = -1;
  mi_pos.col = (xd->width >> 1);
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && xd->up_available) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    if (is_inter_ref_frame(neighbor_mi->ref_frame[0]) &&
        neighbor_mi->ref_frame[0] == mbmi->ref_frame[0]) {
      allow_new_ext |= 1;
      allow_near_ext |= is_warp_mode(neighbor_mi->motion_mode);
      if (p_num_of_warp_neighbors && is_warp_mode(neighbor_mi->motion_mode))
        num_of_warp_neighbors++;
    }
  }

  if (p_num_of_warp_neighbors) {
    *p_num_of_warp_neighbors = num_of_warp_neighbors;
    return num_of_warp_neighbors;
  }

  if (mbmi->mode == NEWMV) {
    return allow_new_ext;
  } else if (mbmi->mode == NEARMV) {
    return allow_near_ext;
  } else {
    return 0;
  }
}

static AOM_INLINE POSITION get_pos_from_pos_idx(const MACROBLOCKD *xd,
                                                int pos_idx) {
  POSITION ret_pos = { 0, 0 };
  if (pos_idx == 5) {
    ret_pos.row = xd->height;
    ret_pos.col = -1;
  } else if (pos_idx == 1) {
    ret_pos.row = (xd->height - 1);
    ret_pos.col = -1;
  } else if (pos_idx == 8) {
    ret_pos.row = (xd->height >> 1);
    ret_pos.col = -1;
  } else if (pos_idx == 3) {
    ret_pos.row = 0;
    ret_pos.col = -1;
  } else if (pos_idx == 7) {
    ret_pos.row = -1;
    ret_pos.col = -1;
  } else if (pos_idx == 4) {
    ret_pos.row = -1;
    ret_pos.col = 0;
  } else if (pos_idx == 9) {
    ret_pos.row = -1;
    ret_pos.col = (xd->width >> 1);
  } else if (pos_idx == 2) {
    ret_pos.row = -1;
    ret_pos.col = (xd->width - 1);
  } else if (pos_idx == 6) {
    ret_pos.row = -1;
    ret_pos.col = xd->width;
  } else {
    assert(0);
  }
  return ret_pos;
}

static AOM_INLINE int get_cand_from_pos_idx(const AV1_COMMON *cm,
                                            const MACROBLOCKD *xd,
                                            int pos_idx) {
#if CONFIG_EXT_RECUR_PARTITIONS
  const int has_tr = has_top_right(cm, xd, xd->mi_row, xd->mi_col, xd->width);
  const int has_bl =
      has_bottom_left(cm, xd, xd->mi_row, xd->mi_col, xd->height);
#else
  const int bs = AOMMAX(xd->width, xd->height);
  const int has_tr = has_top_right(cm, xd, xd->mi_row, xd->mi_col, bs);
  const int has_bl = has_bottom_left(cm, xd, xd->mi_row, xd->mi_col, bs);
#endif  // CONFIG_EXT_RECUR_PARTITIONS

  int ret_cand = 0;

  if (pos_idx == 1 || pos_idx == 3 || pos_idx == 8) {
    ret_cand = xd->left_available;
  } else if (pos_idx == 2 || pos_idx == 4 || pos_idx == 9) {
    ret_cand = xd->up_available;
  } else if (pos_idx == 5) {
    ret_cand = has_bl;
  } else if (pos_idx == 6) {
    ret_cand = has_tr;
  } else if (pos_idx == 7) {
    ret_cand = (xd->up_available && xd->left_available);
  } else {
    assert(0);
  }
  return ret_cand;
}

static AOM_INLINE int check_pos_and_get_base_pos(const AV1_COMMON *cm,
                                                 const MACROBLOCKD *xd,
                                                 const MB_MODE_INFO *mbmi,
                                                 POSITION *base_pos,
                                                 int pos_idx) {
  const TileInfo *const tile = &xd->tile;
  POSITION mi_pos = get_pos_from_pos_idx(xd, pos_idx);
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) &&
      get_cand_from_pos_idx(cm, xd, pos_idx)) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    if ((is_inter_ref_frame(neighbor_mi->ref_frame[0]) &&
         neighbor_mi->ref_frame[0] == mbmi->ref_frame[0]) ||
        (is_inter_ref_frame(neighbor_mi->ref_frame[1]) &&
         neighbor_mi->ref_frame[1] == mbmi->ref_frame[0])) {
      if ((is_warp_mode(neighbor_mi->motion_mode) && mbmi->mode == NEARMV) ||
          mbmi->mode == NEWMV) {
        base_pos->row = mi_pos.row;
        base_pos->col = mi_pos.col;
        return 1;
      }
    }
  }
  return 0;
}

int get_extend_base_pos(const AV1_COMMON *cm, const MACROBLOCKD *xd,
                        const MB_MODE_INFO *mbmi, int mvp_row_offset,
                        int mvp_col_offset, POSITION *base_pos) {
  if (mvp_col_offset == -1 || mvp_row_offset == -1) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mvp_row_offset * xd->mi_stride + mvp_col_offset];
    if ((is_warp_mode(neighbor_mi->motion_mode) && mbmi->mode == NEARMV) ||
        mbmi->mode == NEWMV) {
      base_pos->row = mvp_row_offset;
      base_pos->col = mvp_col_offset;
      return 1;
    }
  }

  for (int pos_idx = 1; pos_idx <= 8; pos_idx++) {
    if (check_pos_and_get_base_pos(cm, xd, mbmi, base_pos, pos_idx)) return 1;
  }
  return 0;
}

int16_t inter_warpmv_mode_ctx(const AV1_COMMON *cm, const MACROBLOCKD *xd,
                              const MB_MODE_INFO *mbmi) {
  int num_of_warp_neighbors = 0;
  int ctx = allow_extend_nb(cm, xd, mbmi, &num_of_warp_neighbors);
  assert(num_of_warp_neighbors == ctx);
  assert(ctx < WARPMV_MODE_CONTEXT);
  return ctx;
}

// return 1 if valid point is found
// return 0 if the point is not valid
static int fill_warp_corner_projected_point(const MB_MODE_INFO *neighbor_mi,
                                            MV_REFERENCE_FRAME this_ref,
                                            const int pos_col,
                                            const int pos_row, int *pts,
                                            int *mvs, int *n_points) {
  // return if the source point is invalid
  if (pos_col < 0 || pos_row < 0) return 0;

  if (!is_inter_ref_frame(neighbor_mi->ref_frame[0])) return 0;
  if (neighbor_mi->ref_frame[0] != this_ref) return 0;
  int mv_row;
  int mv_col;
  if (is_warp_mode(neighbor_mi->motion_mode)) {
    int_mv warp_mv =
        get_warp_motion_vector_xy_pos(&neighbor_mi->wm_params[0], pos_col,
                                      pos_row, MV_PRECISION_ONE_EIGHTH_PEL);
    mv_row = warp_mv.as_mv.row;
    mv_col = warp_mv.as_mv.col;
  } else {
    mv_row = neighbor_mi->mv[0].as_mv.row;
    mv_col = neighbor_mi->mv[0].as_mv.col;
  }
  pts[2 * (*n_points)] = pos_col;
  pts[2 * (*n_points) + 1] = pos_row;
  mvs[2 * (*n_points)] = mv_col;
  mvs[2 * (*n_points) + 1] = mv_row;
  ++(*n_points);
  return 1;
}

// Check all 3 neighbors to generate projected points
int generate_points_from_corners(const MACROBLOCKD *xd, int *pts, int *mvs,
                                 int *np, MV_REFERENCE_FRAME ref_frame) {
  const TileInfo *const tile = &xd->tile;
  POSITION mi_pos;
  int valid_points = 0;
  MV_REFERENCE_FRAME rf[2];
  av1_set_ref_frame(rf, ref_frame);
  MV_REFERENCE_FRAME this_ref = rf[0];
  const int bw = xd->width * MI_SIZE;
  const int bh = xd->height * MI_SIZE;

  // top-left
  mi_pos.row = -1;
  mi_pos.col = -1;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && xd->up_available &&
      xd->left_available) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    int pos_row = xd->mi_row * MI_SIZE;
    int pos_col = xd->mi_col * MI_SIZE;
    int valid = fill_warp_corner_projected_point(neighbor_mi, this_ref, pos_col,
                                                 pos_row, pts, mvs, np);
    if (valid) {
      valid_points++;
    }
  }

  // top-right
  mi_pos.row = -1;
  mi_pos.col = xd->width - 1;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && xd->up_available) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    int pos_row = xd->mi_row * MI_SIZE;
    int pos_col = xd->mi_col * MI_SIZE + bw;
    int valid = fill_warp_corner_projected_point(neighbor_mi, this_ref, pos_col,
                                                 pos_row, pts, mvs, np);
    if (valid) {
      valid_points++;
    }
  }

  // bottom-left
  mi_pos.row = xd->height - 1;
  mi_pos.col = -1;
  if (is_inside(tile, xd->mi_col, xd->mi_row, &mi_pos) && xd->left_available) {
    const MB_MODE_INFO *neighbor_mi =
        xd->mi[mi_pos.row * xd->mi_stride + mi_pos.col];
    int pos_row = xd->mi_row * MI_SIZE + bh;
    int pos_col = xd->mi_col * MI_SIZE;
    int valid = fill_warp_corner_projected_point(neighbor_mi, this_ref, pos_col,
                                                 pos_row, pts, mvs, np);
    if (valid) {
      valid_points++;
    }
  }

  assert(valid_points <= 3);
  return valid_points;
}
#endif  // CONFIG_EXTENDED_WARP_PREDICTION
