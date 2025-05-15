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

#include <math.h>
#include <string.h>

#include "config/aom_dsp_rtcd.h"
#include "config/aom_scale_rtcd.h"

#include "aom/aom_integer.h"
#include "aom_ports/system_state.h"
#include "av1/common/av1_common_int.h"
#include "av1/common/reconinter.h"
#include "av1/encoder/encoder.h"
#include "av1/encoder/pickcdef.h"

#define REDUCED_PRI_STRENGTHS_LVL1 8
#define REDUCED_PRI_STRENGTHS_LVL2 5
#define REDUCED_SEC_STRENGTHS_LVL3 2

#define REDUCED_TOTAL_STRENGTHS_LVL1 \
  (REDUCED_PRI_STRENGTHS_LVL1 * CDEF_SEC_STRENGTHS)
#define REDUCED_TOTAL_STRENGTHS_LVL2 \
  (REDUCED_PRI_STRENGTHS_LVL2 * CDEF_SEC_STRENGTHS)
#define REDUCED_TOTAL_STRENGTHS_LVL3 \
  (REDUCED_PRI_STRENGTHS_LVL2 * REDUCED_SEC_STRENGTHS_LVL3)
#define TOTAL_STRENGTHS (CDEF_PRI_STRENGTHS * CDEF_SEC_STRENGTHS)

static const int priconv_lvl1[REDUCED_PRI_STRENGTHS_LVL1] = { 0, 1, 2,  3,
                                                              5, 7, 10, 13 };
static const int priconv_lvl2[REDUCED_PRI_STRENGTHS_LVL2] = { 0, 2, 4, 8, 14 };
static const int secconv_lvl3[REDUCED_SEC_STRENGTHS_LVL3] = { 0, 2 };
static const int nb_cdef_strengths[CDEF_PICK_METHODS] = {
  TOTAL_STRENGTHS, REDUCED_TOTAL_STRENGTHS_LVL1, REDUCED_TOTAL_STRENGTHS_LVL2,
  REDUCED_TOTAL_STRENGTHS_LVL3, TOTAL_STRENGTHS
};

// Get primary and secondary filter strength for the given strength index and
// search method
static INLINE void get_cdef_filter_strengths(CDEF_PICK_METHOD pick_method,
                                             int *pri_strength,
                                             int *sec_strength,
                                             int strength_idx) {
  const int tot_sec_filter = (pick_method == CDEF_FAST_SEARCH_LVL3)
                                 ? REDUCED_SEC_STRENGTHS_LVL3
                                 : CDEF_SEC_STRENGTHS;
  const int pri_idx = strength_idx / tot_sec_filter;
  const int sec_idx = strength_idx % tot_sec_filter;
  *pri_strength = pri_idx;
  *sec_strength = sec_idx;
  if (pick_method == CDEF_FULL_SEARCH) return;

  switch (pick_method) {
    case CDEF_FAST_SEARCH_LVL1: *pri_strength = priconv_lvl1[pri_idx]; break;
    case CDEF_FAST_SEARCH_LVL2: *pri_strength = priconv_lvl2[pri_idx]; break;
    case CDEF_FAST_SEARCH_LVL3:
      *pri_strength = priconv_lvl2[pri_idx];
      *sec_strength = secconv_lvl3[sec_idx];
      break;
    default: assert(0 && "Invalid CDEF search method");
  }
}

// Store CDEF filter strength calculated from strength index for given search
// method
#define STORE_CDEF_FILTER_STRENGTH(cdef_strength, pick_method, strength_idx) \
  get_cdef_filter_strengths((pick_method), &pri_strength, &sec_strength,     \
                            (strength_idx));                                 \
  cdef_strength = pri_strength * CDEF_SEC_STRENGTHS + sec_strength;

/* Search for the best strength to add as an option, knowing we
   already selected nb_strengths options. */
static uint64_t search_one(int *lev, int nb_strengths,
                           uint64_t mse[][TOTAL_STRENGTHS], int sb_count,
                           CDEF_PICK_METHOD pick_method) {
  uint64_t tot_mse[TOTAL_STRENGTHS];
  const int total_strengths = nb_cdef_strengths[pick_method];
  int i, j;
  uint64_t best_tot_mse = (uint64_t)1 << 63;
  int best_id = 0;
  memset(tot_mse, 0, sizeof(tot_mse));
  for (i = 0; i < sb_count; i++) {
    int gi;
    uint64_t best_mse = (uint64_t)1 << 63;
    /* Find best mse among already selected options. */
    for (gi = 0; gi < nb_strengths; gi++) {
      if (mse[i][lev[gi]] < best_mse) {
        best_mse = mse[i][lev[gi]];
      }
    }
    /* Find best mse when adding each possible new option. */
    for (j = 0; j < total_strengths; j++) {
      uint64_t best = best_mse;
      if (mse[i][j] < best) best = mse[i][j];
      tot_mse[j] += best;
    }
  }
  for (j = 0; j < total_strengths; j++) {
    if (tot_mse[j] < best_tot_mse) {
      best_tot_mse = tot_mse[j];
      best_id = j;
    }
  }
  lev[nb_strengths] = best_id;
  return best_tot_mse;
}

/* Search for the best luma+chroma strength to add as an option, knowing we
   already selected nb_strengths options. */
static uint64_t search_one_dual(int *lev0, int *lev1, int nb_strengths,
                                uint64_t (**mse)[TOTAL_STRENGTHS], int sb_count,
                                CDEF_PICK_METHOD pick_method) {
  uint64_t tot_mse[TOTAL_STRENGTHS][TOTAL_STRENGTHS];
  int i, j;
  uint64_t best_tot_mse = (uint64_t)1 << 63;
  int best_id0 = 0;
  int best_id1 = 0;
  const int total_strengths = nb_cdef_strengths[pick_method];
  memset(tot_mse, 0, sizeof(tot_mse));
  for (i = 0; i < sb_count; i++) {
    int gi;
    uint64_t best_mse = (uint64_t)1 << 63;
    /* Find best mse among already selected options. */
    for (gi = 0; gi < nb_strengths; gi++) {
      uint64_t curr = mse[0][i][lev0[gi]];
      curr += mse[1][i][lev1[gi]];
      if (curr < best_mse) {
        best_mse = curr;
      }
    }
    /* Find best mse when adding each possible new option. */
    for (j = 0; j < total_strengths; j++) {
      int k;
      for (k = 0; k < total_strengths; k++) {
        uint64_t best = best_mse;
        uint64_t curr = mse[0][i][j];
        curr += mse[1][i][k];
        if (curr < best) best = curr;
        tot_mse[j][k] += best;
      }
    }
  }
  for (j = 0; j < total_strengths; j++) {
    int k;
    for (k = 0; k < total_strengths; k++) {
      if (tot_mse[j][k] < best_tot_mse) {
        best_tot_mse = tot_mse[j][k];
        best_id0 = j;
        best_id1 = k;
      }
    }
  }
  lev0[nb_strengths] = best_id0;
  lev1[nb_strengths] = best_id1;
  return best_tot_mse;
}

/* Search for the set of strengths that minimizes mse. */
static uint64_t joint_strength_search(int *best_lev, int nb_strengths,
                                      uint64_t mse[][TOTAL_STRENGTHS],
                                      int sb_count,
                                      CDEF_PICK_METHOD pick_method) {
  uint64_t best_tot_mse;
  int fast = (pick_method >= CDEF_FAST_SEARCH_LVL1 &&
              pick_method <= CDEF_FAST_SEARCH_LVL3);
  int i;
  best_tot_mse = (uint64_t)1 << 63;
  /* Greedy search: add one strength options at a time. */
  for (i = 0; i < nb_strengths; i++) {
    best_tot_mse = search_one(best_lev, i, mse, sb_count, pick_method);
  }
  /* Trying to refine the greedy search by reconsidering each
     already-selected option. */
  if (!fast) {
    for (i = 0; i < 4 * nb_strengths; i++) {
      int j;
      for (j = 0; j < nb_strengths - 1; j++) best_lev[j] = best_lev[j + 1];
      best_tot_mse =
          search_one(best_lev, nb_strengths - 1, mse, sb_count, pick_method);
    }
  }
  return best_tot_mse;
}

/* Search for the set of luma+chroma strengths that minimizes mse. */
static uint64_t joint_strength_search_dual(int *best_lev0, int *best_lev1,
                                           int nb_strengths,
                                           uint64_t (**mse)[TOTAL_STRENGTHS],
                                           int sb_count,
                                           CDEF_PICK_METHOD pick_method) {
  uint64_t best_tot_mse;
  int i;
  best_tot_mse = (uint64_t)1 << 63;
  /* Greedy search: add one strength options at a time. */
  for (i = 0; i < nb_strengths; i++) {
    best_tot_mse =
        search_one_dual(best_lev0, best_lev1, i, mse, sb_count, pick_method);
  }
  /* Trying to refine the greedy search by reconsidering each
     already-selected option. */
  for (i = 0; i < 4 * nb_strengths; i++) {
    int j;
    for (j = 0; j < nb_strengths - 1; j++) {
      best_lev0[j] = best_lev0[j + 1];
      best_lev1[j] = best_lev1[j + 1];
    }
    best_tot_mse = search_one_dual(best_lev0, best_lev1, nb_strengths - 1, mse,
                                   sb_count, pick_method);
  }
  return best_tot_mse;
}

typedef void (*copy_fn_t)(uint16_t *dst, int dstride, const void *src,
                          int src_voffset, int src_hoffset, int sstride,
                          int vsize, int hsize);
typedef uint64_t (*compute_cdef_dist_t)(void *dst, int dstride, uint16_t *src,
                                        cdef_list *dlist, int cdef_count,
                                        BLOCK_SIZE bsize, int coeff_shift,
                                        int row, int col);

static void copy_sb16_16_highbd(uint16_t *dst, int dstride, const void *src,
                                int src_voffset, int src_hoffset, int sstride,
                                int vsize, int hsize) {
  int r;
  const uint16_t *src16 = (uint16_t *)src;
  const uint16_t *base = &src16[src_voffset * sstride + src_hoffset];
  for (r = 0; r < vsize; r++)
    memcpy(dst + r * dstride, base + r * sstride, hsize * sizeof(*base));
}

static INLINE void init_src_params(int *src_stride, int *width, int *height,
                                   int *width_log2, int *height_log2,
                                   BLOCK_SIZE bsize) {
  *src_stride = block_size_wide[bsize];
  *width = block_size_wide[bsize];
  *height = block_size_high[bsize];
  *width_log2 = MI_SIZE_LOG2 + mi_size_wide_log2[bsize];
  *height_log2 = MI_SIZE_LOG2 + mi_size_high_log2[bsize];
}

/* Compute MSE only on the blocks we filtered. */
static uint64_t compute_cdef_dist_highbd(void *dst, int dstride, uint16_t *src,
                                         cdef_list *dlist, int cdef_count,
                                         BLOCK_SIZE bsize, int coeff_shift,
                                         int row, int col) {
  assert(bsize == BLOCK_4X4 || bsize == BLOCK_4X8 || bsize == BLOCK_8X4 ||
         bsize == BLOCK_8X8);
  uint64_t sum = 0;
  int bi, bx, by;
  uint16_t *dst16 = (uint16_t *)dst;
  uint16_t *dst_buff = &dst16[row * dstride + col];
  int src_stride, width, height, width_log2, height_log2;
  init_src_params(&src_stride, &width, &height, &width_log2, &height_log2,
                  bsize);
  for (bi = 0; bi < cdef_count; bi++) {
    by = dlist[bi].by;
    bx = dlist[bi].bx;
    sum += aom_mse_wxh_16bit_highbd(
        &dst_buff[(by << height_log2) * dstride + (bx << width_log2)], dstride,
        &src[bi << (height_log2 + width_log2)], src_stride, width, height);
  }
  return sum >> 2 * coeff_shift;
}

static int sb_all_skip(const CommonModeInfoParams *const mi_params, int mi_row,
                       int mi_col) {
  const int maxr = AOMMIN(mi_params->mi_rows - mi_row, MI_SIZE_64X64);
  const int maxc = AOMMIN(mi_params->mi_cols - mi_col, MI_SIZE_64X64);
  const int stride = mi_params->mi_stride;
  MB_MODE_INFO **mbmi = mi_params->mi_grid_base + mi_row * stride + mi_col;
  for (int r = 0; r < maxr; ++r, mbmi += stride) {
    for (int c = 0; c < maxc; ++c) {
      if (!mbmi[c]->skip_txfm[PLANE_TYPE_Y]) return 0;
    }
  }
  return 1;
}

static void pick_cdef_from_qp(AV1_COMMON *const cm) {
  const int bd = cm->seq_params.bit_depth;
  const int q = av1_ac_quant_QTX(cm->quant_params.base_qindex, 0, 0, bd) >>
                (bd - 8 + QUANT_TABLE_BITS);
  CdefInfo *const cdef_info = &cm->cdef_info;
#if !CONFIG_CDEF_ENHANCEMENTS
  cdef_info->cdef_bits = 0;
#endif  // !CONFIG_CDEF_ENHANCEMENTS
  cdef_info->nb_cdef_strengths = 1;

  int damping_offset =
      clamp(cm->quant_params.base_qindex -
                (cm->seq_params.bit_depth == AOM_BITS_8    ? 0
                 : cm->seq_params.bit_depth == AOM_BITS_10 ? 2 * MAXQ_OFFSET
                                                           : 4 * MAXQ_OFFSET),
            MINQ, MAXQ_8_BITS) >>
      6;
  cdef_info->cdef_damping = AOMMIN(3 + damping_offset, 6);

  int predicted_y_f1 = 0;
  int predicted_y_f2 = 0;
  int predicted_uv_f1 = 0;
  int predicted_uv_f2 = 0;
  aom_clear_system_state();
  if (!frame_is_intra_only(cm)) {
    predicted_y_f1 = clamp((int)roundf(q * q * -0.0000023593946f +
                                       q * 0.0068615186f + 0.02709886f),
                           0, 15);
    predicted_y_f2 = clamp((int)roundf(q * q * -0.00000057629734f +
                                       q * 0.0013993345f + 0.03831067f),
                           0, 3);
    predicted_uv_f1 = clamp((int)roundf(q * q * -0.0000007095069f +
                                        q * 0.0034628846f + 0.00887099f),
                            0, 15);
    predicted_uv_f2 = clamp((int)roundf(q * q * 0.00000023874085f +
                                        q * 0.00028223585f + 0.05576307f),
                            0, 3);
  } else {
    predicted_y_f1 = clamp(
        (int)roundf(q * q * 0.0000033731974f + q * 0.008070594f + 0.0187634f),
        0, 15);
    predicted_y_f2 = clamp(
        (int)roundf(q * q * 0.0000029167343f + q * 0.0027798624f + 0.0079405f),
        0, 3);
    predicted_uv_f1 = clamp(
        (int)roundf(q * q * -0.0000130790995f + q * 0.012892405f - 0.00748388f),
        0, 15);
    predicted_uv_f2 = clamp((int)roundf(q * q * 0.0000032651783f +
                                        q * 0.00035520183f + 0.00228092f),
                            0, 3);
  }
  cdef_info->cdef_strengths[0] =
      predicted_y_f1 * CDEF_SEC_STRENGTHS + predicted_y_f2;
  cdef_info->cdef_uv_strengths[0] =
      predicted_uv_f1 * CDEF_SEC_STRENGTHS + predicted_uv_f2;

  const CommonModeInfoParams *const mi_params = &cm->mi_params;
  const int nvfb = (mi_params->mi_rows + MI_SIZE_64X64 - 1) / MI_SIZE_64X64;
  const int nhfb = (mi_params->mi_cols + MI_SIZE_64X64 - 1) / MI_SIZE_64X64;
  MB_MODE_INFO **mbmi = mi_params->mi_grid_base;
  for (int r = 0; r < nvfb; ++r) {
    for (int c = 0; c < nhfb; ++c) {
#if CONFIG_BRU
      if (mbmi[MI_SIZE_64X64 * c]->sb_active_mode != BRU_ACTIVE_SB) {
        mbmi[MI_SIZE_64X64 * c]->cdef_strength = -1;
      } else
#endif  // CONFIG_BRU
        mbmi[MI_SIZE_64X64 * c]->cdef_strength = 0;
    }
    mbmi += MI_SIZE_64X64 * mi_params->mi_stride;
  }
}

#if CONFIG_CDEF_ENHANCEMENTS
static AOM_INLINE int get_cdef_context(const AV1_COMMON *const cm,
                                       int cur_cdef_pos) {
  const CommonModeInfoParams *const mi_params = &cm->mi_params;

  int mi_row = cur_cdef_pos / mi_params->mi_stride;
  int mi_col = cur_cdef_pos % mi_params->mi_stride;

  const int neighbor0_cdef_x = mi_col - MI_SIZE_64X64;
  const int neighbor0_cdef_y = mi_row;
  const int neighbor1_cdef_x = mi_col;
  const int neighbor1_cdef_y = mi_row - MI_SIZE_64X64;
  if (mi_row > 0 && mi_col > 0) {
    const int neighbor0_cdef_idx =
        neighbor0_cdef_y * mi_params->mi_stride + neighbor0_cdef_x;
    const int neighbor1_cdef_idx =
        neighbor1_cdef_y * mi_params->mi_stride + neighbor1_cdef_x;

    const int neighbor0_strength_0 =
        mi_params->mi_grid_base[neighbor0_cdef_idx]->cdef_strength == 0;
    const int neighbor1_strength_0 =
        mi_params->mi_grid_base[neighbor1_cdef_idx]->cdef_strength == 0;
    return neighbor0_strength_0 && neighbor1_strength_0
               ? 3
               : neighbor0_strength_0 || neighbor1_strength_0;
  } else if (mi_row > 0 || mi_col > 0) {
    int neighbor_cdef_idx = 0;
    if (mi_row > 0) {
      neighbor_cdef_idx =
          neighbor1_cdef_y * mi_params->mi_stride + neighbor1_cdef_x;
    } else {
      neighbor_cdef_idx =
          neighbor0_cdef_y * mi_params->mi_stride + neighbor0_cdef_x;
    }

    const int neighbor_strength_0 =
        mi_params->mi_grid_base[neighbor_cdef_idx]->cdef_strength == 0;
    return neighbor_strength_0 ? 2 : 0;
  } else {
    return 0;
  }
}

static AOM_INLINE void reset_frame_mi_cdef_strength(AV1_COMMON *cm) {
  CommonModeInfoParams *const mi_params = &cm->mi_params;
  const int mi_rows = mi_params->mi_rows;
  const int mi_cols = mi_params->mi_cols;
  const int mi_stride = mi_params->mi_stride;
  for (int mi_row = 0; mi_row < mi_rows; mi_row++) {
    for (int mi_col = 0; mi_col < mi_cols; mi_col++) {
      const int grid_idx = mi_row * mi_stride + mi_col;
      MB_MODE_INFO *const mbmi = mi_params->mi_grid_base[grid_idx];
      mbmi->cdef_strength = -1;
    }
  }
}
#endif  // CONFIG_CDEF_ENHANCEMENTS

void av1_cdef_search(const YV12_BUFFER_CONFIG *frame,
                     const YV12_BUFFER_CONFIG *ref, AV1_COMMON *cm,
                     MACROBLOCKD *xd,
#if CONFIG_CDEF_ENHANCEMENTS && CONFIG_ENTROPY_STATS
                     ThreadData *td,
#endif  // CONFIG_CDEF_ENHANCEMENTS && CONFIG_ENTROPY_STATS
                     CDEF_PICK_METHOD pick_method, int rdmult) {
#if CONFIG_CDEF_ENHANCEMENTS
  if (cm->seq_params.enable_cdef_on_skip_txfm == CDEF_ON_SKIP_TXFM_DISABLED) {
    cm->cdef_info.cdef_on_skip_txfm_frame_enable = 0;
  } else {
    cm->cdef_info.cdef_on_skip_txfm_frame_enable = 1;
  }

  reset_frame_mi_cdef_strength(cm);
#endif  // CONFIG_CDEF_ENHANCEMENTS

  if (pick_method == CDEF_PICK_FROM_Q) {
    pick_cdef_from_qp(cm);
    return;
  }

#if CONFIG_CDEF_ENHANCEMENTS
  aom_cdf_prob cdef_strength_index0_cdf[CDEF_STRENGTH_INDEX0_CTX][CDF_SIZE(2)];
  aom_cdf_prob cdef_cdf[CDEF_STRENGTHS_NUM - 1][CDF_SIZE(CDEF_STRENGTHS_NUM)];
  av1_copy(cdef_strength_index0_cdf, cm->fc->cdef_strength_index0_cdf);
  av1_copy(cdef_cdf, cm->fc->cdef_cdf);
#endif  // CONFIG_CDEF_ENHANCEMENTS

#if CONFIG_EXT_RECUR_PARTITIONS
  cdef_list dlist[MI_SIZE_256X256 * MI_SIZE_256X256];
#else
  cdef_list dlist[MI_SIZE_128X128 * MI_SIZE_128X128];
#endif  // CONFIG_EXT_RECUR_PARTITIONS
  int dir[CDEF_NBLOCKS][CDEF_NBLOCKS] = { { 0 } };
  int var[CDEF_NBLOCKS][CDEF_NBLOCKS] = { { 0 } };
  const CommonModeInfoParams *const mi_params = &cm->mi_params;
  const int nvfb = (mi_params->mi_rows + MI_SIZE_64X64 - 1) / MI_SIZE_64X64;
  const int nhfb = (mi_params->mi_cols + MI_SIZE_64X64 - 1) / MI_SIZE_64X64;
  int *sb_index = aom_malloc(nvfb * nhfb * sizeof(*sb_index));

  int damping_offset =
      clamp(cm->quant_params.base_qindex -
                (cm->seq_params.bit_depth == AOM_BITS_8    ? 0
                 : cm->seq_params.bit_depth == AOM_BITS_10 ? 2 * MAXQ_OFFSET
                                                           : 4 * MAXQ_OFFSET),
            MINQ, MAXQ_8_BITS) >>
      6;
  const int damping = AOMMIN(3 + damping_offset, 6);
  const int fast = (pick_method >= CDEF_FAST_SEARCH_LVL1 &&
                    pick_method <= CDEF_FAST_SEARCH_LVL3);
  const int total_strengths = nb_cdef_strengths[pick_method];
  DECLARE_ALIGNED(32, uint16_t, tmp_dst[1 << (MAX_SB_SIZE_LOG2 * 2)]);
  const int num_planes = av1_num_planes(cm);
  av1_setup_dst_planes(xd->plane, frame, 0, 0, 0, num_planes, NULL);
  uint64_t(*mse[2])[TOTAL_STRENGTHS];
  mse[0] = aom_malloc(sizeof(**mse) * nvfb * nhfb);
  mse[1] = aom_malloc(sizeof(**mse) * nvfb * nhfb);
  uint64_t unfiltered_mse = 0;

  int bsize[3];
  int mi_wide_l2[3];
  int mi_high_l2[3];
  int xdec[3];
  int ydec[3];
  uint16_t *ref_buffer[3] = { ref->y_buffer, ref->u_buffer, ref->v_buffer };
  int ref_stride[3] = { ref->y_stride, ref->uv_stride, ref->uv_stride };

  for (int pli = 0; pli < num_planes; pli++) {
    xdec[pli] = xd->plane[pli].subsampling_x;
    ydec[pli] = xd->plane[pli].subsampling_y;
    bsize[pli] = ydec[pli] ? (xdec[pli] ? BLOCK_4X4 : BLOCK_8X4)
                           : (xdec[pli] ? BLOCK_4X8 : BLOCK_8X8);
    mi_wide_l2[pli] = MI_SIZE_LOG2 - xd->plane[pli].subsampling_x;
    mi_high_l2[pli] = MI_SIZE_LOG2 - xd->plane[pli].subsampling_y;
  }

  copy_fn_t copy_fn;
  compute_cdef_dist_t compute_cdef_dist_fn;

  copy_fn = copy_sb16_16_highbd;
  compute_cdef_dist_fn = compute_cdef_dist_highbd;

  DECLARE_ALIGNED(32, uint16_t, inbuf[CDEF_INBUF_SIZE]);
  uint16_t *const in = inbuf + CDEF_VBORDER * CDEF_BSTRIDE + CDEF_HBORDER;
  const int coeff_shift = AOMMAX(cm->seq_params.bit_depth - 8, 0);
  int sb_count = 0;
  for (int fbr = 0; fbr < nvfb; ++fbr) {
    for (int fbc = 0; fbc < nhfb; ++fbc) {
      // No filtering if the entire filter block is skipped
      if (
#if CONFIG_CDEF_ENHANCEMENTS
          cm->cdef_info.cdef_on_skip_txfm_frame_enable == 0 &&
#endif  // CONFIG_CDEF_ENHANCEMENTS
          sb_all_skip(mi_params, fbr * MI_SIZE_64X64, fbc * MI_SIZE_64X64)) {
        continue;
      }

#if CONFIG_BRU
      MB_MODE_INFO *const mbmi =
          mi_params->mi_grid_base[MI_SIZE_64X64 * fbr * mi_params->mi_stride +
                                  MI_SIZE_64X64 * fbc];
      if (mbmi->sb_active_mode != BRU_ACTIVE_SB) {
        mbmi->cdef_strength = -1;
        continue;
      }
#else
      const MB_MODE_INFO *const mbmi =
          mi_params->mi_grid_base[MI_SIZE_64X64 * fbr * mi_params->mi_stride +
                                  MI_SIZE_64X64 * fbc];
#endif  // CONFIG_BRU
      BLOCK_SIZE bs = mbmi->sb_type[PLANE_TYPE_Y];
#if CONFIG_EXT_RECUR_PARTITIONS
      if (bs > BLOCK_64X64 && bs <= BLOCK_256X256) {
        const int bw = block_size_wide[bs];
        const int bh = block_size_high[bs];
        if ((bw == 256 && (fbc & 3)) || (bh == 256 && (fbr & 3))) {
          continue;
        };
        if ((bw == 128 && (fbc & 1)) || (bh == 128 && (fbr & 1))) {
          continue;
        };
      }
#else
      if (((fbc & 1) && (mbmi->sb_type[PLANE_TYPE_Y] == BLOCK_128X128 ||
                         mbmi->sb_type[PLANE_TYPE_Y] == BLOCK_128X64)) ||
          ((fbr & 1) && (mbmi->sb_type[PLANE_TYPE_Y] == BLOCK_128X128 ||
                         mbmi->sb_type[PLANE_TYPE_Y] == BLOCK_64X128)))
        continue;
#endif  // CONFIG_EXT_RECUR_PARTITIONS

      int nhb = AOMMIN(MI_SIZE_64X64, mi_params->mi_cols - MI_SIZE_64X64 * fbc);
      int nvb = AOMMIN(MI_SIZE_64X64, mi_params->mi_rows - MI_SIZE_64X64 * fbr);
      int hb_step = 1;
      int vb_step = 1;
#if CONFIG_EXT_RECUR_PARTITIONS
      if (bs > BLOCK_64X64 && bs <= BLOCK_256X256) {
        if (block_size_wide[bs] == 256) {
          nhb =
              AOMMIN(MI_SIZE_256X256, mi_params->mi_cols - MI_SIZE_64X64 * fbc);
          hb_step = 4;
        }
        if (block_size_high[bs] == 256) {
          nvb =
              AOMMIN(MI_SIZE_256X256, mi_params->mi_rows - MI_SIZE_64X64 * fbr);
          vb_step = 4;
        }
        if (block_size_wide[bs] == 128) {
          nhb =
              AOMMIN(MI_SIZE_128X128, mi_params->mi_cols - MI_SIZE_64X64 * fbc);
          hb_step = 2;
        }
        if (block_size_high[bs] == 128) {
          nvb =
              AOMMIN(MI_SIZE_128X128, mi_params->mi_rows - MI_SIZE_64X64 * fbr);
          vb_step = 2;
        }
      } else {
        bs = BLOCK_64X64;
      }
#else
      if (mbmi->sb_type[PLANE_TYPE_Y] == BLOCK_128X128 ||
          mbmi->sb_type[PLANE_TYPE_Y] == BLOCK_128X64 ||
          mbmi->sb_type[PLANE_TYPE_Y] == BLOCK_64X128) {
        bs = mbmi->sb_type[PLANE_TYPE_Y];
        if (bs == BLOCK_128X128 || bs == BLOCK_128X64) {
          nhb =
              AOMMIN(MI_SIZE_128X128, mi_params->mi_cols - MI_SIZE_64X64 * fbc);
          hb_step = 2;
        }
        if (bs == BLOCK_128X128 || bs == BLOCK_64X128) {
          nvb =
              AOMMIN(MI_SIZE_128X128, mi_params->mi_rows - MI_SIZE_64X64 * fbr);
          vb_step = 2;
        }
      } else {
        bs = BLOCK_64X64;
      }
#endif  // CONFIG_EXT_RECUR_PARTITIONS

      const int cdef_count = av1_cdef_compute_sb_list(
#if CONFIG_CDEF_ENHANCEMENTS
          cm,
#endif  // CONFIG_CDEF_ENHANCEMENTS
          mi_params, fbr * MI_SIZE_64X64, fbc * MI_SIZE_64X64, dlist, bs);

      const int yoff = CDEF_VBORDER * (fbr != 0);
      const int xoff = CDEF_HBORDER * (fbc != 0);
      int dirinit = 0;
      for (int pli = 0; pli < num_planes; pli++) {
        for (int i = 0; i < CDEF_INBUF_SIZE; i++) inbuf[i] = CDEF_VERY_LARGE;
        /* We avoid filtering the pixels for which some of the pixels to
           average are outside the frame. We could change the filter instead,
           but it would add special cases for any future vectorization. */
        const int ysize = (nvb << mi_high_l2[pli]) +
                          CDEF_VBORDER * (fbr + vb_step < nvfb) + yoff;
        const int xsize = (nhb << mi_wide_l2[pli]) +
                          CDEF_HBORDER * (fbc + hb_step < nhfb) + xoff;
        const int row = fbr * MI_SIZE_64X64 << mi_high_l2[pli];
        const int col = fbc * MI_SIZE_64X64 << mi_wide_l2[pli];
        for (int gi = 0; gi < total_strengths; gi++) {
          int pri_strength, sec_strength;
          get_cdef_filter_strengths(pick_method, &pri_strength, &sec_strength,
                                    gi);
          copy_fn(&in[(-yoff * CDEF_BSTRIDE - xoff)], CDEF_BSTRIDE,
                  xd->plane[pli].dst.buf, row - yoff, col - xoff,
                  xd->plane[pli].dst.stride, ysize, xsize);
          av1_cdef_filter_fb(
              NULL, tmp_dst, CDEF_BSTRIDE, in, xdec[pli], ydec[pli], dir,
              &dirinit, var, pli, dlist, cdef_count, pri_strength,
              sec_strength + (sec_strength == 3), damping, coeff_shift);
          const uint64_t curr_mse = compute_cdef_dist_fn(
              ref_buffer[pli], ref_stride[pli], tmp_dst, dlist, cdef_count,
              bsize[pli], coeff_shift, row, col);
          if (gi == 0) {
            unfiltered_mse += curr_mse;
          }
          if (pli < 2)
            mse[pli][sb_count][gi] = curr_mse;
          else
            mse[1][sb_count][gi] += curr_mse;
        }
      }
      sb_index[sb_count++] =
          MI_SIZE_64X64 * fbr * mi_params->mi_stride + MI_SIZE_64X64 * fbc;
    }
  }

  /* Search for different number of signalling bits. */
#if !CONFIG_CDEF_ENHANCEMENTS
  int nb_strength_bits = 0;
#endif  // !CONFIG_CDEF_ENHANCEMENTS
#if CONFIG_CDEF_ENHANCEMENTS
  int best_nb_strength = 0;
#endif  // CONFIG_CDEF_ENHANCEMENTS
  uint64_t best_rd = UINT64_MAX;
  CdefInfo *const cdef_info = &cm->cdef_info;
  int best_cdef_on = 0;
#if CONFIG_CDEF_ENHANCEMENTS
  for (int nb_strengths = 1; nb_strengths <= 8; nb_strengths++) {
#else
  for (int i = 0; i <= 3; i++) {
    const int nb_strengths = 1 << i;
#endif  // CONFIG_CDEF_ENHANCEMENTS
    int best_lev0[CDEF_MAX_STRENGTHS];
    int best_lev1[CDEF_MAX_STRENGTHS] = { 0 };
    uint64_t tot_mse;
    if (num_planes > 1) {
      tot_mse = joint_strength_search_dual(best_lev0, best_lev1, nb_strengths,
                                           mse, sb_count, pick_method);
    } else {
      tot_mse = joint_strength_search(best_lev0, nb_strengths, mse[0], sb_count,
                                      pick_method);
    }

#if CONFIG_CDEF_ENHANCEMENTS
    int luma_strength_less_4_count = 0;
    int chroma_strength_less_4_count = 0;
    for (int level = 0; level < nb_strengths; ++level) {
      if (best_lev0[level] < 4) {
        luma_strength_less_4_count++;
      }
      if (num_planes > 1 && best_lev1[level] < 4) {
        chroma_strength_less_4_count++;
      }
    }
    int luma_bits = luma_strength_less_4_count * 2;
    luma_bits +=
        (nb_strengths - luma_strength_less_4_count) * CDEF_STRENGTH_BITS;
    // Bits to indicate if each strength is less than 4
    luma_bits += nb_strengths;

    int chroma_bits = 0;
    if (num_planes > 1) {
      chroma_bits += chroma_strength_less_4_count * 2;
      chroma_bits +=
          (nb_strengths - chroma_strength_less_4_count) * CDEF_STRENGTH_BITS;
      // Bits to indicate if each strength is less than 4
      chroma_bits += nb_strengths;
    }
#endif  // CONFIG_CDEF_ENHANCEMENTS

#if CONFIG_FIX_CDEF_SYNTAX
    /* check if cdef is on for the current frame, and assign total bits
     * accordingly. */
    const int cdef_on_bits =
#if CONFIG_CDEF_ENHANCEMENTS
        luma_bits + chroma_bits + 1 + 2 + 3;
#else
        sb_count * i +
        nb_strengths * CDEF_STRENGTH_BITS * (num_planes > 1 ? 2 : 1) + 1 + 2 +
        2;
#endif  // CONFIG_CDEF_ENHANCEMENTS

    const int cdef_off_bit = 1;
#if CONFIG_CDEF_ENHANCEMENTS
    const int is_cdef_on = (nb_strengths != 1 || best_lev0[0] || best_lev1[0]);
#else
    const int is_cdef_on = (i || best_lev0[0] || best_lev1[0]);
#endif  // CONFIG_CDEF_ENHANCEMENTS
    const int total_bits = is_cdef_on ? cdef_on_bits : cdef_off_bit;
#else
    const int total_bits =
#if CONFIG_CDEF_ENHANCEMENTS
        offset_bits + luma_bits +
        chroma_bits
#else
        sb_count * i +
        nb_strengths * CDEF_STRENGTH_BITS * (num_planes > 1 ? 2 : 1);
#endif  // CONFIG_CDEF_ENHANCEMENTS
#endif  // CONFIG_FIX_CDEF_SYNTAX
#if CONFIG_CDEF_ENHANCEMENTS
    int rate_cost = av1_cost_literal(total_bits);
    if (is_cdef_on) {
      tot_mse = 0;
      for (int count_idx = 0; count_idx < sb_count; count_idx++) {
        uint64_t best_mse = UINT64_MAX;
        uint64_t best_rdo = UINT64_MAX;
        int best_sb_rate_cost = 0;
        int best_gi = 0;
#if CONFIG_CDEF_ENHANCEMENTS
        int strength_index0_cost_from_cdf[CDEF_STRENGTH_INDEX0_CTX][2];
        int cost_from_cdf[CDEF_STRENGTHS_NUM - 1][CDEF_STRENGTHS_NUM];
        if (nb_strengths >= 2) {
          for (int ctx = 0; ctx < CDEF_STRENGTH_INDEX0_CTX; ++ctx) {
            av1_cost_tokens_from_cdf(strength_index0_cost_from_cdf[ctx],
                                     cdef_strength_index0_cdf[ctx], NULL);
          }
          if (nb_strengths > 2) {
            av1_cost_tokens_from_cdf(cost_from_cdf[nb_strengths - 3],
                                     cdef_cdf[nb_strengths - 3], NULL);
          }
        }
        const int strength_index0_ctx =
            get_cdef_context(cm, sb_index[count_idx]);
#endif  // CONFIG_CDEF_ENHANCEMENTS
        for (int gi = 0; gi < nb_strengths; gi++) {
          uint64_t curr = mse[0][count_idx][best_lev0[gi]];
          if (num_planes > 1) curr += mse[1][count_idx][best_lev1[gi]];
#if CONFIG_CDEF_ENHANCEMENTS
          int sb_rate_cost = 0;
          if (nb_strengths >= 2) {
            if (gi == 0) {
              sb_rate_cost =
                  strength_index0_cost_from_cdf[strength_index0_ctx][1];
            } else if (gi == 1 && nb_strengths == 2) {
              sb_rate_cost =
                  strength_index0_cost_from_cdf[strength_index0_ctx][0];
            } else {
              sb_rate_cost =
                  strength_index0_cost_from_cdf[strength_index0_ctx][0] +
                  cost_from_cdf[nb_strengths - 3][gi - 1];
            }
          }

          const uint64_t curr_rdo = RDCOST(rdmult, sb_rate_cost, curr * 16);
#endif  // CONFIG_CDEF_ENHANCEMENTS
          if (curr_rdo < best_rdo) {
            best_gi = gi;
            best_sb_rate_cost = sb_rate_cost;
            best_mse = curr;
            best_rdo = curr_rdo;
          }
        }
        rate_cost += best_sb_rate_cost;
        tot_mse += best_mse;
#if CONFIG_CDEF_ENHANCEMENTS
        if (nb_strengths >= 2) {
          update_cdf(cdef_strength_index0_cdf[strength_index0_ctx],
                     best_gi == 0, 2);
          if (nb_strengths > 2 && best_gi >= 1) {
            update_cdf(cdef_cdf[nb_strengths - 3], best_gi - 1,
                       nb_strengths - 1);
          }
        }

        mi_params->mi_grid_base[sb_index[count_idx]]->cdef_strength = best_gi;
        BLOCK_SIZE bsize_y =
            mi_params->mi_grid_base[sb_index[count_idx]]->sb_type[PLANE_TYPE_Y];
        const int bh = mi_size_high[bsize_y];
        const int bw = mi_size_wide[bsize_y];
        int mi_row = sb_index[count_idx] / mi_params->mi_stride;
        int mi_col = sb_index[count_idx] % mi_params->mi_stride;
        if (
#if CONFIG_EXT_RECUR_PARTITIONS
            bsize_y == BLOCK_256X256 || bsize_y == BLOCK_256X128 ||
            bsize_y == BLOCK_128X256 ||
#endif  // CONFIG_EXT_RECUR_PARTITIONS
            bsize_y == BLOCK_128X128 || bsize_y == BLOCK_128X64 ||
            bsize_y == BLOCK_64X128) {
          const int x_inside_boundary = AOMMIN(bw, mi_params->mi_cols - mi_col);
          const int y_inside_boundary = AOMMIN(bh, mi_params->mi_rows - mi_row);
          int idx = mi_params->mi_stride;
          for (int y = 0; y < y_inside_boundary; ++y) {
            for (int x = 0; x < x_inside_boundary; ++x) {
              mi_params->mi_grid_base[sb_index[count_idx] + y * idx + x]
                  ->cdef_strength = best_gi;
            }
          }
        }
#endif  // CONFIG_CDEF_ENHANCEMENTS
      }
    }
#else
        const int rate_cost = av1_cost_literal(total_bits);
#endif  // CONFIG_CDEF_ENHANCEMENTS
    const uint64_t dist = tot_mse * 16;
    const uint64_t rd = RDCOST(rdmult, rate_cost, dist);
    if (rd < best_rd) {
      best_cdef_on = is_cdef_on;
      best_rd = rd;
#if !CONFIG_CDEF_ENHANCEMENTS
      nb_strength_bits = i;
#endif  // !CONFIG_CDEF_ENHANCEMENTS
#if CONFIG_CDEF_ENHANCEMENTS
      best_nb_strength = nb_strengths;
#endif  // CONFIG_CDEF_ENHANCEMENTS
      memcpy(cdef_info->cdef_strengths, best_lev0,
             nb_strengths * sizeof(best_lev0[0]));
      if (num_planes > 1) {
        memcpy(cdef_info->cdef_uv_strengths, best_lev1,
               nb_strengths * sizeof(best_lev1[0]));
      }
    }
  }

  if (best_cdef_on) {
    const uint64_t unfiltered_rd =
        RDCOST(rdmult, av1_cost_literal(1), unfiltered_mse * 16);
    if (unfiltered_rd < best_rd) {
#if !CONFIG_CDEF_ENHANCEMENTS
      nb_strength_bits = 0;
#endif  // !CONFIG_CDEF_ENHANCEMENTS
#if CONFIG_CDEF_ENHANCEMENTS
      best_nb_strength = 1;
#endif  // CONFIG_CDEF_ENHANCEMENTS
      cm->cdef_info.cdef_frame_enable = 0;
#if !CONFIG_CDEF_ENHANCEMENTS
      cm->cdef_info.cdef_bits = 0;
#endif  // !CONFIG_CDEF_ENHANCEMENTS
      cm->cdef_info.cdef_strengths[0] = 0;
      cm->cdef_info.nb_cdef_strengths = 1;
      cm->cdef_info.cdef_uv_strengths[0] = 0;
    }
  }

#if CONFIG_CDEF_ENHANCEMENTS
  av1_copy(cdef_strength_index0_cdf, cm->fc->cdef_strength_index0_cdf);
  av1_copy(cdef_cdf, cm->fc->cdef_cdf);
#endif  // CONFIG_CDEF_ENHANCEMENTS
#if !CONFIG_CDEF_ENHANCEMENTS
  cdef_info->cdef_bits = nb_strength_bits;
#endif  // !CONFIG_CDEF_ENHANCEMENTS
#if CONFIG_CDEF_ENHANCEMENTS
  cdef_info->nb_cdef_strengths = best_nb_strength;
#else
  cdef_info->nb_cdef_strengths = 1 << nb_strength_bits;
#endif  // CONFIG_CDEF_ENHANCEMENTS
  for (int i = 0; i < sb_count; i++) {
    uint64_t best_mse = UINT64_MAX;
    int best_gi = 0;
#if CONFIG_CDEF_ENHANCEMENTS
    int strength_index0_cost_from_cdf[CDEF_STRENGTH_INDEX0_CTX][2];
    int cost_from_cdf[CDEF_STRENGTHS_NUM - 1][CDEF_STRENGTHS_NUM];
    if (best_nb_strength >= 2) {
      for (int ctx = 0; ctx < CDEF_STRENGTH_INDEX0_CTX; ++ctx) {
        av1_cost_tokens_from_cdf(strength_index0_cost_from_cdf[ctx],
                                 cdef_strength_index0_cdf[ctx], NULL);
      }
      if (best_nb_strength > 2) {
        av1_cost_tokens_from_cdf(cost_from_cdf[best_nb_strength - 3],
                                 cdef_cdf[best_nb_strength - 3], NULL);
      }
    }
    const int strength_index0_ctx = get_cdef_context(cm, sb_index[i]);
#endif  // CONFIG_CDEF_ENHANCEMENTS
    for (int gi = 0; gi < best_nb_strength; gi++) {
      uint64_t curr = mse[0][i][cdef_info->cdef_strengths[gi]];
      if (num_planes > 1) curr += mse[1][i][cdef_info->cdef_uv_strengths[gi]];
#if CONFIG_CDEF_ENHANCEMENTS
      int sb_rate_cost = 0;
      if (best_nb_strength >= 2) {
        if (gi == 0) {
          sb_rate_cost = strength_index0_cost_from_cdf[strength_index0_ctx][1];
        } else if (gi == 1 && best_nb_strength == 2) {
          sb_rate_cost = strength_index0_cost_from_cdf[strength_index0_ctx][0];
        } else {
          sb_rate_cost = strength_index0_cost_from_cdf[strength_index0_ctx][0] +
                         cost_from_cdf[best_nb_strength - 3][gi - 1];
        }
      }

      curr = RDCOST(rdmult, sb_rate_cost, curr * 16);
#endif  // CONFIG_CDEF_ENHANCEMENTS
      if (curr < best_mse) {
        best_gi = gi;
        best_mse = curr;
      }
    }
#if CONFIG_CDEF_ENHANCEMENTS
    if (best_nb_strength >= 2) {
#if CONFIG_ENTROPY_STATS
      ++td->counts
            ->cdef_strength_index0_cnts[strength_index0_ctx][best_gi == 0];
      if (best_nb_strength > 2 && best_gi >= 1) {
        ++td->counts->cdef_cnts[best_nb_strength - 3][best_gi];
      }
#endif  // CONFIG_ENTROPY_STATS
      update_cdf(cdef_strength_index0_cdf[strength_index0_ctx], best_gi == 0,
                 2);
      if (best_nb_strength > 2 && best_gi >= 1) {
        update_cdf(cdef_cdf[best_nb_strength - 3], best_gi - 1,
                   best_nb_strength - 1);
      }
    }
#endif  // CONFIG_CDEF_ENHANCEMENTS

    mi_params->mi_grid_base[sb_index[i]]->cdef_strength = best_gi;
    BLOCK_SIZE bsize_y =
        mi_params->mi_grid_base[sb_index[i]]->sb_type[PLANE_TYPE_Y];
    const int bh = mi_size_high[bsize_y];
    const int bw = mi_size_wide[bsize_y];
    int mi_row = sb_index[i] / mi_params->mi_stride;
    int mi_col = sb_index[i] % mi_params->mi_stride;
#if CONFIG_CDEF_ENHANCEMENTS
    int nhb = AOMMIN(AOMMAX(bw, MI_SIZE_64X64), mi_params->mi_cols - mi_col);
    int nvb = AOMMIN(AOMMAX(bh, MI_SIZE_64X64), mi_params->mi_rows - mi_row);
    for (int j = 0; j < nvb; j++) {
      for (int k = 0; k < nhb; k++) {
        const int grid_idx = get_mi_grid_idx(mi_params, mi_row + j, mi_col + k);
        mi_params->mi_grid_base[grid_idx]->cdef_strength = best_gi;
      }
    }
#else
    if (
#if CONFIG_EXT_RECUR_PARTITIONS
        bsize_y == BLOCK_256X256 || bsize_y == BLOCK_256X128 ||
        bsize_y == BLOCK_128X256 ||
#endif  // CONFIG_EXT_RECUR_PARTITIONS
        bsize_y == BLOCK_128X128 || bsize_y == BLOCK_128X64 ||
        bsize_y == BLOCK_64X128) {
      const int x_inside_boundary = AOMMIN(bw, mi_params->mi_cols - mi_col);
      const int y_inside_boundary = AOMMIN(bh, mi_params->mi_rows - mi_row);
      int idx = mi_params->mi_stride;
      for (int y = 0; y < y_inside_boundary; ++y) {
        for (int x = 0; x < x_inside_boundary; ++x) {
          mi_params->mi_grid_base[sb_index[i] + y * idx + x]->cdef_strength =
              best_gi;
        }
      }
    }
#endif  // CONFIG_CDEF_ENHANCEMENTS
  }

  if (fast) {
    for (int j = 0; j < cdef_info->nb_cdef_strengths; j++) {
      const int luma_strength = cdef_info->cdef_strengths[j];
      const int chroma_strength = cdef_info->cdef_uv_strengths[j];
      int pri_strength, sec_strength;

      STORE_CDEF_FILTER_STRENGTH(cdef_info->cdef_strengths[j], pick_method,
                                 luma_strength);
      STORE_CDEF_FILTER_STRENGTH(cdef_info->cdef_uv_strengths[j], pick_method,
                                 chroma_strength);
    }
  }

  cdef_info->cdef_damping = damping;
#if CONFIG_FIX_CDEF_SYNTAX
  cdef_info->cdef_frame_enable = (
#if CONFIG_CDEF_ENHANCEMENTS
      cdef_info->nb_cdef_strengths > 1 ||
#else
      cdef_info->cdef_bits ||
#endif  // CONFIG_CDEF_ENHANCEMENTS
      cdef_info->cdef_strengths[0] || cdef_info->cdef_uv_strengths[0]);
#endif  // CONFIG_FIX_CDEF_SYNTAX

  aom_free(mse[0]);
  aom_free(mse[1]);
  aom_free(sb_index);
}
