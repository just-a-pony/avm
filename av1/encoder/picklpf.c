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

#include <assert.h>
#include <limits.h>

#include "config/aom_scale_rtcd.h"

#include "aom_dsp/aom_dsp_common.h"
#include "aom_dsp/psnr.h"
#include "aom_mem/aom_mem.h"
#include "aom_ports/mem.h"

#include "av1/common/av1_common_int.h"
#include "av1/common/av1_loopfilter.h"
#include "av1/common/quant_common.h"

#include "av1/encoder/av1_quantize.h"
#include "av1/encoder/encoder.h"
#include "av1/encoder/picklpf.h"

#if CONFIG_NEW_DF
#include <float.h>
#define CHROMA_LAMBDA_MULT 6
#endif  // CONFIG_NEW_DF

static void yv12_copy_plane(const YV12_BUFFER_CONFIG *src_bc,
                            YV12_BUFFER_CONFIG *dst_bc, int plane) {
  switch (plane) {
    case 0: aom_yv12_copy_y(src_bc, dst_bc); break;
    case 1: aom_yv12_copy_u(src_bc, dst_bc); break;
    case 2: aom_yv12_copy_v(src_bc, dst_bc); break;
    default: assert(plane >= 0 && plane <= 2); break;
  }
}
#if !CONFIG_NEW_DF
int av1_get_max_filter_level(const AV1_COMP *cpi) {
  (void)cpi;
  return MAX_LOOP_FILTER;
}

#endif
static int64_t try_filter_frame(const YV12_BUFFER_CONFIG *sd,
                                AV1_COMP *const cpi,
#if CONFIG_NEW_DF
                                int q_offset, int side_offset,
#else
                                int filt_level,
#endif
                                int partial_frame, int plane, int dir) {
  MultiThreadInfo *const mt_info = &cpi->mt_info;
  int num_workers = mt_info->num_workers;
  AV1_COMMON *const cm = &cpi->common;
  int64_t filt_err;

  assert(plane >= 0 && plane <= 2);
#if !CONFIG_NEW_DF
  int filter_level[2] = { filt_level, filt_level };
  if (plane == 0 && dir == 0) filter_level[1] = cm->lf.filter_level[1];
  if (plane == 0 && dir == 1) filter_level[0] = cm->lf.filter_level[0];
#endif  // !CONFIG_NEW_DF
#if CONFIG_NEW_DF
  // set base filters for use of av1_get_filter_level when in DELTA_LF mode
  switch (plane) {
    case 0:
#if DF_DUAL
      switch (dir) {
        case 2:
          cm->lf.delta_q_luma[0] = cm->lf.delta_q_luma[1] = q_offset;
          cm->lf.delta_side_luma[0] = cm->lf.delta_side_luma[1] = side_offset;
          break;
        case 1:
        case 0:
          cm->lf.delta_q_luma[dir] = q_offset;
          cm->lf.delta_side_luma[dir] = side_offset;
          break;
      }
#else
      cm->lf.delta_q_luma = q_offset;
      cm->lf.delta_side_luma = side_offset;
#endif  // DF_DUAL
      break;
    case 1:
      cm->lf.delta_q_u = q_offset;
      cm->lf.delta_side_u = side_offset;
      break;
    case 2:
      cm->lf.delta_q_v = q_offset;
      cm->lf.delta_side_v = side_offset;
      break;
  }
#else
  // set base filters for use of av1_get_filter_level when in DELTA_LF mode
  switch (plane) {
    case 0:
      cm->lf.filter_level[0] = filter_level[0];
      cm->lf.filter_level[1] = filter_level[1];
      break;
    case 1: cm->lf.filter_level_u = filter_level[0]; break;
    case 2: cm->lf.filter_level_v = filter_level[0]; break;
  }
#endif  // CONFIG_NEW_DF
  // TODO(any): please enable multi-thread and remove the flag when loop
  // filter mask is compatible with multi-thread.
  if (num_workers > 1)
    av1_loop_filter_frame_mt(&cm->cur_frame->buf, cm, &cpi->td.mb.e_mbd, plane,
                             plane + 1, partial_frame,
#if CONFIG_LPF_MASK
                             0,
#endif
                             mt_info->workers, num_workers,
                             &mt_info->lf_row_sync);
  else
    av1_loop_filter_frame(&cm->cur_frame->buf, cm, &cpi->td.mb.e_mbd,
#if CONFIG_LPF_MASK
                          0,
#endif
                          plane, plane + 1, partial_frame);

  filt_err = aom_get_sse_plane(sd, &cm->cur_frame->buf, plane);

  // Re-instate the unfiltered frame
  yv12_copy_plane(&cpi->last_frame_uf, &cm->cur_frame->buf, plane);

  return filt_err;
}

#if CONFIG_NEW_DF
static int search_filter_offsets(const YV12_BUFFER_CONFIG *sd, AV1_COMP *cpi,
                                 int partial_frame,
                                 const int *last_frame_offsets,
                                 double *best_cost_ret, int plane,
                                 int search_side_offset
#if DF_DUAL
                                 ,
                                 int dir
#endif
) {
  const AV1_COMMON *const cm = &cpi->common;
  const int min_filter_offset = DF_PAR_MIN_VAL;
  const int max_filter_offset = DF_PAR_MAX_VAL;
  int filt_direction = 0;
  int64_t best_err, start_err;
  int offset_best;
  MACROBLOCK *x = &cpi->td.mb;
  int offsets[2];
  int off_ind = search_side_offset;
  int temp_offsets[2];

  assert(plane >= 0 && plane <= 2);

  // Start the search at the previous frame filter level unless it is now out of
  // range.

#if DF_DUAL
  switch (plane) {
    case 0:
      switch (dir) {
        case 2:
          offsets[0] = (last_frame_offsets[0] + last_frame_offsets[2] + 1) >> 1;
          offsets[1] = (last_frame_offsets[1] + last_frame_offsets[3] + 1) >> 1;
          break;
        case 0:
          offsets[0] = last_frame_offsets[0];
          offsets[1] = last_frame_offsets[1];
          break;
        case 1:
          offsets[0] = last_frame_offsets[2];
          offsets[1] = last_frame_offsets[3];
          break;
        default: assert(dir >= 0 && dir <= 2); return 0;
      }
      break;
    case 1:
      offsets[0] = last_frame_offsets[4];
      offsets[1] = last_frame_offsets[5];
      break;
    case 2:
      offsets[0] = last_frame_offsets[6];
      offsets[1] = last_frame_offsets[7];
      break;
    default: assert(plane >= 0 && plane <= 2); return 0;
  }

#else
  offsets[0] = last_frame_offsets[plane << 1];
  offsets[1] = last_frame_offsets[(plane << 1) + 1];
#endif  // DF_DUAL
  int offset_mid =
      clamp(offsets[off_ind], min_filter_offset, max_filter_offset);
  //  int filter_step = abs(offset_mid) < 16 ? 8 : abs(offset_mid) / 2;  // is
  //  this a good number?
  int filter_step = DF_SEARCH_STEP_SIZE;  // is this a good number?

  // Sum squared error at each filter level
  int64_t ss_err[MAX_DF_OFFSETS + 1];

  // Set each entry to -1
  memset(ss_err, 0xFF, sizeof(ss_err));
  yv12_copy_plane(&cm->cur_frame->buf, &cpi->last_frame_uf, plane);

#if DF_TWO_PARAM
  temp_offsets[0] = offsets[0];
  temp_offsets[1] = offsets[1];
  temp_offsets[off_ind] = offset_mid;
#else
  temp_offsets[0] = temp_offsets[1] = offset_mid;
#endif  // DF_TWO_PARAM

#if DF_DUAL
  start_err = best_err = try_filter_frame(
      sd, cpi, temp_offsets[0], temp_offsets[1], partial_frame, plane, dir);
#else
  start_err = best_err = try_filter_frame(
      sd, cpi, temp_offsets[0], temp_offsets[1], partial_frame, plane, 0);
#endif  // DF_DUAL
  offset_best = offset_mid;
  ss_err[offset_mid + ZERO_DF_OFFSET] = best_err;

  while (filter_step > 0) {
    const int offset_high = AOMMIN(offset_mid + filter_step, max_filter_offset);
    const int offset_low = AOMMAX(offset_mid - filter_step, min_filter_offset);

    int64_t bias = 0;

    // yx, bias less for large block size
    if (cm->features.tx_mode != ONLY_4X4) bias >>= 1;

    if (filt_direction <= 0 && offset_low != offset_mid) {
      // Get Low filter error score
      if (ss_err[offset_low + ZERO_DF_OFFSET] < 0) {
#if DF_TWO_PARAM
        temp_offsets[off_ind] = offset_low;
#else
        temp_offsets[0] = temp_offsets[1] = offset_low;
#endif  // DF_TWO_PARAM
        ss_err[offset_low + ZERO_DF_OFFSET] =
#if DF_DUAL
            try_filter_frame(sd, cpi, temp_offsets[0], temp_offsets[1],
                             partial_frame, plane, dir);
#else
            try_filter_frame(sd, cpi, temp_offsets[0], temp_offsets[1],
                             partial_frame, plane, 0);
#endif  // DF_TWO_PARAM
      }
      // If value is close to the best so far then bias towards a lower loop
      // filter value.
      if (ss_err[offset_low + ZERO_DF_OFFSET] < (best_err + bias)) {
        // Was it actually better than the previous best?
        if (ss_err[offset_low + ZERO_DF_OFFSET] < best_err) {
          best_err = ss_err[offset_low + ZERO_DF_OFFSET];
        }
        offset_best = offset_low;
      }
    }

    // Now look at filt_high
    if (filt_direction >= 0 && offset_high != offset_mid) {
      if (ss_err[offset_high + ZERO_DF_OFFSET] < 0) {
#if DF_TWO_PARAM
        temp_offsets[off_ind] = offset_high;
#else
        temp_offsets[0] = temp_offsets[1] = offset_high;
#endif  // DF_TWO_PARAM
        ss_err[offset_high + ZERO_DF_OFFSET] =
#if DF_DUAL
            try_filter_frame(sd, cpi, temp_offsets[0], temp_offsets[1],
                             partial_frame, plane, dir);
#else
            try_filter_frame(sd, cpi, temp_offsets[0], temp_offsets[1],
                             partial_frame, plane, 0);
#endif  // DF_DUAL
      }
      // If value is significantly better than previous best, bias added against
      // raising filter value
      if (ss_err[offset_high + ZERO_DF_OFFSET] < (best_err - bias)) {
        best_err = ss_err[offset_high + ZERO_DF_OFFSET];
        offset_best = offset_high;
      }
    }

    // Half the step distance if the best filter value was the same as last time
    if (offset_best == offset_mid) {
      filter_step /= 2;
      filt_direction = 0;
    } else {
      filt_direction = (offset_best < offset_mid) ? -1 : 1;
      offset_mid = offset_best;
    }
  }

  // Update best error
  best_err = ss_err[offset_best + ZERO_DF_OFFSET];

  int chroma_lambda_mult = plane ? CHROMA_LAMBDA_MULT : 1;
#if DF_DUAL
  int best_bits = 0;
  int start_bits = 0;
  if (dir == 2) {
    start_bits = offsets[off_ind] ? DF_PAR_BITS : 0;
    best_bits = offset_best ? DF_PAR_BITS : 0;
  } else if (dir == 0) {
    start_bits = 0;  // To not bias the first dual search, assumed that dir == 1
                     // will be run later.
    best_bits = 0;
  } else {               // dir == 1
    int vert_q_ind = 0;  // offset for the vert dir
    int vert_offset = last_frame_offsets[vert_q_ind + off_ind];
    int vert_bits = vert_offset ? DF_PAR_BITS : 0;
    start_bits =
        vert_bits + (offsets[off_ind] == vert_offset ? 0 : DF_PAR_BITS);
    best_bits = vert_bits + (offset_best == vert_offset ? 0 : DF_PAR_BITS);
  }

  double best_cost =
      RDCOST_DBL_WITH_NATIVE_BD_DIST(x->rdmult * chroma_lambda_mult, best_bits,
                                     best_err, cm->seq_params.bit_depth);
  double start_cost =
      RDCOST_DBL_WITH_NATIVE_BD_DIST(x->rdmult * chroma_lambda_mult, start_bits,
                                     start_err, cm->seq_params.bit_depth);

#else
  double best_cost = RDCOST_DBL_WITH_NATIVE_BD_DIST(
      x->rdmult * chroma_lambda_mult, offset_best ? DF_PAR_BITS : 0, best_err,
      cm->seq_params.bit_depth);
  double start_cost = RDCOST_DBL_WITH_NATIVE_BD_DIST(
      x->rdmult * chroma_lambda_mult, offsets[off_ind] ? DF_PAR_BITS : 0,
      start_err, cm->seq_params.bit_depth);
#endif  // DF_DUAL

  if (best_cost_ret) *best_cost_ret = AOMMIN(best_cost, start_cost);

  return best_cost < start_cost ? offset_best : offsets[off_ind];
}
#else
static int search_filter_level(const YV12_BUFFER_CONFIG *sd, AV1_COMP *cpi,
                               int partial_frame,
                               const int *last_frame_filter_level,
                               double *best_cost_ret, int plane, int dir) {
  const AV1_COMMON *const cm = &cpi->common;
  const int min_filter_level = 0;
  const int max_filter_level = av1_get_max_filter_level(cpi);
  int filt_direction = 0;
  int64_t best_err;
  int filt_best;
  MACROBLOCK *x = &cpi->td.mb;

  // Start the search at the previous frame filter level unless it is now out of
  // range.
  int lvl;
  switch (plane) {
    case 0:
      switch (dir) {
        case 2:
          lvl = (last_frame_filter_level[0] + last_frame_filter_level[1] + 1) >>
                1;
          break;
        case 0:
        case 1: lvl = last_frame_filter_level[dir]; break;
        default: assert(dir >= 0 && dir <= 2); return 0;
      }
      break;
    case 1: lvl = last_frame_filter_level[2]; break;
    case 2: lvl = last_frame_filter_level[3]; break;
    default: assert(plane >= 0 && plane <= 2); return 0;
  }
  int filt_mid = clamp(lvl, min_filter_level, max_filter_level);
  int filter_step = filt_mid < 16 ? 4 : filt_mid / 4;
  // Sum squared error at each filter level
  int64_t ss_err[MAX_LOOP_FILTER + 1];

  // Set each entry to -1
  memset(ss_err, 0xFF, sizeof(ss_err));
  yv12_copy_plane(&cm->cur_frame->buf, &cpi->last_frame_uf, plane);
  best_err = try_filter_frame(sd, cpi, filt_mid, partial_frame, plane, dir);
  filt_best = filt_mid;
  ss_err[filt_mid] = best_err;

  while (filter_step > 0) {
    const int filt_high = AOMMIN(filt_mid + filter_step, max_filter_level);
    const int filt_low = AOMMAX(filt_mid - filter_step, min_filter_level);

    // Bias against raising loop filter in favor of lowering it.
    int64_t bias = (best_err >> (15 - (filt_mid / 8))) * filter_step;

    // yx, bias less for large block size
    if (cm->features.tx_mode != ONLY_4X4) bias >>= 1;

    if (filt_direction <= 0 && filt_low != filt_mid) {
      // Get Low filter error score
      if (ss_err[filt_low] < 0) {
        ss_err[filt_low] =
            try_filter_frame(sd, cpi, filt_low, partial_frame, plane, dir);
      }
      // If value is close to the best so far then bias towards a lower loop
      // filter value.
      if (ss_err[filt_low] < (best_err + bias)) {
        // Was it actually better than the previous best?
        if (ss_err[filt_low] < best_err) {
          best_err = ss_err[filt_low];
        }
        filt_best = filt_low;
      }
    }

    // Now look at filt_high
    if (filt_direction >= 0 && filt_high != filt_mid) {
      if (ss_err[filt_high] < 0) {
        ss_err[filt_high] =
            try_filter_frame(sd, cpi, filt_high, partial_frame, plane, dir);
      }
      // If value is significantly better than previous best, bias added against
      // raising filter value
      if (ss_err[filt_high] < (best_err - bias)) {
        best_err = ss_err[filt_high];
        filt_best = filt_high;
      }
    }

    // Half the step distance if the best filter value was the same as last time
    if (filt_best == filt_mid) {
      filter_step /= 2;
      filt_direction = 0;
    } else {
      filt_direction = (filt_best < filt_mid) ? -1 : 1;
      filt_mid = filt_best;
    }
  }

  // Update best error
  best_err = ss_err[filt_best];

  if (best_cost_ret)
    *best_cost_ret = RDCOST_DBL_WITH_NATIVE_BD_DIST(
        x->rdmult, 0, (best_err << 4), cm->seq_params.bit_depth);
  return filt_best;
}
#endif  // CONFIG_NEW_DF

#if CONFIG_NEW_DF
void av1_pick_filter_level(const YV12_BUFFER_CONFIG *sd, AV1_COMP *cpi,
                           LPF_PICK_METHOD method) {
  AV1_COMMON *const cm = &cpi->common;
  const int num_planes = av1_num_planes(cm);
  struct loopfilter *const lf = &cm->lf;
  (void)sd;

  cpi->td.mb.rdmult = cpi->rd.RDMULT;

  if (method == LPF_PICK_MINIMAL_LPF) {
    lf->filter_level[0] = 0;
    lf->filter_level[1] = 0;
    lf->filter_level_u = lf->filter_level_v = 0;
  } else if (method >= LPF_PICK_FROM_Q) {
    // TODO(chengchen): retrain the model for Y, U, V filter levels
    lf->filter_level[0] = lf->filter_level[1] = 1;
    if (num_planes > 1) {
      lf->filter_level_u = lf->filter_level_v = 1;
    } else {
      lf->filter_level_u = lf->filter_level_v = 0;
    }
#if DF_DUAL
    lf->delta_q_luma[0] = lf->delta_q_luma[1] = lf->delta_q_u = lf->delta_q_v =
        0;
    lf->delta_side_luma[0] = lf->delta_side_luma[1] = lf->delta_side_u =
        lf->delta_side_v = 0;
#else
    lf->delta_q_luma = lf->delta_q_u = lf->delta_q_v = 0;
    lf->delta_side_luma = lf->delta_side_u = lf->delta_side_v = 0;
#endif  // DF_DUAL
  } else {
    // To make sure the df filters are run
    lf->filter_level[0] = 1;
    lf->filter_level[1] = 1;
    if (num_planes > 1) {
      lf->filter_level_u = lf->filter_level_v = 1;
    } else {
      lf->filter_level_u = lf->filter_level_v = 0;
    }
    // TODO(anyone): What are good initial levels for keyframes?
#if DF_DUAL
    lf->delta_q_luma[0] = lf->delta_q_luma[1] = lf->delta_q_u = lf->delta_q_v =
        0;
    lf->delta_side_luma[0] = lf->delta_side_luma[1] = lf->delta_side_u =
        lf->delta_side_v = 0;
#else
    lf->delta_q_luma = lf->delta_q_u = lf->delta_q_v = 0;
    lf->delta_side_luma = lf->delta_side_u = lf->delta_side_v = 0;
#endif  // DF_DUAL
#if DF_DUAL
    int last_frame_offsets[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
#else
    int last_frame_offsets[6] = { 0, 0, 0, 0, 0, 0 };
#endif  // DF_DUAL

    //    double best_cost[3] = { DBL_MAX, DBL_MAX, DBL_MAX };

#if DF_DUAL
    int dir = 0;
    double best_single_cost = DBL_MAX;
    double best_dual_cost = DBL_MAX;
    int best_single_offsets[4] = { 0, 0, 0, 0 };

    // luma
    last_frame_offsets[1] = last_frame_offsets[3] = lf->delta_side_luma[0] =
        lf->delta_side_luma[1] = search_filter_offsets(
            sd, cpi, method == LPF_PICK_FROM_SUBIMAGE, last_frame_offsets,
            &best_single_cost, 0, 1, 2);
#if DF_TWO_PARAM
    last_frame_offsets[0] = last_frame_offsets[2] = lf->delta_q_luma[0] =
        lf->delta_q_luma[1] = search_filter_offsets(
            sd, cpi, method == LPF_PICK_FROM_SUBIMAGE, last_frame_offsets,
            &best_single_cost, 0, 0, 2);
#else
    last_frame_offsets[0] = last_frame_offsets[2] = lf->delta_q_luma[0] =
        lf->delta_q_luma[1] = lf->delta_side_luma[0];
#endif  // DF_TWO_PARAM
    best_single_offsets[0] = last_frame_offsets[0];
    best_single_offsets[1] = last_frame_offsets[1];
    best_single_offsets[2] = last_frame_offsets[2];
    best_single_offsets[3] = last_frame_offsets[3];

    last_frame_offsets[1] = lf->delta_side_luma[0] =
        search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                              last_frame_offsets, &best_dual_cost, 0, 1, 0);
#if DF_TWO_PARAM
    last_frame_offsets[0] = lf->delta_q_luma[0] =
        search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                              last_frame_offsets, &best_dual_cost, 0, 0, 0);
#else
    last_frame_offsets[0] = lf->delta_q_luma[0] = lf->delta_side_luma[0];
#endif  // DF_TWO_PARAM

    last_frame_offsets[3] = lf->delta_side_luma[1] =
        search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                              last_frame_offsets, &best_dual_cost, 0, 1, 1);
#if DF_TWO_PARAM
    last_frame_offsets[2] = lf->delta_q_luma[1] =
        search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                              last_frame_offsets, &best_dual_cost, 0, 0, 1);
#else
    last_frame_offsets[2] = lf->delta_q_luma[1] = lf->delta_side_luma[1];
#endif  // DF_TWO_PARAM

    if (best_single_cost < best_dual_cost) {
      lf->delta_q_luma[0] = last_frame_offsets[0] = best_single_offsets[0];
      lf->delta_side_luma[0] = last_frame_offsets[1] = best_single_offsets[1];
      lf->delta_q_luma[1] = last_frame_offsets[2] = best_single_offsets[2];
      lf->delta_side_luma[1] = last_frame_offsets[3] = best_single_offsets[3];
    }

    if (num_planes > 1) {
      // Cb
      last_frame_offsets[5] = lf->delta_side_u =
          search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                                last_frame_offsets, NULL, 1, 1, dir);
#if DF_TWO_PARAM
      last_frame_offsets[4] = lf->delta_q_u =
          search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                                last_frame_offsets, NULL, 1, 0, dir);
#else
      last_frame_offsets[4] = lf->delta_q_u = lf->delta_side_u;
#endif  // DF_TWO_PARAM

      last_frame_offsets[5] = lf->delta_side_u =
          search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                                last_frame_offsets, NULL, 1, 1, dir);
#if DF_TWO_PARAM
      last_frame_offsets[4] = lf->delta_q_u =
          search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                                last_frame_offsets, NULL, 1, 0, dir);
#else
      last_frame_offsets[4] = lf->delta_q_u = lf->delta_side_u;
#endif  // DF_TWO_PARAM
      // Cr
      last_frame_offsets[7] = lf->delta_side_v =
          search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                                last_frame_offsets, NULL, 2, 1, dir);
#if DF_TWO_PARAM
      last_frame_offsets[6] = lf->delta_q_v =
          search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                                last_frame_offsets, NULL, 2, 0, dir);
#else
      last_frame_offsets[6] = lf->delta_q_v = lf->delta_side_v;
#endif  // DF_TWO_PARAM
      last_frame_offsets[7] = lf->delta_side_v =
          search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                                last_frame_offsets, NULL, 2, 1, dir);
#if DF_TWO_PARAM
      last_frame_offsets[6] = lf->delta_q_v =
          search_filter_offsets(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                                last_frame_offsets, NULL, 2, 0, dir);
#else
      last_frame_offsets[6] = lf->delta_q_v = lf->delta_side_v;
#endif  // DF_TWO_PARAM

      // to switch off filters if offsets are zero
      if (!df_quant_from_qindex(cm->quant_params.base_qindex +
                                    cm->lf.delta_q_luma[0] * DF_DELTA_SCALE,
                                cm->seq_params.bit_depth) ||
          !df_side_from_qindex(cm->quant_params.base_qindex +
                                   cm->lf.delta_side_luma[0] * DF_DELTA_SCALE,
                               cm->seq_params.bit_depth)) {
        lf->filter_level[0] = 0;
        cm->lf.delta_q_luma[0] = 0;
        cm->lf.delta_side_luma[0] = 0;
      }
      if (!df_quant_from_qindex(cm->quant_params.base_qindex +
                                    cm->lf.delta_q_luma[1] * DF_DELTA_SCALE,
                                cm->seq_params.bit_depth) ||
          !df_side_from_qindex(cm->quant_params.base_qindex +
                                   cm->lf.delta_side_luma[1] * DF_DELTA_SCALE,
                               cm->seq_params.bit_depth)) {
        lf->filter_level[1] = 0;
        cm->lf.delta_q_luma[1] = 0;
        cm->lf.delta_side_luma[1] = 0;
      }
      if (lf->filter_level[0] == 0 && lf->filter_level[1] == 0) {
        lf->filter_level_u = 0;
        lf->filter_level_v = 0;
        cm->lf.delta_q_u = 0;
        cm->lf.delta_side_u = 0;
        cm->lf.delta_q_v = 0;
        cm->lf.delta_side_v = 0;
      } else {
        if (!df_quant_from_qindex(cm->quant_params.base_qindex +
                                      cm->quant_params.u_ac_delta_q +
                                      cm->lf.delta_q_u * DF_DELTA_SCALE,
                                  cm->seq_params.bit_depth) ||
            !df_side_from_qindex(cm->quant_params.base_qindex +
                                     cm->quant_params.u_ac_delta_q +
                                     cm->lf.delta_side_u * DF_DELTA_SCALE,
                                 cm->seq_params.bit_depth)) {
          lf->filter_level_u = 0;
          cm->lf.delta_q_u = 0;
          cm->lf.delta_side_u = 0;
        }
        if (!df_quant_from_qindex(cm->quant_params.base_qindex +
                                      cm->quant_params.v_ac_delta_q +
                                      cm->lf.delta_q_v * DF_DELTA_SCALE,
                                  cm->seq_params.bit_depth) ||
            !df_side_from_qindex(cm->quant_params.base_qindex +
                                     cm->quant_params.v_ac_delta_q +
                                     cm->lf.delta_side_v * DF_DELTA_SCALE,
                                 cm->seq_params.bit_depth)) {
          lf->filter_level_v = 0;
          cm->lf.delta_q_v = 0;
          cm->lf.delta_side_v = 0;
        }
      }
      // to switch off filters if offsets are zero
    }
#else  // not DUAL
    for (int plane = 0; plane < num_planes; ++plane) {
      last_frame_offsets[(plane << 1) + 1] = search_filter_offsets(
          sd, cpi, method == LPF_PICK_FROM_SUBIMAGE, last_frame_offsets,
          &best_cost[plane], plane, 1);
#if DF_TWO_PARAM
      last_frame_offsets[plane << 1] = search_filter_offsets(
          sd, cpi, method == LPF_PICK_FROM_SUBIMAGE, last_frame_offsets,
          &best_cost[plane], plane, 0);
#else
      last_frame_offsets[plane << 1] = last_frame_offsets[(plane << 1) + 1];
#endif  // DF_TWO_PARAM
      last_frame_offsets[(plane << 1) + 1] = search_filter_offsets(
          sd, cpi, method == LPF_PICK_FROM_SUBIMAGE, last_frame_offsets,
          &best_cost[plane], plane, 1);
#if DF_TWO_PARAM
      last_frame_offsets[plane << 1] = search_filter_offsets(
          sd, cpi, method == LPF_PICK_FROM_SUBIMAGE, last_frame_offsets,
          &best_cost[plane], plane, 0);
#else
      last_frame_offsets[plane << 1] = last_frame_offsets[(plane << 1) + 1];
#endif  // DF_TWO_PARAM
    }

    lf->delta_q_luma = last_frame_offsets[0];
    lf->delta_side_luma = last_frame_offsets[1];
    if (num_planes > 1) {
      lf->delta_q_u = last_frame_offsets[2];
      lf->delta_side_u = last_frame_offsets[3];
      lf->delta_q_v = last_frame_offsets[4];
      lf->delta_side_v = last_frame_offsets[5];
    }
#endif  // DF_DUAL
  }
}
#else
void av1_pick_filter_level(const YV12_BUFFER_CONFIG *sd, AV1_COMP *cpi,
                           LPF_PICK_METHOD method) {
  AV1_COMMON *const cm = &cpi->common;
  const int num_planes = av1_num_planes(cm);
  struct loopfilter *const lf = &cm->lf;
  (void)sd;

  lf->sharpness_level = 0;
  cpi->td.mb.rdmult = cpi->rd.RDMULT;

  if (method == LPF_PICK_MINIMAL_LPF) {
    lf->filter_level[0] = 0;
    lf->filter_level[1] = 0;
  } else if (method >= LPF_PICK_FROM_Q) {
    const int min_filter_level = 0;
    const int max_filter_level = av1_get_max_filter_level(cpi);

    const int q =
        ROUND_POWER_OF_TWO(av1_ac_quant_QTX(cm->quant_params.base_qindex, 0,
                                            cm->seq_params.bit_depth),
                           QUANT_TABLE_BITS);

    // based on tests result for rtc test set
    // 0.04590 boosted or 0.02295 non-booseted in 18-bit fixed point
    const int strength_boost_q_treshold = 700;
    const int inter_frame_multiplier =
        q > strength_boost_q_treshold ? 12034 : 6017;
    // These values were determined by linear fitting the result of the
    // searched level for 8 bit depth:
    // Keyframes: filt_guess = q * 0.06699 - 1.60817
    // Other frames: filt_guess = q * inter_frame_multiplier + 2.48225
    //
    // And high bit depth separately:
    // filt_guess = q * 0.316206 + 3.87252
    int filt_guess;
    switch (cm->seq_params.bit_depth) {
      case AOM_BITS_8:
        filt_guess =
            (cm->current_frame.frame_type == KEY_FRAME)
                ? ROUND_POWER_OF_TWO(q * 17563 - 421574, 18)
                : ROUND_POWER_OF_TWO(q * inter_frame_multiplier + 650707, 18);
        break;
      case AOM_BITS_10:
        filt_guess = ROUND_POWER_OF_TWO(q * 20723 + 4060632, 20);
        break;
      case AOM_BITS_12:
        filt_guess = ROUND_POWER_OF_TWO(q * 20723 + 16242526, 22);
        break;
      default:
        assert(0 &&
               "bit_depth should be AOM_BITS_8, AOM_BITS_10 "
               "or AOM_BITS_12");
        return;
    }
    if (cm->seq_params.bit_depth != AOM_BITS_8 &&
        cm->current_frame.frame_type == KEY_FRAME)
      filt_guess -= 4;
    // TODO(chengchen): retrain the model for Y, U, V filter levels
    lf->filter_level[0] = clamp(filt_guess, min_filter_level, max_filter_level);
    lf->filter_level[1] = clamp(filt_guess, min_filter_level, max_filter_level);
    lf->filter_level_u = clamp(filt_guess, min_filter_level, max_filter_level);
    lf->filter_level_v = clamp(filt_guess, min_filter_level, max_filter_level);
  } else {
    // TODO(anyone): What are good initial levels for keyframes?
    int last_frame_filter_level[4] = { 0 };
    if (!frame_is_intra_only(cm)) {
      last_frame_filter_level[0] = lf->filter_level[0];
      last_frame_filter_level[1] = lf->filter_level[1];
      last_frame_filter_level[2] = lf->filter_level_u;
      last_frame_filter_level[3] = lf->filter_level_v;
    }

    lf->filter_level[0] = lf->filter_level[1] =
        search_filter_level(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                            last_frame_filter_level, NULL, 0, 2);
    if (method != LPF_PICK_FROM_FULL_IMAGE_NON_DUAL) {
      lf->filter_level[0] =
          search_filter_level(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                              last_frame_filter_level, NULL, 0, 0);
      lf->filter_level[1] =
          search_filter_level(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                              last_frame_filter_level, NULL, 0, 1);
    }

    if (num_planes > 1) {
      lf->filter_level_u =
          search_filter_level(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                              last_frame_filter_level, NULL, 1, 0);
      lf->filter_level_v =
          search_filter_level(sd, cpi, method == LPF_PICK_FROM_SUBIMAGE,
                              last_frame_filter_level, NULL, 2, 0);
    }
  }
}
#endif  // !CONFIG_NEW_DF
