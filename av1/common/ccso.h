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

#ifndef AOM_AV1_COMMON_CCSO_H_
#define AOM_AV1_COMMON_CCSO_H_

#define min_ccf(X, Y) (((X) < (Y)) ? (X) : (Y))
#define max_ccf(X, Y) (((X) > (Y)) ? (X) : (Y))

#define CCSO_INPUT_INTERVAL 3

#include <float.h>
#include "config/aom_config.h"
#include "aom/aom_integer.h"
#include "aom_ports/mem.h"
#include "av1/common/av1_common_int.h"

#if CONFIG_BRU
#define CCSO_REFACTORING 1
#else
#define CCSO_REFACTORING 0
#endif

#if CONFIG_CCSO_CLEANUP
static const uint16_t quant_sz[4][4] = { { 16, 8, 32, 0 },
                                         { 32, 16, 64, 128 },
                                         { 48, 24, 96, 192 },
                                         { 64, 32, 128, 256 } };
#endif  // CONFIG_CCSO_CLEANUP

static const int edge_clf_to_edge_interval[2] = { 3, 2 };

#ifdef __cplusplus
extern "C" {
#endif

void av1_copy_ccso_filters(CcsoInfo *to, CcsoInfo *from, int plane,
                           bool frame_level, bool block_level, int sb_count);

#if CONFIG_F054_PIC_BOUNDARY
void extend_ccso_border(const YV12_BUFFER_CONFIG *frame, uint16_t *buf,
                        const int d);
#else
void extend_ccso_border(uint16_t *buf, const int d, MACROBLOCKD *xd);
#endif  // CONFIG_F054_PIC_BOUNDARY

void cal_filter_support(int *rec_luma_idx, const uint16_t *rec_y,
                        const int quant_step_size, const int inv_quant_step,
                        const int *rec_idx, const int edge_clf);

// ext_filter_support must be less than 7.
void derive_ccso_sample_pos(int *rec_idx, const int ccso_stride,
                            const uint8_t ext_filter_support);

/* Apply CCSO on one color component */
typedef void (*CCSO_FILTER_FUNC)(AV1_COMMON *cm, MACROBLOCKD *xd,
                                 const int plane, const uint16_t *src_y,
                                 uint16_t *dst_yuv, const int dst_stride,
#if CCSO_REFACTORING
                                 const int proc_unit_log2,
#endif  // CCSO_REFACTORING
                                 const uint16_t thr, const uint8_t filter_sup,
                                 const uint8_t max_band_log2,
                                 const int edge_clf);

void ccso_frame(YV12_BUFFER_CONFIG *frame, AV1_COMMON *cm, MACROBLOCKD *xd,
                uint16_t *ext_rec_y);

typedef void (*ccso_filter_block_func)(
    const uint16_t *src_y, uint16_t *dst_yuv, const int x, const int y,
    const int pic_width, const int pic_height, int *src_cls,
    const int8_t *offset_buf, const int scaled_ext_stride, const int dst_stride,
    const int y_uv_hscale, const int y_uv_vscale, const int thr,
    const int neg_thr, const int *src_loc, const int max_val,
    const int blk_size);

#ifdef __cplusplus
}  // extern "C"
#endif
#endif  // AOM_AV1_COMMON_CCSO_H_
