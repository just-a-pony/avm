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

#ifndef AOM_AV1_COMMON_AV1_TXFM_H_
#define AOM_AV1_COMMON_AV1_TXFM_H_

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "config/aom_config.h"

#include "av1/common/enums.h"
#include "av1/common/blockd.h"
#include "av1/common/secondary_tx.h"
#include "aom/aom_integer.h"
#include "aom_dsp/aom_dsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(DO_RANGE_CHECK_CLAMP)
#define DO_RANGE_CHECK_CLAMP 0
#endif

// Secondary transform coeffs in int precision
#if CONFIG_IST_ANY_SET
#if CONFIG_F105_IST_MEM_REDUCE
extern int32_t ist_4x4_kernel_int32[IST_SET_SIZE][STX_TYPES - 1][16]
                                   [IST_4x4_WIDTH];

extern int32_t ist_8x8_kernel_int32[IST_SET_SIZE][STX_TYPES - 1]
                                   [IST_8x8_HEIGHT_MAX][IST_8x8_WIDTH];
#else
extern int32_t ist_4x4_kernel_int32[IST_SET_SIZE][STX_TYPES - 1][IST_4x4_HEIGHT]
                                   [IST_4x4_WIDTH];

extern int32_t ist_8x8_kernel_int32[IST_SET_SIZE][STX_TYPES - 1]
                                   [IST_8x8_HEIGHT_MAX][IST_8x8_WIDTH];
#endif  // CONFIG_F105_IST_MEM_REDUCE
#else   // CONFIG_IST_ANY_SET
extern int32_t ist_4x4_kernel_int32[IST_SET_SIZE][STX_TYPES - 1][IST_4x4_HEIGHT]
                                   [IST_4x4_WIDTH];

extern int32_t ist_8x8_kernel_int32[IST_SET_SIZE][STX_TYPES - 1][IST_8x8_HEIGHT]
                                   [IST_8x8_WIDTH];
#endif  // CONFIG_IST_ANY_SET

#define REPLACE_ADST4 0
#define REPLACE_ADST8 1
#define REPLACE_ADST16 1

#define CCTX_PREC_BITS 8
extern const int32_t cctx_mtx[CCTX_TYPES - 1][2];

#define NewSqrt2Bits ((int32_t)12)
// 2^12 * sqrt(2)
static const int32_t NewSqrt2 = 5793;
// 2^12 / sqrt(2)
static const int32_t NewInvSqrt2 = 2896;

void av1_init_stxfm_kernels(void);

static INLINE int32_t clamp_value(int32_t value, int8_t bit) {
  if (bit <= 0) return value;  // Do nothing for invalid clamp bit.
  const int64_t max_value = (1LL << (bit - 1)) - 1;
  const int64_t min_value = -(1LL << (bit - 1));
  return (int32_t)clamp64(value, min_value, max_value);
}

static INLINE int32_t range_check_value(int32_t value, int8_t bit) {
#if CONFIG_COEFFICIENT_RANGE_CHECKING
  const int64_t max_value = (1LL << (bit - 1)) - 1;
  const int64_t min_value = -(1LL << (bit - 1));
  if (value < min_value || value > max_value) {
    fprintf(stderr, "coeff out of bit range, value: %d bit %d\n", value, bit);
#if !CONFIG_AV1_ENCODER
    assert(0);
#endif
  }
#endif  // CONFIG_COEFFICIENT_RANGE_CHECKING
#if DO_RANGE_CHECK_CLAMP
  bit = AOMMIN(bit, 31);
  return clamp(value, -(1 << (bit - 1)), (1 << (bit - 1)) - 1);
#endif  // DO_RANGE_CHECK_CLAMP
  (void)bit;
  return value;
}

static INLINE int32_t round_shift(int64_t value, int bit) {
  assert(bit >= 1);
  return (int32_t)((value + (1ll << (bit - 1))) >> bit);
}

static INLINE uint16_t highbd_clip_pixel_add(uint16_t dest, tran_high_t trans,
                                             int bd) {
  return clip_pixel_highbd(dest + (int)trans, bd);
}

typedef void (*TxfmFunc)(const int32_t *input, int32_t *output, int8_t cos_bit,
                         const int8_t *stage_range);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // AOM_AV1_COMMON_AV1_TXFM_H_
