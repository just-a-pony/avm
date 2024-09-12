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
#include <stdlib.h>
#include <string.h>

#include "config/aom_config.h"
#include "config/aom_dsp_rtcd.h"
#include "config/av1_rtcd.h"

#include "aom/aom_integer.h"
#include "aom_ports/mem.h"

#include "aom_dsp/aom_filter.h"
#include "aom_dsp/blend.h"
#include "aom_dsp/variance.h"

#include "av1/common/av1_common_int.h"
#include "av1/common/filter.h"
#include "av1/common/reconinter.h"
#include "av1/encoder/reconinter_enc.h"

uint32_t aom_get_mb_ss_c(const int16_t *a) {
  unsigned int i, sum = 0;

  for (i = 0; i < 256; ++i) {
    sum += a[i] * a[i];
  }

  return sum;
}

// Applies a 1-D 2-tap bilinear filter to the source block in either horizontal
// or vertical direction to produce the filtered output block. Used to implement
// the first-pass of 2-D separable filter.
//
// Produces int16_t output to retain precision for the next pass. Two filter
// taps should sum to FILTER_WEIGHT. pixel_step defines whether the filter is
// applied horizontally (pixel_step = 1) or vertically (pixel_step = stride).
// It defines the offset required to move from one input to the next.
void aom_var_filter_block2d_bil_first_pass_c(const uint8_t *a, uint16_t *b,
                                             unsigned int src_pixels_per_line,
                                             unsigned int pixel_step,
                                             unsigned int output_height,
                                             unsigned int output_width,
                                             const uint8_t *filter) {
  unsigned int i, j;

  for (i = 0; i < output_height; ++i) {
    for (j = 0; j < output_width; ++j) {
      b[j] = ROUND_POWER_OF_TWO(
          (int)a[0] * filter[0] + (int)a[pixel_step] * filter[1], FILTER_BITS);

      ++a;
    }

    a += src_pixels_per_line - output_width;
    b += output_width;
  }
}

// Applies a 1-D 2-tap bilinear filter to the source block in either horizontal
// or vertical direction to produce the filtered output block. Used to implement
// the second-pass of 2-D separable filter.
//
// Requires 16-bit input as produced by filter_block2d_bil_first_pass. Two
// filter taps should sum to FILTER_WEIGHT. pixel_step defines whether the
// filter is applied horizontally (pixel_step = 1) or vertically
// (pixel_step = stride). It defines the offset required to move from one input
// to the next. Output is 8-bit.
void aom_var_filter_block2d_bil_second_pass_c(const uint16_t *a, uint8_t *b,
                                              unsigned int src_pixels_per_line,
                                              unsigned int pixel_step,
                                              unsigned int output_height,
                                              unsigned int output_width,
                                              const uint8_t *filter) {
  unsigned int i, j;

  for (i = 0; i < output_height; ++i) {
    for (j = 0; j < output_width; ++j) {
      b[j] = ROUND_POWER_OF_TWO(
          (int)a[0] * filter[0] + (int)a[pixel_step] * filter[1], FILTER_BITS);
      ++a;
    }

    a += src_pixels_per_line - output_width;
    b += output_width;
  }
}

static void highbd_variance64(const uint16_t *a, int a_stride,
                              const uint16_t *b, int b_stride, int w, int h,
                              uint64_t *sse, int64_t *sum) {
  int64_t tsum = 0;
  uint64_t tsse = 0;
  for (int i = 0; i < h; ++i) {
    int32_t lsum = 0;
    for (int j = 0; j < w; ++j) {
      const int diff = a[j] - b[j];
      lsum += diff;
      tsse += (uint32_t)(diff * diff);
    }
    tsum += lsum;
    a += a_stride;
    b += b_stride;
  }
  *sum = tsum;
  *sse = tsse;
}

uint64_t aom_highbd_sse_odd_size(const uint16_t *a, int a_stride,
                                 const uint16_t *b, int b_stride, int w,
                                 int h) {
  uint64_t sse;
  int64_t sum;
  highbd_variance64(a, a_stride, b, b_stride, w, h, &sse, &sum);
  return sse;
}

static void highbd_8_variance(const uint16_t *a8, int a_stride,
                              const uint16_t *b8, int b_stride, int w, int h,
                              uint32_t *sse, int *sum) {
  uint64_t sse_long = 0;
  int64_t sum_long = 0;
  highbd_variance64(a8, a_stride, b8, b_stride, w, h, &sse_long, &sum_long);
  *sse = (uint32_t)sse_long;
  *sum = (int)sum_long;
}

static void highbd_10_variance(const uint16_t *a8, int a_stride,
                               const uint16_t *b8, int b_stride, int w, int h,
                               uint32_t *sse, int *sum) {
  uint64_t sse_long = 0;
  int64_t sum_long = 0;
  highbd_variance64(a8, a_stride, b8, b_stride, w, h, &sse_long, &sum_long);
  *sse = (uint32_t)ROUND_POWER_OF_TWO(sse_long, 4);
  *sum = (int)ROUND_POWER_OF_TWO(sum_long, 2);
}

static void highbd_12_variance(const uint16_t *a8, int a_stride,
                               const uint16_t *b8, int b_stride, int w, int h,
                               uint32_t *sse, int *sum) {
  uint64_t sse_long = 0;
  int64_t sum_long = 0;
  highbd_variance64(a8, a_stride, b8, b_stride, w, h, &sse_long, &sum_long);
  *sse = (uint32_t)ROUND_POWER_OF_TWO(sse_long, 8);
  *sum = (int)ROUND_POWER_OF_TWO(sum_long, 4);
}

#define HIGHBD_VAR(W, H)                                                       \
  uint32_t aom_highbd_8_variance##W##x##H##_c(const uint16_t *a, int a_stride, \
                                              const uint16_t *b, int b_stride, \
                                              uint32_t *sse) {                 \
    int sum;                                                                   \
    highbd_8_variance(a, a_stride, b, b_stride, W, H, sse, &sum);              \
    return *sse - (uint32_t)(((int64_t)sum * sum) / (W * H));                  \
  }                                                                            \
                                                                               \
  uint32_t aom_highbd_10_variance##W##x##H##_c(                                \
      const uint16_t *a, int a_stride, const uint16_t *b, int b_stride,        \
      uint32_t *sse) {                                                         \
    int sum;                                                                   \
    int64_t var;                                                               \
    highbd_10_variance(a, a_stride, b, b_stride, W, H, sse, &sum);             \
    var = (int64_t)(*sse) - (((int64_t)sum * sum) / (W * H));                  \
    return (var >= 0) ? (uint32_t)var : 0;                                     \
  }                                                                            \
                                                                               \
  uint32_t aom_highbd_12_variance##W##x##H##_c(                                \
      const uint16_t *a, int a_stride, const uint16_t *b, int b_stride,        \
      uint32_t *sse) {                                                         \
    int sum;                                                                   \
    int64_t var;                                                               \
    highbd_12_variance(a, a_stride, b, b_stride, W, H, sse, &sum);             \
    var = (int64_t)(*sse) - (((int64_t)sum * sum) / (W * H));                  \
    return (var >= 0) ? (uint32_t)var : 0;                                     \
  }

#define HIGHBD_GET_VAR(S)                                                     \
  void aom_highbd_8_get##S##x##S##var_c(const uint16_t *src, int src_stride,  \
                                        const uint16_t *ref, int ref_stride,  \
                                        uint32_t *sse, int *sum) {            \
    highbd_8_variance(src, src_stride, ref, ref_stride, S, S, sse, sum);      \
  }                                                                           \
                                                                              \
  void aom_highbd_10_get##S##x##S##var_c(const uint16_t *src, int src_stride, \
                                         const uint16_t *ref, int ref_stride, \
                                         uint32_t *sse, int *sum) {           \
    highbd_10_variance(src, src_stride, ref, ref_stride, S, S, sse, sum);     \
  }                                                                           \
                                                                              \
  void aom_highbd_12_get##S##x##S##var_c(const uint16_t *src, int src_stride, \
                                         const uint16_t *ref, int ref_stride, \
                                         uint32_t *sse, int *sum) {           \
    highbd_12_variance(src, src_stride, ref, ref_stride, S, S, sse, sum);     \
  }

#define HIGHBD_MSE(W, H)                                                       \
  uint32_t aom_highbd_8_mse##W##x##H##_c(const uint16_t *src, int src_stride,  \
                                         const uint16_t *ref, int ref_stride,  \
                                         uint32_t *sse) {                      \
    int sum;                                                                   \
    highbd_8_variance(src, src_stride, ref, ref_stride, W, H, sse, &sum);      \
    return *sse;                                                               \
  }                                                                            \
                                                                               \
  uint32_t aom_highbd_10_mse##W##x##H##_c(const uint16_t *src, int src_stride, \
                                          const uint16_t *ref, int ref_stride, \
                                          uint32_t *sse) {                     \
    int sum;                                                                   \
    highbd_10_variance(src, src_stride, ref, ref_stride, W, H, sse, &sum);     \
    return *sse;                                                               \
  }                                                                            \
                                                                               \
  uint32_t aom_highbd_12_mse##W##x##H##_c(const uint16_t *src, int src_stride, \
                                          const uint16_t *ref, int ref_stride, \
                                          uint32_t *sse) {                     \
    int sum;                                                                   \
    highbd_12_variance(src, src_stride, ref, ref_stride, W, H, sse, &sum);     \
    return *sse;                                                               \
  }

void aom_highbd_var_filter_block2d_bil_first_pass(
    const uint16_t *src_ptr, uint16_t *output_ptr,
    unsigned int src_pixels_per_line, int pixel_step,
    unsigned int output_height, unsigned int output_width,
    const uint8_t *filter) {
  unsigned int i, j;
  for (i = 0; i < output_height; ++i) {
    for (j = 0; j < output_width; ++j) {
      output_ptr[j] = ROUND_POWER_OF_TWO(
          (int)src_ptr[0] * filter[0] + (int)src_ptr[pixel_step] * filter[1],
          FILTER_BITS);

      ++src_ptr;
    }

    // Next row...
    src_ptr += src_pixels_per_line - output_width;
    output_ptr += output_width;
  }
}

void aom_highbd_var_filter_block2d_bil_second_pass(
    const uint16_t *src_ptr, uint16_t *output_ptr,
    unsigned int src_pixels_per_line, unsigned int pixel_step,
    unsigned int output_height, unsigned int output_width,
    const uint8_t *filter) {
  unsigned int i, j;

  for (i = 0; i < output_height; ++i) {
    for (j = 0; j < output_width; ++j) {
      output_ptr[j] = ROUND_POWER_OF_TWO(
          (int)src_ptr[0] * filter[0] + (int)src_ptr[pixel_step] * filter[1],
          FILTER_BITS);
      ++src_ptr;
    }

    src_ptr += src_pixels_per_line - output_width;
    output_ptr += output_width;
  }
}

#define HIGHBD_SUBPIX_VAR(W, H)                                              \
  uint32_t aom_highbd_8_sub_pixel_variance##W##x##H##_c(                     \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,         \
      const uint16_t *dst, int dst_stride, uint32_t *sse) {                  \
    uint16_t fdata3[(H + 1) * W];                                            \
    uint16_t temp2[H * W];                                                   \
                                                                             \
    aom_highbd_var_filter_block2d_bil_first_pass(                            \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]); \
    aom_highbd_var_filter_block2d_bil_second_pass(                           \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);            \
                                                                             \
    return aom_highbd_8_variance##W##x##H##_c((temp2), W, dst, dst_stride,   \
                                              sse);                          \
  }                                                                          \
                                                                             \
  uint32_t aom_highbd_10_sub_pixel_variance##W##x##H##_c(                    \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,         \
      const uint16_t *dst, int dst_stride, uint32_t *sse) {                  \
    uint16_t fdata3[(H + 1) * W];                                            \
    uint16_t temp2[H * W];                                                   \
                                                                             \
    aom_highbd_var_filter_block2d_bil_first_pass(                            \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]); \
    aom_highbd_var_filter_block2d_bil_second_pass(                           \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);            \
                                                                             \
    return aom_highbd_10_variance##W##x##H##_c((temp2), W, dst, dst_stride,  \
                                               sse);                         \
  }                                                                          \
                                                                             \
  uint32_t aom_highbd_12_sub_pixel_variance##W##x##H##_c(                    \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,         \
      const uint16_t *dst, int dst_stride, uint32_t *sse) {                  \
    uint16_t fdata3[(H + 1) * W];                                            \
    uint16_t temp2[H * W];                                                   \
                                                                             \
    aom_highbd_var_filter_block2d_bil_first_pass(                            \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]); \
    aom_highbd_var_filter_block2d_bil_second_pass(                           \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);            \
                                                                             \
    return aom_highbd_12_variance##W##x##H##_c((temp2), W, dst, dst_stride,  \
                                               sse);                         \
  }

#define HIGHBD_SUBPIX_AVG_VAR(W, H)                                           \
  uint32_t aom_highbd_8_sub_pixel_avg_variance##W##x##H##_c(                  \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,          \
      const uint16_t *dst, int dst_stride, uint32_t *sse,                     \
      const uint16_t *second_pred) {                                          \
    uint16_t fdata3[(H + 1) * W];                                             \
    uint16_t temp2[H * W];                                                    \
    DECLARE_ALIGNED(16, uint16_t, temp3[H * W]);                              \
                                                                              \
    aom_highbd_var_filter_block2d_bil_first_pass(                             \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]);  \
    aom_highbd_var_filter_block2d_bil_second_pass(                            \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);             \
                                                                              \
    aom_highbd_comp_avg_pred_c((temp3), second_pred, W, H, (temp2), W);       \
                                                                              \
    return aom_highbd_8_variance##W##x##H##_c((temp3), W, dst, dst_stride,    \
                                              sse);                           \
  }                                                                           \
                                                                              \
  uint32_t aom_highbd_10_sub_pixel_avg_variance##W##x##H##_c(                 \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,          \
      const uint16_t *dst, int dst_stride, uint32_t *sse,                     \
      const uint16_t *second_pred) {                                          \
    uint16_t fdata3[(H + 1) * W];                                             \
    uint16_t temp2[H * W];                                                    \
    DECLARE_ALIGNED(16, uint16_t, temp3[H * W]);                              \
                                                                              \
    aom_highbd_var_filter_block2d_bil_first_pass(                             \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]);  \
    aom_highbd_var_filter_block2d_bil_second_pass(                            \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);             \
                                                                              \
    aom_highbd_comp_avg_pred_c((temp3), second_pred, W, H, (temp2), W);       \
                                                                              \
    return aom_highbd_10_variance##W##x##H##_c((temp3), W, dst, dst_stride,   \
                                               sse);                          \
  }                                                                           \
                                                                              \
  uint32_t aom_highbd_12_sub_pixel_avg_variance##W##x##H##_c(                 \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,          \
      const uint16_t *dst, int dst_stride, uint32_t *sse,                     \
      const uint16_t *second_pred) {                                          \
    uint16_t fdata3[(H + 1) * W];                                             \
    uint16_t temp2[H * W];                                                    \
    DECLARE_ALIGNED(16, uint16_t, temp3[H * W]);                              \
                                                                              \
    aom_highbd_var_filter_block2d_bil_first_pass(                             \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]);  \
    aom_highbd_var_filter_block2d_bil_second_pass(                            \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);             \
                                                                              \
    aom_highbd_comp_avg_pred_c((temp3), second_pred, W, H, (temp2), W);       \
                                                                              \
    return aom_highbd_12_variance##W##x##H##_c((temp3), W, dst, dst_stride,   \
                                               sse);                          \
  }                                                                           \
                                                                              \
  uint32_t aom_highbd_8_dist_wtd_sub_pixel_avg_variance##W##x##H##_c(         \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,          \
      const uint16_t *dst, int dst_stride, uint32_t *sse,                     \
      const uint16_t *second_pred, const DIST_WTD_COMP_PARAMS *jcp_param) {   \
    uint16_t fdata3[(H + 1) * W];                                             \
    uint16_t temp2[H * W];                                                    \
    DECLARE_ALIGNED(16, uint16_t, temp3[H * W]);                              \
                                                                              \
    aom_highbd_var_filter_block2d_bil_first_pass(                             \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]);  \
    aom_highbd_var_filter_block2d_bil_second_pass(                            \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);             \
                                                                              \
    aom_highbd_dist_wtd_comp_avg_pred((temp3), second_pred, W, H, (temp2), W, \
                                      jcp_param);                             \
                                                                              \
    return aom_highbd_8_variance##W##x##H((temp3), W, dst, dst_stride, sse);  \
  }                                                                           \
                                                                              \
  uint32_t aom_highbd_10_dist_wtd_sub_pixel_avg_variance##W##x##H##_c(        \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,          \
      const uint16_t *dst, int dst_stride, uint32_t *sse,                     \
      const uint16_t *second_pred, const DIST_WTD_COMP_PARAMS *jcp_param) {   \
    uint16_t fdata3[(H + 1) * W];                                             \
    uint16_t temp2[H * W];                                                    \
    DECLARE_ALIGNED(16, uint16_t, temp3[H * W]);                              \
                                                                              \
    aom_highbd_var_filter_block2d_bil_first_pass(                             \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]);  \
    aom_highbd_var_filter_block2d_bil_second_pass(                            \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);             \
                                                                              \
    aom_highbd_dist_wtd_comp_avg_pred((temp3), second_pred, W, H, (temp2), W, \
                                      jcp_param);                             \
                                                                              \
    return aom_highbd_10_variance##W##x##H((temp3), W, dst, dst_stride, sse); \
  }                                                                           \
                                                                              \
  uint32_t aom_highbd_12_dist_wtd_sub_pixel_avg_variance##W##x##H##_c(        \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,          \
      const uint16_t *dst, int dst_stride, uint32_t *sse,                     \
      const uint16_t *second_pred, const DIST_WTD_COMP_PARAMS *jcp_param) {   \
    uint16_t fdata3[(H + 1) * W];                                             \
    uint16_t temp2[H * W];                                                    \
    DECLARE_ALIGNED(16, uint16_t, temp3[H * W]);                              \
                                                                              \
    aom_highbd_var_filter_block2d_bil_first_pass(                             \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]);  \
    aom_highbd_var_filter_block2d_bil_second_pass(                            \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);             \
                                                                              \
    aom_highbd_dist_wtd_comp_avg_pred((temp3), second_pred, W, H, (temp2), W, \
                                      jcp_param);                             \
                                                                              \
    return aom_highbd_12_variance##W##x##H((temp3), W, dst, dst_stride, sse); \
  }

/* All three forms of the variance are available in the same sizes. */
#define HIGHBD_VARIANCES(W, H) \
  HIGHBD_VAR(W, H)             \
  HIGHBD_SUBPIX_VAR(W, H)      \
  HIGHBD_SUBPIX_AVG_VAR(W, H)

#if CONFIG_EXT_RECUR_PARTITIONS
HIGHBD_VARIANCES(256, 256)
HIGHBD_VARIANCES(256, 128)
HIGHBD_VARIANCES(128, 256)
#endif  // CONFIG_EXT_RECUR_PARTITIONS
HIGHBD_VARIANCES(128, 128)
HIGHBD_VARIANCES(128, 64)
HIGHBD_VARIANCES(64, 128)
HIGHBD_VARIANCES(64, 64)
HIGHBD_VARIANCES(64, 32)
HIGHBD_VARIANCES(32, 64)
HIGHBD_VARIANCES(32, 32)
HIGHBD_VARIANCES(32, 16)
HIGHBD_VARIANCES(16, 32)
HIGHBD_VARIANCES(16, 16)
HIGHBD_VARIANCES(16, 8)
HIGHBD_VARIANCES(8, 16)
HIGHBD_VARIANCES(8, 8)
HIGHBD_VARIANCES(8, 4)
HIGHBD_VARIANCES(4, 8)
HIGHBD_VARIANCES(4, 4)
HIGHBD_VARIANCES(4, 2)
HIGHBD_VARIANCES(2, 4)
HIGHBD_VARIANCES(2, 2)
HIGHBD_VARIANCES(4, 16)
HIGHBD_VARIANCES(16, 4)
HIGHBD_VARIANCES(8, 32)
HIGHBD_VARIANCES(32, 8)
HIGHBD_VARIANCES(16, 64)
HIGHBD_VARIANCES(64, 16)
#if CONFIG_EXT_RECUR_PARTITIONS
HIGHBD_VARIANCES(4, 32)
HIGHBD_VARIANCES(32, 4)
HIGHBD_VARIANCES(8, 64)
HIGHBD_VARIANCES(64, 8)
HIGHBD_VARIANCES(4, 64)
HIGHBD_VARIANCES(64, 4)
#endif  // CONFIG_EXT_RECUR_PARTITIONS

HIGHBD_GET_VAR(8)
HIGHBD_GET_VAR(16)

HIGHBD_MSE(16, 16)
HIGHBD_MSE(16, 8)
HIGHBD_MSE(8, 16)
HIGHBD_MSE(8, 8)

void aom_highbd_comp_avg_pred_c(uint16_t *comp_pred, const uint16_t *pred,
                                int width, int height, const uint16_t *ref,
                                int ref_stride) {
  int i, j;
  for (i = 0; i < height; ++i) {
    for (j = 0; j < width; ++j) {
      const int tmp = pred[j] + ref[j];
      comp_pred[j] = ROUND_POWER_OF_TWO(tmp, 1);
    }
    comp_pred += width;
    pred += width;
    ref += ref_stride;
  }
}

void aom_highbd_upsampled_pred_c(MACROBLOCKD *xd,
                                 const struct AV1Common *const cm, int mi_row,
                                 int mi_col, const MV *const mv,
                                 uint16_t *comp_pred, int width, int height,
                                 int subpel_x_q3, int subpel_y_q3,
                                 const uint16_t *ref, int ref_stride, int bd,
                                 int subpel_search, int is_scaled_ref) {
  // expect xd == NULL only in tests
  if (xd != NULL) {
    const MB_MODE_INFO *mi = xd->mi[0];
    const int ref_num = 0;
    const int is_intrabc = is_intrabc_block(mi, xd->tree_type);
    const struct scale_factors *const sf =
        (is_intrabc || is_scaled_ref) ? &cm->sf_identity
                                      : xd->block_ref_scale_factors[ref_num];

    const int is_scaled = av1_is_scaled(sf);

    if (is_scaled) {
      int plane = 0;
      const int mi_x = mi_col * MI_SIZE;
      const int mi_y = mi_row * MI_SIZE;
      const struct macroblockd_plane *const pd = &xd->plane[plane];
      const struct buf_2d *const dst_buf = &pd->dst;
      const struct buf_2d *const pre_buf =
          is_intrabc ? dst_buf : &pd->pre[ref_num];

      InterPredParams inter_pred_params;
      inter_pred_params.conv_params = get_conv_params(0, plane, xd->bd);
      const InterpFilter filters = EIGHTTAP_REGULAR;
      av1_init_inter_params(
          &inter_pred_params, width, height, mi_y >> pd->subsampling_y,
          mi_x >> pd->subsampling_x, pd->subsampling_x, pd->subsampling_y,
          xd->bd, is_intrabc, sf, pre_buf, filters);
      av1_enc_build_one_inter_predictor(comp_pred, width, mv,
                                        &inter_pred_params);
      return;
    }
  }

  const InterpFilterParams *filter = av1_get_filter(subpel_search);

  if (!subpel_x_q3 && !subpel_y_q3) {
    for (int i = 0; i < height; i++) {
      memcpy(comp_pred, ref, width * sizeof(*comp_pred));
      comp_pred += width;
      ref += ref_stride;
    }
  } else if (!subpel_y_q3) {
    const int16_t *const kernel =
        av1_get_interp_filter_subpel_kernel(filter, subpel_x_q3 << 1);
    aom_highbd_convolve8_horiz_c(ref, ref_stride, comp_pred, width, kernel, 16,
                                 NULL, -1, width, height, bd);
  } else if (!subpel_x_q3) {
    const int16_t *const kernel =
        av1_get_interp_filter_subpel_kernel(filter, subpel_y_q3 << 1);
    aom_highbd_convolve8_vert_c(ref, ref_stride, comp_pred, width, NULL, -1,
                                kernel, 16, width, height, bd);
  } else {
    DECLARE_ALIGNED(16, uint16_t,
                    temp[((MAX_SB_SIZE + 16) + 16) * MAX_SB_SIZE]);
    const int16_t *const kernel_x =
        av1_get_interp_filter_subpel_kernel(filter, subpel_x_q3 << 1);
    const int16_t *const kernel_y =
        av1_get_interp_filter_subpel_kernel(filter, subpel_y_q3 << 1);
    const int intermediate_height =
        (((height - 1) * 8 + subpel_y_q3) >> 3) + filter->taps;
    assert(intermediate_height <= (MAX_SB_SIZE * 2 + 16) + 16);
    aom_highbd_convolve8_horiz_c(ref - ref_stride * ((filter->taps >> 1) - 1),
                                 ref_stride, (temp), MAX_SB_SIZE, kernel_x, 16,
                                 NULL, -1, width, intermediate_height, bd);
    aom_highbd_convolve8_vert_c(
        (temp + MAX_SB_SIZE * ((filter->taps >> 1) - 1)), MAX_SB_SIZE,
        comp_pred, width, NULL, -1, kernel_y, 16, width, height, bd);
  }
}

void aom_highbd_comp_avg_upsampled_pred_c(
    MACROBLOCKD *xd, const struct AV1Common *const cm, int mi_row, int mi_col,
    const MV *const mv, uint16_t *comp_pred, const uint16_t *pred, int width,
    int height, int subpel_x_q3, int subpel_y_q3, const uint16_t *ref,
    int ref_stride, int bd, int subpel_search, int is_scaled_ref) {
  int i, j;

  aom_highbd_upsampled_pred(xd, cm, mi_row, mi_col, mv, comp_pred, width,
                            height, subpel_x_q3, subpel_y_q3, ref, ref_stride,
                            bd, subpel_search, is_scaled_ref);
  for (i = 0; i < height; ++i) {
    for (j = 0; j < width; ++j) {
      comp_pred[j] = ROUND_POWER_OF_TWO(pred[j] + comp_pred[j], 1);
    }
    comp_pred += width;
    pred += width;
  }
}

void aom_highbd_dist_wtd_comp_avg_pred_c(
    uint16_t *comp_pred, const uint16_t *pred, int width, int height,
    const uint16_t *ref, int ref_stride,
    const DIST_WTD_COMP_PARAMS *jcp_param) {
  int i, j;
  const int fwd_offset = jcp_param->fwd_offset;
  const int bck_offset = jcp_param->bck_offset;

  for (i = 0; i < height; ++i) {
    for (j = 0; j < width; ++j) {
      int tmp = pred[j] * bck_offset + ref[j] * fwd_offset;
      tmp = ROUND_POWER_OF_TWO(tmp, DIST_PRECISION_BITS);
      comp_pred[j] = (uint16_t)tmp;
    }
    comp_pred += width;
    pred += width;
    ref += ref_stride;
  }
}

void aom_highbd_dist_wtd_comp_avg_upsampled_pred_c(
    MACROBLOCKD *xd, const struct AV1Common *const cm, int mi_row, int mi_col,
    const MV *const mv, uint16_t *comp_pred, const uint16_t *pred, int width,
    int height, int subpel_x_q3, int subpel_y_q3, const uint16_t *ref,
    int ref_stride, int bd, const DIST_WTD_COMP_PARAMS *jcp_param,
    int subpel_search, int is_scaled_ref) {
  int i, j;
  const int fwd_offset = jcp_param->fwd_offset;
  const int bck_offset = jcp_param->bck_offset;
  aom_highbd_upsampled_pred_c(xd, cm, mi_row, mi_col, mv, comp_pred, width,
                              height, subpel_x_q3, subpel_y_q3, ref, ref_stride,
                              bd, subpel_search, is_scaled_ref);

  for (i = 0; i < height; i++) {
    for (j = 0; j < width; j++) {
      int tmp = pred[j] * bck_offset + comp_pred[j] * fwd_offset;
      tmp = ROUND_POWER_OF_TWO(tmp, DIST_PRECISION_BITS);
      comp_pred[j] = (uint16_t)tmp;
    }
    comp_pred += width;
    pred += width;
  }
}

void aom_highbd_comp_mask_pred_c(uint16_t *comp_pred, const uint16_t *pred,
                                 int width, int height, const uint16_t *ref,
                                 int ref_stride, const uint8_t *mask,
                                 int mask_stride, int invert_mask) {
  int i, j;
  for (i = 0; i < height; ++i) {
    for (j = 0; j < width; ++j) {
      if (!invert_mask)
        comp_pred[j] = AOM_BLEND_A64(mask[j], ref[j], pred[j]);
      else
        comp_pred[j] = AOM_BLEND_A64(mask[j], pred[j], ref[j]);
    }
    comp_pred += width;
    pred += width;
    ref += ref_stride;
    mask += mask_stride;
  }
}

void aom_highbd_comp_mask_upsampled_pred(
    MACROBLOCKD *xd, const struct AV1Common *const cm, int mi_row, int mi_col,
    const MV *const mv, uint16_t *comp_pred, const uint16_t *pred, int width,
    int height, int subpel_x_q3, int subpel_y_q3, const uint16_t *ref,
    int ref_stride, const uint8_t *mask, int mask_stride, int invert_mask,
    int bd, int subpel_search, int is_scaled_ref) {
  aom_highbd_upsampled_pred(xd, cm, mi_row, mi_col, mv, comp_pred, width,
                            height, subpel_x_q3, subpel_y_q3, ref, ref_stride,
                            bd, subpel_search, is_scaled_ref);
  aom_highbd_comp_mask_pred(comp_pred, pred, width, height, comp_pred, width,
                            mask, mask_stride, invert_mask);
}

#define HIGHBD_MASK_SUBPIX_VAR(W, H)                                         \
  unsigned int aom_highbd_8_masked_sub_pixel_variance##W##x##H##_c(          \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,         \
      const uint16_t *ref, int ref_stride, const uint16_t *second_pred,      \
      const uint8_t *msk, int msk_stride, int invert_mask,                   \
      unsigned int *sse) {                                                   \
    uint16_t fdata3[(H + 1) * W];                                            \
    uint16_t temp2[H * W];                                                   \
    DECLARE_ALIGNED(16, uint16_t, temp3[H * W]);                             \
                                                                             \
    aom_highbd_var_filter_block2d_bil_first_pass(                            \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]); \
    aom_highbd_var_filter_block2d_bil_second_pass(                           \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);            \
                                                                             \
    aom_highbd_comp_mask_pred_c((temp3), second_pred, W, H, (temp2), W, msk, \
                                msk_stride, invert_mask);                    \
                                                                             \
    return aom_highbd_8_variance##W##x##H##_c((temp3), W, ref, ref_stride,   \
                                              sse);                          \
  }                                                                          \
                                                                             \
  unsigned int aom_highbd_10_masked_sub_pixel_variance##W##x##H##_c(         \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,         \
      const uint16_t *ref, int ref_stride, const uint16_t *second_pred,      \
      const uint8_t *msk, int msk_stride, int invert_mask,                   \
      unsigned int *sse) {                                                   \
    uint16_t fdata3[(H + 1) * W];                                            \
    uint16_t temp2[H * W];                                                   \
    DECLARE_ALIGNED(16, uint16_t, temp3[H * W]);                             \
                                                                             \
    aom_highbd_var_filter_block2d_bil_first_pass(                            \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]); \
    aom_highbd_var_filter_block2d_bil_second_pass(                           \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);            \
                                                                             \
    aom_highbd_comp_mask_pred_c((temp3), second_pred, W, H, (temp2), W, msk, \
                                msk_stride, invert_mask);                    \
                                                                             \
    return aom_highbd_10_variance##W##x##H##_c((temp3), W, ref, ref_stride,  \
                                               sse);                         \
  }                                                                          \
                                                                             \
  unsigned int aom_highbd_12_masked_sub_pixel_variance##W##x##H##_c(         \
      const uint16_t *src, int src_stride, int xoffset, int yoffset,         \
      const uint16_t *ref, int ref_stride, const uint16_t *second_pred,      \
      const uint8_t *msk, int msk_stride, int invert_mask,                   \
      unsigned int *sse) {                                                   \
    uint16_t fdata3[(H + 1) * W];                                            \
    uint16_t temp2[H * W];                                                   \
    DECLARE_ALIGNED(16, uint16_t, temp3[H * W]);                             \
                                                                             \
    aom_highbd_var_filter_block2d_bil_first_pass(                            \
        src, fdata3, src_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]); \
    aom_highbd_var_filter_block2d_bil_second_pass(                           \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);            \
                                                                             \
    aom_highbd_comp_mask_pred_c((temp3), second_pred, W, H, (temp2), W, msk, \
                                msk_stride, invert_mask);                    \
                                                                             \
    return aom_highbd_12_variance##W##x##H##_c((temp3), W, ref, ref_stride,  \
                                               sse);                         \
  }

HIGHBD_MASK_SUBPIX_VAR(4, 4)
HIGHBD_MASK_SUBPIX_VAR(4, 8)
HIGHBD_MASK_SUBPIX_VAR(8, 4)
HIGHBD_MASK_SUBPIX_VAR(8, 8)
HIGHBD_MASK_SUBPIX_VAR(8, 16)
HIGHBD_MASK_SUBPIX_VAR(16, 8)
HIGHBD_MASK_SUBPIX_VAR(16, 16)
HIGHBD_MASK_SUBPIX_VAR(16, 32)
HIGHBD_MASK_SUBPIX_VAR(32, 16)
HIGHBD_MASK_SUBPIX_VAR(32, 32)
HIGHBD_MASK_SUBPIX_VAR(32, 64)
HIGHBD_MASK_SUBPIX_VAR(64, 32)
HIGHBD_MASK_SUBPIX_VAR(64, 64)
HIGHBD_MASK_SUBPIX_VAR(64, 128)
HIGHBD_MASK_SUBPIX_VAR(128, 64)
HIGHBD_MASK_SUBPIX_VAR(128, 128)
#if CONFIG_EXT_RECUR_PARTITIONS
HIGHBD_MASK_SUBPIX_VAR(128, 256)
HIGHBD_MASK_SUBPIX_VAR(256, 128)
HIGHBD_MASK_SUBPIX_VAR(256, 256)
#endif  // CONFIG_EXT_RECUR_PARTITIONS
HIGHBD_MASK_SUBPIX_VAR(4, 16)
HIGHBD_MASK_SUBPIX_VAR(16, 4)
HIGHBD_MASK_SUBPIX_VAR(8, 32)
HIGHBD_MASK_SUBPIX_VAR(32, 8)
HIGHBD_MASK_SUBPIX_VAR(16, 64)
HIGHBD_MASK_SUBPIX_VAR(64, 16)
#if CONFIG_EXT_RECUR_PARTITIONS
HIGHBD_MASK_SUBPIX_VAR(4, 32)
HIGHBD_MASK_SUBPIX_VAR(32, 4)
HIGHBD_MASK_SUBPIX_VAR(8, 64)
HIGHBD_MASK_SUBPIX_VAR(64, 8)
HIGHBD_MASK_SUBPIX_VAR(4, 64)
HIGHBD_MASK_SUBPIX_VAR(64, 4)
#endif  // CONFIG_EXT_RECUR_PARTITIONS

static INLINE void highbd_obmc_variance64(const uint16_t *pre, int pre_stride,
                                          const int32_t *wsrc,
                                          const int32_t *mask, int w, int h,
                                          uint64_t *sse, int64_t *sum) {
  int i, j;

  *sse = 0;
  *sum = 0;

  for (i = 0; i < h; i++) {
    for (j = 0; j < w; j++) {
      int diff = ROUND_POWER_OF_TWO_SIGNED(wsrc[j] - pre[j] * mask[j], 12);
      *sum += diff;
      *sse += diff * diff;
    }

    pre += pre_stride;
    wsrc += w;
    mask += w;
  }
}

static INLINE void highbd_obmc_variance(const uint16_t *pre, int pre_stride,
                                        const int32_t *wsrc,
                                        const int32_t *mask, int w, int h,
                                        unsigned int *sse, int *sum) {
  int64_t sum64;
  uint64_t sse64;
  highbd_obmc_variance64(pre, pre_stride, wsrc, mask, w, h, &sse64, &sum64);
  *sum = (int)sum64;
  *sse = (unsigned int)sse64;
}

static INLINE void highbd_10_obmc_variance(const uint16_t *pre, int pre_stride,
                                           const int32_t *wsrc,
                                           const int32_t *mask, int w, int h,
                                           unsigned int *sse, int *sum) {
  int64_t sum64;
  uint64_t sse64;
  highbd_obmc_variance64(pre, pre_stride, wsrc, mask, w, h, &sse64, &sum64);
  *sum = (int)ROUND_POWER_OF_TWO(sum64, 2);
  *sse = (unsigned int)ROUND_POWER_OF_TWO(sse64, 4);
}

static INLINE void highbd_12_obmc_variance(const uint16_t *pre, int pre_stride,
                                           const int32_t *wsrc,
                                           const int32_t *mask, int w, int h,
                                           unsigned int *sse, int *sum) {
  int64_t sum64;
  uint64_t sse64;
  highbd_obmc_variance64(pre, pre_stride, wsrc, mask, w, h, &sse64, &sum64);
  *sum = (int)ROUND_POWER_OF_TWO(sum64, 4);
  *sse = (unsigned int)ROUND_POWER_OF_TWO(sse64, 8);
}

#define HIGHBD_OBMC_VAR(W, H)                                              \
  unsigned int aom_highbd_obmc_variance##W##x##H##_c(                      \
      const uint16_t *pre, int pre_stride, const int32_t *wsrc,            \
      const int32_t *mask, unsigned int *sse) {                            \
    int sum;                                                               \
    highbd_obmc_variance(pre, pre_stride, wsrc, mask, W, H, sse, &sum);    \
    return *sse - (unsigned int)(((int64_t)sum * sum) / (W * H));          \
  }                                                                        \
                                                                           \
  unsigned int aom_highbd_10_obmc_variance##W##x##H##_c(                   \
      const uint16_t *pre, int pre_stride, const int32_t *wsrc,            \
      const int32_t *mask, unsigned int *sse) {                            \
    int sum;                                                               \
    int64_t var;                                                           \
    highbd_10_obmc_variance(pre, pre_stride, wsrc, mask, W, H, sse, &sum); \
    var = (int64_t)(*sse) - (((int64_t)sum * sum) / (W * H));              \
    return (var >= 0) ? (uint32_t)var : 0;                                 \
  }                                                                        \
                                                                           \
  unsigned int aom_highbd_12_obmc_variance##W##x##H##_c(                   \
      const uint16_t *pre, int pre_stride, const int32_t *wsrc,            \
      const int32_t *mask, unsigned int *sse) {                            \
    int sum;                                                               \
    int64_t var;                                                           \
    highbd_12_obmc_variance(pre, pre_stride, wsrc, mask, W, H, sse, &sum); \
    var = (int64_t)(*sse) - (((int64_t)sum * sum) / (W * H));              \
    return (var >= 0) ? (uint32_t)var : 0;                                 \
  }

#define HIGHBD_OBMC_SUBPIX_VAR(W, H)                                           \
  unsigned int aom_highbd_obmc_sub_pixel_variance##W##x##H##_c(                \
      const uint16_t *pre, int pre_stride, int xoffset, int yoffset,           \
      const int32_t *wsrc, const int32_t *mask, unsigned int *sse) {           \
    uint16_t fdata3[(H + 1) * W];                                              \
    uint16_t temp2[H * W];                                                     \
                                                                               \
    aom_highbd_var_filter_block2d_bil_first_pass(                              \
        pre, fdata3, pre_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]);   \
    aom_highbd_var_filter_block2d_bil_second_pass(                             \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);              \
                                                                               \
    return aom_highbd_obmc_variance##W##x##H##_c((temp2), W, wsrc, mask, sse); \
  }                                                                            \
                                                                               \
  unsigned int aom_highbd_10_obmc_sub_pixel_variance##W##x##H##_c(             \
      const uint16_t *pre, int pre_stride, int xoffset, int yoffset,           \
      const int32_t *wsrc, const int32_t *mask, unsigned int *sse) {           \
    uint16_t fdata3[(H + 1) * W];                                              \
    uint16_t temp2[H * W];                                                     \
                                                                               \
    aom_highbd_var_filter_block2d_bil_first_pass(                              \
        pre, fdata3, pre_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]);   \
    aom_highbd_var_filter_block2d_bil_second_pass(                             \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);              \
                                                                               \
    return aom_highbd_10_obmc_variance##W##x##H##_c((temp2), W, wsrc, mask,    \
                                                    sse);                      \
  }                                                                            \
                                                                               \
  unsigned int aom_highbd_12_obmc_sub_pixel_variance##W##x##H##_c(             \
      const uint16_t *pre, int pre_stride, int xoffset, int yoffset,           \
      const int32_t *wsrc, const int32_t *mask, unsigned int *sse) {           \
    uint16_t fdata3[(H + 1) * W];                                              \
    uint16_t temp2[H * W];                                                     \
                                                                               \
    aom_highbd_var_filter_block2d_bil_first_pass(                              \
        pre, fdata3, pre_stride, 1, H + 1, W, bilinear_filters_2t[xoffset]);   \
    aom_highbd_var_filter_block2d_bil_second_pass(                             \
        fdata3, temp2, W, W, H, W, bilinear_filters_2t[yoffset]);              \
                                                                               \
    return aom_highbd_12_obmc_variance##W##x##H##_c((temp2), W, wsrc, mask,    \
                                                    sse);                      \
  }

HIGHBD_OBMC_VAR(4, 4)
HIGHBD_OBMC_SUBPIX_VAR(4, 4)

HIGHBD_OBMC_VAR(4, 8)
HIGHBD_OBMC_SUBPIX_VAR(4, 8)

HIGHBD_OBMC_VAR(8, 4)
HIGHBD_OBMC_SUBPIX_VAR(8, 4)

HIGHBD_OBMC_VAR(8, 8)
HIGHBD_OBMC_SUBPIX_VAR(8, 8)

HIGHBD_OBMC_VAR(8, 16)
HIGHBD_OBMC_SUBPIX_VAR(8, 16)

HIGHBD_OBMC_VAR(16, 8)
HIGHBD_OBMC_SUBPIX_VAR(16, 8)

HIGHBD_OBMC_VAR(16, 16)
HIGHBD_OBMC_SUBPIX_VAR(16, 16)

HIGHBD_OBMC_VAR(16, 32)
HIGHBD_OBMC_SUBPIX_VAR(16, 32)

HIGHBD_OBMC_VAR(32, 16)
HIGHBD_OBMC_SUBPIX_VAR(32, 16)

HIGHBD_OBMC_VAR(32, 32)
HIGHBD_OBMC_SUBPIX_VAR(32, 32)

HIGHBD_OBMC_VAR(32, 64)
HIGHBD_OBMC_SUBPIX_VAR(32, 64)

HIGHBD_OBMC_VAR(64, 32)
HIGHBD_OBMC_SUBPIX_VAR(64, 32)

HIGHBD_OBMC_VAR(64, 64)
HIGHBD_OBMC_SUBPIX_VAR(64, 64)

HIGHBD_OBMC_VAR(64, 128)
HIGHBD_OBMC_SUBPIX_VAR(64, 128)

HIGHBD_OBMC_VAR(128, 64)
HIGHBD_OBMC_SUBPIX_VAR(128, 64)

HIGHBD_OBMC_VAR(128, 128)
HIGHBD_OBMC_SUBPIX_VAR(128, 128)

#if CONFIG_EXT_RECUR_PARTITIONS
HIGHBD_OBMC_VAR(128, 256)
HIGHBD_OBMC_SUBPIX_VAR(128, 256)

HIGHBD_OBMC_VAR(256, 128)
HIGHBD_OBMC_SUBPIX_VAR(256, 128)

HIGHBD_OBMC_VAR(256, 256)
HIGHBD_OBMC_SUBPIX_VAR(256, 256)
#endif  // CONFIG_EXT_RECUR_PARTITIONS

HIGHBD_OBMC_VAR(4, 16)
HIGHBD_OBMC_SUBPIX_VAR(4, 16)
HIGHBD_OBMC_VAR(16, 4)
HIGHBD_OBMC_SUBPIX_VAR(16, 4)
HIGHBD_OBMC_VAR(8, 32)
HIGHBD_OBMC_SUBPIX_VAR(8, 32)
HIGHBD_OBMC_VAR(32, 8)
HIGHBD_OBMC_SUBPIX_VAR(32, 8)
HIGHBD_OBMC_VAR(16, 64)
HIGHBD_OBMC_SUBPIX_VAR(16, 64)
HIGHBD_OBMC_VAR(64, 16)
HIGHBD_OBMC_SUBPIX_VAR(64, 16)

#if CONFIG_EXT_RECUR_PARTITIONS
HIGHBD_OBMC_VAR(4, 32)
HIGHBD_OBMC_SUBPIX_VAR(4, 32)
HIGHBD_OBMC_VAR(32, 4)
HIGHBD_OBMC_SUBPIX_VAR(32, 4)
HIGHBD_OBMC_VAR(8, 64)
HIGHBD_OBMC_SUBPIX_VAR(8, 64)
HIGHBD_OBMC_VAR(64, 8)
HIGHBD_OBMC_SUBPIX_VAR(64, 8)
HIGHBD_OBMC_VAR(4, 64)
HIGHBD_OBMC_SUBPIX_VAR(4, 64)
HIGHBD_OBMC_VAR(64, 4)
HIGHBD_OBMC_SUBPIX_VAR(64, 4)
#endif  // CONFIG_EXT_RECUR_PARTITIONS

uint64_t aom_mse_wxh_16bit_highbd_c(uint16_t *dst, int dstride, uint16_t *src,
                                    int sstride, int w, int h) {
  uint64_t sum = 0;
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      int e = dst[i * dstride + j] - src[i * sstride + j];
      sum += e * e;
    }
  }
  return sum;
}
