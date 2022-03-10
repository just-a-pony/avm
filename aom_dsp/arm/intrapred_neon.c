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

#include <arm_neon.h>

#include "common/tools_common.h"

#include "config/aom_config.h"
#include "config/aom_dsp_rtcd.h"

#include "aom/aom_integer.h"

//------------------------------------------------------------------------------
// DC 4x4

// 'do_above' and 'do_left' facilitate branch removal when inlined.
static INLINE void dc_4x4(uint8_t *dst, ptrdiff_t stride, const uint8_t *above,
                          const uint8_t *left, int do_above, int do_left) {
  uint16x8_t sum_top;
  uint16x8_t sum_left;
  uint8x8_t dc0;

  if (do_above) {
    const uint8x8_t A = vld1_u8(above);  // top row
    const uint16x4_t p0 = vpaddl_u8(A);  // cascading summation of the top
    const uint16x4_t p1 = vpadd_u16(p0, p0);
    sum_top = vcombine_u16(p1, p1);
  }

  if (do_left) {
    const uint8x8_t L = vld1_u8(left);   // left border
    const uint16x4_t p0 = vpaddl_u8(L);  // cascading summation of the left
    const uint16x4_t p1 = vpadd_u16(p0, p0);
    sum_left = vcombine_u16(p1, p1);
  }

  if (do_above && do_left) {
    const uint16x8_t sum = vaddq_u16(sum_left, sum_top);
    dc0 = vrshrn_n_u16(sum, 3);
  } else if (do_above) {
    dc0 = vrshrn_n_u16(sum_top, 2);
  } else if (do_left) {
    dc0 = vrshrn_n_u16(sum_left, 2);
  } else {
    dc0 = vdup_n_u8(0x80);
  }

  {
    const uint8x8_t dc = vdup_lane_u8(dc0, 0);
    int i;
    for (i = 0; i < 4; ++i) {
      vst1_lane_u32((uint32_t *)(dst + i * stride), vreinterpret_u32_u8(dc), 0);
    }
  }
}

void aom_dc_predictor_4x4_neon(uint8_t *dst, ptrdiff_t stride,
                               const uint8_t *above, const uint8_t *left) {
  dc_4x4(dst, stride, above, left, 1, 1);
}

void aom_dc_left_predictor_4x4_neon(uint8_t *dst, ptrdiff_t stride,
                                    const uint8_t *above, const uint8_t *left) {
  (void)above;
  dc_4x4(dst, stride, NULL, left, 0, 1);
}

void aom_dc_top_predictor_4x4_neon(uint8_t *dst, ptrdiff_t stride,
                                   const uint8_t *above, const uint8_t *left) {
  (void)left;
  dc_4x4(dst, stride, above, NULL, 1, 0);
}

void aom_dc_128_predictor_4x4_neon(uint8_t *dst, ptrdiff_t stride,
                                   const uint8_t *above, const uint8_t *left) {
  (void)above;
  (void)left;
  dc_4x4(dst, stride, NULL, NULL, 0, 0);
}

//------------------------------------------------------------------------------
// DC 8x8

// 'do_above' and 'do_left' facilitate branch removal when inlined.
static INLINE void dc_8x8(uint8_t *dst, ptrdiff_t stride, const uint8_t *above,
                          const uint8_t *left, int do_above, int do_left) {
  uint16x8_t sum_top;
  uint16x8_t sum_left;
  uint8x8_t dc0;

  if (do_above) {
    const uint8x8_t A = vld1_u8(above);  // top row
    const uint16x4_t p0 = vpaddl_u8(A);  // cascading summation of the top
    const uint16x4_t p1 = vpadd_u16(p0, p0);
    const uint16x4_t p2 = vpadd_u16(p1, p1);
    sum_top = vcombine_u16(p2, p2);
  }

  if (do_left) {
    const uint8x8_t L = vld1_u8(left);   // left border
    const uint16x4_t p0 = vpaddl_u8(L);  // cascading summation of the left
    const uint16x4_t p1 = vpadd_u16(p0, p0);
    const uint16x4_t p2 = vpadd_u16(p1, p1);
    sum_left = vcombine_u16(p2, p2);
  }

  if (do_above && do_left) {
    const uint16x8_t sum = vaddq_u16(sum_left, sum_top);
    dc0 = vrshrn_n_u16(sum, 4);
  } else if (do_above) {
    dc0 = vrshrn_n_u16(sum_top, 3);
  } else if (do_left) {
    dc0 = vrshrn_n_u16(sum_left, 3);
  } else {
    dc0 = vdup_n_u8(0x80);
  }

  {
    const uint8x8_t dc = vdup_lane_u8(dc0, 0);
    int i;
    for (i = 0; i < 8; ++i) {
      vst1_u32((uint32_t *)(dst + i * stride), vreinterpret_u32_u8(dc));
    }
  }
}

void aom_dc_predictor_8x8_neon(uint8_t *dst, ptrdiff_t stride,
                               const uint8_t *above, const uint8_t *left) {
  dc_8x8(dst, stride, above, left, 1, 1);
}

void aom_dc_left_predictor_8x8_neon(uint8_t *dst, ptrdiff_t stride,
                                    const uint8_t *above, const uint8_t *left) {
  (void)above;
  dc_8x8(dst, stride, NULL, left, 0, 1);
}

void aom_dc_top_predictor_8x8_neon(uint8_t *dst, ptrdiff_t stride,
                                   const uint8_t *above, const uint8_t *left) {
  (void)left;
  dc_8x8(dst, stride, above, NULL, 1, 0);
}

void aom_dc_128_predictor_8x8_neon(uint8_t *dst, ptrdiff_t stride,
                                   const uint8_t *above, const uint8_t *left) {
  (void)above;
  (void)left;
  dc_8x8(dst, stride, NULL, NULL, 0, 0);
}

//------------------------------------------------------------------------------
// DC 16x16

// 'do_above' and 'do_left' facilitate branch removal when inlined.
static INLINE void dc_16x16(uint8_t *dst, ptrdiff_t stride,
                            const uint8_t *above, const uint8_t *left,
                            int do_above, int do_left) {
  uint16x8_t sum_top;
  uint16x8_t sum_left;
  uint8x8_t dc0;

  if (do_above) {
    const uint8x16_t A = vld1q_u8(above);  // top row
    const uint16x8_t p0 = vpaddlq_u8(A);   // cascading summation of the top
    const uint16x4_t p1 = vadd_u16(vget_low_u16(p0), vget_high_u16(p0));
    const uint16x4_t p2 = vpadd_u16(p1, p1);
    const uint16x4_t p3 = vpadd_u16(p2, p2);
    sum_top = vcombine_u16(p3, p3);
  }

  if (do_left) {
    const uint8x16_t L = vld1q_u8(left);  // left row
    const uint16x8_t p0 = vpaddlq_u8(L);  // cascading summation of the left
    const uint16x4_t p1 = vadd_u16(vget_low_u16(p0), vget_high_u16(p0));
    const uint16x4_t p2 = vpadd_u16(p1, p1);
    const uint16x4_t p3 = vpadd_u16(p2, p2);
    sum_left = vcombine_u16(p3, p3);
  }

  if (do_above && do_left) {
    const uint16x8_t sum = vaddq_u16(sum_left, sum_top);
    dc0 = vrshrn_n_u16(sum, 5);
  } else if (do_above) {
    dc0 = vrshrn_n_u16(sum_top, 4);
  } else if (do_left) {
    dc0 = vrshrn_n_u16(sum_left, 4);
  } else {
    dc0 = vdup_n_u8(0x80);
  }

  {
    const uint8x16_t dc = vdupq_lane_u8(dc0, 0);
    int i;
    for (i = 0; i < 16; ++i) {
      vst1q_u8(dst + i * stride, dc);
    }
  }
}

void aom_dc_predictor_16x16_neon(uint8_t *dst, ptrdiff_t stride,
                                 const uint8_t *above, const uint8_t *left) {
  dc_16x16(dst, stride, above, left, 1, 1);
}

void aom_dc_left_predictor_16x16_neon(uint8_t *dst, ptrdiff_t stride,
                                      const uint8_t *above,
                                      const uint8_t *left) {
  (void)above;
  dc_16x16(dst, stride, NULL, left, 0, 1);
}

void aom_dc_top_predictor_16x16_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  (void)left;
  dc_16x16(dst, stride, above, NULL, 1, 0);
}

void aom_dc_128_predictor_16x16_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  (void)above;
  (void)left;
  dc_16x16(dst, stride, NULL, NULL, 0, 0);
}

//------------------------------------------------------------------------------
// DC 32x32

// 'do_above' and 'do_left' facilitate branch removal when inlined.
static INLINE void dc_32x32(uint8_t *dst, ptrdiff_t stride,
                            const uint8_t *above, const uint8_t *left,
                            int do_above, int do_left) {
  uint16x8_t sum_top;
  uint16x8_t sum_left;
  uint8x8_t dc0;

  if (do_above) {
    const uint8x16_t A0 = vld1q_u8(above);  // top row
    const uint8x16_t A1 = vld1q_u8(above + 16);
    const uint16x8_t p0 = vpaddlq_u8(A0);  // cascading summation of the top
    const uint16x8_t p1 = vpaddlq_u8(A1);
    const uint16x8_t p2 = vaddq_u16(p0, p1);
    const uint16x4_t p3 = vadd_u16(vget_low_u16(p2), vget_high_u16(p2));
    const uint16x4_t p4 = vpadd_u16(p3, p3);
    const uint16x4_t p5 = vpadd_u16(p4, p4);
    sum_top = vcombine_u16(p5, p5);
  }

  if (do_left) {
    const uint8x16_t L0 = vld1q_u8(left);  // left row
    const uint8x16_t L1 = vld1q_u8(left + 16);
    const uint16x8_t p0 = vpaddlq_u8(L0);  // cascading summation of the left
    const uint16x8_t p1 = vpaddlq_u8(L1);
    const uint16x8_t p2 = vaddq_u16(p0, p1);
    const uint16x4_t p3 = vadd_u16(vget_low_u16(p2), vget_high_u16(p2));
    const uint16x4_t p4 = vpadd_u16(p3, p3);
    const uint16x4_t p5 = vpadd_u16(p4, p4);
    sum_left = vcombine_u16(p5, p5);
  }

  if (do_above && do_left) {
    const uint16x8_t sum = vaddq_u16(sum_left, sum_top);
    dc0 = vrshrn_n_u16(sum, 6);
  } else if (do_above) {
    dc0 = vrshrn_n_u16(sum_top, 5);
  } else if (do_left) {
    dc0 = vrshrn_n_u16(sum_left, 5);
  } else {
    dc0 = vdup_n_u8(0x80);
  }

  {
    const uint8x16_t dc = vdupq_lane_u8(dc0, 0);
    int i;
    for (i = 0; i < 32; ++i) {
      vst1q_u8(dst + i * stride, dc);
      vst1q_u8(dst + i * stride + 16, dc);
    }
  }
}

void aom_dc_predictor_32x32_neon(uint8_t *dst, ptrdiff_t stride,
                                 const uint8_t *above, const uint8_t *left) {
  dc_32x32(dst, stride, above, left, 1, 1);
}

void aom_dc_left_predictor_32x32_neon(uint8_t *dst, ptrdiff_t stride,
                                      const uint8_t *above,
                                      const uint8_t *left) {
  (void)above;
  dc_32x32(dst, stride, NULL, left, 0, 1);
}

void aom_dc_top_predictor_32x32_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  (void)left;
  dc_32x32(dst, stride, above, NULL, 1, 0);
}

void aom_dc_128_predictor_32x32_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  (void)above;
  (void)left;
  dc_32x32(dst, stride, NULL, NULL, 0, 0);
}

// -----------------------------------------------------------------------------

void aom_d135_predictor_4x4_neon(uint8_t *dst, ptrdiff_t stride,
                                 const uint8_t *above, const uint8_t *left) {
  const uint8x8_t XABCD_u8 = vld1_u8(above - 1);
  const uint64x1_t XABCD = vreinterpret_u64_u8(XABCD_u8);
  const uint64x1_t ____XABC = vshl_n_u64(XABCD, 32);
  const uint32x2_t zero = vdup_n_u32(0);
  const uint32x2_t IJKL = vld1_lane_u32((const uint32_t *)left, zero, 0);
  const uint8x8_t IJKL_u8 = vreinterpret_u8_u32(IJKL);
  const uint64x1_t LKJI____ = vreinterpret_u64_u8(vrev32_u8(IJKL_u8));
  const uint64x1_t LKJIXABC = vorr_u64(LKJI____, ____XABC);
  const uint8x8_t KJIXABC_ = vreinterpret_u8_u64(vshr_n_u64(LKJIXABC, 8));
  const uint8x8_t JIXABC__ = vreinterpret_u8_u64(vshr_n_u64(LKJIXABC, 16));
  const uint8_t D = vget_lane_u8(XABCD_u8, 4);
  const uint8x8_t JIXABCD_ = vset_lane_u8(D, JIXABC__, 6);
  const uint8x8_t LKJIXABC_u8 = vreinterpret_u8_u64(LKJIXABC);
  const uint8x8_t avg1 = vhadd_u8(JIXABCD_, LKJIXABC_u8);
  const uint8x8_t avg2 = vrhadd_u8(avg1, KJIXABC_);
  const uint64x1_t avg2_u64 = vreinterpret_u64_u8(avg2);
  const uint32x2_t r3 = vreinterpret_u32_u8(avg2);
  const uint32x2_t r2 = vreinterpret_u32_u64(vshr_n_u64(avg2_u64, 8));
  const uint32x2_t r1 = vreinterpret_u32_u64(vshr_n_u64(avg2_u64, 16));
  const uint32x2_t r0 = vreinterpret_u32_u64(vshr_n_u64(avg2_u64, 24));
  vst1_lane_u32((uint32_t *)(dst + 0 * stride), r0, 0);
  vst1_lane_u32((uint32_t *)(dst + 1 * stride), r1, 0);
  vst1_lane_u32((uint32_t *)(dst + 2 * stride), r2, 0);
  vst1_lane_u32((uint32_t *)(dst + 3 * stride), r3, 0);
}

void aom_v_predictor_4x4_neon(uint8_t *dst, ptrdiff_t stride,
                              const uint8_t *above, const uint8_t *left) {
  int i;
  uint32x2_t d0u32 = vdup_n_u32(0);
  (void)left;

  d0u32 = vld1_lane_u32((const uint32_t *)above, d0u32, 0);
  for (i = 0; i < 4; i++, dst += stride)
    vst1_lane_u32((uint32_t *)dst, d0u32, 0);
}

void aom_v_predictor_8x8_neon(uint8_t *dst, ptrdiff_t stride,
                              const uint8_t *above, const uint8_t *left) {
  int i;
  uint8x8_t d0u8 = vdup_n_u8(0);
  (void)left;

  d0u8 = vld1_u8(above);
  for (i = 0; i < 8; i++, dst += stride) vst1_u8(dst, d0u8);
}

void aom_v_predictor_16x16_neon(uint8_t *dst, ptrdiff_t stride,
                                const uint8_t *above, const uint8_t *left) {
  int i;
  uint8x16_t q0u8 = vdupq_n_u8(0);
  (void)left;

  q0u8 = vld1q_u8(above);
  for (i = 0; i < 16; i++, dst += stride) vst1q_u8(dst, q0u8);
}

void aom_v_predictor_32x32_neon(uint8_t *dst, ptrdiff_t stride,
                                const uint8_t *above, const uint8_t *left) {
  int i;
  uint8x16_t q0u8 = vdupq_n_u8(0);
  uint8x16_t q1u8 = vdupq_n_u8(0);
  (void)left;

  q0u8 = vld1q_u8(above);
  q1u8 = vld1q_u8(above + 16);
  for (i = 0; i < 32; i++, dst += stride) {
    vst1q_u8(dst, q0u8);
    vst1q_u8(dst + 16, q1u8);
  }
}

void aom_h_predictor_4x4_neon(uint8_t *dst, ptrdiff_t stride,
                              const uint8_t *above, const uint8_t *left) {
  uint8x8_t d0u8 = vdup_n_u8(0);
  uint32x2_t d1u32 = vdup_n_u32(0);
  (void)above;

  d1u32 = vld1_lane_u32((const uint32_t *)left, d1u32, 0);

  d0u8 = vdup_lane_u8(vreinterpret_u8_u32(d1u32), 0);
  vst1_lane_u32((uint32_t *)dst, vreinterpret_u32_u8(d0u8), 0);
  dst += stride;
  d0u8 = vdup_lane_u8(vreinterpret_u8_u32(d1u32), 1);
  vst1_lane_u32((uint32_t *)dst, vreinterpret_u32_u8(d0u8), 0);
  dst += stride;
  d0u8 = vdup_lane_u8(vreinterpret_u8_u32(d1u32), 2);
  vst1_lane_u32((uint32_t *)dst, vreinterpret_u32_u8(d0u8), 0);
  dst += stride;
  d0u8 = vdup_lane_u8(vreinterpret_u8_u32(d1u32), 3);
  vst1_lane_u32((uint32_t *)dst, vreinterpret_u32_u8(d0u8), 0);
}

void aom_h_predictor_8x8_neon(uint8_t *dst, ptrdiff_t stride,
                              const uint8_t *above, const uint8_t *left) {
  uint8x8_t d0u8 = vdup_n_u8(0);
  uint64x1_t d1u64 = vdup_n_u64(0);
  (void)above;

  d1u64 = vld1_u64((const uint64_t *)left);

  d0u8 = vdup_lane_u8(vreinterpret_u8_u64(d1u64), 0);
  vst1_u8(dst, d0u8);
  dst += stride;
  d0u8 = vdup_lane_u8(vreinterpret_u8_u64(d1u64), 1);
  vst1_u8(dst, d0u8);
  dst += stride;
  d0u8 = vdup_lane_u8(vreinterpret_u8_u64(d1u64), 2);
  vst1_u8(dst, d0u8);
  dst += stride;
  d0u8 = vdup_lane_u8(vreinterpret_u8_u64(d1u64), 3);
  vst1_u8(dst, d0u8);
  dst += stride;
  d0u8 = vdup_lane_u8(vreinterpret_u8_u64(d1u64), 4);
  vst1_u8(dst, d0u8);
  dst += stride;
  d0u8 = vdup_lane_u8(vreinterpret_u8_u64(d1u64), 5);
  vst1_u8(dst, d0u8);
  dst += stride;
  d0u8 = vdup_lane_u8(vreinterpret_u8_u64(d1u64), 6);
  vst1_u8(dst, d0u8);
  dst += stride;
  d0u8 = vdup_lane_u8(vreinterpret_u8_u64(d1u64), 7);
  vst1_u8(dst, d0u8);
}

void aom_h_predictor_16x16_neon(uint8_t *dst, ptrdiff_t stride,
                                const uint8_t *above, const uint8_t *left) {
  int j;
  uint8x8_t d2u8 = vdup_n_u8(0);
  uint8x16_t q0u8 = vdupq_n_u8(0);
  uint8x16_t q1u8 = vdupq_n_u8(0);
  (void)above;

  q1u8 = vld1q_u8(left);
  d2u8 = vget_low_u8(q1u8);
  for (j = 0; j < 2; j++, d2u8 = vget_high_u8(q1u8)) {
    q0u8 = vdupq_lane_u8(d2u8, 0);
    vst1q_u8(dst, q0u8);
    dst += stride;
    q0u8 = vdupq_lane_u8(d2u8, 1);
    vst1q_u8(dst, q0u8);
    dst += stride;
    q0u8 = vdupq_lane_u8(d2u8, 2);
    vst1q_u8(dst, q0u8);
    dst += stride;
    q0u8 = vdupq_lane_u8(d2u8, 3);
    vst1q_u8(dst, q0u8);
    dst += stride;
    q0u8 = vdupq_lane_u8(d2u8, 4);
    vst1q_u8(dst, q0u8);
    dst += stride;
    q0u8 = vdupq_lane_u8(d2u8, 5);
    vst1q_u8(dst, q0u8);
    dst += stride;
    q0u8 = vdupq_lane_u8(d2u8, 6);
    vst1q_u8(dst, q0u8);
    dst += stride;
    q0u8 = vdupq_lane_u8(d2u8, 7);
    vst1q_u8(dst, q0u8);
    dst += stride;
  }
}

void aom_h_predictor_32x32_neon(uint8_t *dst, ptrdiff_t stride,
                                const uint8_t *above, const uint8_t *left) {
  int j, k;
  uint8x8_t d2u8 = vdup_n_u8(0);
  uint8x16_t q0u8 = vdupq_n_u8(0);
  uint8x16_t q1u8 = vdupq_n_u8(0);
  (void)above;

  for (k = 0; k < 2; k++, left += 16) {
    q1u8 = vld1q_u8(left);
    d2u8 = vget_low_u8(q1u8);
    for (j = 0; j < 2; j++, d2u8 = vget_high_u8(q1u8)) {
      q0u8 = vdupq_lane_u8(d2u8, 0);
      vst1q_u8(dst, q0u8);
      vst1q_u8(dst + 16, q0u8);
      dst += stride;
      q0u8 = vdupq_lane_u8(d2u8, 1);
      vst1q_u8(dst, q0u8);
      vst1q_u8(dst + 16, q0u8);
      dst += stride;
      q0u8 = vdupq_lane_u8(d2u8, 2);
      vst1q_u8(dst, q0u8);
      vst1q_u8(dst + 16, q0u8);
      dst += stride;
      q0u8 = vdupq_lane_u8(d2u8, 3);
      vst1q_u8(dst, q0u8);
      vst1q_u8(dst + 16, q0u8);
      dst += stride;
      q0u8 = vdupq_lane_u8(d2u8, 4);
      vst1q_u8(dst, q0u8);
      vst1q_u8(dst + 16, q0u8);
      dst += stride;
      q0u8 = vdupq_lane_u8(d2u8, 5);
      vst1q_u8(dst, q0u8);
      vst1q_u8(dst + 16, q0u8);
      dst += stride;
      q0u8 = vdupq_lane_u8(d2u8, 6);
      vst1q_u8(dst, q0u8);
      vst1q_u8(dst + 16, q0u8);
      dst += stride;
      q0u8 = vdupq_lane_u8(d2u8, 7);
      vst1q_u8(dst, q0u8);
      vst1q_u8(dst + 16, q0u8);
      dst += stride;
    }
  }
}

static INLINE void highbd_dc_predictor(uint16_t *dst, ptrdiff_t stride, int bw,
                                       const uint16_t *above,
                                       const uint16_t *left) {
  assert(bw >= 4);
  assert(IS_POWER_OF_TWO(bw));
  int expected_dc, sum = 0;
  const int count = bw * 2;
  uint32x4_t sum_q = vdupq_n_u32(0);
  uint32x2_t sum_d;
  uint16_t *dst_1;
  if (bw >= 8) {
    for (int i = 0; i < bw; i += 8) {
      sum_q = vpadalq_u16(sum_q, vld1q_u16(above));
      sum_q = vpadalq_u16(sum_q, vld1q_u16(left));
      above += 8;
      left += 8;
    }
    sum_d = vadd_u32(vget_low_u32(sum_q), vget_high_u32(sum_q));
    sum = vget_lane_s32(vreinterpret_s32_u64(vpaddl_u32(sum_d)), 0);
    expected_dc = (sum + (count >> 1)) / count;
    const uint16x8_t dc = vdupq_n_u16((uint16_t)expected_dc);
    for (int r = 0; r < bw; r++) {
      dst_1 = dst;
      for (int i = 0; i < bw; i += 8) {
        vst1q_u16(dst_1, dc);
        dst_1 += 8;
      }
      dst += stride;
    }
  } else {  // 4x4
    sum_q = vaddl_u16(vld1_u16(above), vld1_u16(left));
    sum_d = vadd_u32(vget_low_u32(sum_q), vget_high_u32(sum_q));
    sum = vget_lane_s32(vreinterpret_s32_u64(vpaddl_u32(sum_d)), 0);
    expected_dc = (sum + (count >> 1)) / count;
    const uint16x4_t dc = vdup_n_u16((uint16_t)expected_dc);
    for (int r = 0; r < bw; r++) {
      vst1_u16(dst, dc);
      dst += stride;
    }
  }
}

#define intra_pred_highbd_sized_neon(type, width)               \
  void aom_highbd_##type##_predictor_##width##x##width##_neon(  \
      uint16_t *dst, ptrdiff_t stride, const uint16_t *above,   \
      const uint16_t *left, int bd) {                           \
    (void)bd;                                                   \
    highbd_##type##_predictor(dst, stride, width, above, left); \
  }

#define intra_pred_square(type)           \
  intra_pred_highbd_sized_neon(type, 4);  \
  intra_pred_highbd_sized_neon(type, 8);  \
  intra_pred_highbd_sized_neon(type, 16); \
  intra_pred_highbd_sized_neon(type, 32); \
  intra_pred_highbd_sized_neon(type, 64);

intra_pred_square(dc);
#undef intra_pred_square

static const int sm_weight_log2_scale = 8;

// max(block_size_wide[BLOCK_LARGEST], block_size_high[BLOCK_LARGEST])
#define MAX_BLOCK_DIM 64

/* clang-format off */
static const uint8_t sm_weight_arrays[2 * MAX_BLOCK_DIM] = {
    // Unused, because we always offset by bs, which is at least 2.
    0, 0,
    // bs = 2
    255, 128,
    // bs = 4
    255, 149, 85, 64,
    // bs = 8
    255, 197, 146, 105, 73, 50, 37, 32,
    // bs = 16
    255, 225, 196, 170, 145, 123, 102, 84, 68, 54, 43, 33, 26, 20, 17, 16,
    // bs = 32
    255, 240, 225, 210, 196, 182, 169, 157, 145, 133, 122, 111, 101, 92, 83, 74,
    66, 59, 52, 45, 39, 34, 29, 25, 21, 17, 14, 12, 10, 9, 8, 8,
    // bs = 64
    255, 248, 240, 233, 225, 218, 210, 203, 196, 189, 182, 176, 169, 163, 156,
    150, 144, 138, 133, 127, 121, 116, 111, 106, 101, 96, 91, 86, 82, 77, 73,
    69, 65, 61, 57, 54, 50, 47, 44, 41, 38, 35, 32, 29, 27, 25, 22, 20, 18, 16,
    15, 13, 12, 10, 9, 8, 7, 6, 6, 5, 5, 4, 4, 4,
};
/* clang-format on */

// -----------------------------------------------------------------------------
// SMOOTH_PRED

// pixels[0]: above and below_pred interleave vector
// pixels[1]: left vector
// pixels[2]: right_pred vector
static INLINE void load_pixel_w4(const uint8_t *above, const uint8_t *left,
                                 int height, uint8x16_t *pixels) {
  uint32x4_t zero = vdupq_n_u32(0);
  const uint8x8_t d = vcreate_u8(((const uint32_t *)above)[0]);
  if (height == 4)
    pixels[1] =
        vreinterpretq_u8_u32(vld1q_lane_u32((const uint32_t *)left, zero, 0));
  else if (height == 8) {
    pixels[1] = vreinterpretq_u8_u64(vsetq_lane_u64(
        ((const uint64_t *)left)[0], vreinterpretq_u64_u32(zero), 0));
  } else {
    pixels[1] = vld1q_u8(left);
  }

  pixels[2] = vreinterpretq_u8_u16(vdupq_n_u16(above[3]));

  const uint16x8_t bp = vdupq_n_u16(left[height - 1]);
#if defined(__aarch64__)
  pixels[0] = vreinterpretq_u8_u16(vzip1q_u16(vmovl_u8(d), bp));
#else
  pixels[0] = vreinterpretq_u8_u16(vzipq_u16(vmovl_u8(d), bp).val[0]);
#endif  // (__aarch64__)
}

// weight_h[0]: weight_h vector
// weight_h[1]: scale - weight_h vector
// weight_h[2]: same as [0], second half for height = 16 only
// weight_h[3]: same as [1], second half for height = 16 only
// weight_w[0]: weights_w and scale - weights_w interleave vector
static INLINE void load_weight_w4(const uint8_t *weight_array, int height,
                                  uint16x8_t *weight_h, uint16x8_t *weight_w) {
  const uint16x8_t d = vdupq_n_u16((uint16_t)(1 << sm_weight_log2_scale));
  const uint8x8_t t = vcreate_u8(((const uint32_t *)(weight_array))[1]);
  weight_h[0] = vmovl_u8(t);
  weight_h[1] = vsubw_u8(d, t);
#if defined(__aarch64__)
  weight_w[0] = vzip1q_u16(weight_h[0], weight_h[1]);
#else
  weight_w[0] = vzipq_u16(weight_h[0], weight_h[1]).val[0];
#endif  // (__aarch64__)

  if (height == 8) {
    const uint8x8_t weight = vld1_u8(&weight_array[8]);
    weight_h[0] = vmovl_u8(weight);
    weight_h[1] = vsubw_u8(d, weight);
  } else if (height == 16) {
    const uint8x16_t zero = vdupq_n_u8(0);
    const uint8x16_t weight = vld1q_u8(&weight_array[16]);
    const uint8x16x2_t weight_h_02 = vzipq_u8(weight, zero);
    weight_h[0] = vreinterpretq_u16_u8(weight_h_02.val[0]);
    weight_h[1] = vsubq_u16(d, vreinterpretq_u16_u8(weight_h_02.val[0]));
    weight_h[2] = vreinterpretq_u16_u8(weight_h_02.val[1]);
    weight_h[3] = vsubq_u16(d, vreinterpretq_u16_u8(weight_h_02.val[1]));
  }
}

static INLINE void smooth_pred_4xh(const uint8x16_t *pixel,
                                   const uint16x8_t *wh, const uint16x8_t *ww,
                                   int h, uint8_t *dst, ptrdiff_t stride,
                                   int second_half) {
  const uint16x4_t one = vdup_n_u16(1);
  const uint16x4_t inc = vdup_n_u16(0x202);
  uint16x4_t rep =
      second_half ? vdup_n_u16((uint16_t)0x8008) : vdup_n_u16((uint16_t)0x8000);
  uint16x4_t d = vdup_n_u16(0x100);
  const uint16x4_t v_pixel_0_lo = vmovn_u32(vreinterpretq_u32_u8(pixel[0]));
  const uint16x4_t v_pixel_0_hi =
      vmovn_u32(vreinterpretq_u32_u8(vextq_u8(pixel[0], pixel[0], 2)));
  const uint16x4_t v_pixel_2 = vget_low_u16(vreinterpretq_u16_u8(pixel[2]));
  const uint16x4_t ww_0_lo = vmovn_u32(vreinterpretq_u32_u16(ww[0]));
  const uint16x4_t ww_0_hi =
      vmovn_u32(vreinterpretq_u32_u16(vextq_u16(ww[0], ww[0], 1)));
  const uint8x8_t save_mask = vcreate_u8(0 + (2 << 8) + (4 << 16) + (6 << 24));

#if !defined(__aarch64__)
  const uint8x8x2_t v_split1 = { { vget_low_u8(vreinterpretq_u8_u16(wh[0])),
                                   vget_high_u8(
                                       vreinterpretq_u8_u16(wh[0])) } };
  const uint8x8x2_t v_split2 = { { vget_low_u8(vreinterpretq_u8_u16(wh[1])),
                                   vget_high_u8(
                                       vreinterpretq_u8_u16(wh[1])) } };
  const uint8x8x2_t v_split3 = { { vget_low_u8(pixel[1]),
                                   vget_high_u8(pixel[1]) } };
#endif  // (__aarch64__)

  for (int i = 0; i < h; ++i) {
#if defined(__aarch64__)
    const uint8x8_t wg =
        vqtbl1_u8(vreinterpretq_u8_u16(wh[0]), vreinterpret_u8_u16(d));
    const uint8x8_t sc =
        vqtbl1_u8(vreinterpretq_u8_u16(wh[1]), vreinterpret_u8_u16(d));
#else
    const uint8x8_t wg = vtbl2_u8(v_split1, vreinterpret_u8_u16(d));
    const uint8x8_t sc = vtbl2_u8(v_split2, vreinterpret_u8_u16(d));
#endif  // (__aarch64__)

    uint32x4_t sum = vmull_u16(v_pixel_0_lo, vreinterpret_u16_u8(wg));
    sum = vmlal_u16(sum, v_pixel_0_hi, vreinterpret_u16_u8(sc));

#if defined(__aarch64__)
    uint8x8_t b = vqtbl1_u8(pixel[1], vreinterpret_u8_u16(rep));
#else
    uint8x8_t b = vtbl2_u8(v_split3, vreinterpret_u8_u16(rep));
#endif  // (__aarch64__)

    sum = vmlal_u16(sum, vreinterpret_u16_u8(b), ww_0_lo);
    sum = vmlal_u16(sum, v_pixel_2, ww_0_hi);
    uint8x8_t sum_l = vreinterpret_u8_u16(vqrshrn_n_u32(sum, 9));
    uint32x2_t predsh = vreinterpret_u32_u8(vtbl1_u8(sum_l, save_mask));
    vst1_lane_u32((uint32_t *)dst, predsh, 0);

    dst += stride;

    rep = vadd_u16(rep, one);
    d = vadd_u16(d, inc);
  }
}

void aom_smooth_predictor_4x4_neon(uint8_t *dst, ptrdiff_t stride,
                                   const uint8_t *above, const uint8_t *left) {
  uint8x16_t pixels[3];
  load_pixel_w4(above, left, 4, pixels);

  uint16x8_t wh[4], ww[2];
  load_weight_w4(sm_weight_arrays, 4, wh, ww);

  smooth_pred_4xh(pixels, wh, ww, 4, dst, stride, 0);
}

void aom_smooth_predictor_4x8_neon(uint8_t *dst, ptrdiff_t stride,
                                   const uint8_t *above, const uint8_t *left) {
  uint8x16_t pixels[3];
  load_pixel_w4(above, left, 8, pixels);

  uint16x8_t wh[4], ww[2];
  load_weight_w4(sm_weight_arrays, 8, wh, ww);

  smooth_pred_4xh(pixels, wh, ww, 8, dst, stride, 0);
}

void aom_smooth_predictor_4x16_neon(uint8_t *dst, ptrdiff_t stride,
                                    const uint8_t *above, const uint8_t *left) {
  uint8x16_t pixels[3];
  load_pixel_w4(above, left, 16, pixels);

  uint16x8_t wh[4], ww[2];
  load_weight_w4(sm_weight_arrays, 16, wh, ww);

  smooth_pred_4xh(pixels, wh, ww, 8, dst, stride, 0);
  dst += stride << 3;
  smooth_pred_4xh(pixels, &wh[2], ww, 8, dst, stride, 1);
}

// pixels[0]: above and below_pred interleave vector, first half
// pixels[1]: above and below_pred interleave vector, second half
// pixels[2]: left vector
// pixels[3]: right_pred vector
// pixels[4]: above and below_pred interleave vector, first half
// pixels[5]: above and below_pred interleave vector, second half
// pixels[6]: left vector + 16
// pixels[7]: right_pred vector
static INLINE void load_pixel_w8(const uint8_t *above, const uint8_t *left,
                                 int height, uint8x16_t *pixels) {
  pixels[0] = vreinterpretq_u8_u16(vmovl_u8(vld1_u8(above)));
  pixels[1] = vreinterpretq_u8_u16(vdupq_n_u16((uint16_t)left[height - 1]));
  pixels[3] = vreinterpretq_u8_u16(vdupq_n_u16((uint16_t)above[7]));

  if (height == 4) {
    const uint32x4_t zero32 = vdupq_n_u32(0);
    pixels[2] =
        vreinterpretq_u8_u32(vld1q_lane_u32((const uint32_t *)left, zero32, 0));
  } else if (height == 8) {
    const uint64x2_t zero64 = vdupq_n_u64(0);
    pixels[2] = vreinterpretq_u8_u64(
        vsetq_lane_u64(((const uint64_t *)left)[0], zero64, 0));
  } else if (height == 16) {
    pixels[2] = vld1q_u8(left);
  } else {
    pixels[2] = vld1q_u8(left);
    pixels[4] = pixels[0];
    pixels[5] = pixels[1];
    pixels[6] = vld1q_u8(left + 16);
    pixels[7] = pixels[3];
  }
}

// weight_h[0]: weight_h vector
// weight_h[1]: scale - weight_h vector
// weight_h[2]: same as [0], offset 8
// weight_h[3]: same as [1], offset 8
// weight_h[4]: same as [0], offset 16
// weight_h[5]: same as [1], offset 16
// weight_h[6]: same as [0], offset 24
// weight_h[7]: same as [1], offset 24
// weight_w[0]: weights_w and scale - weights_w interleave vector, first half
// weight_w[1]: weights_w and scale - weights_w interleave vector, second half
static INLINE void load_weight_w8(const uint8_t *weight_array, int height,
                                  uint16x8_t *weight_h, uint16x8_t *weight_w) {
  const uint8x16_t zero = vdupq_n_u8(0);
  const int we_offset = height < 8 ? 4 : 8;
  uint8x16_t we = vld1q_u8(&weight_array[we_offset]);
#if defined(__aarch64__)
  weight_h[0] = vreinterpretq_u16_u8(vzip1q_u8(we, zero));
#else
  weight_h[0] = vreinterpretq_u16_u8(vzipq_u8(we, zero).val[0]);
#endif  // (__aarch64__)
  const uint16x8_t d = vdupq_n_u16(256);
  weight_h[1] = vsubq_u16(d, weight_h[0]);

  if (height == 4) {
    we = vextq_u8(we, zero, 4);
#if defined(__aarch64__)
    weight_w[0] = vreinterpretq_u16_u8(vzip1q_u8(we, zero));
#else
    weight_w[0] = vmovl_u8(vget_low_u8(we));
#endif  // (__aarch64__)
    weight_w[1] = vsubq_u16(d, weight_w[0]);
  } else {
    weight_w[0] = weight_h[0];
    weight_w[1] = weight_h[1];
  }

  if (height == 16) {
    we = vld1q_u8(&weight_array[16]);
    const uint8x16x2_t weight_h_02 = vzipq_u8(we, zero);
    weight_h[0] = vreinterpretq_u16_u8(weight_h_02.val[0]);
    weight_h[1] = vsubq_u16(d, weight_h[0]);
    weight_h[2] = vreinterpretq_u16_u8(weight_h_02.val[1]);
    weight_h[3] = vsubq_u16(d, weight_h[2]);
  } else if (height == 32) {
    const uint8x16_t weight_lo = vld1q_u8(&weight_array[32]);
    const uint8x16x2_t weight_h_02 = vzipq_u8(weight_lo, zero);
    weight_h[0] = vreinterpretq_u16_u8(weight_h_02.val[0]);
    weight_h[1] = vsubq_u16(d, weight_h[0]);
    weight_h[2] = vreinterpretq_u16_u8(weight_h_02.val[1]);
    weight_h[3] = vsubq_u16(d, weight_h[2]);
    const uint8x16_t weight_hi = vld1q_u8(&weight_array[32 + 16]);
    const uint8x16x2_t weight_h_46 = vzipq_u8(weight_hi, zero);
    weight_h[4] = vreinterpretq_u16_u8(weight_h_46.val[0]);
    weight_h[5] = vsubq_u16(d, weight_h[4]);
    weight_h[6] = vreinterpretq_u16_u8(weight_h_46.val[1]);
    weight_h[7] = vsubq_u16(d, weight_h[6]);
  }
}

static INLINE void smooth_pred_8xh(const uint8x16_t *pixels,
                                   const uint16x8_t *wh, const uint16x8_t *ww,
                                   int h, uint8_t *dst, ptrdiff_t stride,
                                   int second_half) {
  const uint16x8_t one = vdupq_n_u16(1);
  const uint16x8_t inc = vdupq_n_u16(0x202);
  uint16x8_t rep = second_half ? vdupq_n_u16((uint16_t)0x8008)
                               : vdupq_n_u16((uint16_t)0x8000);
  uint16x8_t d = vdupq_n_u16(0x100);

#if !defined(__aarch64__)
  const uint8x8x2_t v_split1 = { { vget_low_u8(vreinterpretq_u8_u16(wh[0])),
                                   vget_high_u8(
                                       vreinterpretq_u8_u16(wh[0])) } };
  const uint8x8x2_t v_split2 = { { vget_low_u8(vreinterpretq_u8_u16(wh[1])),
                                   vget_high_u8(
                                       vreinterpretq_u8_u16(wh[1])) } };
  const uint8x8x2_t v_split3 = { { vget_low_u8(pixels[2]),
                                   vget_high_u8(pixels[2]) } };
#endif

  for (int i = 0; i < h; ++i) {
#if defined(__aarch64__)
    const uint8x16_t wg_wg =
        vqtbl1q_u8(vreinterpretq_u8_u16(wh[0]), vreinterpretq_u8_u16(d));
    const uint8x16_t sc_sc =
        vqtbl1q_u8(vreinterpretq_u8_u16(wh[1]), vreinterpretq_u8_u16(d));
#else
    const uint8x8_t v_d_lo = vreinterpret_u8_u16(vget_low_u16(d));
    const uint8x8_t v_d_hi = vreinterpret_u8_u16(vget_high_u16(d));
    const uint8x16_t wg_wg =
        vcombine_u8(vtbl2_u8(v_split1, v_d_lo), vtbl2_u8(v_split1, v_d_hi));
    const uint8x16_t sc_sc =
        vcombine_u8(vtbl2_u8(v_split2, v_d_lo), vtbl2_u8(v_split2, v_d_hi));
#endif  // (__aarch64__)
    uint16x8_t s01 =
        vmulq_u16(vreinterpretq_u16_u8(pixels[0]), vreinterpretq_u16_u8(wg_wg));
    s01 = vmlaq_u16(s01, vreinterpretq_u16_u8(pixels[1]),
                    vreinterpretq_u16_u8(sc_sc));
#if defined(__aarch64__)
    const uint8x16_t b = vqtbl1q_u8(pixels[2], vreinterpretq_u8_u16(rep));
#else
    const uint8x16_t b = vcombine_u8(
        vtbl2_u8(v_split3, vget_low_u8(vreinterpretq_u8_u16(rep))),
        vtbl2_u8(v_split3, vget_high_u8(vreinterpretq_u8_u16(rep))));
#endif  // (__aarch64__)
    uint16x8_t sum0 = vmulq_u16(vreinterpretq_u16_u8(b), ww[0]);
    sum0 = vmlaq_u16(sum0, vreinterpretq_u16_u8(pixels[3]), ww[1]);

    uint32x4_t s0 = vaddl_u16(vget_low_u16(s01), vget_low_u16(sum0));
#if defined(__aarch64__)
    uint32x4_t s1 = vaddl_high_u16(s01, sum0);
#else
    uint32x4_t s1 = vaddl_u16(vget_high_u16(s01), vget_high_u16(sum0));
#endif  // (__aarch64__)

    sum0 = vcombine_u16(vqrshrn_n_u32(s0, 9), vqrshrn_n_u32(s1, 9));
    uint8x8_t predsh = vqmovn_u16(sum0);
    vst1_u8(dst, predsh);

    dst += stride;
    rep = vaddq_u16(rep, one);
    d = vaddq_u16(d, inc);
  }
}

void aom_smooth_predictor_8x4_neon(uint8_t *dst, ptrdiff_t stride,
                                   const uint8_t *above, const uint8_t *left) {
  uint8x16_t pixels[4];
  load_pixel_w8(above, left, 4, pixels);

  uint16x8_t wh[4], ww[2];
  load_weight_w8(sm_weight_arrays, 4, wh, ww);

  smooth_pred_8xh(pixels, wh, ww, 4, dst, stride, 0);
}

void aom_smooth_predictor_8x8_neon(uint8_t *dst, ptrdiff_t stride,
                                   const uint8_t *above, const uint8_t *left) {
  uint8x16_t pixels[4];
  load_pixel_w8(above, left, 8, pixels);

  uint16x8_t wh[4], ww[2];
  load_weight_w8(sm_weight_arrays, 8, wh, ww);

  smooth_pred_8xh(pixels, wh, ww, 8, dst, stride, 0);
}

void aom_smooth_predictor_8x16_neon(uint8_t *dst, ptrdiff_t stride,
                                    const uint8_t *above, const uint8_t *left) {
  uint8x16_t pixels[4];
  load_pixel_w8(above, left, 16, pixels);

  uint16x8_t wh[4], ww[2];
  load_weight_w8(sm_weight_arrays, 16, wh, ww);

  smooth_pred_8xh(pixels, wh, ww, 8, dst, stride, 0);
  dst += stride << 3;
  smooth_pred_8xh(pixels, &wh[2], ww, 8, dst, stride, 1);
}

void aom_smooth_predictor_8x32_neon(uint8_t *dst, ptrdiff_t stride,
                                    const uint8_t *above, const uint8_t *left) {
  uint8x16_t pixels[8];
  load_pixel_w8(above, left, 32, pixels);

  uint16x8_t wh[8], ww[2];
  load_weight_w8(sm_weight_arrays, 32, wh, ww);

  smooth_pred_8xh(&pixels[0], wh, ww, 8, dst, stride, 0);
  dst += stride << 3;
  smooth_pred_8xh(&pixels[0], &wh[2], ww, 8, dst, stride, 1);
  dst += stride << 3;
  smooth_pred_8xh(&pixels[4], &wh[4], ww, 8, dst, stride, 0);
  dst += stride << 3;
  smooth_pred_8xh(&pixels[4], &wh[6], ww, 8, dst, stride, 1);
}

static INLINE void smooth_predictor_wxh(uint8_t *dst, ptrdiff_t stride,
                                        const uint8_t *above,
                                        const uint8_t *left, uint32_t bw,
                                        uint32_t bh) {
  const uint8_t *const sm_weights_w = sm_weight_arrays + bw;
  const uint8_t *const sm_weights_h = sm_weight_arrays + bh;
  const uint16x8_t scale_value = vdupq_n_u16(256);

  for (uint32_t y = 0; y < bh; ++y) {
    const uint8x8_t left_y = vdup_n_u8(left[y]);
    const uint8x8_t weights_y_dup = vdup_n_u8(sm_weights_h[y]);
    const uint32x4_t pred_scaled_bl =
        vdupq_n_u32(256 + (256 - sm_weights_h[y]) * left[bh - 1]);

    for (uint32_t x = 0; x < bw; x += 8) {
      const uint8x8_t weights_x = vld1_u8(sm_weights_w + x);
      const uint8x8_t top_x = vld1_u8(above + x);

      uint16x8_t pred_m1, pred_m2;
      uint32x4_t pred_lo, pred_hi;
      pred_m1 = vmull_u8(top_x, weights_y_dup);
      pred_m2 = vmull_u8(weights_x, left_y);

      pred_lo = vaddl_u16(vget_low_u16(pred_m1), vget_low_u16(pred_m2));
#if defined(__aarch64__)
      pred_hi = vaddl_high_u16(pred_m1, pred_m2);
#else
      pred_hi = vaddl_u16(vget_high_u16(pred_m1), vget_high_u16(pred_m2));
#endif  // (__aarch64__)

      const uint16x8_t scale_m_weights_x = vsubw_u8(scale_value, weights_x);

      const uint16x8_t swxtr = vmulq_n_u16(scale_m_weights_x, above[bw - 1]);

      pred_lo = vaddq_u32(pred_lo, pred_scaled_bl);
      pred_hi = vaddq_u32(pred_hi, pred_scaled_bl);

      pred_lo = vaddw_u16(pred_lo, vget_low_u16(swxtr));
#if defined(__aarch64__)
      pred_hi = vaddw_high_u16(pred_hi, swxtr);
#else
      pred_hi = vaddw_u16(pred_hi, vget_high_u16(swxtr));
#endif  // (__aarch64__)

      uint16x8_t pred =
          vcombine_u16(vshrn_n_u32(pred_lo, 9), vshrn_n_u32(pred_hi, 9));

      uint8x8_t predsh = vqmovn_u16(pred);

      vst1_u8(dst + x, predsh);
    }

    dst += stride;
  }
}

void aom_smooth_predictor_16x4_neon(uint8_t *dst, ptrdiff_t stride,
                                    const uint8_t *above, const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 16, 4);
}

void aom_smooth_predictor_16x8_neon(uint8_t *dst, ptrdiff_t stride,
                                    const uint8_t *above, const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 16, 8);
}

void aom_smooth_predictor_16x16_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 16, 16);
}

void aom_smooth_predictor_16x32_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 16, 32);
}

void aom_smooth_predictor_32x8_neon(uint8_t *dst, ptrdiff_t stride,
                                    const uint8_t *above, const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 32, 8);
}

void aom_smooth_predictor_32x16_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 32, 16);
}

void aom_smooth_predictor_32x32_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 32, 32);
}

void aom_smooth_predictor_32x64_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 32, 64);
}

void aom_smooth_predictor_64x64_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 64, 64);
}

void aom_smooth_predictor_64x32_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 64, 32);
}

void aom_smooth_predictor_64x16_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 64, 16);
}

void aom_smooth_predictor_16x64_neon(uint8_t *dst, ptrdiff_t stride,
                                     const uint8_t *above,
                                     const uint8_t *left) {
  smooth_predictor_wxh(dst, stride, above, left, 16, 64);
}
