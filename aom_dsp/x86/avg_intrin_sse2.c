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

#include <immintrin.h>

#include "config/aom_dsp_rtcd.h"
#include "aom/aom_integer.h"
#include "aom_dsp/x86/bitdepth_conversion_sse2.h"
#include "aom_ports/mem.h"

int aom_satd_sse2(const tran_low_t *coeff, int length) {
  int i;
  const __m128i zero = _mm_setzero_si128();
  __m128i accum = zero;

  for (i = 0; i < length; i += 8) {
    const __m128i src_line = load_tran_low(coeff);
    const __m128i inv = _mm_sub_epi16(zero, src_line);
    const __m128i abs = _mm_max_epi16(src_line, inv);  // abs(src_line)
    const __m128i abs_lo = _mm_unpacklo_epi16(abs, zero);
    const __m128i abs_hi = _mm_unpackhi_epi16(abs, zero);
    const __m128i sum = _mm_add_epi32(abs_lo, abs_hi);
    accum = _mm_add_epi32(accum, sum);
    coeff += 8;
  }

  {  // cascading summation of accum
    __m128i hi = _mm_srli_si128(accum, 8);
    accum = _mm_add_epi32(accum, hi);
    hi = _mm_srli_epi64(accum, 32);
    accum = _mm_add_epi32(accum, hi);
  }

  return _mm_cvtsi128_si32(accum);
}

void aom_int_pro_row_sse2(int16_t hbuf[16], const uint8_t *ref,
                          const int ref_stride, const int height) {
  int idx = 1;
  __m128i zero = _mm_setzero_si128();
  __m128i src_line = _mm_loadu_si128((const __m128i *)ref);
  __m128i s0 = _mm_unpacklo_epi8(src_line, zero);
  __m128i s1 = _mm_unpackhi_epi8(src_line, zero);
  __m128i t0, t1;
  int height_1 = height - 1;
  ref += ref_stride;
  do {
    src_line = _mm_loadu_si128((const __m128i *)ref);
    t0 = _mm_unpacklo_epi8(src_line, zero);
    t1 = _mm_unpackhi_epi8(src_line, zero);
    s0 = _mm_adds_epu16(s0, t0);
    s1 = _mm_adds_epu16(s1, t1);
    ref += ref_stride;

    src_line = _mm_loadu_si128((const __m128i *)ref);
    t0 = _mm_unpacklo_epi8(src_line, zero);
    t1 = _mm_unpackhi_epi8(src_line, zero);
    s0 = _mm_adds_epu16(s0, t0);
    s1 = _mm_adds_epu16(s1, t1);
    ref += ref_stride;
    idx += 2;
  } while (idx < height_1);

  src_line = _mm_loadu_si128((const __m128i *)ref);
  t0 = _mm_unpacklo_epi8(src_line, zero);
  t1 = _mm_unpackhi_epi8(src_line, zero);
  s0 = _mm_adds_epu16(s0, t0);
  s1 = _mm_adds_epu16(s1, t1);
  if (height == 128) {
    s0 = _mm_srai_epi16(s0, 6);
    s1 = _mm_srai_epi16(s1, 6);
  } else if (height == 64) {
    s0 = _mm_srai_epi16(s0, 5);
    s1 = _mm_srai_epi16(s1, 5);
  } else if (height == 32) {
    s0 = _mm_srai_epi16(s0, 4);
    s1 = _mm_srai_epi16(s1, 4);
  } else {
    assert(height == 16);
    s0 = _mm_srai_epi16(s0, 3);
    s1 = _mm_srai_epi16(s1, 3);
  }

  _mm_storeu_si128((__m128i *)hbuf, s0);
  _mm_storeu_si128((__m128i *)(hbuf + 8), s1);
}

int16_t aom_int_pro_col_sse2(const uint8_t *ref, const int width) {
  __m128i zero = _mm_setzero_si128();
  __m128i src_line = _mm_loadu_si128((const __m128i *)ref);
  __m128i s0 = _mm_sad_epu8(src_line, zero);
  __m128i s1;
  int i;

  for (i = 16; i < width; i += 16) {
    ref += 16;
    src_line = _mm_loadu_si128((const __m128i *)ref);
    s1 = _mm_sad_epu8(src_line, zero);
    s0 = _mm_adds_epu16(s0, s1);
  }

  s1 = _mm_srli_si128(s0, 8);
  s0 = _mm_adds_epu16(s0, s1);

  return _mm_extract_epi16(s0, 0);
}
