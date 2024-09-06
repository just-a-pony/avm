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

#ifndef AOM_AOM_DSP_BITWRITER_H_
#define AOM_AOM_DSP_BITWRITER_H_

#include <assert.h>

#include "config/aom_config.h"

#include "aom_dsp/entenc.h"
#include "aom_dsp/prob.h"
#include "aom_dsp_common.h"
#include "aom_dsp/recenter.h"

#if CONFIG_RD_DEBUG
#include "av1/common/blockd.h"
#include "av1/encoder/cost.h"
#endif

#if CONFIG_BITSTREAM_DEBUG
#include "aom_util/debug_util.h"
#endif  // CONFIG_BITSTREAM_DEBUG

#ifdef __cplusplus
extern "C" {
#endif

struct aom_writer {
  unsigned int pos;
  uint8_t *buffer;
  od_ec_enc ec;
  uint8_t allow_update_cdf;
};

typedef struct aom_writer aom_writer;

typedef struct TOKEN_STATS {
  int cost;
#if CONFIG_RD_DEBUG
  int txb_coeff_cost_map[TXB_COEFF_COST_MAP_SIZE][TXB_COEFF_COST_MAP_SIZE];
#endif
} TOKEN_STATS;

static INLINE void init_token_stats(TOKEN_STATS *token_stats) {
#if CONFIG_RD_DEBUG
  int r, c;
  for (r = 0; r < TXB_COEFF_COST_MAP_SIZE; ++r) {
    for (c = 0; c < TXB_COEFF_COST_MAP_SIZE; ++c) {
      token_stats->txb_coeff_cost_map[r][c] = 0;
    }
  }
#endif
  token_stats->cost = 0;
}

void aom_start_encode(aom_writer *w, uint8_t *buffer);

int aom_stop_encode(aom_writer *w);

#if CONFIG_BITSTREAM_DEBUG
// Push a literal (one or more equi-probably symbols) into
// the bitstream debug queue, in the same order it is
// encoded into the stream (msb to lsb).
static INLINE void bitstream_queue_push_literal(int data, int bits) {
  aom_cdf_prob cdf[2] = { 128, 32767 };
  for (int bit = bits - 1; bit >= 0; bit--) {
    bitstream_queue_push(1 & (data >> bit), cdf, 2);
  }
}
#endif  // CONFIG_BITSTREAM_DEBUG

static INLINE void aom_write(aom_writer *w, int bit, int probability) {
  int p = (0x7FFFFF - (probability << 15) + probability) >> 8;
#if CONFIG_BITSTREAM_DEBUG
  aom_cdf_prob cdf[2] = { (aom_cdf_prob)p, 32767 };
  bitstream_queue_push(bit, cdf, 2);
#endif  // CONFIG_BITSTREAM_DEBUG

  od_ec_encode_bool_q15(&w->ec, bit, p);
}

static INLINE void aom_write_bit(aom_writer *w, int bit) {
#if CONFIG_BYPASS_IMPROVEMENT
#if CONFIG_BITSTREAM_DEBUG
  bitstream_queue_push_literal(bit, 1);
#endif  // CONFIG_BITSTREAM_DEBUG
  od_ec_encode_literal_bypass(&w->ec, bit, 1);
#else
  aom_write(w, bit, 128);  // aom_prob_half
#endif  // CONFIG_BYPASS_IMPROVEMENT
}

static INLINE void aom_write_literal(aom_writer *w, int data, int bits) {
#if CONFIG_BYPASS_IMPROVEMENT
#if CONFIG_BITSTREAM_DEBUG
  bitstream_queue_push_literal(data, bits);
#endif  // CONFIG_BITSTREAM_DEBUG
  int n;
  while (bits > 0) {
    n = bits >= 8 ? 8 : bits;
    od_ec_encode_literal_bypass(&w->ec, (data >> (bits - n)), n);
    bits -= n;
    data &= ((1 << bits) - 1);
  }
#else
  int bit;

  for (bit = bits - 1; bit >= 0; bit--) aom_write_bit(w, 1 & (data >> bit));
#endif  // CONFIG_BYPASS_IMPROVEMENT
}

static INLINE void aom_write_cdf(aom_writer *w, int symb,
                                 const aom_cdf_prob *cdf, int nsymbs) {
#if CONFIG_BITSTREAM_DEBUG
  bitstream_queue_push(symb, cdf, nsymbs);
#endif  // CONFIG_BITSTREAM_DEBUG

  od_ec_encode_cdf_q15(&w->ec, symb, cdf, nsymbs);
}

static INLINE void aom_write_symbol(aom_writer *w, int symb, aom_cdf_prob *cdf,
                                    int nsymbs) {
  aom_write_cdf(w, symb, cdf, nsymbs);
  if (w->allow_update_cdf) update_cdf(cdf, symb, nsymbs);
}

// Implements a code where a symbol with an alphabet size a power of 2 with
// nsymb_bits bits (with nsymb_bits >= 3), is coded by decomposing the symbol
// into 4 parts covering 1/8, 1/8, 1/4, 1/2 of the total number of symbols.
// The part is arithmetically coded using the provided cdf of size 4. The
// offset within each part is coded using fixed length binary codes with
// (nsymb_bits - 3), (nsymb_bits - 3), (nsymb_bits - 2) or (nsymb_bits - 1)
// bits, depending on the part.
//
static INLINE int symb_to_part(int symb, int nsymb_bits) {
  assert(nsymb_bits >= 3);
  int part_offs[4] = { 0, 1 << (nsymb_bits - 3), 1 << (nsymb_bits - 2),
                       1 << (nsymb_bits - 1) };
  if (symb < part_offs[1])
    return 0;
  else if (symb < part_offs[2])
    return 1;
  else if (symb < part_offs[3])
    return 2;
  else
    return 3;
}

static INLINE void aom_write_4part(aom_writer *w, int symb, aom_cdf_prob *cdf,
                                   int nsymb_bits) {
  assert(nsymb_bits >= 3);
  int part;
  int part_bits[4] = { (nsymb_bits - 3), (nsymb_bits - 3), (nsymb_bits - 2),
                       (nsymb_bits - 1) };
  int part_offs[4] = { 0, 1 << (nsymb_bits - 3), 1 << (nsymb_bits - 2),
                       1 << (nsymb_bits - 1) };
  if (symb < part_offs[1])
    part = 0;
  else if (symb < part_offs[2])
    part = 1;
  else if (symb < part_offs[3])
    part = 2;
  else
    part = 3;
  aom_write_symbol(w, part, cdf, 4);
  aom_write_literal(w, symb - part_offs[part], part_bits[part]);
}

// Implements a nsymb_bits bit 4-part code that codes a symbol symb given a
// reference ref_symb after recentering symb around ref_symb.
static INLINE void aom_write_4part_wref(aom_writer *w, int ref_symb, int symb,
                                        aom_cdf_prob *cdf, int nsymb_bits) {
  const int recentered_symb =
      recenter_finite_nonneg(1 << nsymb_bits, ref_symb, symb);
  aom_write_4part(w, recentered_symb, cdf, nsymb_bits);
}

static INLINE int64_t aom_count_4part(int symb, const int *part_cost,
                                      int nsymb_bits, int scale_shift) {
  assert(nsymb_bits >= 3);
  int part_bits[4] = { (nsymb_bits - 3), (nsymb_bits - 3), (nsymb_bits - 2),
                       (nsymb_bits - 1) };
  int part_offs[4] = { 0, 1 << (nsymb_bits - 3), 1 << (nsymb_bits - 2),
                       1 << (nsymb_bits - 1) };
  if (symb < part_offs[1])
    return part_cost[0] + (part_bits[0] << scale_shift);
  else if (symb < part_offs[2])
    return part_cost[1] + (part_bits[1] << scale_shift);
  else if (symb < part_offs[3])
    return part_cost[2] + (part_bits[2] << scale_shift);
  else
    return part_cost[3] + (part_bits[3] << scale_shift);
}

static INLINE int64_t aom_count_4part_wref(int ref_symb, int symb,
                                           const int *part_cost, int nsymb_bits,
                                           int scale_shift) {
  const int recentered_symb =
      recenter_finite_nonneg(1 << nsymb_bits, ref_symb, symb);
  return aom_count_4part(recentered_symb, part_cost, nsymb_bits, scale_shift);
}

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AOM_AOM_DSP_BITWRITER_H_
