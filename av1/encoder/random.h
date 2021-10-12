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

#ifndef AOM_AV1_ENCODER_RANDOM_H_
#define AOM_AV1_ENCODER_RANDOM_H_

#ifdef __cplusplus
extern "C" {
#endif

// Generate a random number in the range [0, 32768).
static INLINE unsigned int lcg_rand16(unsigned int *state) {
  *state = (unsigned int)(*state * 1103515245ULL + 12345);
  return *state / 65536 % 32768;
}

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AOM_AV1_ENCODER_RANDOM_H_
