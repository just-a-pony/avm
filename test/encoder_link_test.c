/*
 * Copyright (c) 2025, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 3-Clause Clear License
 * and the Alliance for Open Media Patent License 1.0. If the BSD 3-Clause Clear
 * License was not distributed with this source code in the LICENSE file, you
 * can obtain it at aomedia.org/license/software-license/bsd-3-c-c/.  If the
 * Alliance for Open Media Patent License 1.0 was not distributed with this
 * source code in the PATENTS file, you can obtain it at
 * aomedia.org/license/patent-license/.
 */

// A program that calls three AVM public functions. It should only need to be
// linked with the AVM library.

#include <stdio.h>

#include "aom/aomcx.h"
#include "aom/aom_codec.h"
#include "aom/aom_encoder.h"

int main(void) {
  aom_codec_iface_t *iface = aom_codec_av1_cx();
  aom_codec_enc_cfg_t cfg;
  if (aom_codec_enc_config_default(iface, &cfg, AOM_USAGE_GOOD_QUALITY) !=
      AOM_CODEC_OK) {
    fprintf(stderr, "aom_codec_enc_config_default() failed\n");
    return 1;
  }
  aom_codec_ctx_t ctx;
  if (aom_codec_enc_init(&ctx, iface, &cfg, 0) != AOM_CODEC_OK) {
    fprintf(stderr, "aom_codec_enc_init() failed\n");
    return 1;
  }
  if (aom_codec_destroy(&ctx) != AOM_CODEC_OK) {
    fprintf(stderr, "aom_codec_destroy() failed\n");
    return 1;
  }

  printf("Hello, world!\n");
  return 0;
}
