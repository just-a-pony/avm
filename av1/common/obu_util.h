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
#ifndef AOM_AV1_COMMON_OBU_UTIL_H_
#define AOM_AV1_COMMON_OBU_UTIL_H_

#include "aom/aom_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  size_t size;  // Size (1 or 2 bytes) of the OBU header (including the
                // optional second byte) in the bitstream.
  int obu_extension_flag;
  OBU_TYPE type;
  int obu_tlayer_id;
  int obu_mlayer_id;  // same as spatial_layer_id in the old design
  int obu_xlayer_id;
} ObuHeader;

aom_codec_err_t aom_read_obu_header_and_size(const uint8_t *data,
                                             size_t bytes_available,
                                             int is_annexb,
                                             ObuHeader *obu_header,
                                             size_t *const payload_size,
                                             size_t *const bytes_read);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AOM_AV1_COMMON_OBU_UTIL_H_
