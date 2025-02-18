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

#include "aom_ports/system_state.h"

#include "av1/encoder/bitstream.h"
#include "av1/encoder/encodeframe.h"
#include "av1/encoder/encoder.h"
#include "av1/encoder/encoder_alloc.h"
#include "av1/encoder/encodetxb.h"
#include "av1/encoder/encoder_utils.h"
#include "av1/encoder/grain_test_vectors.h"
#include "av1/encoder/mv_prec.h"
#include "av1/encoder/rc_utils.h"
#include "av1/encoder/rdopt.h"
#include "av1/encoder/segmentation.h"
#include "av1/encoder/superres_scale.h"
#if CONFIG_ENHANCED_FRAME_CONTEXT_INIT
#include "av1/encoder/encodeframe_utils.h"
#endif  // CONFIG_ENHANCED_FRAME_CONTEXT_INIT

#if CONFIG_TUNE_VMAF
#include "av1/encoder/tune_vmaf.h"
#endif

#define MIN_BOOST_COMBINE_FACTOR 4.0
#define MAX_BOOST_COMBINE_FACTOR 12.0
// TODO(urvang): Augment array for FLEX_PARTITION: used in speed >= 3.

const int default_tx_type_probs[FRAME_UPDATE_TYPES][TX_SIZES_ALL][TX_TYPES] = {
  { { 221, 189, 214, 292, 0, 0, 0, 0, 0, 2, 38, 68, 0, 0, 0, 0 },
    { 262, 203, 216, 239, 0, 0, 0, 0, 0, 1, 37, 66, 0, 0, 0, 0 },
    { 315, 231, 239, 226, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 222, 188, 214, 287, 0, 0, 0, 0, 0, 2, 50, 61, 0, 0, 0, 0 },
    { 256, 182, 205, 282, 0, 0, 0, 0, 0, 2, 21, 76, 0, 0, 0, 0 },
    { 281, 214, 217, 222, 0, 0, 0, 0, 0, 1, 48, 41, 0, 0, 0, 0 },
    { 263, 194, 225, 225, 0, 0, 0, 0, 0, 2, 15, 100, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 170, 192, 242, 293, 0, 0, 0, 0, 0, 1, 68, 58, 0, 0, 0, 0 },
    { 199, 210, 213, 291, 0, 0, 0, 0, 0, 1, 14, 96, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
  { { 106, 69, 107, 278, 9, 15, 20, 45, 49, 23, 23, 88, 36, 74, 25, 57 },
    { 105, 72, 81, 98, 45, 49, 47, 50, 56, 72, 30, 81, 33, 95, 27, 83 },
    { 211, 105, 109, 120, 57, 62, 43, 49, 52, 58, 42, 116, 0, 0, 0, 0 },
    { 1008, 0, 0, 0, 0, 0, 0, 0, 0, 16, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 131, 57, 98, 172, 19, 40, 37, 64, 69, 22, 41, 52, 51, 77, 35, 59 },
    { 176, 83, 93, 202, 22, 24, 28, 47, 50, 16, 12, 93, 26, 76, 17, 59 },
    { 136, 72, 89, 95, 46, 59, 47, 56, 61, 68, 35, 51, 32, 82, 26, 69 },
    { 122, 80, 87, 105, 49, 47, 46, 46, 57, 52, 13, 90, 19, 103, 15, 93 },
    { 1009, 0, 0, 0, 0, 0, 0, 0, 0, 15, 0, 0, 0, 0, 0, 0 },
    { 1011, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 202, 20, 84, 114, 14, 60, 41, 79, 99, 21, 41, 15, 50, 84, 34, 66 },
    { 196, 44, 23, 72, 30, 22, 28, 57, 67, 13, 4, 165, 15, 148, 9, 131 },
    { 882, 0, 0, 0, 0, 0, 0, 0, 0, 142, 0, 0, 0, 0, 0, 0 },
    { 840, 0, 0, 0, 0, 0, 0, 0, 0, 184, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
  { { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 } },
  { { 213, 110, 141, 269, 12, 16, 15, 19, 21, 11, 38, 68, 22, 29, 16, 24 },
    { 216, 119, 128, 143, 38, 41, 26, 30, 31, 30, 42, 70, 23, 36, 19, 32 },
    { 367, 149, 154, 154, 38, 35, 17, 21, 21, 10, 22, 36, 0, 0, 0, 0 },
    { 1022, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 219, 96, 127, 191, 21, 40, 25, 32, 34, 18, 45, 45, 33, 39, 26, 33 },
    { 296, 99, 122, 198, 23, 21, 19, 24, 25, 13, 20, 64, 23, 32, 18, 27 },
    { 275, 128, 142, 143, 35, 48, 23, 30, 29, 18, 42, 36, 18, 23, 14, 20 },
    { 239, 132, 166, 175, 36, 27, 19, 21, 24, 14, 13, 85, 9, 31, 8, 25 },
    { 1022, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0 },
    { 1022, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 309, 25, 79, 59, 25, 80, 34, 53, 61, 25, 49, 23, 43, 64, 36, 59 },
    { 270, 57, 40, 54, 50, 42, 41, 53, 56, 28, 17, 81, 45, 86, 34, 70 },
    { 1005, 0, 0, 0, 0, 0, 0, 0, 0, 19, 0, 0, 0, 0, 0, 0 },
    { 992, 0, 0, 0, 0, 0, 0, 0, 0, 32, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
  { { 133, 63, 55, 83, 57, 87, 58, 72, 68, 16, 24, 35, 29, 105, 25, 114 },
    { 131, 75, 74, 60, 71, 77, 65, 66, 73, 33, 21, 79, 20, 83, 18, 78 },
    { 276, 95, 82, 58, 86, 93, 63, 60, 64, 17, 38, 92, 0, 0, 0, 0 },
    { 1006, 0, 0, 0, 0, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 147, 49, 75, 78, 50, 97, 60, 67, 76, 17, 42, 35, 31, 93, 27, 80 },
    { 157, 49, 58, 75, 61, 52, 56, 67, 69, 12, 15, 79, 24, 119, 11, 120 },
    { 178, 69, 83, 77, 69, 85, 72, 77, 77, 20, 35, 40, 25, 48, 23, 46 },
    { 174, 55, 64, 57, 73, 68, 62, 61, 75, 15, 12, 90, 17, 99, 16, 86 },
    { 1008, 0, 0, 0, 0, 0, 0, 0, 0, 16, 0, 0, 0, 0, 0, 0 },
    { 1018, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 266, 31, 63, 64, 21, 52, 39, 54, 63, 30, 52, 31, 48, 89, 46, 75 },
    { 272, 26, 32, 44, 29, 31, 32, 53, 51, 13, 13, 88, 22, 153, 16, 149 },
    { 923, 0, 0, 0, 0, 0, 0, 0, 0, 101, 0, 0, 0, 0, 0, 0 },
    { 969, 0, 0, 0, 0, 0, 0, 0, 0, 55, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
  { { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 } },
  { { 158, 92, 125, 298, 12, 15, 20, 29, 31, 12, 29, 67, 34, 44, 23, 35 },
    { 147, 94, 103, 123, 45, 48, 38, 41, 46, 48, 37, 78, 33, 63, 27, 53 },
    { 268, 126, 125, 136, 54, 53, 31, 38, 38, 33, 35, 87, 0, 0, 0, 0 },
    { 1018, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 159, 72, 103, 194, 20, 35, 37, 50, 56, 21, 39, 40, 51, 61, 38, 48 },
    { 259, 86, 95, 188, 32, 20, 25, 34, 37, 13, 12, 85, 25, 53, 17, 43 },
    { 189, 99, 113, 123, 45, 59, 37, 46, 48, 44, 39, 41, 31, 47, 26, 37 },
    { 175, 110, 113, 128, 58, 38, 33, 33, 43, 29, 13, 100, 14, 68, 12, 57 },
    { 1017, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0 },
    { 1019, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 208, 22, 84, 101, 21, 59, 44, 70, 90, 25, 59, 13, 64, 67, 49, 48 },
    { 277, 52, 32, 63, 43, 26, 33, 48, 54, 11, 6, 130, 18, 119, 11, 101 },
    { 963, 0, 0, 0, 0, 0, 0, 0, 0, 61, 0, 0, 0, 0, 0, 0 },
    { 979, 0, 0, 0, 0, 0, 0, 0, 0, 45, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }
};

/* clang-format off */
// TODO(urvang): update for CONFIG_EXT_RECUR_PARTITIONS. Used for speed >= 1.
const int default_obmc_probs[FRAME_UPDATE_TYPES][BLOCK_SIZES_ALL] = { {
    // BLOCK_4X4
    0,
    // BLOCK_4X8 .. BLOCK_8X8
    0, 0, 0,
    // BLOCK_8X16 .. BLOCK_16X16
    0, 0, 0,
    // BLOCK_16X32 .. BLOCK_32X32
    0, 0, 0,
    // BLOCK_32X64 .. BLOCK_64X64
    0, 0, 0,
    // BLOCK_64X128 .. BLOCK_128X128
    0, 0, 0,
#if CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_128X256 .. BLOCK_256X256
    0, 0, 0,
#endif  // CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_4X16, BLOCK_16X4
    0, 0,
    // BLOCK_8X32, BLOCK_32X8
    0, 0,
    // BLOCK_16X64, BLOCK_64X16
    0, 0
  }, {
    // BLOCK_4X4
    0,
    // BLOCK_4X8 .. BLOCK_8X8
    0,  0,  106,
    // BLOCK_8X16 .. BLOCK_16X16
    90, 90, 97,
    // BLOCK_16X32 .. BLOCK_32X32
    67, 59, 70,
    // BLOCK_32X64 .. BLOCK_64X64
    28, 30, 38,
    // BLOCK_64X128 .. BLOCK_128X128
    16, 16,  16,
#if CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_128X256 .. BLOCK_256X256
    16, 16,  16,
#endif  // CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_4X16, BLOCK_16X4
    0,  0,
    // BLOCK_8X32, BLOCK_32X8
    44, 50,
    // BLOCK_16X64, BLOCK_64X16
    26, 25
  }, {
    // BLOCK_4X4
    0,
    // BLOCK_4X8 .. BLOCK_8X8
    0, 0, 0,
    // BLOCK_8X16 .. BLOCK_16X16
    0, 0, 0,
    // BLOCK_16X32 .. BLOCK_32X32
    0, 0, 0,
    // BLOCK_32X64 .. BLOCK_64X64
    0, 0, 0,
    // BLOCK_64X128 .. BLOCK_128X128
    0, 0, 0,
#if CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_128X256 .. BLOCK_256X256
    0, 0, 0,
#endif  // CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_4X16, BLOCK_16X4
    0, 0,
    // BLOCK_8X32, BLOCK_32X8
    0, 0,
    // BLOCK_16X64, BLOCK_64X16
    0, 0
  }, {
    // BLOCK_4X4
    0,
    // BLOCK_4X8 .. BLOCK_8X8
    0,  0,  98,
    // BLOCK_8X16 .. BLOCK_16X16
    93, 97, 68,
    // BLOCK_16X32 .. BLOCK_32X32
    82, 85, 33,
    // BLOCK_32X64 .. BLOCK_64X64
    30, 33, 16,
    // BLOCK_64X128 .. BLOCK_128X128
    16, 16, 16,
#if CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_128X256 .. BLOCK_256X256
    16, 16, 16,
#endif  // CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_4X16, BLOCK_16X4
    0,  0,
    // BLOCK_8X32, BLOCK_32X8
    43, 37,
    // BLOCK_16X64, BLOCK_64X16
    26, 16
  }, {
    // BLOCK_4X4
    0,
    // BLOCK_4X8 .. BLOCK_8X8
    0,  0,  91,
    // BLOCK_8X16 .. BLOCK_16X16
    80, 76, 78,
    // BLOCK_16X32 .. BLOCK_32X32
    55, 49, 24,
    // BLOCK_32X64 .. BLOCK_64X64
    16, 16, 16,
    // BLOCK_64X128 .. BLOCK_128X128
    16, 16, 16,
    // BLOCK_128X256 .. BLOCK_256X256
#if CONFIG_EXT_RECUR_PARTITIONS
    16, 16, 16,
#endif  // CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_4X16, BLOCK_16X4
    0,  0,
    // BLOCK_8X32, BLOCK_32X8
    29, 45,
    // BLOCK_16X64, BLOCK_64X16
    16, 38
  }, {
    // BLOCK_4X4
    0,
    // BLOCK_4X8 .. BLOCK_8X8
    0, 0, 0,
    // BLOCK_8X16 .. BLOCK_16X16
    0, 0, 0,
    // BLOCK_16X32 .. BLOCK_32X32
    0, 0, 0,
    // BLOCK_32X64 .. BLOCK_64X64
    0, 0, 0,
    // BLOCK_64X128 .. BLOCK_128X128
    0, 0, 0,
#if CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_128X256 .. BLOCK_256X256
    0, 0, 0,
#endif  // CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_4X16, BLOCK_16X4
    0, 0,
    // BLOCK_8X32, BLOCK_32X8
    0, 0,
    // BLOCK_16X64, BLOCK_64X16
    0, 0
  }, {
    // BLOCK_4X4
    0,
    // BLOCK_4X8 .. BLOCK_8X8
    0,  0,  103,
    // BLOCK_8X16 .. BLOCK_16X16
    89, 89, 89,
    // BLOCK_16X32 .. BLOCK_32X32
    62, 63, 76,
    // BLOCK_32X64 .. BLOCK_64X64
    34, 35, 32,
    // BLOCK_64X128 .. BLOCK_128X128
    19, 16,  16,
#if CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_128X256 .. BLOCK_256X256
    19, 16,  16,
#endif  // CONFIG_EXT_RECUR_PARTITIONS
    // BLOCK_4X16, BLOCK_16X4
    0,  0,
    // BLOCK_8X32, BLOCK_32X8
    49, 55,
    // BLOCK_16X64, BLOCK_64X16
    29, 19
  }
};
/* clang-format on */

const int default_warped_probs[FRAME_UPDATE_TYPES] = { 64, 64, 64, 64,
                                                       64, 64, 64 };

// TODO(yunqing): the default probs can be trained later from better
// performance.
const int default_switchable_interp_probs[FRAME_UPDATE_TYPES]
                                         [SWITCHABLE_FILTER_CONTEXTS]
                                         [SWITCHABLE_FILTERS] = {
                                           { { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 } },
                                           { { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 } },
                                           { { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 } },
                                           { { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 } },
                                           { { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 } },
                                           { { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 } },
                                           { { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 },
                                             { 512, 512, 512 } }
                                         };

void av1_apply_active_map(AV1_COMP *cpi) {
  struct segmentation *const seg = &cpi->common.seg;
  unsigned char *const seg_map = cpi->enc_seg.map;
  const unsigned char *const active_map = cpi->active_map.map;
  int i;

  assert(AM_SEGMENT_ID_ACTIVE == CR_SEGMENT_ID_BASE);

  if (frame_is_intra_only(&cpi->common)) {
    cpi->active_map.enabled = 0;
    cpi->active_map.update = 1;
  }

  if (cpi->active_map.update) {
    if (cpi->active_map.enabled) {
      for (i = 0;
           i < cpi->common.mi_params.mi_rows * cpi->common.mi_params.mi_cols;
           ++i)
        if (seg_map[i] == AM_SEGMENT_ID_ACTIVE) seg_map[i] = active_map[i];
      av1_enable_segmentation(seg);
      av1_enable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_SKIP);
      av1_enable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_Y_H);
      av1_enable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_Y_V);
      av1_enable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_U);
      av1_enable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_V);

      av1_set_segdata(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_Y_H,
                      -MAX_LOOP_FILTER);
      av1_set_segdata(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_Y_V,
                      -MAX_LOOP_FILTER);
      av1_set_segdata(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_U,
                      -MAX_LOOP_FILTER);
      av1_set_segdata(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_V,
                      -MAX_LOOP_FILTER);
    } else {
      av1_disable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_SKIP);
      av1_disable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_Y_H);
      av1_disable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_Y_V);
      av1_disable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_U);
      av1_disable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF_V);
      if (seg->enabled) {
        seg->update_data = 1;
        seg->update_map = 1;
      }
    }
    cpi->active_map.update = 0;
  }
}

static void process_tpl_stats_frame(AV1_COMP *cpi) {
  const GF_GROUP *const gf_group = &cpi->gf_group;
  AV1_COMMON *const cm = &cpi->common;

  assert(IMPLIES(gf_group->size > 0, gf_group->index < gf_group->size));

  const int tpl_idx = gf_group->index;
  TplParams *const tpl_data = &cpi->tpl_data;
  TplDepFrame *tpl_frame = &tpl_data->tpl_frame[tpl_idx];
  TplDepStats *tpl_stats = tpl_frame->tpl_stats_ptr;

  if (tpl_frame->is_valid) {
    int tpl_stride = tpl_frame->stride;
    int64_t intra_cost_base = 0;
    int64_t mc_dep_cost_base = 0;
    const int step = 1 << tpl_data->tpl_stats_block_mis_log2;
    const int row_step = step;
    const int col_step_sr =
        coded_to_superres_mi(step, cm->superres_scale_denominator);
    const int mi_cols_sr = av1_pixels_to_mi(cm->superres_upscaled_width);

    for (int row = 0; row < cm->mi_params.mi_rows; row += row_step) {
      for (int col = 0; col < mi_cols_sr; col += col_step_sr) {
        TplDepStats *this_stats = &tpl_stats[av1_tpl_ptr_pos(
            row, col, tpl_stride, tpl_data->tpl_stats_block_mis_log2)];
        int64_t mc_dep_delta =
            RDCOST(tpl_frame->base_rdmult, this_stats->mc_dep_rate,
                   this_stats->mc_dep_dist);
        intra_cost_base += (this_stats->recrf_dist << RDDIV_BITS);
        mc_dep_cost_base +=
            (this_stats->recrf_dist << RDDIV_BITS) + mc_dep_delta;
      }
    }

    if (mc_dep_cost_base == 0) {
      tpl_frame->is_valid = 0;
    } else {
      aom_clear_system_state();
      cpi->rd.r0 = (double)intra_cost_base / mc_dep_cost_base;
      if (is_frame_tpl_eligible(gf_group, gf_group->index)) {
        if (cpi->lap_enabled) {
          double min_boost_factor = sqrt(cpi->rc.baseline_gf_interval);
          const int gfu_boost = get_gfu_boost_from_r0_lap(
              min_boost_factor, MAX_GFUBOOST_FACTOR, cpi->rd.r0,
              cpi->rc.num_stats_required_for_gfu_boost);
          // printf("old boost %d new boost %d\n", cpi->rc.gfu_boost,
          //        gfu_boost);
          cpi->rc.gfu_boost = combine_prior_with_tpl_boost(
              min_boost_factor, MAX_BOOST_COMBINE_FACTOR, cpi->rc.gfu_boost,
              gfu_boost, cpi->rc.num_stats_used_for_gfu_boost);
        } else {
          const int gfu_boost = (int)(200.0 / cpi->rd.r0);
          cpi->rc.gfu_boost = combine_prior_with_tpl_boost(
              MIN_BOOST_COMBINE_FACTOR, MAX_BOOST_COMBINE_FACTOR,
              cpi->rc.gfu_boost, gfu_boost, cpi->rc.frames_to_key);
        }
      }
      aom_clear_system_state();
    }
  }
}

void av1_set_size_dependent_vars(AV1_COMP *cpi, int *q, int *bottom_index,
                                 int *top_index) {
  AV1_COMMON *const cm = &cpi->common;

  // Setup variables that depend on the dimensions of the frame.
  av1_set_speed_features_framesize_dependent(cpi, cpi->speed);

  GF_GROUP *gf_group = &cpi->gf_group;
  if (cpi->oxcf.algo_cfg.enable_tpl_model &&
      is_frame_tpl_eligible(gf_group, gf_group->index)) {
    process_tpl_stats_frame(cpi);
    av1_tpl_rdmult_setup(cpi);
  }

  // Decide q and q bounds.
  *q = av1_rc_pick_q_and_bounds(cpi, &cpi->rc, cm->width, cm->height,
                                cpi->gf_group.index, bottom_index, top_index);
}

static void reset_film_grain_chroma_params(aom_film_grain_t *pars) {
  pars->num_cr_points = 0;
  pars->cr_mult = 0;
  pars->cr_luma_mult = 0;
  memset(pars->scaling_points_cr, 0, sizeof(pars->scaling_points_cr));
  memset(pars->ar_coeffs_cr, 0, sizeof(pars->ar_coeffs_cr));
  pars->num_cb_points = 0;
  pars->cb_mult = 0;
  pars->cb_luma_mult = 0;
  pars->chroma_scaling_from_luma = 0;
  memset(pars->scaling_points_cb, 0, sizeof(pars->scaling_points_cb));
  memset(pars->ar_coeffs_cb, 0, sizeof(pars->ar_coeffs_cb));
}

void av1_update_film_grain_parameters(struct AV1_COMP *cpi,
                                      const AV1EncoderConfig *oxcf) {
  AV1_COMMON *const cm = &cpi->common;
  cpi->oxcf = *oxcf;
  const TuneCfg *const tune_cfg = &oxcf->tune_cfg;

  if (cpi->film_grain_table) {
    aom_film_grain_table_free(cpi->film_grain_table);
    aom_free(cpi->film_grain_table);
    cpi->film_grain_table = NULL;
  }

  if (tune_cfg->film_grain_test_vector) {
    cm->seq_params.film_grain_params_present = 1;
    if (cm->current_frame.frame_type == KEY_FRAME) {
      memcpy(&cm->film_grain_params,
             film_grain_test_vectors + tune_cfg->film_grain_test_vector - 1,
             sizeof(cm->film_grain_params));
      if (oxcf->tool_cfg.enable_monochrome)
        reset_film_grain_chroma_params(&cm->film_grain_params);
      cm->film_grain_params.bit_depth = cm->seq_params.bit_depth;
      if (cm->seq_params.color_range == AOM_CR_FULL_RANGE) {
        cm->film_grain_params.clip_to_restricted_range = 0;
      }
    }
  } else if (tune_cfg->film_grain_table_filename) {
    cm->seq_params.film_grain_params_present = 1;

    cpi->film_grain_table = aom_malloc(sizeof(*cpi->film_grain_table));
    memset(cpi->film_grain_table, 0, sizeof(aom_film_grain_table_t));

    aom_film_grain_table_read(cpi->film_grain_table,
                              tune_cfg->film_grain_table_filename, &cm->error);
  } else {
#if CONFIG_DENOISE
    cm->seq_params.film_grain_params_present = (cpi->oxcf.noise_level > 0);
#else
    cm->seq_params.film_grain_params_present = 0;
#endif
    memset(&cm->film_grain_params, 0, sizeof(cm->film_grain_params));
  }
}

void av1_scale_references(AV1_COMP *cpi, const InterpFilter filter,
                          const int phase, const int use_optimized_scaler) {
  AV1_COMMON *cm = &cpi->common;
  const int num_planes = av1_num_planes(cm);
  MV_REFERENCE_FRAME ref_frame;

  for (ref_frame = 0; ref_frame < INTER_REFS_PER_FRAME; ++ref_frame) {
    // Need to convert from AOM_REFFRAME to index into ref_mask (subtract 1).
    if (cm->ref_frame_flags & (1 << ref_frame)) {
      BufferPool *const pool = cm->buffer_pool;
      const YV12_BUFFER_CONFIG *const ref =
          get_ref_frame_yv12_buf(cm, ref_frame);

      if (ref == NULL) {
        cpi->scaled_ref_buf[ref_frame] = NULL;
        continue;
      }

      if (ref->y_crop_width != cm->width || ref->y_crop_height != cm->height) {
        // Replace the reference buffer with a copy having a thicker border,
        // if the reference buffer is higher resolution than the current
        // frame, and the border is thin.
        if ((ref->y_crop_width > cm->width ||
             ref->y_crop_height > cm->height) &&
            ref->border < AOM_BORDER_IN_PIXELS) {
          RefCntBuffer *ref_fb = get_ref_frame_buf(cm, ref_frame);
          if (aom_yv12_realloc_with_new_border(
                  &ref_fb->buf, AOM_BORDER_IN_PIXELS,
                  cm->features.byte_alignment, num_planes,
                  cpi->alloc_pyramid) != 0) {
            aom_internal_error(&cm->error, AOM_CODEC_MEM_ERROR,
                               "Failed to allocate frame buffer");
          }
        }
        int force_scaling = 0;
        RefCntBuffer *new_fb = cpi->scaled_ref_buf[ref_frame];
        if (new_fb == NULL) {
          const int new_fb_idx = get_free_fb(cm);
          if (new_fb_idx == INVALID_IDX) {
            aom_internal_error(&cm->error, AOM_CODEC_MEM_ERROR,
                               "Unable to find free frame buffer");
          }
          force_scaling = 1;
          new_fb = &pool->frame_bufs[new_fb_idx];
        }

        if (force_scaling || new_fb->buf.y_crop_width != cm->width ||
            new_fb->buf.y_crop_height != cm->height) {
          if (aom_realloc_frame_buffer(
                  &new_fb->buf, cm->width, cm->height,
                  cm->seq_params.subsampling_x, cm->seq_params.subsampling_y,
                  AOM_BORDER_IN_PIXELS, cm->features.byte_alignment, NULL, NULL,
                  NULL, cpi->alloc_pyramid)) {
            if (force_scaling) {
              // Release the reference acquired in the get_free_fb() call above.
              --new_fb->ref_count;
            }
            aom_internal_error(&cm->error, AOM_CODEC_MEM_ERROR,
                               "Failed to allocate frame buffer");
          }
          if (use_optimized_scaler && cm->seq_params.bit_depth == AOM_BITS_8)
            av1_resize_and_extend_frame(ref, &new_fb->buf, filter, phase,
                                        num_planes);
          else
            av1_resize_and_extend_frame_nonnormative(
                ref, &new_fb->buf, (int)cm->seq_params.bit_depth, num_planes);
          cpi->scaled_ref_buf[ref_frame] = new_fb;
          alloc_frame_mvs(cm, new_fb);
        }
      } else {
        RefCntBuffer *buf = get_ref_frame_buf(cm, ref_frame);
        buf->buf.y_crop_width = ref->y_crop_width;
        buf->buf.y_crop_height = ref->y_crop_height;
        cpi->scaled_ref_buf[ref_frame] = buf;
        ++buf->ref_count;
      }
    } else {
      if (!has_no_stats_stage(cpi)) cpi->scaled_ref_buf[ref_frame] = NULL;
    }
  }
}

BLOCK_SIZE av1_select_sb_size(const AV1_COMP *const cpi) {
  const AV1_COMMON *const cm = &cpi->common;
  const AV1EncoderConfig *const oxcf = &cpi->oxcf;

  if (oxcf->tool_cfg.superblock_size == AOM_SUPERBLOCK_SIZE_64X64)
    return BLOCK_64X64;
  if (oxcf->tool_cfg.superblock_size == AOM_SUPERBLOCK_SIZE_128X128)
    return BLOCK_128X128;

  assert(oxcf->tool_cfg.superblock_size == AOM_SUPERBLOCK_SIZE_DYNAMIC);

  if (oxcf->resize_cfg.resize_mode != RESIZE_NONE) {
    // Use the configured size (top resolution) for spatial layers or
    // on resize.
    return AOMMIN(oxcf->frm_dim_cfg.width, oxcf->frm_dim_cfg.height) > 480
               ? BLOCK_128X128
               : BLOCK_64X64;
  }

  // TODO(any): Possibly could improve this with a heuristic.
  // When superres / resize is on, 'cm->width / height' can change between
  // calls, so we don't apply this heuristic there.
  // Things break if superblock size changes between the first pass and second
  // pass encoding, which is why this heuristic is not configured as a
  // speed-feature.
  if (oxcf->superres_cfg.superres_mode == AOM_SUPERRES_NONE &&
      oxcf->resize_cfg.resize_mode == RESIZE_NONE && oxcf->speed > 1) {
    return AOMMIN(cm->width, cm->height) > 480 ? BLOCK_128X128 : BLOCK_64X64;
  }
#if CONFIG_EXT_RECUR_PARTITIONS
  return AOMMIN(oxcf->frm_dim_cfg.width, oxcf->frm_dim_cfg.height) >= 720
             ? BLOCK_256X256
             : BLOCK_128X128;
#else
  return BLOCK_128X128;
#endif  // CONFIG_EXT_RECUR_PARTITIONS
}

void reallocate_sb_size_dependent_buffers(AV1_COMP *cpi) {
  // Note: this is heavier than it needs to be. We can avoid reallocating some
  // of the buffers.
  AV1_COMMON *const cm = &cpi->common;
  const int num_planes = av1_num_planes(cm);

  av1_set_tile_info(cm, &cpi->oxcf.tile_cfg);
  av1_free_context_buffers(cm);
  av1_free_shared_coeff_buffer(&cpi->td.shared_coeff_buf);
  av1_free_sms_tree(&cpi->td);
#if CONFIG_EXT_RECUR_PARTITIONS
  av1_free_sms_bufs(&cpi->td);
#endif  // CONFIG_EXT_RECUR_PARTITIONS
  av1_free_pmc(cpi->td.firstpass_ctx, num_planes);
  cpi->td.firstpass_ctx = NULL;
  alloc_compressor_data(cpi);
  if (av1_alloc_above_context_buffers(&cm->above_contexts, cm->tiles.rows,
                                      cm->mi_params.mi_cols, num_planes)) {
    aom_internal_error(&cm->error, AOM_CODEC_MEM_ERROR,
                       "Failed to allocate context buffers");
  }

  const int old_restoration_unit_size = cm->rst_info[0].restoration_unit_size;

  const SequenceHeader *const seq_params = &cm->seq_params;
  const int frame_width = cm->superres_upscaled_width;
  const int frame_height = cm->superres_upscaled_height;

  set_restoration_unit_size(frame_width, frame_height,
                            seq_params->subsampling_x,
                            seq_params->subsampling_y, cm->rst_info);
  if (old_restoration_unit_size != cm->rst_info[0].restoration_unit_size) {
    for (int i = 0; i < num_planes; ++i)
      cm->rst_info[i].frame_restoration_type = RESTORE_NONE;

    av1_alloc_restoration_buffers(cm);
  }
}

void av1_setup_frame(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  // Set up entropy context depending on frame type. The decoder mandates
  // the use of the default context, index 0, for keyframes and inter
  // frames where the error_resilient_mode or intra_only flag is set. For
  // other inter-frames the encoder currently uses only two contexts;
  // context 1 for ALTREF frames and context 0 for the others.

  if (frame_is_intra_only(cm) || cm->features.error_resilient_mode ||
      cpi->ext_flags.use_primary_ref_none) {
    av1_setup_past_independence(cm);
  }

  const BLOCK_SIZE old_sb_size = cm->sb_size;
  if ((cm->current_frame.frame_type == KEY_FRAME && cm->show_frame) ||
      frame_is_sframe(cm)) {
    if (!cpi->seq_params_locked) {
      set_sb_size(cm, av1_select_sb_size(cpi));
    }
  } else {
#if CONFIG_PRIMARY_REF_FRAME_OPT
    const RefCntBuffer *const primary_ref_buf =
        get_primary_ref_frame_buf(cm, cm->features.primary_ref_frame);
#else
    const RefCntBuffer *const primary_ref_buf = get_primary_ref_frame_buf(cm);
#endif  // CONFIG_PRIMARY_REF_FRAME_OPT
    if (primary_ref_buf == NULL) {
      av1_setup_past_independence(cm);
      cm->seg.update_map = 1;
      cm->seg.update_data = 1;
    } else {
      *cm->fc = primary_ref_buf->frame_context;
#if CONFIG_ENHANCED_FRAME_CONTEXT_INIT
      const int ref_frame_used = (cm->features.primary_ref_frame ==
                                  cm->features.derived_primary_ref_frame)
                                     ? cm->features.derived_secondary_ref_frame
                                     : cm->features.derived_primary_ref_frame;
      const int map_idx = get_ref_frame_map_idx(cm, ref_frame_used);
      if ((map_idx != INVALID_IDX) &&
          (ref_frame_used != cm->features.primary_ref_frame) &&
          (cm->seq_params.enable_avg_cdf && !cm->seq_params.avg_cdf_type) &&
          !(cm->features.error_resilient_mode || frame_is_sframe(cm)) &&
          (ref_frame_used != PRIMARY_REF_NONE)) {
        av1_avg_cdf_symbols(cm->fc, &cm->ref_frame_map[map_idx]->frame_context,
                            AVG_CDF_WEIGHT_PRIMARY, AVG_CDF_WEIGHT_NON_PRIMARY);
      }
#endif  // CONFIG_ENHANCED_FRAME_CONTEXT_INIT
    }
  }
  av1_set_frame_sb_size(cm, cm->seq_params.sb_size);
#if CONFIG_EXT_RECUR_PARTITIONS
  cpi->td.sb_size = cm->sb_size;
#endif  // CONFIG_EXT_RECUR_PARTITIONS
  av1_set_tile_info(cm, &cpi->oxcf.tile_cfg);
  if (cm->sb_size != old_sb_size) {
    // Reallocate sb_size-dependent buffers if the sb_size has changed.
    reallocate_sb_size_dependent_buffers(cpi);
  } else if (cpi->alloc_width < cm->width || cpi->alloc_height < cm->height) {
    av1_free_context_buffers(cm);
    av1_free_shared_coeff_buffer(&cpi->td.shared_coeff_buf);
    av1_free_sms_tree(&cpi->td);
#if CONFIG_EXT_RECUR_PARTITIONS
    av1_free_sms_bufs(&cpi->td);
#endif  // CONFIG_EXT_RECUR_PARTITIONS
    av1_free_pmc(cpi->td.firstpass_ctx, av1_num_planes(cm));
    cpi->td.firstpass_ctx = NULL;
    alloc_compressor_data(cpi);
    if (av1_alloc_above_context_buffers(&cm->above_contexts, cm->tiles.rows,
                                        cm->mi_params.mi_cols,
                                        av1_num_planes(cm))) {
      aom_internal_error(&cm->error, AOM_CODEC_MEM_ERROR,
                         "Failed to allocate context buffers");
    }
  }

  av1_zero(cm->cur_frame->interp_filter_selected);
#if CONFIG_PRIMARY_REF_FRAME_OPT
  cm->prev_frame =
      get_primary_ref_frame_buf(cm, cm->features.derived_primary_ref_frame);
#else
  cm->prev_frame = get_primary_ref_frame_buf(cm);
#endif  // CONFIG_PRIMARY_REF_FRAME_OPT
  cpi->vaq_refresh = 0;
}

#if CONFIG_NEW_TX_PARTITION
#define STRICT_PSNR_DIFF_THRESH 0.88
#else
#define STRICT_PSNR_DIFF_THRESH 0.9
#endif  // CONFIG_NEW_TX_PARTITION

// Encode key frame with/without screen content tools to determine whether
// screen content tools should be enabled for this key frame group or not.
// The first encoding is without screen content tools.
// The second encoding is with screen content tools.
// We compare the psnr and frame size to make the decision.
static void screen_content_tools_determination(
    AV1_COMP *cpi, const int allow_screen_content_tools_orig_decision,
    const int allow_intrabc_orig_decision,
    const int is_screen_content_type_orig_decision, const int pass,
    int *projected_size_pass, PSNR_STATS *psnr) {
  AV1_COMMON *const cm = &cpi->common;
  FeatureFlags *const features = &cm->features;
  projected_size_pass[pass] = cpi->rc.projected_frame_size;

  const uint32_t in_bit_depth = cpi->oxcf.input_cfg.input_bit_depth;
  const uint32_t bit_depth = cpi->td.mb.e_mbd.bd;
  aom_calc_highbd_psnr(cpi->source, &cpi->common.cur_frame->buf, &psnr[pass],
                       bit_depth, in_bit_depth);

  if (pass != 1) return;

  const bool use_hbd_psnr = (cpi->b_calculate_psnr == 2);
  const double psnr_diff = use_hbd_psnr
                               ? psnr[1].psnr_hbd[0] - psnr[0].psnr_hbd[0]
                               : psnr[1].psnr[0] - psnr[0].psnr[0];
#if CONFIG_SCC_DETERMINATION
  // Calculate % of palette mode to be chosen in a frame from mode decision.
  const double palette_ratio =
      (double)cpi->palette_pixel_num / (double)(cm->height * cm->width);
  const int psnr_diff_is_large = (psnr_diff > STRICT_PSNR_DIFF_THRESH);
  // Note: These two conditions are intentionally separated because combining
  // them can cause division by zero under some compiler configurations (e.g.
  // clang-16 + address sanitizer) when palette_ratio == 0.0.
  int ratio_is_large = (palette_ratio >= 0.0001);
  if (ratio_is_large) {
    ratio_is_large = ((psnr_diff / palette_ratio) > 4);
  }
  const int is_sc_encoding_much_better = (psnr_diff_is_large || ratio_is_large);

  // the following two flags are used to determine if we enable intrabc or not.
  // Since intrabc is an expensive tool, we raise the threshold of
  // palette_ratio.
  // Note: these two conditions are intentionally separated (see note above).
  int ratio_is_large_2 = (palette_ratio >= 0.25);
  if (ratio_is_large_2) {
    ratio_is_large_2 = ((psnr_diff / palette_ratio) > 4);
  }
  const int is_sc_encoding_much_better_2 =
      (psnr_diff_is_large || ratio_is_large_2);
#else
  const int is_sc_encoding_much_better = psnr_diff > STRICT_PSNR_DIFF_THRESH;
#endif  // CONFIG_SCC_DETERMINATION
  if (is_sc_encoding_much_better) {
    // Use screen content tools, if we get coding gain.
    features->allow_screen_content_tools = 1;
#if CONFIG_SCC_DETERMINATION
    features->allow_intrabc =
        (is_sc_encoding_much_better_2 || cpi->intrabc_used ||
         allow_intrabc_orig_decision);
#else
    features->allow_intrabc = cpi->intrabc_used;
#endif  // CONFIG_SCC_DETERMINATION
    cpi->is_screen_content_type = 1;
  } else {
    // Use original screen content decision.
    features->allow_screen_content_tools =
        allow_screen_content_tools_orig_decision;
    features->allow_intrabc = allow_intrabc_orig_decision;
    cpi->is_screen_content_type = is_screen_content_type_orig_decision;
  }
}

// Set some encoding parameters to make the encoding process fast.
// A fixed block partition size, and a large q is used.
static void set_encoding_params_for_screen_content(AV1_COMP *cpi,
                                                   const int pass) {
  AV1_COMMON *const cm = &cpi->common;
  if (pass == 0) {
    // In the first pass, encode without screen content tools.
    // Use a high q, and a fixed block size for fast encoding.
    cm->features.allow_screen_content_tools = 0;
    cm->features.allow_intrabc = 0;
    cpi->is_screen_content_type = 0;
    cpi->sf.part_sf.partition_search_type = FIXED_PARTITION;
    cpi->sf.part_sf.fixed_partition_size = BLOCK_32X32;
    return;
  }
  assert(pass == 1);
  // In the second pass, encode with screen content tools.
  // Use a high q, and a fixed block size for fast encoding.
  cm->features.allow_screen_content_tools = 1;
  // TODO(chengchen): turn intrabc on could lead to data race issue.
  // cm->allow_intrabc = 1;
  cpi->is_screen_content_type = 1;
  cpi->sf.part_sf.partition_search_type = FIXED_PARTITION;
  cpi->sf.part_sf.fixed_partition_size = BLOCK_32X32;
}

// Determines whether to use screen content tools for the key frame group.
// This function modifies "cm->features.allow_screen_content_tools",
// "cm->features.allow_intrabc" and "cpi->is_screen_content_type".
void av1_determine_sc_tools_with_encoding(AV1_COMP *cpi, const int q_orig) {
  AV1_COMMON *const cm = &cpi->common;
  const AV1EncoderConfig *const oxcf = &cpi->oxcf;
  const QuantizationCfg *const q_cfg = &oxcf->q_cfg;
  // Variables to help determine if we should allow screen content tools.
  int projected_size_pass[3] = { 0 };
  PSNR_STATS psnr[3];
  const int is_key_frame = cm->current_frame.frame_type == KEY_FRAME;
  const int allow_screen_content_tools_orig_decision =
      cm->features.allow_screen_content_tools;
  const int allow_intrabc_orig_decision = cm->features.allow_intrabc;
  const int is_screen_content_type_orig_decision = cpi->is_screen_content_type;
#if CONFIG_IBC_SR_EXT
  if (cpi->oxcf.tune_cfg.content == AOM_CONTENT_SCREEN) {
    cm->features.allow_screen_content_tools = cm->features.allow_intrabc = 1;
    return;
  }
#endif  // CONFIG_IBC_SR_EXT

  // Turn off the encoding trial for forward key frame and superres.
  if (oxcf->kf_cfg.fwd_kf_enabled || cpi->superres_mode != AOM_SUPERRES_NONE ||
      is_screen_content_type_orig_decision || !is_key_frame) {
    return;
  }

  // TODO(chengchen): multiple encoding for the lossless mode is time consuming.
  // Find a better way to determine whether screen content tools should be used
  // for lossless coding.
  // Use a high q and a fixed partition to do quick encoding.
  const int q_for_screen_content_quick_run =
#if CONFIG_SCC_DETERMINATION
      is_lossless_requested(&oxcf->rc_cfg) ? q_orig : AOMMAX(q_orig, 200);
#else
      is_lossless_requested(&oxcf->rc_cfg) ? q_orig : AOMMAX(q_orig, 244);
#endif  // CONFIG_SCC_DETERMINATION
  const int partition_search_type_orig = cpi->sf.part_sf.partition_search_type;
  const BLOCK_SIZE fixed_partition_block_size_orig =
      cpi->sf.part_sf.fixed_partition_size;

  // Setup necessary params for encoding, including frame source, etc.
  aom_clear_system_state();

  cpi->source =
      av1_scale_if_required(cm, cpi->unscaled_source, &cpi->scaled_source,
                            cm->features.interp_filter, 0, false, false);
  if (cpi->unscaled_last_source != NULL) {
    cpi->last_source = av1_scale_if_required(
        cm, cpi->unscaled_last_source, &cpi->scaled_last_source,
        cm->features.interp_filter, 0, false, false);
  }

  if (cm->seg.enabled) {
    if (!cm->seg.update_data && cm->prev_frame) {
      segfeatures_copy(&cm->seg, &cm->prev_frame->seg);
      cm->seg.enabled = cm->prev_frame->seg.enabled;
    } else {
      av1_calculate_segdata(&cm->seg);
    }
  } else {
    memset(&cm->seg, 0, sizeof(cm->seg));
  }
  segfeatures_copy(&cm->cur_frame->seg, &cm->seg);
  cm->cur_frame->seg.enabled = cm->seg.enabled;

  // The two encoding passes aim to help determine whether to use screen
  // content tools, with a high q and fixed partition.
  for (int pass = 0; pass < 2; ++pass) {
    set_encoding_params_for_screen_content(cpi, pass);
    av1_set_quantizer(cm, q_cfg->qm_minlevel, q_cfg->qm_maxlevel,
                      q_for_screen_content_quick_run,
                      q_cfg->enable_chroma_deltaq);
    av1_set_speed_features_qindex_dependent(cpi, oxcf->speed);

    av1_init_quantizer(&cm->seq_params, &cpi->enc_quant_dequant_params,
                       &cm->quant_params);

    av1_setup_frame(cpi);

    // transform / motion compensation build reconstruction frame
    av1_encode_frame(cpi);
    // Screen content decision
    screen_content_tools_determination(
        cpi, allow_screen_content_tools_orig_decision,
        allow_intrabc_orig_decision, is_screen_content_type_orig_decision, pass,
        projected_size_pass, psnr);
  }
#if CONFIG_SCC_DETERMINATION
  assert(is_key_frame);
  cm->features.kf_allow_sc_tools = cm->features.allow_screen_content_tools;
#endif  // CONFIG_SCC_DETERMINATION
  // Set partition speed feature back.
  cpi->sf.part_sf.partition_search_type = partition_search_type_orig;
  cpi->sf.part_sf.fixed_partition_size = fixed_partition_block_size_orig;
}

static void fix_interp_filter(InterpFilter *const interp_filter,
                              const FRAME_COUNTS *const counts) {
  if (*interp_filter == SWITCHABLE) {
    // Check to see if only one of the filters is actually used
    int count[SWITCHABLE_FILTERS] = { 0 };
    int num_filters_used = 0;
    for (int i = 0; i < SWITCHABLE_FILTERS; ++i) {
      for (int j = 0; j < SWITCHABLE_FILTER_CONTEXTS; ++j)
        count[i] += counts->switchable_interp[j][i];
      num_filters_used += (count[i] > 0);
    }
    if (num_filters_used == 1) {
      // Only one filter is used. So set the filter at frame level
      for (int i = 0; i < SWITCHABLE_FILTERS; ++i) {
        if (count[i]) {
          if (i == EIGHTTAP_REGULAR) *interp_filter = i;
          break;
        }
      }
    }
  }
}

void av1_finalize_encoded_frame(AV1_COMP *const cpi) {
  AV1_COMMON *const cm = &cpi->common;
  CurrentFrame *const current_frame = &cm->current_frame;

  if (!cm->seq_params.reduced_still_picture_hdr &&
      (encode_show_existing_frame(cm) || cm->show_existing_frame)) {
    RefCntBuffer *const frame_to_show =
        cm->ref_frame_map[cpi->existing_fb_idx_to_show];

    if (frame_to_show == NULL) {
      aom_internal_error(&cm->error, AOM_CODEC_UNSUP_BITSTREAM,
                         "Buffer does not contain a reconstructed frame");
    }
    assert(frame_to_show->ref_count > 0);
    assign_frame_buffer_p(&cm->cur_frame, frame_to_show);
  }

  if (!encode_show_existing_frame(cm) &&
      cm->seq_params.film_grain_params_present &&
      (cm->show_frame || cm->showable_frame)) {
    // Copy the current frame's film grain params to the its corresponding
    // RefCntBuffer slot.
    cm->cur_frame->film_grain_params = cm->film_grain_params;

    // We must update the parameters if this is not an INTER_FRAME
    if (current_frame->frame_type != INTER_FRAME)
      cm->cur_frame->film_grain_params.update_parameters = 1;

    // Iterate the random seed for the next frame.
    cm->film_grain_params.random_seed += 3381;
    if (cm->film_grain_params.random_seed == 0)
      cm->film_grain_params.random_seed = 7391;
  }

  // Initialise all tiles' contexts from the global frame context
  for (int tile_col = 0; tile_col < cm->tiles.cols; tile_col++) {
    for (int tile_row = 0; tile_row < cm->tiles.rows; tile_row++) {
      const int tile_idx = tile_row * cm->tiles.cols + tile_col;
      cpi->tile_data[tile_idx].tctx = *cm->fc;
    }
  }

  fix_interp_filter(&cm->features.interp_filter, cpi->td.counts);
}

int av1_is_integer_mv(const YV12_BUFFER_CONFIG *cur_picture,
                      const YV12_BUFFER_CONFIG *last_picture,
                      ForceIntegerMVInfo *const force_intpel_info) {
  aom_clear_system_state();
  // check use hash ME
  int k;

  const int block_size = FORCE_INT_MV_DECISION_BLOCK_SIZE;
  const double threshold_current = 0.8;
  const double threshold_average = 0.95;
  const int max_history_size = 32;
  int T = 0;  // total block
  int C = 0;  // match with collocated block
  int S = 0;  // smooth region but not match with collocated block

  const int pic_width = cur_picture->y_width;
  const int pic_height = cur_picture->y_height;
  for (int i = 0; i + block_size <= pic_height; i += block_size) {
    for (int j = 0; j + block_size <= pic_width; j += block_size) {
      const int x_pos = j;
      const int y_pos = i;
      int match = 1;
      T++;

      // check whether collocated block match with current
      uint16_t *p_cur = cur_picture->y_buffer;
      uint16_t *p_ref = last_picture->y_buffer;
      int stride_cur = cur_picture->y_stride;
      int stride_ref = last_picture->y_stride;
      p_cur += (y_pos * stride_cur + x_pos);
      p_ref += (y_pos * stride_ref + x_pos);

      for (int tmpY = 0; tmpY < block_size && match; tmpY++) {
        for (int tmpX = 0; tmpX < block_size && match; tmpX++) {
          if (p_cur[tmpX] != p_ref[tmpX]) {
            match = 0;
          }
        }
        p_cur += stride_cur;
        p_ref += stride_ref;
      }

      if (match) {
        C++;
        continue;
      }

      if (av1_hash_is_horizontal_perfect(cur_picture, block_size, x_pos,
                                         y_pos) ||
          av1_hash_is_vertical_perfect(cur_picture, block_size, x_pos, y_pos)) {
        S++;
        continue;
      }
    }
  }

  assert(T > 0);
  double cs_rate = ((double)(C + S)) / ((double)(T));

  force_intpel_info->cs_rate_array[force_intpel_info->rate_index] = cs_rate;

  force_intpel_info->rate_index =
      (force_intpel_info->rate_index + 1) % max_history_size;
  force_intpel_info->rate_size++;
  force_intpel_info->rate_size =
      AOMMIN(force_intpel_info->rate_size, max_history_size);

  if (cs_rate < threshold_current) {
    return 0;
  }

  if (C == T) {
    return 1;
  }

  double cs_average = 0.0;

  for (k = 0; k < force_intpel_info->rate_size; k++) {
    cs_average += force_intpel_info->cs_rate_array[k];
  }
  cs_average /= force_intpel_info->rate_size;

  if (cs_average < threshold_average) {
    return 0;
  }

  if ((T - C - S) < 0) {
    return 1;
  }

  if (cs_average > 1.01) {
    return 1;
  }

  return 0;
}

void av1_set_mb_ssim_rdmult_scaling(AV1_COMP *cpi) {
  const CommonModeInfoParams *const mi_params = &cpi->common.mi_params;
  ThreadData *td = &cpi->td;
  MACROBLOCK *x = &td->mb;
  MACROBLOCKD *xd = &x->e_mbd;
  uint16_t *y_buffer = cpi->source->y_buffer;
  const int y_stride = cpi->source->y_stride;
  const int block_size = BLOCK_16X16;

  const int num_mi_w = mi_size_wide[block_size];
  const int num_mi_h = mi_size_high[block_size];
  const int num_cols = (mi_params->mi_cols + num_mi_w - 1) / num_mi_w;
  const int num_rows = (mi_params->mi_rows + num_mi_h - 1) / num_mi_h;
  double log_sum = 0.0;

  // Loop through each 16x16 block.
  for (int row = 0; row < num_rows; ++row) {
    for (int col = 0; col < num_cols; ++col) {
      double var = 0.0, num_of_var = 0.0;
      const int index = row * num_cols + col;

      // Loop through each 8x8 block.
      for (int mi_row = row * num_mi_h;
           mi_row < mi_params->mi_rows && mi_row < (row + 1) * num_mi_h;
           mi_row += 2) {
        for (int mi_col = col * num_mi_w;
             mi_col < mi_params->mi_cols && mi_col < (col + 1) * num_mi_w;
             mi_col += 2) {
          struct buf_2d buf;
          const int row_offset_y = mi_row << 2;
          const int col_offset_y = mi_col << 2;

          buf.buf = y_buffer + row_offset_y * y_stride + col_offset_y;
          buf.stride = y_stride;

          var +=
              av1_high_get_sby_perpixel_variance(cpi, &buf, BLOCK_8X8, xd->bd);

          num_of_var += 1.0;
        }
      }
      var = var / num_of_var;

      // Curve fitting with an exponential model on all 16x16 blocks from the
      // midres dataset.
      var = 67.035434 * (1 - exp(-0.0021489 * var)) + 17.492222;
      cpi->ssim_rdmult_scaling_factors[index] = var;
      log_sum += log(var);
    }
  }
  log_sum = exp(log_sum / (double)(num_rows * num_cols));

  for (int row = 0; row < num_rows; ++row) {
    for (int col = 0; col < num_cols; ++col) {
      const int index = row * num_cols + col;
      cpi->ssim_rdmult_scaling_factors[index] /= log_sum;
    }
  }
}

// Coding context that only needs to be saved when recode loop includes
// filtering (deblocking, CDEF, superres post-encode upscale and/or loop
// restoraton).
static void save_extra_coding_context(AV1_COMP *cpi) {
  CODING_CONTEXT *const cc = &cpi->coding_context;
  AV1_COMMON *cm = &cpi->common;

  cc->lf = cm->lf;
  cc->cdef_info = cm->cdef_info;
  cc->rc = cpi->rc;
  cc->mv_stats = cpi->mv_stats;
  cc->features = cm->features;
}

void av1_save_all_coding_context(AV1_COMP *cpi) {
  save_extra_coding_context(cpi);
  if (!frame_is_intra_only(&cpi->common)) release_scaled_references(cpi);
}
