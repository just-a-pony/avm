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

#ifndef TOOLS_STREAM_MUX_H_
#define TOOLS_STREAM_MUX_H_

#include <stdlib.h>
#include <string.h>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <string>

#include "config/aom_config.h"
#include "common/ivfdec.h"
#include "common/obudec.h"
#include "common/tools_common.h"
#include "common/webmdec.h"
#include "av1/common/obu_util.h"
#include "av1/common/blockd.h"
#include "aom_dsp/bitwriter_buffer.h"
#include "aom_dsp/bitreader_buffer.h"
#include "aom/aom_codec.h"
#include "av1/encoder/bitstream.h"

#define PRINT_TU_INFO 0

#define AOM_MAX_NUM_STREAMS 4

const size_t kInitialBufferSize = 100 * 1024;

// Basic OBU syntax
// 8 bits: Header
//   7,6,5,4
//     type bits
//   3
//     extension flag bit
//   2,1,0
//     tlayer ID
const uint32_t kObuTypeBitsMask = 0x1F;
const uint32_t kObuTypeBitsShift = 2;
const uint32_t kObuExtensionFlagBitMask = 0x1;
const uint32_t kObuExtensionFlagBitShift = 7;
const uint32_t kObuExtTlayerIdBitsMask = 0x3;
const uint32_t kObuExtTlayerIdBitsShift = 0;

// When extension flag bit is set:
// 8 bits: extension header
// 7,6,5
//   mlayer ID
// 4,3,2,1,0
//   xlayer ID
const uint32_t kObuExtMlayerIdBitsMask = 0x7;
const uint32_t kObuExtMlayerIdBitsShift = 5;
const uint32_t kObuExtXlayerIdBitsMask = 0x1F;
const uint32_t kObuExtXlayerIdBitsShift = 0;

struct InputContext {
  InputContext() = default;
  ~InputContext() { free(unit_buffer); }

  void Init() {
    memset(avx_ctx, 0, sizeof(*avx_ctx));
    memset(obu_ctx, 0, sizeof(*obu_ctx));
    obu_ctx->avx_ctx = avx_ctx;
#if CONFIG_WEBM_IO
    memset(webm_ctx, 0, sizeof(*webm_ctx));
#endif
    obu_ctx->is_annexb = 1;
  }

  AvxInputContext *avx_ctx = nullptr;
  ObuDecInputContext *obu_ctx = nullptr;
#if CONFIG_WEBM_IO
  WebmInputContext *webm_ctx = nullptr;
#endif
  uint8_t *unit_buffer = nullptr;
  size_t unit_buffer_size = 0;
};

VideoFileType GetFileType(InputContext *ctx) {
  if (file_is_ivf(ctx->avx_ctx)) return FILE_TYPE_IVF;
  if (file_is_obu(ctx->obu_ctx)) return FILE_TYPE_OBU;
#if CONFIG_WEBM_IO
  if (file_is_webm(ctx->webm_ctx, ctx->avx_ctx)) return FILE_TYPE_WEBM;
#endif
  return FILE_TYPE_RAW;
}

bool ReadTemporalUnit(InputContext *ctx, size_t *unit_size) {
  const VideoFileType file_type = ctx->avx_ctx->file_type;
  switch (file_type) {
    case FILE_TYPE_IVF: {
      if (ivf_read_frame(ctx->avx_ctx->file, &ctx->unit_buffer, unit_size,
                         &ctx->unit_buffer_size, NULL)) {
        return false;
      }
      break;
    }
    case FILE_TYPE_OBU: {
      if (obudec_read_temporal_unit(ctx->obu_ctx, &ctx->unit_buffer, unit_size,
                                    &ctx->unit_buffer_size)) {
        return false;
      }
      break;
    }
#if CONFIG_WEBM_IO
    case FILE_TYPE_WEBM: {
      if (webm_read_frame(ctx->webm_ctx, &ctx->unit_buffer, unit_size,
                          &ctx->unit_buffer_size)) {
        return false;
      }
      break;
    }
#endif
    default: fprintf(stderr, "Error: Unsupported file type.\n"); return false;
  }

  return true;
}

bool ValidObuType(int obu_type) {
  switch (obu_type) {
    case OBU_SEQUENCE_HEADER:
    case OBU_TEMPORAL_DELIMITER:
#if CONFIG_MULTI_FRAME_HEADER
    case OBU_MULTI_FRAME_HEADER:
#endif  // CONFIG_MULTI_FRAME_HEADER
#if CONFIG_F106_OBU_TILEGROUP
#if CONFIG_F106_OBU_SWITCH
    case OBU_SWITCH:
#endif  // CONFIG_F106_OBU_SWITCH
#if CONFIG_F106_OBU_SEF
    case OBU_SEF:
#endif  // CONFIG_F106_OBU_SEF
#if CONFIG_F106_OBU_TIP
    case OBU_TIP:
#endif  // CONFIG_F106_OBU_TIP
    case OBU_TILE_GROUP:
#else
    case OBU_FRAME_HEADER:
    case OBU_TILE_GROUP:
#endif  // CONFIG_F106_OBU_TILEGROUP
    case OBU_METADATA:
#if !CONFIG_F106_OBU_TILEGROUP
    case OBU_FRAME:
#if !CONFIG_REMOVAL_REDUNDANT_FRAME_HEADER
    case OBU_REDUNDANT_FRAME_HEADER:
#endif  // !CONFIG_REMOVAL_REDUNDANT_FRAME_HEADER
#endif  // CONFIG_F106_OBU_TILEGROUP
#if CONFIG_MULTILAYER_HLS
    case OBU_LAYER_CONFIGURATION_RECORD:
    case OBU_ATLAS_SEGMENT:
    case OBU_OPERATING_POINT_SET:
#endif  // CONFIG_MULTILAYER_HLS
#if CONFIG_MULTI_STREAM
    case OBU_MSDO:
#endif  // CONFIG_MULTI_STREAM
    case OBU_PADDING: return true;
  }
  return false;
}

void PrintObuHeader(const ObuHeader *header) {
  printf(
      "      OBU type:  %s\n"
      "      obu_tlayer_id: %d\n"
      "      obu_mlayer_id: %d\n"
      "      obu_xlayer_id:  %d\n",
      aom_obu_type_to_string(static_cast<OBU_TYPE>(header->type)),
      header->obu_tlayer_id, header->obu_mlayer_id, header->obu_xlayer_id);
}

// Initialize a read bit buffer
struct aom_read_bit_buffer *init_read_bit_buffer(struct aom_read_bit_buffer *rb,
                                                 const uint8_t *data,
                                                 const uint8_t *data_end) {
  rb->bit_offset = 0;
  rb->error_handler = NULL;
  rb->error_handler_data = NULL;
  rb->bit_buffer = data;
  rb->bit_buffer_end = data_end;
  return rb;
}

bool ParseAV2ObuHeader(uint8_t obu_header_byte, ObuHeader *obu_header) {
  obu_header->type = static_cast<OBU_TYPE>(
      (obu_header_byte >> kObuTypeBitsShift) & kObuTypeBitsMask);
  if (!ValidObuType(obu_header->type)) {
    fprintf(stderr, "Invalid OBU type: %d.\n", obu_header->type);
    return false;
  }

  obu_header->obu_extension_flag =
      (obu_header_byte >> kObuExtensionFlagBitShift) & kObuExtensionFlagBitMask;
  obu_header->obu_tlayer_id =
      (obu_header_byte >> kObuExtTlayerIdBitsShift) & kObuExtTlayerIdBitsMask;
  return true;
}

bool ParseAV2ObuExtensionHeader(uint8_t ext_header_byte,
                                ObuHeader *obu_header) {
  obu_header->obu_mlayer_id =
      (ext_header_byte >> kObuExtMlayerIdBitsShift) & kObuExtMlayerIdBitsMask;
  obu_header->obu_xlayer_id =
      (ext_header_byte >> kObuExtXlayerIdBitsShift) & kObuExtXlayerIdBitsMask;

  return true;
}

#endif  // TOOLS_STREAM_MUX_H_
