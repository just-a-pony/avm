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

#include "av1/common/banding_metadata.h"

#if CONFIG_BAND_METADATA

#include <string.h>
#include "aom_dsp/bitwriter_buffer.h"
#include "aom_dsp/bitreader_buffer.h"
#include "aom/internal/aom_image_internal.h"

int aom_encode_banding_hints_metadata(
    const aom_banding_hints_metadata_t *metadata, uint8_t *payload,
    size_t *payload_size) {
  if (!metadata || !payload || !payload_size) {
    return -1;
  }

  struct aom_write_bit_buffer wb = { payload, 0 };

  // Write basic flags (3 bits)
  aom_wb_write_bit(&wb, metadata->coding_banding_present_flag);
  aom_wb_write_bit(&wb, metadata->source_banding_present_flag);

  if (metadata->coding_banding_present_flag) {
    aom_wb_write_bit(&wb, metadata->banding_hints_flag);

    if (metadata->banding_hints_flag) {
      aom_wb_write_bit(&wb, metadata->three_color_components);

      const int num_components = metadata->three_color_components ? 3 : 1;

      // Write per-component information
      for (int plane = 0; plane < num_components; plane++) {
        aom_wb_write_bit(&wb,
                         metadata->banding_in_component_present_flag[plane]);
        if (metadata->banding_in_component_present_flag[plane]) {
          aom_wb_write_literal(&wb, metadata->max_band_width_minus4[plane], 6);
          aom_wb_write_literal(&wb, metadata->max_band_step_minus1[plane], 4);
        }
      }

      // Write band units information
      aom_wb_write_bit(&wb, metadata->band_units_information_present_flag);
      if (metadata->band_units_information_present_flag) {
        aom_wb_write_literal(&wb, metadata->num_band_units_rows_minus_1, 5);
        aom_wb_write_literal(&wb, metadata->num_band_units_cols_minus_1, 5);
        aom_wb_write_bit(&wb, metadata->varying_size_band_units_flag);

        if (metadata->varying_size_band_units_flag) {
          aom_wb_write_literal(&wb, metadata->band_block_in_luma_samples, 3);

          // Write vertical sizes
          for (int r = 0; r <= metadata->num_band_units_rows_minus_1; r++) {
            aom_wb_write_literal(
                &wb, metadata->vert_size_in_band_blocks_minus1[r], 5);
          }

          // Write horizontal sizes
          for (int c = 0; c <= metadata->num_band_units_cols_minus_1; c++) {
            aom_wb_write_literal(
                &wb, metadata->horz_size_in_band_blocks_minus1[c], 5);
          }
        }

        // Write per-tile banding flags
        for (int r = 0; r <= metadata->num_band_units_rows_minus_1; r++) {
          for (int c = 0; c <= metadata->num_band_units_cols_minus_1; c++) {
            aom_wb_write_bit(&wb,
                             metadata->banding_in_band_unit_present_flag[r][c]);
          }
        }
      }
    }
  }

  // Calculate actual payload size in bytes
  *payload_size = aom_wb_bytes_written(&wb);

  // Add byte alignment if needed
  if (wb.bit_offset != 0) {
    (*payload_size)++;
  }

  return 0;
}

int aom_decode_banding_hints_metadata(const uint8_t *payload,
                                      size_t payload_size,
                                      aom_banding_hints_metadata_t *metadata) {
  if (!payload || !metadata || payload_size == 0) {
    return -1;
  }

  // Clear the metadata structure
  memset(metadata, 0, sizeof(*metadata));

  struct aom_read_bit_buffer rb = { payload, payload + payload_size, 0, NULL,
                                    NULL };

  // Read basic flags (3 bits)
  metadata->coding_banding_present_flag = aom_rb_read_bit(&rb);
  metadata->source_banding_present_flag = aom_rb_read_bit(&rb);

  if (metadata->coding_banding_present_flag) {
    metadata->banding_hints_flag = aom_rb_read_bit(&rb);

    if (metadata->banding_hints_flag) {
      metadata->three_color_components = aom_rb_read_bit(&rb);

      const int num_components = metadata->three_color_components ? 3 : 1;

      // Read per-component information
      for (int plane = 0; plane < num_components; plane++) {
        metadata->banding_in_component_present_flag[plane] =
            aom_rb_read_bit(&rb);
        if (metadata->banding_in_component_present_flag[plane]) {
          metadata->max_band_width_minus4[plane] = aom_rb_read_literal(&rb, 6);
          metadata->max_band_step_minus1[plane] = aom_rb_read_literal(&rb, 4);
        }
      }

      // Read band units information
      metadata->band_units_information_present_flag = aom_rb_read_bit(&rb);
      if (metadata->band_units_information_present_flag) {
        metadata->num_band_units_rows_minus_1 = aom_rb_read_literal(&rb, 5);
        metadata->num_band_units_cols_minus_1 = aom_rb_read_literal(&rb, 5);
        metadata->varying_size_band_units_flag = aom_rb_read_bit(&rb);

        if (metadata->varying_size_band_units_flag) {
          metadata->band_block_in_luma_samples = aom_rb_read_literal(&rb, 3);

          // Read vertical sizes
          for (int r = 0; r <= metadata->num_band_units_rows_minus_1; r++) {
            metadata->vert_size_in_band_blocks_minus1[r] =
                aom_rb_read_literal(&rb, 5);
          }

          // Read horizontal sizes
          for (int c = 0; c <= metadata->num_band_units_cols_minus_1; c++) {
            metadata->horz_size_in_band_blocks_minus1[c] =
                aom_rb_read_literal(&rb, 5);
          }
        }

        // Read per-tile banding flags
        for (int r = 0; r <= metadata->num_band_units_rows_minus_1; r++) {
          for (int c = 0; c <= metadata->num_band_units_cols_minus_1; c++) {
            metadata->banding_in_band_unit_present_flag[r][c] =
                aom_rb_read_bit(&rb);
          }
        }
      }
    }
  }

  return 0;
}

int aom_img_add_banding_hints_metadata(
    aom_image_t *img, const aom_banding_hints_metadata_t *banding_metadata,
    aom_metadata_insert_flags_t insert_flag) {
  if (!img || !banding_metadata) {
    return -1;
  }

  // Encode the banding metadata to a payload
  uint8_t payload_buffer[256];  // Should be sufficient for banding metadata
  size_t payload_size = sizeof(payload_buffer);

  if (aom_encode_banding_hints_metadata(banding_metadata, payload_buffer,
                                        &payload_size) != 0) {
    return -1;
  }

  // Add the metadata to the image
  return aom_img_add_metadata(img, OBU_METADATA_TYPE_BANDING_HINTS,
                              payload_buffer, payload_size, insert_flag);
}

#endif  // CONFIG_BAND_METADATA
