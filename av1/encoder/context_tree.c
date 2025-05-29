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

#include "av1/encoder/context_tree.h"
#include "av1/common/av1_common_int.h"
#include "av1/encoder/encoder.h"
#include "av1/encoder/rd.h"

static const BLOCK_SIZE square[MAX_SB_SIZE_LOG2 - 1] = {
  BLOCK_4X4,   BLOCK_8X8,     BLOCK_16X16,   BLOCK_32X32,
  BLOCK_64X64, BLOCK_128X128, BLOCK_256X256,
};

void av1_copy_tree_context(PICK_MODE_CONTEXT *dst_ctx,
                           PICK_MODE_CONTEXT *src_ctx, int num_planes) {
  dst_ctx->mic = src_ctx->mic;
#if CONFIG_C071_SUBBLK_WARPMV
  if (is_warp_mode(src_ctx->mic.motion_mode)) {
    av1_copy_array(dst_ctx->submic, src_ctx->submic, src_ctx->num_4x4_blk);
  }
#endif  // CONFIG_C071_SUBBLK_WARPMV
  dst_ctx->mbmi_ext_best = src_ctx->mbmi_ext_best;

  assert(dst_ctx->num_4x4_blk == src_ctx->num_4x4_blk);
  assert(dst_ctx->num_4x4_blk_chroma == src_ctx->num_4x4_blk_chroma);
  dst_ctx->skippable = src_ctx->skippable;

  for (int i = 0; i < num_planes; ++i) {
    const int num_blk_plane =
        (i == 0) ? src_ctx->num_4x4_blk : src_ctx->num_4x4_blk_chroma;
    memcpy(dst_ctx->blk_skip[i], src_ctx->blk_skip[i],
           sizeof(*src_ctx->blk_skip[i]) * num_blk_plane);
  }
  av1_copy_array(dst_ctx->tx_type_map, src_ctx->tx_type_map,
                 src_ctx->num_4x4_blk);
  av1_copy_array(dst_ctx->cctx_type_map, src_ctx->cctx_type_map,
                 src_ctx->num_4x4_blk_chroma);

  dst_ctx->hybrid_pred_diff = src_ctx->hybrid_pred_diff;
  dst_ctx->comp_pred_diff = src_ctx->comp_pred_diff;
  dst_ctx->single_pred_diff = src_ctx->single_pred_diff;

  dst_ctx->rd_stats = src_ctx->rd_stats;
  dst_ctx->rd_mode_is_ready = src_ctx->rd_mode_is_ready;
  const int num_pix = src_ctx->num_4x4_blk * 16;
  if (num_pix <= MAX_PALETTE_SQUARE) {
    for (int i = 0; i < 2; ++i) {
      const int num_blk =
          (i == 0) ? src_ctx->num_4x4_blk : src_ctx->num_4x4_blk_chroma;
      const int color_map_size = num_blk * 16;
      memcpy(dst_ctx->color_index_map[i], src_ctx->color_index_map[i],
             sizeof(src_ctx->color_index_map[i][0]) * color_map_size);
    }
  }
}

void av1_setup_shared_coeff_buffer(AV1_COMMON *cm,
                                   PC_TREE_SHARED_BUFFERS *shared_bufs) {
  for (int i = 0; i < 3; i++) {
    const int max_num_pix = MAX_SB_SIZE * MAX_SB_SIZE;
    CHECK_MEM_ERROR(cm, shared_bufs->coeff_buf[i],
                    aom_memalign(32, max_num_pix * sizeof(tran_low_t)));
    CHECK_MEM_ERROR(cm, shared_bufs->qcoeff_buf[i],
                    aom_memalign(32, max_num_pix * sizeof(tran_low_t)));
    CHECK_MEM_ERROR(cm, shared_bufs->dqcoeff_buf[i],
                    aom_memalign(32, max_num_pix * sizeof(tran_low_t)));
  }
}

void av1_free_shared_coeff_buffer(PC_TREE_SHARED_BUFFERS *shared_bufs) {
  for (int i = 0; i < 3; i++) {
    aom_free(shared_bufs->coeff_buf[i]);
    aom_free(shared_bufs->qcoeff_buf[i]);
    aom_free(shared_bufs->dqcoeff_buf[i]);
    shared_bufs->coeff_buf[i] = NULL;
    shared_bufs->qcoeff_buf[i] = NULL;
    shared_bufs->dqcoeff_buf[i] = NULL;
  }
}

// Get base block size for pick mode context allocation.
static INLINE int get_num_pix_bsize_base(
    BLOCK_SIZE bsize, TREE_TYPE tree_type,
    const CHROMA_REF_INFO *const chroma_ref_info, int num_planes) {
  if (num_planes == 1 || tree_type == LUMA_PART) {
    // No chroma pixels required in this case. But we return value 1 to avoid
    // special cases for malloc(), memcpy() etc.
    return 1;
  }
  BLOCK_SIZE bsize_base = BLOCK_INVALID;
  if (tree_type == SHARED_PART) {
    bsize_base = chroma_ref_info->bsize_base;
  } else {
    assert(tree_type == CHROMA_PART);
    bsize_base = bsize;
  }
  assert(bsize_base != BLOCK_INVALID);
  return block_size_wide[bsize_base] * block_size_high[bsize_base];
}

PICK_MODE_CONTEXT *av1_alloc_pmc(const AV1_COMMON *cm, TREE_TYPE tree_type,
                                 int mi_row, int mi_col, BLOCK_SIZE bsize,
                                 PC_TREE *parent,
                                 PARTITION_TYPE parent_partition, int index,
                                 int subsampling_x, int subsampling_y,
                                 PC_TREE_SHARED_BUFFERS *shared_bufs) {
  PICK_MODE_CONTEXT *ctx = NULL;
  struct aom_internal_error_info error;

  AOM_CHECK_MEM_ERROR(&error, ctx, aom_calloc(1, sizeof(*ctx)));
  ctx->rd_mode_is_ready = 0;
  ctx->parent = parent;
  ctx->index = index;
  set_chroma_ref_info(tree_type, mi_row, mi_col, index, bsize,
                      &ctx->chroma_ref_info,
                      parent ? &parent->chroma_ref_info : NULL,
                      parent ? parent->block_size : BLOCK_INVALID,
                      parent_partition, subsampling_x, subsampling_y);
  ctx->mic.chroma_ref_info = ctx->chroma_ref_info;

  const int num_planes = av1_num_planes(cm);
  const int num_pix = block_size_wide[bsize] * block_size_high[bsize];
  const int num_blk = num_pix / 16;

  // We need to get actual chroma block size due to possible sub-8x8 luma block
  // sizes.
  const int num_pix_bize_base = get_num_pix_bsize_base(
      bsize, tree_type, &ctx->chroma_ref_info, num_planes);
  ctx->num_4x4_blk = num_blk;
  ctx->num_4x4_blk_chroma = num_pix_bize_base / 16;

  AOM_CHECK_MEM_ERROR(&error, ctx->tx_type_map,
                      aom_calloc(num_blk, sizeof(*ctx->tx_type_map)));
#if CONFIG_C071_SUBBLK_WARPMV
  if (!frame_is_intra_only(cm)) {
    ctx->submic = malloc(num_blk * sizeof(*ctx->submic));
  }
#endif  // CONFIG_C071_SUBBLK_WARPMV
  AOM_CHECK_MEM_ERROR(
      &error, ctx->cctx_type_map,
      aom_calloc(ctx->num_4x4_blk_chroma, sizeof(*ctx->cctx_type_map)));

  for (int i = 0; i < num_planes; ++i) {
    ctx->coeff[i] = shared_bufs->coeff_buf[i];
    ctx->qcoeff[i] = shared_bufs->qcoeff_buf[i];
    ctx->dqcoeff[i] = shared_bufs->dqcoeff_buf[i];
    const int num_blk_plane =
        (i == 0) ? ctx->num_4x4_blk : ctx->num_4x4_blk_chroma;
    AOM_CHECK_MEM_ERROR(&error, ctx->blk_skip[i],
                        aom_calloc(num_blk_plane, sizeof(*ctx->blk_skip[i])));
    AOM_CHECK_MEM_ERROR(
        &error, ctx->eobs[i],
        aom_memalign(32, num_blk_plane * sizeof(*ctx->eobs[i])));
    AOM_CHECK_MEM_ERROR(
        &error, ctx->bobs[i],
        aom_memalign(32, num_blk_plane * sizeof(*ctx->bobs[i])));
    AOM_CHECK_MEM_ERROR(
        &error, ctx->txb_entropy_ctx[i],
        aom_memalign(32, num_blk_plane * sizeof(*ctx->txb_entropy_ctx[i])));
  }

  if (num_pix <= MAX_PALETTE_SQUARE) {
    for (int i = 0; i < 2; ++i) {
      const int color_map_size = (i == 0) ? num_pix : num_pix_bize_base;
      AOM_CHECK_MEM_ERROR(
          &error, ctx->color_index_map[i],
          aom_memalign(32, color_map_size * sizeof(*ctx->color_index_map[i])));
    }
  }
  av1_invalid_rd_stats(&ctx->rd_stats);
  return ctx;
}

void av1_free_pmc(PICK_MODE_CONTEXT *ctx, int num_planes) {
  if (ctx == NULL) return;

#if CONFIG_C071_SUBBLK_WARPMV
  if (ctx->submic) {
    free(ctx->submic);
  }
#endif  // CONFIG_C071_SUBBLK_WARPMV
  for (int i = 0; i < MAX_MB_PLANE; ++i) {
    aom_free(ctx->blk_skip[i]);
    ctx->blk_skip[i] = NULL;
  }
  aom_free(ctx->tx_type_map);
  ctx->tx_type_map = NULL;
  aom_free(ctx->cctx_type_map);
  ctx->cctx_type_map = NULL;
  for (int i = 0; i < num_planes; ++i) {
    ctx->coeff[i] = NULL;
    ctx->qcoeff[i] = NULL;
    ctx->dqcoeff[i] = NULL;
    aom_free(ctx->eobs[i]);
    ctx->eobs[i] = NULL;
    aom_free(ctx->bobs[i]);
    ctx->bobs[i] = NULL;
    aom_free(ctx->txb_entropy_ctx[i]);
    ctx->txb_entropy_ctx[i] = NULL;
  }

  for (int i = 0; i < 2; ++i) {
    aom_free(ctx->color_index_map[i]);
    ctx->color_index_map[i] = NULL;
  }

  aom_free(ctx);
}

PC_TREE *av1_alloc_pc_tree_node(TREE_TYPE tree_type, int mi_row, int mi_col,
                                BLOCK_SIZE bsize, PC_TREE *parent,
                                PARTITION_TYPE parent_partition, int index,
                                int is_last, int subsampling_x,
                                int subsampling_y) {
  PC_TREE *pc_tree = NULL;
  struct aom_internal_error_info error;

  AOM_CHECK_MEM_ERROR(&error, pc_tree, aom_calloc(1, sizeof(*pc_tree)));

  pc_tree->mi_row = mi_row;
  pc_tree->mi_col = mi_col;
  pc_tree->parent = parent;
  pc_tree->index = index;
  pc_tree->partitioning = PARTITION_NONE;
  if (parent) {
    pc_tree->region_type = parent->region_type;
    if (parent->extended_sdp_allowed_flag)
      pc_tree->extended_sdp_allowed_flag =
          is_extended_sdp_allowed(1, parent->block_size, parent_partition);
    else
      pc_tree->extended_sdp_allowed_flag = 0;
  } else {
    pc_tree->extended_sdp_allowed_flag = 1;
  }
  pc_tree->block_size = bsize;
  pc_tree->is_last_subblock = is_last;
  av1_invalid_rd_stats(&pc_tree->rd_cost);
  av1_invalid_rd_stats(&pc_tree->none_rd);
  pc_tree->skippable = false;
  set_chroma_ref_info(tree_type, mi_row, mi_col, index, bsize,
                      &pc_tree->chroma_ref_info,
                      parent ? &parent->chroma_ref_info : NULL,
                      parent ? parent->block_size : BLOCK_INVALID,
                      parent_partition, subsampling_x, subsampling_y);
  pc_tree->none[INTRA_REGION] = NULL;
  pc_tree->none[MIXED_INTER_INTRA_REGION] = NULL;
  pc_tree->none_chroma = NULL;

  for (REGION_TYPE cur_region_type = INTRA_REGION;
       cur_region_type < REGION_TYPES; ++cur_region_type) {
    for (int i = 0; i < 2; ++i) {
      pc_tree->horizontal[cur_region_type][i] = NULL;
      pc_tree->vertical[cur_region_type][i] = NULL;
    }
    for (int i = 0; i < 4; ++i) {
      pc_tree->horizontal4a[cur_region_type][i] = NULL;
      pc_tree->horizontal4b[cur_region_type][i] = NULL;
      pc_tree->vertical4a[cur_region_type][i] = NULL;
      pc_tree->vertical4b[cur_region_type][i] = NULL;
    }
    for (int i = 0; i < 4; ++i) {
      pc_tree->horizontal3[cur_region_type][i] = NULL;
      pc_tree->vertical3[cur_region_type][i] = NULL;
    }
    for (int i = 0; i < 4; ++i) {
      pc_tree->split[cur_region_type][i] = NULL;
    }
  }
  return pc_tree;
}

#define FREE_PMC_NODE(CTX)         \
  do {                             \
    av1_free_pmc(CTX, num_planes); \
    CTX = NULL;                    \
  } while (0)

void av1_free_pc_tree_recursive(PC_TREE *pc_tree, int num_planes, int keep_best,
                                int keep_none) {
  if (pc_tree == NULL) return;

  const PARTITION_TYPE partition = pc_tree->partitioning;

  PC_TREE *parent = pc_tree->parent;
  if (!keep_none &&
      (!keep_best || (pc_tree->region_type != INTRA_REGION ||
                      (parent && parent->region_type == INTRA_REGION))))
    FREE_PMC_NODE(pc_tree->none_chroma);

  for (int cur_region_type = INTRA_REGION; cur_region_type < REGION_TYPES;
       ++cur_region_type) {
    if (!keep_none && (!keep_best || (partition != PARTITION_NONE)))
      FREE_PMC_NODE(pc_tree->none[cur_region_type]);

    for (int i = 0; i < 2; ++i) {
      if ((!keep_best || (partition != PARTITION_HORZ) ||
           (cur_region_type != pc_tree->region_type)) &&
          pc_tree->horizontal[cur_region_type][i] != NULL) {
        av1_free_pc_tree_recursive(pc_tree->horizontal[cur_region_type][i],
                                   num_planes, 0, 0);
        pc_tree->horizontal[cur_region_type][i] = NULL;
      }
      if ((!keep_best || (partition != PARTITION_VERT) ||
           (cur_region_type != pc_tree->region_type)) &&
          pc_tree->vertical[cur_region_type][i] != NULL) {
        av1_free_pc_tree_recursive(pc_tree->vertical[cur_region_type][i],
                                   num_planes, 0, 0);
        pc_tree->vertical[cur_region_type][i] = NULL;
      }
    }

    if (!keep_best || (partition != PARTITION_HORZ_4A) ||
        (cur_region_type != pc_tree->region_type)) {
      for (int i = 0; i < 4; ++i) {
        if (pc_tree->horizontal4a[cur_region_type][i] != NULL) {
          av1_free_pc_tree_recursive(pc_tree->horizontal4a[cur_region_type][i],
                                     num_planes, 0, 0);
          pc_tree->horizontal4a[cur_region_type][i] = NULL;
        }
      }
    }

    if (!keep_best || (partition != PARTITION_HORZ_4B) ||
        (cur_region_type != pc_tree->region_type)) {
      for (int i = 0; i < 4; ++i) {
        if (pc_tree->horizontal4b[cur_region_type][i] != NULL) {
          av1_free_pc_tree_recursive(pc_tree->horizontal4b[cur_region_type][i],
                                     num_planes, 0, 0);
          pc_tree->horizontal4b[cur_region_type][i] = NULL;
        }
      }
    }

    if (!keep_best || (partition != PARTITION_VERT_4A) ||
        (cur_region_type != pc_tree->region_type)) {
      for (int i = 0; i < 4; ++i) {
        if (pc_tree->vertical4a[cur_region_type][i] != NULL) {
          av1_free_pc_tree_recursive(pc_tree->vertical4a[cur_region_type][i],
                                     num_planes, 0, 0);
          pc_tree->vertical4a[cur_region_type][i] = NULL;
        }
      }
    }

    if (!keep_best || (partition != PARTITION_VERT_4B) ||
        (cur_region_type != pc_tree->region_type)) {
      for (int i = 0; i < 4; ++i) {
        if (pc_tree->vertical4b[cur_region_type][i] != NULL) {
          av1_free_pc_tree_recursive(pc_tree->vertical4b[cur_region_type][i],
                                     num_planes, 0, 0);
          pc_tree->vertical4b[cur_region_type][i] = NULL;
        }
      }
    }
    for (int i = 0; i < 4; ++i) {
      if ((!keep_best || (partition != PARTITION_HORZ_3) ||
           (cur_region_type != pc_tree->region_type)) &&
          pc_tree->horizontal3[cur_region_type][i] != NULL) {
        av1_free_pc_tree_recursive(pc_tree->horizontal3[cur_region_type][i],
                                   num_planes, 0, 0);
        pc_tree->horizontal3[cur_region_type][i] = NULL;
      }
      if ((!keep_best || (partition != PARTITION_VERT_3) ||
           (cur_region_type != pc_tree->region_type)) &&
          pc_tree->vertical3[cur_region_type][i] != NULL) {
        av1_free_pc_tree_recursive(pc_tree->vertical3[cur_region_type][i],
                                   num_planes, 0, 0);
        pc_tree->vertical3[cur_region_type][i] = NULL;
      }
    }

    if (!keep_best || (partition != PARTITION_SPLIT) ||
        (cur_region_type != pc_tree->region_type)) {
      for (int i = 0; i < 4; ++i) {
        if (pc_tree->split[cur_region_type][i] != NULL) {
          av1_free_pc_tree_recursive(pc_tree->split[cur_region_type][i],
                                     num_planes, 0, 0);
          pc_tree->split[cur_region_type][i] = NULL;
        }
      }
    }
  }  // region type index
  if (!keep_best && !keep_none) aom_free(pc_tree);
}

void av1_copy_pc_tree_recursive(MACROBLOCKD *xd, const AV1_COMMON *cm,
                                PC_TREE *dst, PC_TREE *src, int ss_x, int ss_y,
                                PC_TREE_SHARED_BUFFERS *shared_bufs,
                                TREE_TYPE tree_type, int num_planes) {
  // Copy the best partition type. For basic information like bsize and index,
  // we assume they have been set properly when initializing the dst PC_TREE
  dst->partitioning = src->partitioning;
  dst->region_type = src->region_type;
  dst->extended_sdp_allowed_flag = src->extended_sdp_allowed_flag;
  REGION_TYPE cur_region_type = src->region_type;
  dst->rd_cost = src->rd_cost;
  dst->none_rd = src->none_rd;
  dst->skippable = src->skippable;

  const BLOCK_SIZE bsize = dst->block_size;
  const BLOCK_SIZE subsize = get_partition_subsize(bsize, src->partitioning);
  const int mi_row = src->mi_row;
  const int mi_col = src->mi_col;

  PC_TREE *src_parent = src->parent;
  if ((src->region_type == INTRA_REGION &&
       (src_parent && src_parent->region_type == MIXED_INTER_INTRA_REGION))) {
    if (dst->none_chroma) av1_free_pmc(dst->none_chroma, num_planes);
    dst->none_chroma = NULL;
    if (src->none_chroma) {
      dst->none_chroma =
          av1_alloc_pmc(cm, CHROMA_PART, mi_row, mi_col, bsize, dst,
                        PARTITION_NONE, 0, ss_x, ss_y, shared_bufs);
      av1_copy_tree_context(dst->none_chroma, src->none_chroma, num_planes);
    }
  }

  switch (src->partitioning) {
    // PARTITION_NONE
    case PARTITION_NONE:
      if (dst->none[cur_region_type])
        av1_free_pmc(dst->none[cur_region_type], num_planes);
      dst->none[cur_region_type] = NULL;
      if (src->none[cur_region_type]) {
        dst->none[cur_region_type] = av1_alloc_pmc(
            cm,
            !frame_is_intra_only(cm) && cur_region_type == INTRA_REGION
                ? LUMA_PART
                : tree_type,
            mi_row, mi_col, bsize, dst, PARTITION_NONE, 0, ss_x, ss_y,
            shared_bufs);
        av1_copy_tree_context(dst->none[cur_region_type],
                              src->none[cur_region_type], num_planes);
#if CONFIG_MVP_IMPROVEMENT
        if (is_inter_block(&src->none[cur_region_type]->mic, xd->tree_type)) {
#if CONFIG_BANK_IMPROVE
          xd->mi_row = mi_row;
          xd->mi_col = mi_col;
#endif  // CONFIG_BANK_IMPROVE
#if WARP_CU_BANK
          av1_update_warp_param_bank(cm, xd, &dst->none[cur_region_type]->mic);
#endif  // WARP_CU_BANK
          if (cm->seq_params.enable_refmvbank) {
            av1_update_ref_mv_bank(cm, xd,
#if CONFIG_BANK_IMPROVE
                                   1,
#endif  // CONFIG_BANK_IMPROVE
                                   &dst->none[cur_region_type]->mic);
          }
        }
#if CONFIG_BANK_IMPROVE
        else {
          xd->mi_row = mi_row;
          xd->mi_col = mi_col;
          decide_rmb_unit_update_count(cm, xd,
                                       &dst->none[cur_region_type]->mic);
        }
#endif  // CONFIG_BANK_IMPROVE
#endif  // CONFIG_MVP_IMPROVEMENT
      }
      break;
    // PARTITION_SPLIT
    case PARTITION_SPLIT:
      if (is_partition_valid(bsize, PARTITION_SPLIT)) {
        for (int i = 0; i < 4; ++i) {
          if (dst->split[cur_region_type][i]) {
            av1_free_pc_tree_recursive(dst->split[cur_region_type][i],
                                       num_planes, 0, 0);
            dst->split[cur_region_type][i] = NULL;
          }
          if (src->split[cur_region_type][i]) {
            const int x_idx = (i & 1) * (mi_size_wide[bsize] >> 1);
            const int y_idx = (i >> 1) * (mi_size_high[bsize] >> 1);
            dst->split[cur_region_type][i] = av1_alloc_pc_tree_node(
                tree_type, mi_row + y_idx, mi_col + x_idx, subsize, dst,
                PARTITION_SPLIT, i, i == 3, ss_x, ss_y);
            av1_copy_pc_tree_recursive(xd, cm, dst->split[cur_region_type][i],
                                       src->split[cur_region_type][i], ss_x,
                                       ss_y, shared_bufs, tree_type,
                                       num_planes);
          }
        }
      }
      break;
    // PARTITION_HORZ
    case PARTITION_HORZ:
      if (is_partition_valid(bsize, PARTITION_HORZ)) {
        for (int i = 0; i < 2; ++i) {
          if (dst->horizontal[cur_region_type][i]) {
            av1_free_pc_tree_recursive(dst->horizontal[cur_region_type][i],
                                       num_planes, 0, 0);
            dst->horizontal[cur_region_type][i] = NULL;
          }
          if (src->horizontal[cur_region_type][i]) {
            const int this_mi_row = mi_row + i * (mi_size_high[bsize] >> 1);
            dst->horizontal[cur_region_type][i] = av1_alloc_pc_tree_node(
                tree_type, this_mi_row, mi_col, subsize, dst, PARTITION_HORZ, i,
                i == 1, ss_x, ss_y);
            av1_copy_pc_tree_recursive(
                xd, cm, dst->horizontal[cur_region_type][i],
                src->horizontal[cur_region_type][i], ss_x, ss_y, shared_bufs,
                tree_type, num_planes);
          }
        }
      }
      break;
    // PARTITION_VERT
    case PARTITION_VERT:
      if (is_partition_valid(bsize, PARTITION_VERT)) {
        for (int i = 0; i < 2; ++i) {
          if (dst->vertical[cur_region_type][i]) {
            av1_free_pc_tree_recursive(dst->vertical[cur_region_type][i],
                                       num_planes, 0, 0);
            dst->vertical[cur_region_type][i] = NULL;
          }
          if (src->vertical[cur_region_type][i]) {
            const int this_mi_col = mi_col + i * (mi_size_wide[bsize] >> 1);
            dst->vertical[cur_region_type][i] = av1_alloc_pc_tree_node(
                tree_type, mi_row, this_mi_col, subsize, dst, PARTITION_VERT, i,
                i == 1, ss_x, ss_y);
            av1_copy_pc_tree_recursive(
                xd, cm, dst->vertical[cur_region_type][i],
                src->vertical[cur_region_type][i], ss_x, ss_y, shared_bufs,
                tree_type, num_planes);
          }
        }
      }
      break;
    // PARTITION_HORZ_4A
    case PARTITION_HORZ_4A:
      if (is_partition_valid(bsize, PARTITION_HORZ_4A)) {
        const int ebh = (mi_size_high[bsize] >> 3);
        const int mi_rows[4] = { mi_row, mi_row + ebh, mi_row + ebh * 3,
                                 mi_row + ebh * 7 };
        const BLOCK_SIZE bsize_big =
            get_partition_subsize(bsize, PARTITION_HORZ);
        const BLOCK_SIZE bsize_med = subsize_lookup[PARTITION_HORZ][bsize_big];
        assert(subsize == subsize_lookup[PARTITION_HORZ][bsize_med]);
        const BLOCK_SIZE subsizes[4] = { subsize, bsize_med, bsize_big,
                                         subsize };
        for (int i = 0; i < 4; ++i) {
          if (dst->horizontal4a[cur_region_type][i]) {
            av1_free_pc_tree_recursive(dst->horizontal4a[cur_region_type][i],
                                       num_planes, 0, 0);
            dst->horizontal4a[cur_region_type][i] = NULL;
          }
          if (src->horizontal4a[cur_region_type][i]) {
            dst->horizontal4a[cur_region_type][i] = av1_alloc_pc_tree_node(
                tree_type, mi_rows[i], mi_col, subsizes[i], dst,
                PARTITION_HORZ_4A, i, i == 3, ss_x, ss_y);
            av1_copy_pc_tree_recursive(
                xd, cm, dst->horizontal4a[cur_region_type][i],
                src->horizontal4a[cur_region_type][i], ss_x, ss_y, shared_bufs,
                tree_type, num_planes);
          }
        }
      }
      break;
    // PARTITION_HORZ_4B
    case PARTITION_HORZ_4B:
      if (is_partition_valid(bsize, PARTITION_HORZ_4B)) {
        const int ebh = (mi_size_high[bsize] >> 3);
        const int mi_rows[4] = { mi_row, mi_row + ebh, mi_row + ebh * 5,
                                 mi_row + ebh * 7 };
        const BLOCK_SIZE bsize_big =
            get_partition_subsize(bsize, PARTITION_HORZ);
        const BLOCK_SIZE bsize_med = subsize_lookup[PARTITION_HORZ][bsize_big];
        assert(subsize == subsize_lookup[PARTITION_HORZ][bsize_med]);
        const BLOCK_SIZE subsizes[4] = { subsize, bsize_big, bsize_med,
                                         subsize };
        for (int i = 0; i < 4; ++i) {
          if (dst->horizontal4b[cur_region_type][i]) {
            av1_free_pc_tree_recursive(dst->horizontal4b[cur_region_type][i],
                                       num_planes, 0, 0);
            dst->horizontal4b[cur_region_type][i] = NULL;
          }
          if (src->horizontal4b[cur_region_type][i]) {
            dst->horizontal4b[cur_region_type][i] = av1_alloc_pc_tree_node(
                tree_type, mi_rows[i], mi_col, subsizes[i], dst,
                PARTITION_HORZ_4B, i, i == 3, ss_x, ss_y);
            av1_copy_pc_tree_recursive(
                xd, cm, dst->horizontal4b[cur_region_type][i],
                src->horizontal4b[cur_region_type][i], ss_x, ss_y, shared_bufs,
                tree_type, num_planes);
          }
        }
      }
      break;
    // PARTITION_VERT_4A
    case PARTITION_VERT_4A:
      if (is_partition_valid(bsize, PARTITION_VERT_4A)) {
        const int ebw = (mi_size_wide[bsize] >> 3);
        const int mi_cols[4] = { mi_col, mi_col + ebw, mi_col + ebw * 3,
                                 mi_col + ebw * 7 };
        const BLOCK_SIZE bsize_big =
            get_partition_subsize(bsize, PARTITION_VERT);
        const BLOCK_SIZE bsize_med = subsize_lookup[PARTITION_VERT][bsize_big];
        assert(subsize == subsize_lookup[PARTITION_VERT][bsize_med]);
        const BLOCK_SIZE subsizes[4] = { subsize, bsize_med, bsize_big,
                                         subsize };
        for (int i = 0; i < 4; ++i) {
          if (dst->vertical4a[cur_region_type][i]) {
            av1_free_pc_tree_recursive(dst->vertical4a[cur_region_type][i],
                                       num_planes, 0, 0);
            dst->vertical4a[cur_region_type][i] = NULL;
          }
          if (src->vertical4a[cur_region_type][i]) {
            dst->vertical4a[cur_region_type][i] = av1_alloc_pc_tree_node(
                tree_type, mi_row, mi_cols[i], subsizes[i], dst,
                PARTITION_VERT_4A, i, i == 3, ss_x, ss_y);
            av1_copy_pc_tree_recursive(
                xd, cm, dst->vertical4a[cur_region_type][i],
                src->vertical4a[cur_region_type][i], ss_x, ss_y, shared_bufs,
                tree_type, num_planes);
          }
        }
      }
      break;
    // PARTITION_VERT_4B
    case PARTITION_VERT_4B:
      if (is_partition_valid(bsize, PARTITION_VERT_4B)) {
        const int ebw = (mi_size_wide[bsize] >> 3);
        const int mi_cols[4] = { mi_col, mi_col + ebw, mi_col + ebw * 5,
                                 mi_col + ebw * 7 };
        const BLOCK_SIZE bsize_big =
            get_partition_subsize(bsize, PARTITION_VERT);
        const BLOCK_SIZE bsize_med = subsize_lookup[PARTITION_VERT][bsize_big];
        assert(subsize == subsize_lookup[PARTITION_VERT][bsize_med]);
        const BLOCK_SIZE subsizes[4] = { subsize, bsize_big, bsize_med,
                                         subsize };
        for (int i = 0; i < 4; ++i) {
          if (dst->vertical4b[cur_region_type][i]) {
            av1_free_pc_tree_recursive(dst->vertical4b[cur_region_type][i],
                                       num_planes, 0, 0);
            dst->vertical4b[cur_region_type][i] = NULL;
          }
          if (src->vertical4b[cur_region_type][i]) {
            dst->vertical4b[cur_region_type][i] = av1_alloc_pc_tree_node(
                tree_type, mi_row, mi_cols[i], subsizes[i], dst,
                PARTITION_VERT_4B, i, i == 3, ss_x, ss_y);
            av1_copy_pc_tree_recursive(
                xd, cm, dst->vertical4b[cur_region_type][i],
                src->vertical4b[cur_region_type][i], ss_x, ss_y, shared_bufs,
                tree_type, num_planes);
          }
        }
      }
      break;

    // PARTITION_HORZ_3
    case PARTITION_HORZ_3:
      if (is_partition_valid(bsize, PARTITION_HORZ_3)) {
        for (int i = 0; i < 4; ++i) {
          const BLOCK_SIZE this_subsize =
              get_h_partition_subsize(bsize, i, PARTITION_HORZ_3);
          const int offset_mr =
              get_h_partition_offset_mi_row(bsize, i, PARTITION_HORZ_3);
          const int offset_mc =
              get_h_partition_offset_mi_col(bsize, i, PARTITION_HORZ_3);
          if (dst->horizontal3[cur_region_type][i]) {
            av1_free_pc_tree_recursive(dst->horizontal3[cur_region_type][i],
                                       num_planes, 0, 0);
            dst->horizontal3[cur_region_type][i] = NULL;
          }
          if (src->horizontal3[cur_region_type][i]) {
            dst->horizontal3[cur_region_type][i] = av1_alloc_pc_tree_node(
                tree_type, mi_row + offset_mr, mi_col + offset_mc, this_subsize,
                dst, PARTITION_HORZ_3, i, i == 3, ss_x, ss_y);
            av1_copy_pc_tree_recursive(
                xd, cm, dst->horizontal3[cur_region_type][i],
                src->horizontal3[cur_region_type][i], ss_x, ss_y, shared_bufs,
                tree_type, num_planes);
          }
        }
      }
      break;
    // PARTITION_VERT_3
    case PARTITION_VERT_3:
      if (is_partition_valid(bsize, PARTITION_VERT_3)) {
        for (int i = 0; i < 4; ++i) {
          const BLOCK_SIZE this_subsize =
              get_h_partition_subsize(bsize, i, PARTITION_VERT_3);
          const int offset_mr =
              get_h_partition_offset_mi_row(bsize, i, PARTITION_VERT_3);
          const int offset_mc =
              get_h_partition_offset_mi_col(bsize, i, PARTITION_VERT_3);
          if (dst->vertical3[cur_region_type][i]) {
            av1_free_pc_tree_recursive(dst->vertical3[cur_region_type][i],
                                       num_planes, 0, 0);
            dst->vertical3[cur_region_type][i] = NULL;
          }
          if (src->vertical3[cur_region_type][i]) {
            dst->vertical3[cur_region_type][i] = av1_alloc_pc_tree_node(
                tree_type, mi_row + offset_mr, mi_col + offset_mc, this_subsize,
                dst, PARTITION_VERT_3, i, i == 3, ss_x, ss_y);
            av1_copy_pc_tree_recursive(
                xd, cm, dst->vertical3[cur_region_type][i],
                src->vertical3[cur_region_type][i], ss_x, ss_y, shared_bufs,
                tree_type, num_planes);
          }
        }
      }
      break;
    default: assert(0 && "Not a valid partition."); break;
  }
}

static AOM_INLINE int get_pc_tree_nodes(const BLOCK_SIZE sb_size,
                                        int stat_generation_stage) {
  const int is_sb_size_128 = sb_size == BLOCK_128X128;
  const int is_sb_size_256 = sb_size == BLOCK_256X256;
  const int tree_nodes_inc = is_sb_size_256   ? (1024 + 4 * 1024)
                             : is_sb_size_128 ? 1024
                                              : 0;
  const int tree_nodes =
      stat_generation_stage ? 1 : (tree_nodes_inc + 256 + 64 + 16 + 4 + 1);
  return tree_nodes;
}

void av1_setup_sms_tree(AV1_COMP *const cpi, ThreadData *td) {
  AV1_COMMON *const cm = &cpi->common;
  const int stat_generation_stage = is_stat_generation_stage(cpi);
  const int is_sb_size_128 = cm->sb_size == BLOCK_128X128;
  const int is_sb_size_256 = cm->sb_size == BLOCK_256X256;
  const int tree_nodes = get_pc_tree_nodes(cm->sb_size, stat_generation_stage);
  int sms_tree_index = 0;
  SIMPLE_MOTION_DATA_TREE *this_sms;
  int square_index = 1;
  int nodes;

  aom_free(td->sms_tree);
  CHECK_MEM_ERROR(cm, td->sms_tree,
                  aom_calloc(tree_nodes, sizeof(*td->sms_tree)));
  this_sms = &td->sms_tree[0];

  if (!stat_generation_stage) {
    const int leaf_factor = is_sb_size_256 ? 16 : is_sb_size_128 ? 4 : 1;

    const int leaf_nodes = 256 * leaf_factor;

    // Sets up all the leaf nodes in the tree.
    for (sms_tree_index = 0; sms_tree_index < leaf_nodes; ++sms_tree_index) {
      SIMPLE_MOTION_DATA_TREE *const tree = &td->sms_tree[sms_tree_index];
      tree->block_size = square[0];
    }

    // Each node has 4 leaf nodes, fill each block_size level of the tree
    // from leafs to the root.
    for (nodes = leaf_nodes >> 2; nodes > 0; nodes >>= 2) {
      for (int i = 0; i < nodes; ++i) {
        SIMPLE_MOTION_DATA_TREE *const tree = &td->sms_tree[sms_tree_index];
        tree->block_size = square[square_index];
        for (int j = 0; j < 4; j++) tree->split[j] = this_sms++;
        ++sms_tree_index;
      }
      ++square_index;
    }
  } else {
    // Allocation for firstpass/LAP stage
    // TODO(Mufaddal): refactor square_index to use a common block_size macro
    // from firstpass.c
    SIMPLE_MOTION_DATA_TREE *const tree = &td->sms_tree[sms_tree_index];
    square_index = 2;
    tree->block_size = square[square_index];
  }

  // Set up the root node for the largest superblock size
  td->sms_root = &td->sms_tree[tree_nodes - 1];
}

void av1_free_sms_tree(ThreadData *td) {
  if (td->sms_tree != NULL) {
    aom_free(td->sms_tree);
    td->sms_tree = NULL;
  }
}

void av1_setup_sms_bufs(AV1_COMMON *cm, ThreadData *td) {
  CHECK_MEM_ERROR(cm, td->sms_bufs, aom_malloc(sizeof(*td->sms_bufs)));
}

void av1_free_sms_bufs(ThreadData *td) {
  if (td->sms_bufs != NULL) {
    aom_free(td->sms_bufs);
    td->sms_bufs = NULL;
  }
}

PC_TREE *counterpart_from_different_partition(PC_TREE *pc_tree,
                                              const PC_TREE *target);

static PC_TREE *look_for_counterpart_helper(PC_TREE *cur,
                                            const PC_TREE *target) {
  if (cur == NULL || cur == target) return NULL;

  const BLOCK_SIZE current_bsize = cur->block_size;
  const BLOCK_SIZE target_bsize = target->block_size;
  // Note: To find the counterpart block, we don't actually need to check the
  // whole chroma_ref_info -- checking bsize_base should suffice due to
  // constraints in the partitioning scheme. However, we are checking the whole
  // struct for now as we are still experimenting with new partition schemes.
  if (current_bsize == target_bsize &&
      memcmp(&cur->chroma_ref_info, &target->chroma_ref_info,
             sizeof(cur->chroma_ref_info)) == 0) {
    return cur;
  } else {
    if (mi_size_wide[current_bsize] >= mi_size_wide[target_bsize] &&
        mi_size_high[current_bsize] >= mi_size_high[target_bsize]) {
      return counterpart_from_different_partition(cur, target);
    } else {
      return NULL;
    }
  }
}

/*! \brief Searches for a partition tree node that does not change any context
 * and has the same position and bsize as the current target. */
PC_TREE *counterpart_from_different_partition(PC_TREE *pc_tree,
                                              const PC_TREE *target) {
  if (pc_tree == NULL || pc_tree == target) return NULL;

  PC_TREE *result;
  REGION_TYPE cur_region_type = pc_tree->region_type;
  result =
      look_for_counterpart_helper(pc_tree->split[cur_region_type][0], target);
  if (result) return result;
  result = look_for_counterpart_helper(pc_tree->horizontal[cur_region_type][0],
                                       target);
  if (result) return result;
  result = look_for_counterpart_helper(pc_tree->vertical[cur_region_type][0],
                                       target);
  if (result) return result;
  result = look_for_counterpart_helper(
      pc_tree->horizontal4a[cur_region_type][0], target);
  if (result) return result;
  result = look_for_counterpart_helper(
      pc_tree->horizontal4b[cur_region_type][0], target);
  if (result) return result;
  result = look_for_counterpart_helper(pc_tree->vertical4a[cur_region_type][0],
                                       target);
  if (result) return result;
  result = look_for_counterpart_helper(pc_tree->vertical4b[cur_region_type][0],
                                       target);
  if (result) return result;
  result = look_for_counterpart_helper(pc_tree->horizontal3[cur_region_type][0],
                                       target);
  if (result) return result;
  result = look_for_counterpart_helper(pc_tree->vertical3[cur_region_type][0],
                                       target);
  if (result) return result;
  return NULL;
}

/*! \brief Searches for a partition tree node with the same context, position,
 * and bsize as the current node. */
PC_TREE *av1_look_for_counterpart_block(PC_TREE *pc_tree) {
  if (!pc_tree) return 0;

  // Find the highest possible common parent node
  PC_TREE *current = pc_tree;
  while (current->index == 0 && current->parent) {
    current = current->parent;
  }

  // Search from the highest common ancestor
  return counterpart_from_different_partition(current, pc_tree);
}
