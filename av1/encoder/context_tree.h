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

#ifndef AOM_AV1_ENCODER_CONTEXT_TREE_H_
#define AOM_AV1_ENCODER_CONTEXT_TREE_H_

#include "config/aom_config.h"

#include "av1/common/blockd.h"
#include "av1/encoder/block.h"

#ifdef __cplusplus
extern "C" {
#endif

struct AV1_COMP;
struct AV1Common;
struct ThreadData;

typedef struct {
  tran_low_t *coeff_buf[MAX_MB_PLANE];
  tran_low_t *qcoeff_buf[MAX_MB_PLANE];
  tran_low_t *dqcoeff_buf[MAX_MB_PLANE];
} PC_TREE_SHARED_BUFFERS;

// Structure to hold snapshot of coding context during the mode picking process
typedef struct PICK_MODE_CONTEXT {
  MB_MODE_INFO mic;
#if CONFIG_C071_SUBBLK_WARPMV
  SUBMB_INFO *submic;
#endif  // CONFIG_C071_SUBBLK_WARPMV
  MB_MODE_INFO_EXT_FRAME mbmi_ext_best;
  uint8_t *color_index_map[2];
  uint8_t *blk_skip[MAX_MB_PLANE];

  tran_low_t *coeff[MAX_MB_PLANE];
  tran_low_t *qcoeff[MAX_MB_PLANE];
  tran_low_t *dqcoeff[MAX_MB_PLANE];
  uint16_t *eobs[MAX_MB_PLANE];
  uint16_t *bobs[MAX_MB_PLANE];
  uint8_t *txb_entropy_ctx[MAX_MB_PLANE];
  TX_TYPE *tx_type_map;
  CctxType *cctx_type_map;

  int num_4x4_blk;
  int num_4x4_blk_chroma;
  // For current partition, only if all Y, U, and V transform blocks'
  // coefficients are quantized to 0, skippable is set to 1.
  int skippable;
  int hybrid_pred_diff;
  int comp_pred_diff;
  int single_pred_diff;

  RD_STATS rd_stats;

  int rd_mode_is_ready;  // Flag to indicate whether rd pick mode decision has
                         // been made.
  CHROMA_REF_INFO chroma_ref_info;
  struct PC_TREE *parent;
  int index;
} PICK_MODE_CONTEXT;

typedef struct PC_TREE {
  PARTITION_TYPE partitioning;
  /*! \brief The region type used for the current block. */
  REGION_TYPE region_type;
  BLOCK_SIZE block_size;
  int extended_sdp_allowed_flag;
  PICK_MODE_CONTEXT *none[REGION_TYPES];
  // record the chroma information in intra region.
  PICK_MODE_CONTEXT *none_chroma;
#if CONFIG_EXT_RECUR_PARTITIONS
  struct PC_TREE *horizontal[REGION_TYPES][2];
  struct PC_TREE *vertical[REGION_TYPES][2];
  struct PC_TREE *horizontal4a[REGION_TYPES][4];
  struct PC_TREE *horizontal4b[REGION_TYPES][4];
  struct PC_TREE *vertical4a[REGION_TYPES][4];
  struct PC_TREE *vertical4b[REGION_TYPES][4];
  struct PC_TREE *horizontal3[REGION_TYPES][4];
  struct PC_TREE *vertical3[REGION_TYPES][4];
#else
  PICK_MODE_CONTEXT *horizontal[REGION_TYPES][2];
  PICK_MODE_CONTEXT *vertical[REGION_TYPES][2];
  PICK_MODE_CONTEXT *horizontala[REGION_TYPES][3];
  PICK_MODE_CONTEXT *horizontalb[REGION_TYPES][3];
  PICK_MODE_CONTEXT *verticala[REGION_TYPES][3];
  PICK_MODE_CONTEXT *verticalb[REGION_TYPES][3];
  PICK_MODE_CONTEXT *horizontal4[REGION_TYPES][4];
  PICK_MODE_CONTEXT *vertical4[REGION_TYPES][4];
#endif  // CONFIG_EXT_RECUR_PARTITIONS
  struct PC_TREE *split[REGION_TYPES][4];
  struct PC_TREE *parent;
  int mi_row;
  int mi_col;
  int index;
  int is_last_subblock;
  CHROMA_REF_INFO chroma_ref_info;
  RD_STATS rd_cost;
#if CONFIG_EXT_RECUR_PARTITIONS
  RD_STATS none_rd;
  bool skippable;
#endif  // CONFIG_EXT_RECUR_PARTITIONS
} PC_TREE;

typedef struct SIMPLE_MOTION_DATA_TREE {
  BLOCK_SIZE block_size;
  PARTITION_TYPE partitioning;
  struct SIMPLE_MOTION_DATA_TREE *split[4];

  // Simple motion search_features
  FULLPEL_MV start_mvs[REF_FRAMES];
  unsigned int sms_none_feat[2];
  unsigned int sms_rect_feat[8];
  int sms_none_valid;
  int sms_rect_valid;
} SIMPLE_MOTION_DATA_TREE;

#if CONFIG_EXT_RECUR_PARTITIONS
PC_TREE *av1_look_for_counterpart_block(PC_TREE *pc_tree);
#endif  // CONFIG_EXT_RECUR_PARTITIONS

void av1_setup_shared_coeff_buffer(AV1_COMMON *cm,
                                   PC_TREE_SHARED_BUFFERS *shared_bufs);
void av1_free_shared_coeff_buffer(PC_TREE_SHARED_BUFFERS *shared_bufs);

PC_TREE *av1_alloc_pc_tree_node(TREE_TYPE tree_type, int mi_row, int mi_col,
                                BLOCK_SIZE bsize, PC_TREE *parent,
                                PARTITION_TYPE parent_partition, int index,
                                int is_last, int subsampling_x,
                                int subsampling_y);
void av1_free_pc_tree_recursive(PC_TREE *tree, int num_planes, int keep_best,
                                int keep_none);
#if CONFIG_EXT_RECUR_PARTITIONS
void av1_copy_pc_tree_recursive(MACROBLOCKD *xd, const AV1_COMMON *cm,
                                PC_TREE *dst, PC_TREE *src, int ss_x, int ss_y,
                                PC_TREE_SHARED_BUFFERS *shared_bufs,
                                TREE_TYPE tree_type, int num_planes);
#endif  // CONFIG_EXT_RECUR_PARTITIONS

PICK_MODE_CONTEXT *av1_alloc_pmc(const AV1_COMMON *cm, TREE_TYPE tree_type,
                                 int mi_row, int mi_col, BLOCK_SIZE bsize,
                                 PC_TREE *parent,
                                 PARTITION_TYPE parent_partition, int index,
                                 int subsampling_x, int subsampling_y,
                                 PC_TREE_SHARED_BUFFERS *shared_bufs);
void av1_free_pmc(PICK_MODE_CONTEXT *ctx, int num_planes);
void av1_copy_tree_context(PICK_MODE_CONTEXT *dst_ctx,
                           PICK_MODE_CONTEXT *src_ctx, int num_planes);

void av1_setup_sms_tree(struct AV1_COMP *const cpi, struct ThreadData *td);
void av1_free_sms_tree(struct ThreadData *td);
#if CONFIG_EXT_RECUR_PARTITIONS
void av1_setup_sms_bufs(struct AV1Common *cm, struct ThreadData *td);
void av1_free_sms_bufs(struct ThreadData *td);
#endif  // CONFIG_EXT_RECUR_PARTITIONS

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AOM_AV1_ENCODER_CONTEXT_TREE_H_
