#!/usr/bin/env python3
## Copyright (c) 2023, Alliance for Open Media. All rights reserved
##
## This source code is subject to the terms of the BSD 3-Clause Clear License and the
## Alliance for Open Media Patent License 1.0. If the BSD 3-Clause Clear License was
## not distributed with this source code in the LICENSE file, you can obtain it
## at aomedia.org/license/software-license/bsd-3-c-c/.  If the Alliance for Open Media Patent
## License 1.0 was not distributed with this source code in the PATENTS file, you
## can obtain it at aomedia.org/license/patent-license/.
##
import collections

from avm_stats.extract_proto import *
from avm_stats.frame_visualizations import *
from avm_stats.proto_helpers import *
from avm_stats.stats_aggregation import *

def iter_partitions(partition_block):
  yield partition_block
  if not partition_block.is_leaf_node:
    for child in partition_block.children:
      yield from iter_partitions(child)

class PartitionTypeExtractor(SuperblockExtractor):
  PartitionType = collections.namedtuple(
      "PartitionType",
      ["width", "height", "block_size", "partition_type", "is_intra_frame", "stream_name"],
  )

  def sample(self, superblock: Superblock):
    stream_name = superblock.frame.proto.stream_params.stream_name.removesuffix(".bin")
    is_intra_frame = superblock.frame.is_intra_frame
    for partition_block in iter_partitions(superblock.proto.luma_partition_tree):
      width = partition_block.size.width
      height = partition_block.size.height
      block_size = f"{width}x{height}"
      partition_type = partition_block.partition_type
      partition_type_name = superblock.frame.proto.enum_mappings.partition_type_mapping[partition_type]
      yield self.PartitionType(width, height, block_size, partition_type_name, is_intra_frame, stream_name)
