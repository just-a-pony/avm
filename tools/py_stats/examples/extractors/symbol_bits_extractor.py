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

class SymbolBitsExtractor(SuperblockExtractor):
  SymbolBits = collections.namedtuple(
      "SymbolBits",
      ["symbol_name", "symbol_tags", "bits", "is_intra_frame", "stream_path"],
  )

  def sample(self, superblock: Superblock):
    stream_path = superblock.frame.proto.stream_params.stream_path
    is_intra_frame = superblock.frame.is_intra_frame
    for symbol in superblock.proto.symbols:
      symbol_info = superblock.frame.proto.symbol_info[symbol.info_id]
      symbol_tags = "/".join(symbol_info.tags)
      yield self.SymbolBits(symbol_info.source_function, symbol_tags, symbol.bits, is_intra_frame, stream_path)
