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

def prediction_mode_symbol_filter(symbol: Symbol):
  return symbol.source_function in (
      "read_inter_mode",
      "read_intra_luma_mode",
      "read_drl_index",
      "read_inter_compound_mode",
  )

class PredictionModeExtractor(CodingUnitExtractor):
  PredictionMode = collections.namedtuple(
      "PredictionMode",
      ["width", "height", "block_size", "mode", "mode_bits", "is_intra_frame", "stream_path"],
  )

  def sample(self, coding_unit: CodingUnit):
    stream_path = coding_unit.frame.proto.stream_params.stream_path
    width = coding_unit.rect.width
    height = coding_unit.rect.height
    block_size = f"{width}x{height}"
    mode = coding_unit.get_prediction_mode()
    mode_bits = sum(
        sym.bits
        for sym in coding_unit.get_symbols(prediction_mode_symbol_filter)
    )
    is_intra_frame = coding_unit.frame.is_intra_frame
    yield self.PredictionMode(width, height, block_size, mode, mode_bits, is_intra_frame, stream_path)
