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


class TxTypeExtractor(CodingUnitExtractor):
  TxSize = collections.namedtuple(
      "TxSize",
      ["width", "height", "tx_size", "tx_type", "is_intra_frame", "stream_name"],
  )

  def sample(self, coding_unit: CodingUnit):
    stream_name = coding_unit.frame.proto.stream_params.stream_name.removesuffix(".bin")
    # Luma only
    is_chroma = len(coding_unit.proto.transform_planes) == 2
    if is_chroma:
      return
    is_intra_frame = coding_unit.frame.is_intra_frame
    for transform_unit in coding_unit.proto.transform_planes[0].transform_units:
      width = transform_unit.size.width
      height = transform_unit.size.height
      tx_size = f"{width}x{height}"
      # TODO(comc): Add method on transform_unit for this.
      tx_type = transform_unit.tx_type & 0xF
      tx_type_name = coding_unit.frame.proto.enum_mappings.transform_type_mapping[tx_type]
      yield self.TxSize(width, height, tx_size, tx_type_name, is_intra_frame, stream_name)
