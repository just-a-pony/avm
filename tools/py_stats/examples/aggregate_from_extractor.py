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
from __future__ import annotations

from functools import partial
import gc
import glob
import pathlib
import sys
import tempfile
from typing import Sequence, Type

from absl import app
from absl import flags
from absl import logging
from avm_stats.extract_proto import *
from avm_stats.frame_visualizations import *
from avm_stats.proto_helpers import *
from avm_stats.stats_aggregation import *
import matplotlib.pyplot as plt
import multiprocessing
import pandas as pd

from extractors.partition_similarity_extractor import PartitionSimilarityExtractor
from extractors.partition_type_extractor import PartitionTypeExtractor
from extractors.prediction_mode_extractor import PredictionModeExtractor
from extractors.symbol_bits_extractor import SymbolBitsExtractor
from extractors.tx_type_extractor import TxTypeExtractor

_EXTRACTORS = {
    "partition_similarity": PartitionSimilarityExtractor,
    "partition_type": PartitionTypeExtractor,
    "prediction_mode": PredictionModeExtractor,
    "symbol_bits": SymbolBitsExtractor,
    "tx_type": TxTypeExtractor,
}

_STREAM_GLOB = flags.DEFINE_multi_string(
    "stream_glob", None, "Path to AVM streams."
)
flags.mark_flag_as_required("stream_glob")

_EXTRACT_PROTO_BIN = flags.DEFINE_string(
    "extract_proto_bin", None, "Path to extract_proto binary."
)
flags.mark_flag_as_required("extract_proto_bin")

_OUTPUT_CSV = flags.DEFINE_string(
    "output_csv", None, "Path to output CSV file (optional)."
)

_AGGREGATED_FIELD = flags.DEFINE_string(
    "aggregated_field", "count", "Field to aggregate on. By default will use the count."
)

_GROUP_BY = flags.DEFINE_string(
    "group_by", None, "Group by these fields (comma separated)."
)

_EXTRACTOR = flags.DEFINE_enum(
    "extractor", None, _EXTRACTORS.keys(), "Data extractor to use."
)
flags.mark_flag_as_required("extractor")

_THREADS = flags.DEFINE_integer(
    "threads", 1, "Number of parallel workers to spawn."
)

_FRAME_LIMIT = flags.DEFINE_integer(
    "frame_limit", None, "Use at most this many frames from each stream."
)

_PLOT = flags.DEFINE_multi_string(
    "plot", None, """Plot command args, e.g.: --plot 'title:"Block Sizes", field:"block_size", filter:"not is_intra_frame", limit:10'"""
)

_TMP_DIR = flags.DEFINE_string(
    "tmp_dir", None, "Temp working dir."
)

@dataclasses.dataclass
class PlotArgs:
  title: str = ""
  field: str = ""
  filter: str = ""
  limit: str = ""

  def __init__(self, args_str: str):
    def kv_pair(kv: str) -> tuple[str, str]:
      k, v = [i.strip() for i in kv.split(":")]
      if v.startswith('"') and v.endswith('"'):
        v = v[1:-1]
      return (k, v)

    for k, v in [kv_pair(kv) for kv in args_str.split(",")]:
      if not hasattr(self, k):
        raise ValueError(f"Unknown plot arg: {k}")
      setattr(self, k, v)


def filter_dataframe(df: pd.DataFrame, *, group_by: list[str], aggregated_field: str = "count", filt: str = "", limit: int | None = None):
  if filt:
    filtered_df = df.query(filt)[group_by + [aggregated_field]].groupby(group_by, as_index=False)
  else:
    filtered_df = df[group_by + [aggregated_field]].groupby(group_by, as_index=False)
  filtered_df = filtered_df.sum()
  filtered_df["percent"] = filtered_df[aggregated_field].transform(lambda x: x / x.sum() * 100)
  filtered_df = filtered_df.sort_values(by=[aggregated_field], ascending=False)
  if limit is not None:
    filtered_df_top_n = filtered_df.iloc[:limit, :]
    other_total = filtered_df.iloc[limit:, :][aggregated_field].sum()
    other_percent = filtered_df.iloc[limit:, :]["percent"].sum()
    others = pd.DataFrame({group_by[0]: ["Other"], aggregated_field: other_total, "percent": other_percent})
    filtered_df = pd.concat([filtered_df_top_n, others])
  return filtered_df


def sum_dataframe(df: pd.DataFrame, *, group_by: list[str], aggregated_field: str = "count"):
  aggregated_df = df[group_by + [aggregated_field]].groupby(group_by, as_index=False)
  aggregated_df = aggregated_df.sum()
  return aggregated_df


def create_plot(
    df: pd.DataFrame,
    plot_args: PlotArgs,
    ax: plt.Axes,
    legend_ax: plt.Axes,
    aggregated_field: str,
):
  limit = int(plot_args.limit) if plot_args.limit else None
  aggregated_df = filter_dataframe(
      df, group_by=[plot_args.field], aggregated_field=aggregated_field, filt=plot_args.filter, limit=limit)
  plot_title = plot_args.title or plot_args.field
  ax.set_title(f"{plot_title}")
  patches, _ = ax.pie(
      x=aggregated_df[aggregated_field], labels=aggregated_df[plot_args.field], autopct=None)
  labels = [f"{n} - {p:.1f}% ({v:.1f})" for n, p, v in zip(aggregated_df[plot_args.field],
                                                           aggregated_df["percent"], aggregated_df[aggregated_field])]
  legend_ax.legend(patches, labels, loc="best", ncol=2)
  legend_ax.axis("off")


def extract_to_temp_dir(stream_path: pathlib.Path, frame_limit: int | None = None) -> Iterator[proto_helpers.Frame]:
  with tempfile.TemporaryDirectory(dir=_TMP_DIR.value) as tmp_dir:
    tmp_path = pathlib.Path(tmp_dir)
    extract_proto_path = pathlib.Path(_EXTRACT_PROTO_BIN.value)
    stream_name = stream_path.stem
    output_path = tmp_path / stream_name
    try:
      output_path.mkdir()
    except FileExistsError:
      logging.fatal(f"Duplicate stream name: {stream_name}")
    yield from extract_and_load_protos(
        extract_proto_path=extract_proto_path,
        stream_path=stream_path,
        output_path=output_path,
        frame_limit=frame_limit
    )


def process_stream(stream_path: pathlib.Path, *, extractor_class: Type[Extractor]) -> pd.DataFrame:
  group_by = _GROUP_BY.value.split(",")
  frames = extract_to_temp_dir(stream_path, _FRAME_LIMIT.value)
  df = None
  for frame in frames:
    frame_df = aggregate_to_dataframe([frame], extractor_class())
    frame_df["count"] = 1
    if df is None:
      df = frame_df
    else:
      df = pd.concat([df, frame_df])
    if len(df):
      df = sum_dataframe(df, group_by=group_by, aggregated_field=_AGGREGATED_FIELD.value)
    gc.collect()
  if len(df):
    df = sum_dataframe(df, group_by=group_by, aggregated_field=_AGGREGATED_FIELD.value)
  gc.collect()
  with open(f"/opt/tmp/progress/{stream_path.stem}.txt", "w") as f:
    f.write("Done")
  return df


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  extractor_name = _EXTRACTOR.value
  try:
    extractor_class = _EXTRACTORS[extractor_name]
  except KeyError:
    print(f"Unknown extractor: {extractor_name}", file=sys.stderr)
    sys.exit(1)

  stream_paths = []
  for stream_glob in _STREAM_GLOB.value:
    for stream in glob.glob(stream_glob, recursive=True):
      path = pathlib.Path(stream)
      stream_paths.append(path)

  stream_paths = sorted(stream_paths)
  with multiprocessing.Pool(_THREADS.value) as pool:
    dfs = pool.map(
        partial(process_stream, extractor_class=extractor_class), stream_paths)

  df = pd.concat(dfs)
  group_by = _GROUP_BY.value.split(",")
  aggregated_df = sum_dataframe(df, group_by=group_by, aggregated_field=_AGGREGATED_FIELD.value)

  if _OUTPUT_CSV.value:
    aggregated_df.to_csv(_OUTPUT_CSV.value)

  plots = _PLOT.value
  if plots:
    _, axes = plt.subplots(2, len(plots), squeeze=False, figsize=(10, 5))
    for i, plot in enumerate(plots):
      args = PlotArgs(plot)
      create_plot(df, args, axes[0, i], axes[1, i], aggregated_field=_AGGREGATED_FIELD.value)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
  app.run(main)
