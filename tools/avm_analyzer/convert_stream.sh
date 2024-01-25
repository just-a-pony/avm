#!/bin/bash
set -xe

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -b|--avm_build_dir) avm_build_dir="$2"; shift ;;
        -s|--stream) stream="$2"; shift ;;
        -o|--output) output="$2"; shift ;;
        -y|--yuv) yuv="$2"; shift ;;
        -n|--limit) limit="$2"; shift ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
    shift
done

if [[ -z ${avm_build_dir} || -z ${stream} || -z ${output} ]]; then
  echo "Usage: ./convert_stream.sh --avm_build_dir <AVM_BUILD_PATH> --stream <STREAM_PATH> --output <ZIP_OUTPUT_PATH> [--yuv <PATH_TO_ORIG_YUV>] [--limit <NUM_FRAMES>]"
  exit 1
fi

orig_yuv_arg=""
if [[ -n ${yuv} ]]; then
orig_yuv_arg="--orig_yuv=${yuv}"
fi
limit_arg=""
if [[ -n ${limit} ]]; then
limit_arg="--limit=${limit}"
fi
tmpdir=$(mktemp -d)
${avm_build_dir}/extract_proto --stream ${stream} --output_folder ${tmpdir} ${orig_yuv_arg} ${limit_arg}
zip --filesync --recurse-paths --junk-paths ${output} ${tmpdir}/*.pb
rm ${tmpdir}/*.pb
