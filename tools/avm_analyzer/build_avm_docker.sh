#!/bin/bash
set -e
GIT_ROOT=$(git rev-parse --show-toplevel)

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --avm_build_dir) avm_build_dir="$2"; shift ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
    shift
done

if [[ -z ${avm_build_dir} ]]; then
  echo "Usage: ./build_avm_docker.sh --avm_build_dir <OUTPUT_PATH>"
  exit 1
fi
mkdir -p ${avm_build_dir}

AVM_BUILD_CMD="/scripts/build_avm.sh --avm_build_dir /avm_build --avm_source_dir /avm"

docker run -it --rm -v ${GIT_ROOT}:/avm:ro -v $(realpath ${avm_build_dir}):/avm_build avm_analyzer_runtime bash -c "${AVM_BUILD_CMD}"
