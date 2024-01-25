#!/bin/bash
set -e

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --avm_build_dir) avm_build_dir="$2"; shift ;;
        --avm_source_dir) avm_source_dir="$2"; shift ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
    shift
done

if [[ -z ${avm_build_dir} || -z ${avm_source_dir} ]]; then
  echo "Usage: ./build_avm.sh --avm_source_dir <AVM_GIT_ROOT> --avm_build_dir <OUTPUT_PATH>"
  exit 1
fi
mkdir -p ${avm_build_dir}

cd ${avm_build_dir}
cmake ${avm_source_dir} -DCMAKE_CXX_COMPILER=g++ -DCMAKE_C_COMPILER=gcc \
  -DCMAKE_BUILD_TYPE=Release -DCONFIG_ACCOUNTING=1 -DCONFIG_INSPECTION=1 \
  -DCONFIG_EXTRACT_PROTO=1
make -j
