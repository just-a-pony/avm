#!/bin/bash
set -e
GIT_ROOT=$(git rev-parse --show-toplevel)

port=8080
build_avm=0
build_docker=1
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --port) port="$2"; shift ;;
        --streams_dir) streams_dir="$2"; shift ;;
        --avm_build_dir) build_avm=0; avm_build_dir="$2"; shift ;;
        --build_avm_standalone) build_avm=1 ;;
        --nobuild) build_docker=0 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
    shift
done

if [[ -z ${streams_dir} ]]; then
  echo "Usage: ./launch_server_docker.sh --streams_dir <STREAMS_PATH> [--nobuild] [--avm_build_dir <CUSTOM_AVM_BUILD_DIR>] [--port <PORT>] [--build_avm_standalone]"
  exit 1
fi
# Note: When switching between local and docker builds, there may be permissions errors on the streams dir since the file owner with docker will be root.
# Wiping the dir before switching is recommended.
mkdir -p ${streams_dir}

if [[ ${build_docker} -eq 1 ]]; then
# Build the runtime base image first. This contains no Rust dependencies, only what's needed to build libavm.
docker build -t avm_analyzer_runtime -f Dockerfile.runtime ${GIT_ROOT}
docker build -t avm_analyzer  -f Dockerfile.builder ${GIT_ROOT}
fi

if [[ ${build_avm} -eq 1 ]]; then
./build_avm_docker.sh --avm_build_dir ${avm_build_dir}
fi

avm_build_dir_mount=()
if [[ -n ${avm_build_dir} ]]; then
avm_build_dir_mount=("-v $(realpath ${avm_build_dir}):/avm_build:ro")
fi

SERVER_CMD="/app/avm-analyzer-server --extract-proto /avm_build/extract_proto\
  --dump-obu /avm_build/tools/dump_obu --working-dir /streams\
  --frontend-root /app/dist --ip 0.0.0.0 --port 8080"


docker run -it --init --rm -p 127.0.0.1:${port}:8080 -v ${streams_dir}:/streams ${avm_build_dir_mount[@]} avm_analyzer ${SERVER_CMD}
