#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export HF_ENDPOINT="https://hf-mirror.com"
export PYTORCH_JIT_LOG_LEVEL='profiling_graph_executor_impl'
export HYDRA_FULL_ERROR=1
export PYTHONPATH="${SCRIPT_DIR}/../:${PYTHONPATH:-}"

# TensorRT 路径：通过 TRT_LIB_DIR 环境变量覆盖；不存在则跳过（OpenPI 模式不需要）。
TRT_LIB_DIR="${TRT_LIB_DIR:-/data/TensorRT-10.13.0.35/lib}"
if [ -d "${TRT_LIB_DIR}" ]; then
  export LD_LIBRARY_PATH="${TRT_LIB_DIR}:/usr/lib/x86_64-linux-gnu/:${LD_LIBRARY_PATH:-}"
fi

python3 "${SCRIPT_DIR}/../run.py" "$@"
