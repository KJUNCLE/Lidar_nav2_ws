#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# CPU-only 实车部署默认只构建车端必需包，避免 CUDA/Open3D/RTAB-Map/仿真包进入构建。
exec "$SCRIPT_DIR/build_cpu_real.sh" "$@"
