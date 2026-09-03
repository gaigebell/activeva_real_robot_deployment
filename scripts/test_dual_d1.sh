#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/activeva-uv-cache}"
exec uv run python -m robot.dual_d1_test "$@"
