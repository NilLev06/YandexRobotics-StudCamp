#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec conda run --no-capture-output -n gfsx-real \
  python "$SCRIPT_DIR/gfsx_real_vision.py" "$@"
