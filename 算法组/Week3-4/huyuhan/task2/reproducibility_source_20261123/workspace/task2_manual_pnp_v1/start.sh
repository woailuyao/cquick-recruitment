#!/usr/bin/env bash
set -euo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd -P)"
exec "$here/../.venv/bin/python" -B "$here/prototype.py" "$@"
