#!/usr/bin/env bash
set -e

PIXI=/home/robot/.pixi/bin/pixi
FAIRO=/home/robot/droid/droid/fairo
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

cd "$SCRIPT_DIR"

exec "$PIXI" run \
  --manifest-path "$FAIRO/pixi.toml" \
  -e polymetis-local \
  python run_server.py