#!/usr/bin/env bash
set -e

PIXI=/home/robot/.pixi/bin/pixi
FAIRO=/home/robot/droid/droid/fairo

exec "$PIXI" run \
  --manifest-path "$FAIRO/pixi.toml" \
  -e polymetis-local \
  env HYDRA_FULL_ERROR=1 \
  launch_gripper.py gripper=franka_hand