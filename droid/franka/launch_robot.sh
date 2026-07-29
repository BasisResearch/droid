#!/usr/bin/env bash
set -e

PIXI=/home/robot/.pixi/bin/pixi
FAIRO=/home/robot/droid/droid/fairo

exec "$PIXI" run \
  --manifest-path "$FAIRO/pixi.toml" \
  -e polymetis-local \
  env HYDRA_FULL_ERROR=1 \
  launch_robot.py \
  robot_client=franka_hardware \
  robot_client.executable_cfg.robot_ip=172.16.0.4