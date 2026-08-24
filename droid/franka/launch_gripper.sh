#!/usr/bin/env bash
set -e

PIXI=/home/robot/.pixi/bin/pixi
FAIRO=/home/robot/droid/droid/fairo

# ROBOT_IP (overridable from the environment, e.g. a unit's Environment=)
# overrides the robot_ip baked into the polymetis conf.
ROBOT_IP=${ROBOT_IP:-172.16.0.4}

exec "$PIXI" run \
  --manifest-path "$FAIRO/pixi.toml" \
  -e polymetis-local \
  env HYDRA_FULL_ERROR=1 \
  launch_gripper.py gripper=franka_hand \
  gripper.executable_cfg.robot_ip="$ROBOT_IP"