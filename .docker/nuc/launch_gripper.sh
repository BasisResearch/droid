#!/usr/bin/env bash
set -e

# Container replacement for droid/franka/launch_gripper.sh — see launch_robot.sh.
exec /usr/local/bin/micromamba run -r /opt/micromamba -n polymetis-local \
  env HYDRA_FULL_ERROR=1 \
  launch_gripper.py gripper=franka_hand
