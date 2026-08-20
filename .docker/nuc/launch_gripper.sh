#!/usr/bin/env bash
# Container replacement for droid/franka/launch_gripper.sh — see launch_robot.sh.
exec "$(dirname "$0")/supervise_launch.sh" 1338 50052 franka_hand_cli \
  launch_gripper.py gripper=franka_hand
