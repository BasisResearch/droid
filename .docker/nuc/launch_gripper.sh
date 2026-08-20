#!/usr/bin/env bash
# Container replacement for droid/franka/launch_gripper.sh — see launch_robot.sh.
export ROBOT_IP=${ROBOT_IP:-172.16.0.4}
exec "$(dirname "$0")/supervise_launch.sh" 1338 50052 franka_hand_cli \
  launch_gripper.py gripper=franka_hand \
  gripper.executable_cfg.robot_ip="$ROBOT_IP"
