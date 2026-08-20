#!/usr/bin/env bash
# Container replacement for droid/franka/launch_robot.sh (which targets the
# bare-metal pixi setup), run as the droid-robot quadlet unit. Wrapped in
# supervise_launch.sh so the service waits for the robot to become reachable
# instead of dying while it is off. ROBOT_IP (overridable from the unit's
# Environment=) feeds both the supervisor's reachability probe and the
# polymetis conf override, so the two can never disagree.
export ROBOT_IP=${ROBOT_IP:-172.16.0.4}
exec "$(dirname "$0")/supervise_launch.sh" 1337 50051 franka_panda_cl \
  launch_robot.py robot_client=franka_hardware \
  robot_client.executable_cfg.robot_ip="$ROBOT_IP"
