#!/usr/bin/env bash
# Container replacement for droid/franka/launch_robot.sh (which targets the
# bare-metal pixi setup), run as the droid-robot quadlet unit. Wrapped in
# supervise_launch.sh so the service waits for the robot to become reachable
# instead of dying while it is off. robot_ip and controller settings come from
# the polymetis conf baked into the image (droid/fairo/polymetis/polymetis/conf).
exec "$(dirname "$0")/supervise_launch.sh" 1337 50051 franka_panda_cl \
  launch_robot.py robot_client=franka_hardware
