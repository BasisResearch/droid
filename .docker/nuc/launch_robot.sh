#!/usr/bin/env bash
set -e

# Container replacement for droid/franka/launch_robot.sh (which targets the
# bare-metal pixi setup). Invoked by droid.franka.robot through `sudo -S bash`,
# which strips the environment — hence the absolute micromamba path and the
# explicit root prefix. robot_ip and controller settings come from the
# polymetis conf baked into the image (droid/fairo/polymetis/polymetis/conf).
exec /usr/local/bin/micromamba run -r /opt/micromamba -n polymetis-local \
  env HYDRA_FULL_ERROR=1 \
  launch_robot.py robot_client=franka_hardware
