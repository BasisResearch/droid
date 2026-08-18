# Overview

This directory contains the Docker setups for the two machines on the platform: the control server (nuc) and the client machine (laptop).

    ├── nuc                                      # control server (Franka Panda)
    ├──── Dockerfile.nuc                         # image definition
    ├──── docker-compose-nuc.yaml                # runtime contract (mounts, networking, RT limits)
    ├──── launch_robot.sh                        # container replacement for droid/franka/launch_robot.sh
    ├──── launch_gripper.sh                      # container replacement for droid/franka/launch_gripper.sh
    ├──── droid-nuc.build                        # quadlet build unit (systemd-managed deployment)
    ├──── droid-nuc.container                    # quadlet container unit (systemd-managed deployment)
    ├── laptop                                   # laptop docker setup files
    ├──── Dockerfile.laptop                      # laptop image definition
    ├──── docker-compose-laptop.yaml             # laptop container deployment settings
    ├──── entrypoint.sh                          # script that is run on entrypoint of Docker container

# NUC control server

The NUC image targets the **Franka Panda only** (libfranka 0.9.0, pinned in the Dockerfile).

## What is baked into the image (build time)

* The droid codebase and the `droid/fairo` submodule, installed into the `polymetis-local` conda environment (managed by micromamba, whose solver resolves the environment in minutes rather than the hours classic conda takes).
* Compiled libfranka 0.9.0 and polymetis.
* The polymetis configuration — robot IP, controller gains, safety limits — which lives in the fairo submodule at `droid/fairo/polymetis/polymetis/conf/` (`robot_client/franka_hardware.yaml`, `robot_model/franka_panda.yaml`, `gripper/franka_hand.yaml`). To change these, edit them there and rebuild; the image is the single source of truth at runtime.
* Container-specific launch scripts from this directory, which replace the bare-metal pixi scripts in `droid/franka/` (those hardcode `/home/robot` paths and a pixi manifest that only exists on the physical NUC). The entrypoint itself is inlined in the Dockerfile: `tini` starts the zerorpc control server (`scripts/server/run_server.py`) in the `polymetis-local` conda environment.

Submodules must be checked out before building: `git submodule update --init --recursive`.

## What the container needs at runtime (docker-compose-nuc.yaml)

* **Mount:** `droid/misc/parameters.py` → `/app/droid/misc/parameters.py`. Machine-specific parameters, read at runtime (the server uses `sudo_password` to launch the real-time controller processes). Mounted so it can be edited without a rebuild.
* **Host networking:** the laptop connects to the zerorpc server on port 4242 and to polymetis gRPC on 50051; libfranka connects out to the robot control box at its static IP.
* **Real-time scheduling:** `cap_add: SYS_NICE` plus `ulimits` `rtprio: 99` and `memlock: -1` — required for the 1 kHz libfranka control loop (SCHED_FIFO threads, and `mlockall` of the full libtorch-loaded process). The host kernel must be RT-patched (see `scripts/setup/nuc_setup.sh`).
* **Device:** `/dev/cpu_dma_latency`, which the RT setup writes to pin CPU C-states — the only device node the stack touches; the robot and gripper are network clients. No `privileged` needed.
* **`restart: always`** so the control server comes back up on boot.

## Build and run

```bash
cd .docker/nuc
docker compose -f docker-compose-nuc.yaml build
docker compose -f docker-compose-nuc.yaml up
```

No environment variables are required; the compose file is self-contained.

# Laptop Setup

In order to set up the user client on your laptop run `sudo ./laptop_setup.sh` from this [directory](https://github.com/droid-dataset/droid/tree/main/scripts/setup). Running through all the steps in this script will install host system dependencies and ensure the user client can be run in a docker container.

Further details can be found in the [README.md](https://github.com/droid-dataset/droid/tree/main/scripts/setup/README.md) in the `scripts/setup` directory.

# Docker resources

* [Docker Overview](https://docs.docker.com/get-started/overview/)
* [Dockerfile Reference](https://docs.docker.com/engine/reference/builder/)
* [Docker Compose Overview](https://docs.docker.com/compose/)
