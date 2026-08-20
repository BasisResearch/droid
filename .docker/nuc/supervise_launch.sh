#!/usr/bin/env bash
# Keep a polymetis launcher running without ever failing the container: wait
# until the robot answers on its FCI port, run the launcher, and go back to
# waiting when it exits (robot powered off, unrecoverable fault, ...). The
# quadlet units thus stay active while the robot is down instead of cycling
# through failed states — Restart=always in the unit is only a backstop.
#
# The TCP probe is an accurate readiness gate: libfranka's command ports
# (robot 1337, gripper 1338) only accept connections once FCI is available.
#
# Usage: supervise_launch.sh <fci_port> <launcher> [args...]
set -u

ROBOT_IP=${ROBOT_IP:-172.16.0.4}  # must match robot_ip in the polymetis conf
PROBE_INTERVAL=5
RESTART_DELAY=5

PORT=$1
shift

reachable() {
    timeout 1 bash -c "exec 3<>/dev/tcp/$ROBOT_IP/$PORT" 2>/dev/null
}

# job control gives the launcher its own process group, so SIGTERM from a
# container stop reaches micromamba's whole subtree, letting launch_robot.py
# run its cleanup (killing the realtime control server it spawned)
set -m

child=
term() {
    trap - TERM INT
    if [ -n "$child" ]; then
        kill -TERM -- "-$child" 2>/dev/null
        wait "$child" 2>/dev/null
    fi
    exit 0
}
trap term TERM INT

while true; do
    if ! reachable; then
        echo "[supervise] robot at $ROBOT_IP:$PORT not reachable; waiting..."
        until reachable; do
            sleep "$PROBE_INTERVAL"
        done
    fi

    echo "[supervise] robot at $ROBOT_IP:$PORT is reachable; starting: $*"
    /usr/local/bin/micromamba run -r /opt/micromamba -n polymetis-local \
        env HYDRA_FULL_ERROR=1 "$@" &
    child=$!
    wait "$child"
    status=$?
    child=

    echo "[supervise] launcher exited with status $status; restarting in ${RESTART_DELAY}s"
    sleep "$RESTART_DELAY"
done
