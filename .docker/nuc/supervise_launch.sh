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
# Usage: supervise_launch.sh <fci_port> <server_port> <client_comm> <launcher> [args...]
#   fci_port:    robot-side FCI TCP port gating the launch (1337 arm, 1338 gripper)
#   server_port: local port the polymetis server binds (50051 robot, 50052 gripper)
#   client_comm: kernel comm name (15-char truncated) of the hardware client
#                process that must stay alive while the launcher runs
set -u

ROBOT_IP=${ROBOT_IP:-172.16.0.4}  # must match robot_ip in the polymetis conf
PROBE_INTERVAL=5
RESTART_DELAY=5
STARTUP_GRACE=30
CHECK_INTERVAL=2

FCI_PORT=$1
SERVER_PORT=$2
CLIENT_COMM=$3
shift 3

reachable() {
    timeout 1 bash -c "exec 3<>/dev/tcp/$ROBOT_IP/$FCI_PORT" 2>/dev/null
}

reap_stale() {
    # A dead launcher can leak its polymetis server, which then blocks the
    # port forever (upstream's port-in-use assert even advises 'pkill -9
    # run_server'). The old crash-the-container behavior reaped leaks via PID
    # namespace teardown; staying alive means we must free the port ourselves.
    # Only sees this container's processes — a holder elsewhere (host, other
    # container) is out of reach and keeps tripping the launcher's own check.
    if fuser -k -9 -n tcp "$SERVER_PORT" 2>/dev/null; then
        echo "[supervise] killed stale process holding port $SERVER_PORT"
        sleep 1
    fi
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
        echo "[supervise] robot at $ROBOT_IP:$FCI_PORT not reachable; waiting..."
        until reachable; do
            sleep "$PROBE_INTERVAL"
        done
    fi

    reap_stale

    echo "[supervise] robot at $ROBOT_IP:$FCI_PORT is reachable; starting: $*"
    /usr/local/bin/micromamba run -r /opt/micromamba -n polymetis-local \
        env HYDRA_FULL_ERROR=1 "$@" &
    child=$!

    # Watch the hardware client as well as the launcher: launch_gripper.py's
    # server keeps running after its forked client dies, which would leave a
    # half-alive service that accepts commands with no hardware behind them.
    started=$SECONDS
    while kill -0 "$child" 2>/dev/null; do
        if [ $((SECONDS - started)) -gt "$STARTUP_GRACE" ] && ! pgrep -x "$CLIENT_COMM" > /dev/null; then
            echo "[supervise] hardware client ($CLIENT_COMM) is gone; recycling the launcher"
            kill -TERM -- "-$child" 2>/dev/null
            sleep 2
            kill -KILL -- "-$child" 2>/dev/null
            break
        fi
        sleep "$CHECK_INTERVAL"
    done
    wait "$child" 2>/dev/null
    status=$?
    child=

    echo "[supervise] launcher exited with status $status; restarting in ${RESTART_DELAY}s"
    sleep "$RESTART_DELAY"
done
