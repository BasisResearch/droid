import logging
import os

import zerorpc
from omegaconf import OmegaConf

from droid.franka.robot import FrankaRobot
from droid.misc import parameters
from polymetis.utils.data_dir import PKG_ROOT_DIR

CONF_DIR = os.path.abspath(os.path.join(PKG_ROOT_DIR, "..", "..", "conf"))

log = logging.getLogger("droid.server")


def log_effective_config():
    """Log the configuration the server is actually running with, so running
    against baked-in defaults (e.g. a missing bind mount) is visible."""
    log.info("droid parameters from %s", parameters.__file__)
    for name in ("robot_type", "robot_serial_number", "nuc_ip", "robot_ip", "laptop_ip"):
        log.info("  %s = %r", name, getattr(parameters, name, None))

    for rel, robot_ip_key in (
        ("robot_client/franka_hardware.yaml", ("robot_client", "executable_cfg", "robot_ip")),
        ("gripper/franka_hand.yaml", ("gripper", "executable_cfg", "robot_ip")),
        ("robot_model/franka_panda.yaml", None),
    ):
        path = os.path.join(CONF_DIR, rel)
        if not os.path.exists(path):
            log.warning("polymetis conf missing: %s", path)
            continue
        line = f"polymetis conf {path}"
        if robot_ip_key is not None:
            node = OmegaConf.load(path)
            for key in robot_ip_key:
                node = node[key]
            line += f" (robot_ip = {node!r})"
        log.info(line)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s %(message)s")
    log_effective_config()
    robot_client = FrankaRobot()
    try:
        robot_client.launch_robot()
        log.info("connected to robot and gripper controllers")
    except Exception as e:
        log.warning("controllers not reachable yet (%s); will retry on each call", e)
    s = zerorpc.Server(robot_client)
    s.bind("tcp://0.0.0.0:4242")
    log.info("serving on tcp://0.0.0.0:4242")
    s.run()
