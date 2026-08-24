import os
# from cv2 import aruco

# Robot Params #
# Defaults only: the NUC_IP / ROBOT_IP / LAPTOP_IP environment variables
# (set by ansible / the systemd units / docker-compose) override them below.
# Keep these lines plain `name = "literal"` — the setup scripts scrape them
# with awk.
nuc_ip = "172.16.0.2"
robot_ip = "172.16.0.4"
laptop_ip = "172.16.0.1"

# Environment overrides. Written so the setup scripts' awk scrape of
# top-level assignments skips them; an empty value (docker-compose passing an
# unset host variable) counts as unset.
for _name, _env in (("nuc_ip", "NUC_IP"), ("robot_ip", "ROBOT_IP"), ("laptop_ip", "LAPTOP_IP")):
    if os.environ.get(_env):
        globals()[_name] = os.environ[_env]
sudo_password = "robot"
robot_type = "panda"  # 'panda' or 'fr3'
robot_serial_number = "295341-1324910"

# Camera ID's #
hand_camera_id = ""
varied_camera_1_id = ""
varied_camera_2_id = ""

# Charuco Board Params #
CHARUCOBOARD_ROWCOUNT = 9
CHARUCOBOARD_COLCOUNT = 14
CHARUCOBOARD_CHECKER_SIZE = 0.020
CHARUCOBOARD_MARKER_SIZE = 0.016
#ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_100)

# Ubuntu Pro Token (RT PATCH) #
ubuntu_pro_token = ""

# Code Version [DONT CHANGE] #
droid_version = "1.3"

