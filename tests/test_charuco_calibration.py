"""The third-person hand-eye solver recovers a known camera pose from synthetic board views."""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from droid.calibration.calibration_utils import CHARUCO_BOARD, ThirdPersonCameraCalibrator, calibration_traj
from droid.misc.transformations import change_pose_frame

# A 1280x720 landscape view whose principal point (800, 360) lies past the image height:
# an image size passed as (height, width) puts it outside the image and the solve fails.
K = np.array([[900.0, 0, 800], [0, 920, 360], [0, 0, 1]])
CAMERA_POSE = np.array([0.2, -0.1, 0.4, 0.1, -0.2, 0.3])  # camera in the robot base frame


def transform(pose: np.ndarray) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", pose[3:]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def render(gripper_pose: np.ndarray, board: np.ndarray) -> np.ndarray:
    """The board, fixed to the gripper frame, as the camera sees it."""
    board_in_camera = np.linalg.inv(transform(CAMERA_POSE)) @ transform(gripper_pose)
    rvec = Rotation.from_matrix(board_in_camera[:3, :3]).as_rotvec()
    corners = np.array([[0, 0, 0], [0.28, 0, 0], [0.28, 0.18, 0], [0, 0.18, 0]])
    projected, _ = cv2.projectPoints(corners, rvec, board_in_camera[:3, 3], K, np.zeros(5))
    source = np.array([[0, 0], [1399, 0], [1399, 899], [0, 899]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(source, projected.reshape(4, 2).astype(np.float32))
    gray = cv2.warpPerspective(board, H, (1280, 720), borderValue=(255,))
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def test_solver_recovers_the_camera_pose_from_its_first_sample_on() -> None:
    board = CHARUCO_BOARD.draw((1400, 900))
    board_in_camera = np.eye(4)
    board_in_camera[:3, 3] = [-0.14, -0.09, 0.7]
    start = transform(CAMERA_POSE) @ board_in_camera
    origin = np.r_[start[:3, 3], Rotation.from_matrix(start[:3, :3]).as_euler("xyz")]
    calibrator = ThirdPersonCameraCalibrator({"top_color": {"cameraMatrix": K, "distCoeffs": np.zeros(5)}})
    # No preview before the first sample: add_sample alone must select the view's intrinsics.
    for t in np.arange(0.0, 2 * np.pi, 0.12):
        gripper_pose = change_pose_frame(calibration_traj(t, hand_camera=False), origin)
        calibrator.add_sample("top_color", render(gripper_pose, board), gripper_pose)
    assert calibrator.is_calibration_accurate("top_color")
    solved = np.asarray(calibrator.calibrate("top_color"))
    np.testing.assert_allclose(solved[:3], CAMERA_POSE[:3], atol=0.01)
    error = Rotation.from_euler("xyz", solved[3:]).inv() * Rotation.from_euler("xyz", CAMERA_POSE[3:])
    assert error.magnitude() < 0.02
