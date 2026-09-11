# ROBOT SPECIFIC IMPORTS
import time

import grpc
import numpy as np
import torch
from polymetis import GripperInterface, RobotInterface

from droid.misc.subprocess_utils import run_threaded_command

# UTILITY SPECIFIC IMPORTS
from droid.misc.transformations import add_poses, euler_to_quat, quat_to_euler


class FrankaRobot:
    def __init__(self):
        self._robot = None
        self._gripper = None
        self._max_gripper_width = 0.08  # gripper.metadata.max_width
        self._controller_not_loaded = False

    def launch_controller(self):
        # No-op kept for zerorpc API compatibility: the robot and gripper
        # controllers are long-running services supervised on the NUC, so
        # clients connecting with launch=True must no longer kill and respawn
        # them. Restart a wedged controller on the NUC via systemctl instead.
        pass

    def kill_controller(self):
        # No-op: see launch_controller.
        pass

    def launch_robot(self):
        self._ensure_connected()

    def is_ready(self):
        # Whether both controllers are currently reachable; lets clients poll
        # instead of crashing into a RemoteError while the robot is off.
        try:
            self._ensure_connected()
            self._robot.get_robot_state()
            self._gripper.get_state()
            return True
        except Exception:
            return False

    def _ensure_connected(self):
        # The controller services only come up once the robot is powered on,
        # and can die and be restarted underneath us (robot power-cycled), so
        # connect lazily and leave a clean retry path for the next call.
        if self._robot is None:
            try:
                self._robot = RobotInterface(ip_address="localhost")
            except Exception as e:
                raise RuntimeError("robot controller not reachable (is the robot powered on?): %r" % (e,))

        # GripperInterface construction succeeds even with its server down, so
        # probe with a direct get_state call. Its command-executor thread dies
        # permanently once a queued command hits a gRPC error — after which
        # every blocking command would hang on the queue — so reconnect then.
        if self._gripper is not None and not self._gripper._command_thr.is_alive():
            self._gripper = None
        if self._gripper is None:
            gripper = GripperInterface(ip_address="localhost")
            try:
                gripper.get_state()
            except Exception as e:
                raise RuntimeError("gripper controller not reachable: %r" % (e,))
            self._gripper = gripper

    def _ik_solver_removed(self):
        raise NotImplementedError(
            "This command needs the mujoco-based RobotIKSolver from droid/robot_ik, which was"
            " removed as unused: velocity control and update_command are unsupported. Use"
            " blocking position commands instead, or restore droid/robot_ik from commit dbd8835."
        )

    def update_command(self, command, action_space="cartesian_velocity", gripper_action_space=None, blocking=False):
        self._ensure_connected()
        action_dict = self.create_action_dict(command, action_space=action_space, gripper_action_space=gripper_action_space)

        self.update_joints(action_dict["joint_position"], velocity=False, blocking=blocking)
        self.update_gripper(action_dict["gripper_position"], velocity=False, blocking=blocking)

        return action_dict

    def update_pose(self, command, velocity=False, blocking=False):
        self._ensure_connected()
        if velocity or not blocking:
            self._ik_solver_removed()

        pos = torch.Tensor(command[:3])
        quat = torch.Tensor(euler_to_quat(command[3:6]))
        curr_joints = self._robot.get_joint_positions()
        desired_joints = self._robot.solve_inverse_kinematics(pos, quat, curr_joints)
        self.update_joints(desired_joints, velocity=False, blocking=True)

    def update_joints(self, command, velocity=False, blocking=False, cartesian_noise=None):
        self._ensure_connected()
        if velocity:
            self._ik_solver_removed()
        if cartesian_noise is not None:
            command = self.add_noise_to_joints(command, cartesian_noise)
        command = torch.Tensor(command)

        def helper_non_blocking():
            if not self._robot.is_running_policy():
                self._controller_not_loaded = True
                self._robot.start_cartesian_impedance()
                timeout = time.time() + 5
                while not self._robot.is_running_policy():
                    time.sleep(0.01)
                    if time.time() > timeout:
                        self._robot.start_cartesian_impedance()
                        timeout = time.time() + 5

                self._controller_not_loaded = False
            try:
                self._robot.update_desired_joint_positions(command)
            except grpc.RpcError:
                pass

        if blocking:
            if self._robot.is_running_policy():
                self._robot.terminate_current_policy()
            try:
                time_to_go = self.adaptive_time_to_go(command)
                self._robot.move_to_joint_positions(command, time_to_go=time_to_go)
            except grpc.RpcError:
                pass

            self._robot.start_cartesian_impedance()
        else:
            if not self._controller_not_loaded:
                run_threaded_command(helper_non_blocking)

    def _gstate(self):
        # One-line gripper state for logs. is_moving / prev_command_successful
        # reveal the stuck-latch: a failed grasp/goto that never returns leaves
        # is_moving=True, and the C++ run loop then skips ALL new commands.
        try:
            s = self._gripper.get_state()
            return ("width=%.4f is_moving=%s is_grasped=%s prev_ok=%s"
                    % (s.width, s.is_moving, s.is_grasped, s.prev_command_successful))
        except Exception as e:
            return "state?(%r)" % (e,)

    def update_gripper(self, command, velocity=True, blocking=False):
        self._ensure_connected()
        if velocity:
            self._ik_solver_removed()

        command = float(np.clip(command, 0, 1))
        width = self._max_gripper_width * (1 - command)
        print("[gripper] goto width=%.4fm (cmd=%.3f) blocking=%s  before: %s"
              % (width, command, blocking, self._gstate()), flush=True)
        self._gripper.goto(width=width, speed=0.05, force=0.1, blocking=blocking)
        print("[gripper] goto returned; after: %s" % self._gstate(), flush=True)

    def grasp(self, speed=0.05, force=5.0, grasp_width=0.0, blocking=True):
        # Force grasp: close until contact and keep exerting force. Exposed over
        # zerorpc for clients without polymetis; grips a solid object where a
        # plain goto would just stall on it.
        self._ensure_connected()
        print("[gripper] grasp width=%.4f force=%.1f speed=%.3f blocking=%s  before: %s"
              % (grasp_width, force, speed, blocking, self._gstate()), flush=True)
        self._gripper.grasp(speed=speed, force=force, grasp_width=grasp_width, blocking=blocking)
        print("[gripper] grasp returned; after: %s" % self._gstate(), flush=True)

    def stop_gripper(self, blocking=True):
        # Unstick the gripper controller: after a grasp/goto that couldn't reach
        # its target width (blocked by an object) the controller reports failure
        # and IGNORES all future commands until stopped. Call this before the
        # next goto/grasp. Requires GripperInterface.stop (fairo PR #1417).
        self._ensure_connected()
        print("[gripper] STOP blocking=%s  before: %s" % (blocking, self._gstate()), flush=True)
        self._gripper.stop(blocking=blocking)
        print("[gripper] STOP returned; after: %s" % self._gstate(), flush=True)

    def add_noise_to_joints(self, original_joints, cartesian_noise):
        original_joints = torch.Tensor(original_joints)

        pos, quat = self._robot.robot_model.forward_kinematics(original_joints)
        curr_pose = pos.tolist() + quat_to_euler(quat).tolist()
        new_pose = add_poses(cartesian_noise, curr_pose)

        new_pos = torch.Tensor(new_pose[:3])
        new_quat = torch.Tensor(euler_to_quat(new_pose[3:]))

        noisy_joints, success = self._robot.solve_inverse_kinematics(new_pos, new_quat, original_joints)

        if success:
            desired_joints = noisy_joints
        else:
            desired_joints = original_joints

        return desired_joints.tolist()

    def get_joint_positions(self):
        self._ensure_connected()
        return self._robot.get_joint_positions().tolist()

    def get_joint_velocities(self):
        self._ensure_connected()
        return self._robot.get_joint_velocities().tolist()

    def get_gripper_position(self):
        self._ensure_connected()
        return 1 - (self._gripper.get_state().width / self._max_gripper_width)

    def get_ee_pose(self):
        self._ensure_connected()
        pos, quat = self._robot.get_ee_pose()
        angle = quat_to_euler(quat.numpy())
        return np.concatenate([pos, angle]).tolist()

    def get_robot_state(self):
        self._ensure_connected()
        robot_state = self._robot.get_robot_state()
        gripper_position = self.get_gripper_position()
        pos, quat = self._robot.robot_model.forward_kinematics(torch.Tensor(robot_state.joint_positions))
        cartesian_position = pos.tolist() + quat_to_euler(quat.numpy()).tolist()

        state_dict = {
            "cartesian_position": cartesian_position,
            "gripper_position": gripper_position,
            "joint_positions": list(robot_state.joint_positions),
            "joint_velocities": list(robot_state.joint_velocities),
            "joint_torques_computed": list(robot_state.joint_torques_computed),
            "prev_joint_torques_computed": list(robot_state.prev_joint_torques_computed),
            "prev_joint_torques_computed_safened": list(robot_state.prev_joint_torques_computed_safened),
            "motor_torques_measured": list(robot_state.motor_torques_measured),
            # libfranka's tau_ext_hat_filtered, filled every cycle by
            # franka_panda_client.cpp. The link-side external-torque estimate
            # that a guarded press needs: the measured-minus-computed residual
            # reads ~37 N at rest on this bench, which no contact threshold
            # can sit above.
            "motor_torques_external": list(robot_state.motor_torques_external),
            "prev_controller_latency_ms": robot_state.prev_controller_latency_ms,
            "prev_command_successful": robot_state.prev_command_successful,
        }

        timestamp_dict = {
            "robot_timestamp_seconds": robot_state.timestamp.seconds,
            "robot_timestamp_nanos": robot_state.timestamp.nanos,
        }

        return state_dict, timestamp_dict

    def adaptive_time_to_go(self, desired_joint_position, t_min=0, t_max=4):
        curr_joint_position = self._robot.get_joint_positions()
        displacement = desired_joint_position - curr_joint_position
        time_to_go = self._robot._adaptive_time_to_go(displacement)
        clamped_time_to_go = min(t_max, max(time_to_go, t_min))
        return clamped_time_to_go

    def create_action_dict(self, action, action_space, gripper_action_space=None, robot_state=None):
        self._ik_solver_removed()
