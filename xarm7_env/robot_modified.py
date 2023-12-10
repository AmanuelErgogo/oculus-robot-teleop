import sys
import os
import math
import numpy as np
import torch
from typing import Tuple
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI

# p.connect(p.GUI)
# p.setAdditionalSearchPath(pd.getDataPath())
# arm = XArmAPI(ip)
# arm.motion_enable(enable=True)
# arm.set_mode(0)
# arm.set_state(state=0)

def euler_to_quaternion(roll, pitch, yaw):
    """
    in degree
    """
    # Convert angles to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    # Calculate half angles
    roll_half = 0.5 * roll
    pitch_half = 0.5 * pitch
    yaw_half = 0.5 * yaw

    # Calculate sin and cos of half angles
    sin_roll_half = np.sin(roll_half)
    cos_roll_half = np.cos(roll_half)
    sin_pitch_half = np.sin(pitch_half)
    cos_pitch_half = np.cos(pitch_half)
    sin_yaw_half = np.sin(yaw_half)
    cos_yaw_half = np.cos(yaw_half)

    # Calculate quaternion components
    w = cos_roll_half * cos_pitch_half * cos_yaw_half + sin_roll_half * sin_pitch_half * sin_yaw_half
    x = sin_roll_half * cos_pitch_half * cos_yaw_half - cos_roll_half * sin_pitch_half * sin_yaw_half
    y = cos_roll_half * sin_pitch_half * cos_yaw_half + sin_roll_half * cos_pitch_half * sin_yaw_half
    z = cos_roll_half * cos_pitch_half * sin_yaw_half - sin_roll_half * sin_pitch_half * cos_yaw_half

    return w, x, y, z


def quat_from_angle_axis(angle, axis):
    """
    Convert angle-axis representation to quaternion using PyTorch.

    Args:
        angle (torch.Tensor): Tensor of angles.
        axis (torch.Tensor): Tensor of axes.

    Returns:
        torch.Tensor: Quaternion representation.
    """
    half_angle = 0.5 * angle
    sin_half = torch.sin(half_angle)

    # Quaternion components
    w = torch.cos(half_angle)
    x = sin_half * axis[0]
    y = sin_half * axis[1]
    z = sin_half * axis[2]

    quat = torch.stack([w, x, y, z])
    return quat

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    result = np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])

    return result


def degrees_to_radians(degrees_list):
    radians_list = [math.radians(deg) for deg in degrees_list]
    return radians_list

def apply_delta_pose(
    source_pos: np.ndarray, source_rot:  np.ndarray, delta_pose: np.ndarray, eps: float = 1.0e-6
) -> Tuple[ np.ndarray,  np.ndarray]:
    """Applies delta pose transformation on source pose.

    The first three elements of `delta_pose` are interpreted as cartesian position displacement.
    The remaining three elements of `delta_pose` are interpreted as orientation displacement
    in the angle-axis format.

    Args:
        frame_pos (torch.Tensor): Position of source frame. Shape: [N, 3]
        frame_rot (torch.Tensor): Quaternion orientation of source frame in (w, x, y,z).
        delta_pose (torch.Tensor): Position and orientation displacements. Shape [N, 6].
        eps (float): The tolerance to consider orientation displacement as zero.

    Returns:
        torch.Tensor: A tuple containing the displaced position and orientation frames. Shape: ([N, 3], [N, 4])
    """
    # number of poses given
    #num_poses = source_pos.shape[0]
    #device = source_pos.device

    # interpret delta_pose[:, 0:3] as target position displacements
    target_pos = source_pos + delta_pose[0:3]
    rot_delta_quat = np.array([euler_to_quaternion(roll=delta_pose[3], pitch=delta_pose[4], yaw=delta_pose[5])])
   #  # interpret delta_pose[:, 3:6] as target rotation displacements
   #  rot_actions = delta_pose[3:6]
   #  angle = torch.linalg.vector_norm(rot_actions)
   #  print (f"angles: {angle}")
   #  axis = rot_actions / angle.unsqueeze(-1)
   #  # change from axis-angle to quat convention
   #  identity_quat = torch.tensor([1.0, 0.0, 0.0, 0.0], device=device).repeat(num_poses, 1)
   #  rot_delta_quat = torch.where(
   #      angle.unsqueeze(-1).repeat(1, 4) > eps, quat_from_angle_axis(angle, axis), identity_quat
   #  )
   #  print(f"rot-delat-quat: {rot_delta_quat}")
   #  # TODO: Check if this is the correct order for this multiplication.
    print(rot_delta_quat)
    target_rot = quat_mul(rot_delta_quat[0], source_rot)

    return target_pos, target_rot


class XArm7Robot():
    def __init__(self, ip='192.168.1.238'):
        # super().__init__()
        # Robot initializations
        self.arm = XArmAPI(ip, is_radian=False)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)

        self.joint_positions = np.array(self.arm.get_servo_angle()[1])
        # print(f"servo angle: {self.arm.get_servo_angle()[1]}")
        w, x, y, z = euler_to_quaternion(roll=self.arm.position[3], pitch=self.arm.position[4], yaw=self.arm.position[5])
        self.end_effector_pose = np.array((self.arm.position[0], self.arm.position[2], self.arm.position[2],
                                           w, x, y, z))
        self.joint_velocities = np.array(self.arm.realtime_joint_speeds)
        # self.end_effector_velocities = np.array(self.arm.realtime_tcp_speed)
        self.joint_torques = np.array(self.arm.joints_torque)

        self._command = np.array([0, 0, 0, 0, 0, 0])
        self._position_command_scale = np.array([0.1, 0.1, 0.1])
        self._rotation_command_scale = np.array([0.1, 0.1, 0.1])

        # ik
        # compute method rel_pos to abs pos
    
    def compute_command(self, current_ee_pos:  np.ndarray, current_ee_rot:  np.ndarray):
        """
        we assume comand is pose_relative
        Returns:
         The target joint positions commands.
        """
        # scale command
        self._command[0:3] = self._command[0:3] @ self._position_command_scale
        self._command[3:6] = self._command[3:6] @ self._rotation_command_scale
        self.desired_eef_pose = apply_delta_pose(current_ee_pos, current_ee_rot, self._command)
        print(f"current_ee_pos: {current_ee_pos} current_ee_rot: {current_ee_rot}")
        print(f"desired_pose: {self.desired_eef_pose} self._command: {self._command}")

        return self.desired_eef_pose

    def joint_servo_j(self, joint_positions):
        # Dummy joint_servo_j function
        self.joint_positions = joint_positions
        # Perform any other dummy calculations here

    def get_joint_positions(self):
        self.joint_positions = np.array(self.arm.get_servo_angle()[1])
        return self.joint_positions
    
    def get_end_effector_pose(self):
        w, x, y, z = euler_to_quaternion(roll=self.arm.position[3], pitch=self.arm.position[4], yaw=self.arm.position[5])

        self.end_effector_pose = np.array((self.arm.position[0], self.arm.position[2], self.arm.position[2],
                                           w, x, y, z))
        return self.end_effector_pose

    def get_joint_velocities(self):
        self.joint_velocities = np.array(self.arm.realtime_joint_speeds)
        return self.joint_velocities

    """Returning a single value!!??"""
    # def get_end_effector_velocities(self):
    #     return self.end_effector_velocities

    def get_joint_torques(self):
        self.joint_torques = np.array(self.arm.joints_torque)
        return self.joint_torques