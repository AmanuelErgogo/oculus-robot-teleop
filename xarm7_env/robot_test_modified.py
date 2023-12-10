import numpy as np
import torch
import time
import math
import pybullet as p
import pybullet_data as pd

from robot_modified import XArm7Robot
from xarm_sim_modified import XArm7Sim

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
timeStep=1./30.
p.setTimeStep(timeStep)
p.setGravity(0, 0, -9.8)

robot = XArm7Robot()
ik = XArm7Sim(p,[0,0,0])

joint_pos = robot.get_joint_positions()
# print(f"Joint positions: {joint_pos} \njoint_velocities: {robot.get_joint_velocities()} \njoint_torque: {robot.get_joint_torques()}")
# print(f"eef_pose: {robot.get_end_effector_pose()}")

# set command
# robot._command = np.zeros(6)  
# robot._command[1] = -100

# get current pose
# current_ee_pose = robot.get_end_effector_pose()
# print("current_ee_pose", current_ee_pose)

# compute desired pose
# compute joint pose
# step the action
# desired_eef_pose = robot.compute_command(current_ee_pos=current_ee_pose[0:3], current_ee_rot=current_ee_pose[3:7])
# print(f"desired_pose:{desired_eef_pose}")
#print("current_ee_pose", current_ee_pose)

#current_ee_pos = current_ee_pose
#robot.compute_command(current_ee_pos, current_ee_rot)
"""
Joint positions: [-8.43307931  0.67442862 12.0876906  -0.22832368  5.10677283 -0.12427455
 -2.18497455]
"""

def generate_circle_path(t):
    pos = [0.407+0.2 * math.sin(1.5 * t), 0.2 * math.cos(1.5 * t), 0.12]
    orn = [1,0,0,0]
    return pos, orn

def generate_line_path(num_points, start_point, end_point):
    # Ensure the number of points is at least 2
    num_points = max(num_points, 2)
    
    # Generate linearly spaced values between start and end points in the XY plane
    x_values = np.linspace(start_point[0], end_point[0], num_points)
    y_values = np.linspace(start_point[1], end_point[1], num_points)

    # Create an array with constant z values (assuming the line is in the XY plane)
    z_values = np.full_like(x_values, start_point[2])

    # Combine x, y, and z values to create the 3D path
    path = np.column_stack((x_values, y_values, z_values))

    return path

t = 0

num_points = 50
start = (0.606, 0.05, 0.15)
end = (0.1, 0.5, 0.15)

line = generate_line_path(num_points=num_points, start_point=start, end_point=end)
# print(line)
while(1):
    t += 1.0/60.

    # pos = [0.407+0.2 * math.sin(1.5 * t), 0.0+0.2 * math.cos(1.5 * t), 0.12]
    # # orn = self.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])
    # orn = [1,0,0,0]

    # get current pose
    # current_ee_pose = robot.get_end_effector_pose()

    # compute desired pose
    # desired_eef_pose = robot.compute_command(current_ee_pos=current_ee_pose[0:3], current_ee_rot=current_ee_pose[3:7])

    # print("pos", pos)
    # pos = desired_eef_pose[0] / 1000
    # orn = desired_eef_pose[1]
    # print(f"desired_pos:{pos}")

    pos, orn = generate_circle_path(t)
    # print(f"pos: {pos}")
    # compute ik
    ik.step(pos=pos, orn=orn)
    print("joint_positions: ", ik.jointPoses)

    # print("JointPoses", robot.arm_ik.jointPoses)
    # robot

    # set servo pos
    ret = robot.arm.set_servo_angle_j(angles=ik.jointPoses, speed=math.radians(50), is_radian=True)
    print(f"ret: {ret}")

    # delay for 60 Hz
    time.sleep(1/60)

    # update comand
    # robot._command[1] = robot._command[1] - 100
    print("mode: ", robot.arm.mode)

    print("t: ", t)
# robot.arm.reset(wait=True)
robot.arm.disconnect()