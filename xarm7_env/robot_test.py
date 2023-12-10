import rospy
import numpy as np
import torch
import time
import math

from robot import XArm7Robot

robot = XArm7Robot()
# robot.arm.set_tcp_offset([0, 0, 0, 0, 0, 0])
joint_pos = robot.get_joint_positions()
eef_pose = robot.get_end_effector_pose()

# robot.arm.set_mode(1)
# robot.arm.set_state(state=0)

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

num_points = 500
end = (0.5, 0.3, 0.5)
start = (eef_pose[0]/1000, eef_pose[1]/1000, eef_pose[2]/1000+0.2)
# print("start pose: ", start)

line = generate_line_path(num_points=num_points, start_point=start, end_point=end)
# print(line)
while(t < num_points):
    t += 1

    # pos = [0.407+0.2 * math.sin(1.5 * t), 0.0+0.2 * math.cos(1.5 * t), 0.12]
    # # orn = self.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])
    # orn = [1,0,0,0]

    # get current pose
    current_ee_pose = robot.get_end_effector_pose()
    # print("current pose before pass: ", current_ee_pose)

    # update command
    if current_ee_pose[2]/1000 < 0.6:
        robot._command[2] = 200
        # print("command updated!")


    # compute desired pose
    t_start = time.time()
    desired_eef_pose = robot.compute_command(current_ee_pos=current_ee_pose[0:3], current_ee_rot=current_ee_pose[3:7])
    # print("cmd_time: ", t_start - time.time())
    # print("pos", pos)
    pos = desired_eef_pose[0] / 1000
    # orn = desired_eef_pose[1]
    # print(f"desired_pos:{pos}")

    # pos, orn = generate_circle_path(t)
    # pos = [line[t,0], line[t, 1], line[t, 2]]
    orn = [1, 0, 0, 0]
    # print(f"pos: {pos}")
    # compute ik
    joint_positions = robot.arm_ik.step(pos=pos, orn=orn)
    # print("joint_positions: ", joint_positions)

    # print("JointPoses", robot.arm_ik.jointPoses)
    # robot

    # set servo pos
    ret = robot.arm.set_servo_angle_j(angles=joint_positions, speed=math.radians(200), is_radian=True)
    # print(f"ret: {ret}")

    # step ik
    robot.arm_ik.bullet_client.stepSimulation()

    # delay for 60 Hz
    time.sleep(1/600)

    # update comand
    # robot._command[1] = robot._command[1] - 100

    # print(f"t: {t},  command: {robot._command},  desired_pose: {desired_eef_pose}")
    # print("tcp_offset: ", robot.arm.tcp_offset)
# robot.arm.reset(wait=True)
robot.arm.disconnect()