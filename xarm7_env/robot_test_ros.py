import rospy
import numpy as np
import torch
import time
import math
from unity_robotics_demo_msgs.msg import PosRot
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from robot import XArm7Robot

robot = XArm7Robot()
global pos, orn, gripper_action, rz


current_ee_pose = robot.get_end_effector_pose()
pos = [current_ee_pose[0]/1000, current_ee_pose[1]/1000, current_ee_pose[2]/1000]
orn = [1, 0, 0, 0]
gripper_action = 0  # close: 0, open: 1
stop = 0

robot.set_gripper_pos(action=0)

global is_init, is_update
is_init, is_update = False, False

def normalize_radians(radians):
    """
    Normalize radians from the range [-pi, pi] to [-pi/4, pi/4].

    Parameters:
    - radians (float): The input angle in radians.

    Returns:
    - float: The normalized angle in radians.
    """
    # Ensure the input is a NumPy array
    radians = np.array(radians)

    # Normalize the angle using NumPy operations
    normalized_radians = (radians + np.pi) % (2 * np.pi) - np.pi
    normalized_radians = np.clip(normalized_radians, np.pi / 4, 3 * np.pi / 4)

    return normalized_radians.item() if np.isscalar(radians) else normalized_radians.tolist()

def scale_position(old_range=[[-1, -1, -1], [1, 1, 1]], new_range=[[0.1, -0.6, 0.05], [0.6, 0.6, 0.6]]):
    global pos, orn
    pos[0] = (pos[0] + 1) * ((0.6 - 0.1) / 2) + 0.1
    # pos[1] = ((-msg.pos_x) + 1) * ((0.6 - 0.1) / 2) + 0.1
    pos[1] = ((pos[1] - (-1)) / (1 - (-1))) * (0.6 - (-0.6)) + (-0.6)
    pos[2] = (pos[2] + 1) * ((0.6 - 0.1) / 2) +  0.1

    # scale orn
    orn[0] = orn[0] * 1e-6
    orn[1] = orn[1] * 1e-6
    orn[2] = orn[2] * 1e-6
    orn[3] = orn[3] * 1e-6
    # normalize quat
    magnitude = math.sqrt(
        orn[1]**2 + orn[2]**2 + orn[3]**2 + orn[0]**2
    )
    orn[0] = orn[0] / magnitude
    orn[1] = orn[1] / magnitude
    orn[2] = orn[2] / magnitude
    orn[3] = orn[3] / magnitude


    """
    Scale a position from old range to new range.

    Parameters:
    - pos (list): The position to be scaled [x, y, z].
    - old_range (list): The old range [[min_x, min_y, min_z], [max_x, max_y, max_z]].
    - new_range (list): The new range [[min_x, min_y, min_z], [max_x, max_y, max_z]].

    Returns:
    - list: The scaled position in the new range.
    """
    """
    pos_arr = np.array(pos, float)
    old_range = np.array(old_range, float)
    new_range = np.array(new_range, float)

    # Scale the position for each dimension using NumPy multiplication
    scaled_pos = ((pos_arr - old_range[0]) / (old_range[1] - old_range[0])) * (new_range[1] - new_range[0]) + new_range[0]

    pos = scaled_pos.tolist()
    print("pos_scaled: ", type(pos)) 
    """

def joy_cmd_clb(msg):
    """
    R_trigger: open/close gripper 
    R_B: stop
    """
    global gripper_action, stop
    if msg.axes[2] > 0:
        gripper_action = 1
    else:
        gripper_action = 0
    if msg.buttons[3] == 1:
        stop = 1
    # print("Joy.msg: ", msg)

def oculus_delta_pose_cb(msg):
    global pos, orn, is_update

    pos[0] = (msg.pos_z + 1) * ((0.7 - 0.02) / 2) + 0.02
    # pos[1] = ((-msg.pos_x) + 1) * ((0.6 - 0.1) / 2) + 0.1
    pos[1] = ((-msg.pos_x - (-1)) / (1 - (-1))) * (0.6 - (-0.6)) + (-0.6)
    pos[2] = (msg.pos_y + 1) * ((0.6 - 0.02) / 2) +  0.02

    # scale orn
    # orn[0], orn[1], orn[2], orn[3] = msg.rot_w, -msg.rot_z, -msg.rot_x, msg.rot_y
    # orn[0] = -msg.rot_z * 1e-6
    # orn[1] = -msg.rot_x * 1e-6
    # orn[2] =  msg.rot_y * 1e-6
    # orn[3] =  msg.rot_w * 1e-6 
# 
    # # normilize in -pi/4 and pi/4 range
    # euler_ang = euler_from_quaternion([orn[0], orn[1], orn[2], orn[3]])
    # 
    # # approach 1: 
    # # euler_ang_norm  = normalize_radians(euler_ang)
    # # quat = quaternion_from_euler(euler_ang_norm[0], euler_ang_norm[1], euler_ang_norm[2])
    # # orn[0], orn[1], orn[2], orn[3] = quat[0], quat[1], quat[2], quat[3]
# 
    # # # normalize quat
    # magnitude = math.sqrt(
    #     orn[1]**2 + orn[2]**2 + orn[3]**2 + orn[0]**2
    # )
    # orn[0] = orn[0] / magnitude
    # orn[1] = orn[1] / magnitude
    # orn[2] = orn[2] / magnitude
    # orn[3] = orn[3] / magnitude
# 
    # # new_range = [0.1, 0.1, 0.1]
    # # Callback function to handle the received Pose-Rotation message
    # # Replace 'msg' with the actual variable name you want to use for the received message

    # Extract relevant information from the message
    # pos[0] = -msg.pos_z
    # pos[1] = -msg.pos_z
    # pos[2] = msg.pos_y
    # pos[0] = ((-msg.pos_z) + 1) * ((0.6 - 0.1) / 2) + 0.1
    # # pos[1] = ((-msg.pos_x) + 1) * ((0.6 - 0.1) / 2) + 0.1
    # pos[1] = ((-msg.pos_z - (-1)) / (1 - (-1))) * (0.6 - (-0.6)) + (-0.6)
    # pos[2] = ((msg.pos_y) + 1) * ((0.6 - 0.1) / 2) +  0.1

    # normilize and scale orientation TODO
    # orn[0], orn[1], orn[2], orn[3] = msg.rot_w, -msg.rot_z, -msg.rot_x, msg.rot_y

    # Process the received data as needed
    # ...
    

    # rospy.loginfo("Received Pose-Rotation Message:\nPosition: %s\nRotation: %s", pos, orn)
    # rospy.loginfo("Received Pose-Rotation Message: % s", msg)
    is_update = True

def init():
    global is_init, pos, orn, gripper_action
    # use position controller to set pose to controller pose: mode 0
    rospy.loginfo("Dont't move a controller while the robot reset to controller Pose!")
    # close gripper
    robot.set_gripper_pos(action=0)
    euler_ang = euler_from_quaternion([orn[0], orn[1], orn[2], orn[3]])
    # scale pos
    # scale_position()
    robot.arm.set_position(*[pos[0]*1000, pos[1]*1000, pos[2]*1000, euler_ang[0], euler_ang[1], euler_ang[2]], is_radian=True, wait=True)
    # robot.arm.set_position(*[pos[0]*1000, pos[1]*1000, pos[2]*1000, 3.1415927, 0, 0], is_radian=True, wait=True)
    print("pos: ", pos, "orn: ", euler_ang)
    #robot.arm.set_position(*[200, 0, 200, 180, 0, 0], wait=True)

    # change mode to mode 1
    robot.arm.set_mode(1)
    robot.arm.set_state(state=0)

    is_init = True
    rospy.loginfo("Ready to mimic controller motion!")

def on_shutdown():
    """reset or"""
    rospy.logdebug_once("shutting down!")

def main():
    global pos, orn, is_update, is_init, gripper_action, stop
    rospy.init_node('robot_env_node')  # Replace 'your_ros_node' with your desired node name

    rospy.loginfo("Please put down the right controller!")
    # Set up a subscriber for the "/vr/right_controller_pose" topic
    rospy.Subscriber("/vr/right_controller_pose", PosRot, oculus_delta_pose_cb)
    rospy.Subscriber("/vr/joy_command", Joy, joy_cmd_clb)

    # Instead of rospy.spin(), use a while loop
    rate = rospy.Rate(60)  # Set the desired loop rate (e.g., 10 Hz)

    while not rospy.is_shutdown():
        if is_update and not is_init:
            init()
        if is_init:
            # scale pos
            # scale_position()
            # compute ik
            joint_positions = robot.arm_ik.step(pos=pos, orn=orn)
            # set servo pos
            ret = robot.arm.set_servo_angle_j(angles=joint_positions, speed=math.radians(150), is_radian=True)
            robot.set_gripper_pos(action=gripper_action)
            print(f"gripper_action: {gripper_action}")
            # step ik
            robot.arm_ik.bullet_client.stepSimulation()
        if not is_update:
            rospy.loginfo("Check your controller pose is being published!")
        sim_only = False
        if sim_only:
            # scale_position()
            joint_positions = robot.arm_ik.step(pos=pos, orn=orn)
            robot.arm_ik.bullet_client.stepSimulation()
        
        if stop:
            """shutdown the node"""
            rospy.signal_shutdown("shutdown evoked by key B")
        rate.sleep()
    
    rospy.on_shutdown(on_shutdown)


if __name__ == '__main__':
    main()
    robot.arm.disconnect()