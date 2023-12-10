import rospy
import numpy as np
import torch
import time
import math
from unity_robotics_demo_msgs.msg import PosRot
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from robot import XArm7Robot

# robot = XArm7Robot()
global pos, orn

# current_ee_pose = robot.get_end_effector_pose()
# pos = [current_ee_pose[0]/1000, current_ee_pose[1]/1000, current_ee_pose[2]/1000]
pos = [0.1, 0.0, 0.1]
orn = [1, 0, 0, 0]

global is_init, is_update
is_init, is_update = False, False

def oculus_delta_pose_cb(msg):
    global pos, orn, is_update
    new_range = [0.1, 0.1, 0.1]
    # Callback function to handle the received Pose-Rotation message
    # Replace 'msg' with the actual variable name you want to use for the received message

    # Extract relevant information from the message
    pos[0] = ((-msg.pos_z) + 1) * ((0.6 - 0.1) / 2) + 0.1
    pos[1] = ((-msg.pos_x) + 1) * ((0.6 - 0.1) / 2) + 0.1
    pos[2] = ((msg.pos_y) + 1) * ((0.6 - 0.1) / 2) + 0.1

    # normilize and scale orientation TODO
    # orn[0], orn[1], orn[2], orn[3] = msg.rot_w*0.1, -msg.rot_z*0.1, -msg.rot_x*0.1, msg.rot_y*0.1

    # Process the received data as needed
    # ...
    

    # rospy.loginfo("Received Pose-Rotation Message:\nPosition: %s\nRotation: %s", pos, orn)
    # rospy.loginfo("Received Pose-Rotation Message: % s", msg)
    is_update = True
def init():
    global is_init, pos, orn
    # use position controller to set pose to controller pose: mode 0
    rospy.loginfo("Dont't move a controller while the robot reset to controller Pose!")
    euler_ang = euler_from_quaternion([orn[0], orn[1], orn[2], orn[3]])
    # robot.arm.set_position(x=pos[0], y=pos[1], z=pos[2], roll=euler_ang[0], pitch=euler_ang[1], yaw=euler_ang[2], is_radian=False, wait=True)
    # robot.arm.set_position(*[pos[0]*1000, pos[1]*1000, pos[2]*1000, euler_ang[0], euler_ang[1], euler_ang[2]], wait=True)
    # print("pos: ", pos, "orn: ", euler_ang)
    #robot.arm.set_position(*[200, 0, 200, 180, 0, 0], wait=True)

    # change mode to mode 1
    # robot.arm.set_mode(1)
    # robot.arm.set_state(state=0)

    is_init = True
    rospy.loginfo("Ready to mimic controller motion!")

def main():
    global pos, orn, is_update, is_init
    rospy.init_node('robot_env_node')  # Replace 'your_ros_node' with your desired node name

    rospy.loginfo("Please put down the right controller!")
    # Set up a subscriber for the "/vr/right_controller_pose" topic
    rospy.Subscriber("/vr/right_controller_pose", PosRot, oculus_delta_pose_cb)

    # Instead of rospy.spin(), use a while loop
    rate = rospy.Rate(30)  # Set the desired loop rate (e.g., 10 Hz)

    while not rospy.is_shutdown():
        if is_update and not is_init:
            init()
        if is_init:
            # compute ik
            # joint_positions = robot.arm_ik.step(pos=pos, orn=orn)
            # set servo pos
            # ret = robot.arm.set_servo_angle_j(angles=joint_positions, speed=math.radians(200), is_radian=True)
            # step ik
            XArm7Robot.arm_ik.bullet_client.stepSimulation()
        if not is_update:
            rospy.loginfo("Check your controller pose is being published!")
        sim_only = False
        if sim_only:
            joint_positions = XArm7Robot.arm_ik.step(pos=pos, orn=orn)
            XArm7Robot.arm_ik.bullet_client.stepSimulation()
        rate.sleep()


if __name__ == '__main__':
    main()
    #XArm7Robot.arm.disconnect()