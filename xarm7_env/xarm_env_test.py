import rospy
import gym
import numpy as np
import math
from unity_robotics_demo_msgs.msg import PosRot
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Import your XArm7Env and XArm7Robot classes (assuming they are defined in a separate file)
from xarm7_robot_env import XArm7Env
global is_update, is_init, action, stop
is_update, is_init = False, False
stop = 0

def joy_cmd_clb(msg):
    global stop
    """
    R_trigger: open/close gripper 
    R_B: Stop
    R_A: Done
    """
    action[3] = msg.axes[2]
    env.done_trigger = 1 if msg.buttons[2] == 1 else 0
    if msg.buttons[3] == 1:
        stop = 1

def oculus_delta_pose_cb(msg):
    global is_update, previous_command
    action[0] = msg.pos_z
    action[1] = -msg.pos_x
    action[2] = msg.pos_y

    r_controller_pos = [action[0], action[1], action[2]]

    if not is_update: # first command
        previous_command = np.array(r_controller_pos).copy()
    delta_command_raw = np.array(r_controller_pos) - previous_command
    normalized_delta_cmd = ((delta_command_raw - (-0.1)) / (0.1 - (-0.1))) * (1 - (-1)) + (-1)
    delta_command = np.clip(normalized_delta_cmd, -1.0, 1.0)
    previous_command = np.array(r_controller_pos).copy()
    print ("delta_command: ", delta_command)

    is_update = True


def init():
    global is_init
    env.reset_pos[0] = (action[0] + 1) * ((0.7 - 0.02) / 2) + 0.02
    env.reset_pos[1] = ((action[1] - (-1)) / (1 - (-1))) * (0.6 - (-0.6)) + (-0.6)
    env.reset_pos[2] = (action[2] + 1) * ((0.6 - 0.02) / 2) +  0.02

    env.reset()
    is_init = True

def test_xarm_env():
    global is_init, is_update, stop
    # Create the XArm7Env environment

    rospy.init_node('robot_env_node', anonymous=True) 
    rospy.Subscriber("/vr/right_controller_pose", PosRot, oculus_delta_pose_cb)
    rospy.Subscriber("/vr/joy_command", Joy, joy_cmd_clb)

    rate = rospy.Rate(60)  # Set the desired loop rate (e.g., 10 Hz)

    # Define the number of episodes and steps per episode for testing
    num_episodes = 1
    steps_per_episode = 1

    while not rospy.is_shutdown():

        if is_update and not is_init:
            init()
        if not is_update:
            rospy.loginfo("Check your controller pose is being published!")
        if is_init:
            for episode in range(num_episodes):
                 for step in range(steps_per_episode):
                     # Sample a random action from the action space
                     # Take a step in the environment
                     observation, reward, done, _ = env.step(action)

                     # Print some information (you can modify this based on your needs)
                     # print(f"Step: {step + 1}")
                     # print("Observation:", observation)
                     # print("Reward:", reward)
                     # print("Done:", done)

                     if done:
                         break
        if stop:
            """shutdown the node"""
            rospy.signal_shutdown("shutdown evoked by key B")
        rate.sleep()
    

    # Close the environment
    env.close()

if __name__ == "__main__":
    env = XArm7Env()
    action = [env.reset_pos[0], env.reset_pos[1], env.reset_pos[2], 0]
    test_xarm_env()
