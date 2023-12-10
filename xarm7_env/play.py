import rospy
import tempfile

import gymnasium as gym
import numpy as np
from stable_baselines3.common import vec_env

from xarm7_env import XArm7Env
from unity_robotics_demo_msgs.msg import PosRot
from sensor_msgs.msg import Joy

env_cfg  = XArm7Env()

action_type = "absolute"
stop = 0
is_update = False
done_trigger = 0
action = [0.0, 0.0, 0.0, 0.0]
previous_command = np.array([0.0, 0.0, 0.0])

def joy_cmd_clb(msg):
    global action, action
    """
    R_trigger: open/close gripper 
    R_B: Stop
    R_A: Done
    """
    action[3] = msg.axes[2]
    # self.done_trigger = 1 if msg.buttons[2] == 1 else 0
    if msg.buttons[3] == 1:
        stop = 1

def oculus_delta_pose_cb(msg):
    """Obtains expert action from oculus quest 2 interface."""
    global action_type, action, previous_command, is_update
    r_controller_pos = [msg.pos_z, -msg.pos_x, msg.pos_y]
    if action_type == "relative":
        """action = delta_pos between -1 and 1
        - calculate delta 
        - normalize between -1 and 1  to make it similar with agents action
        """
        if not is_update: # first command
            previous_command = np.array(r_controller_pos).copy()
        delta_command = np.array(r_controller_pos) - previous_command
        normalized_delta_command = np.clip(delta_command, -1.0, 1.0)
        previous_command = np.array(r_controller_pos).copy()
        action[0] = normalized_delta_command[0]
        action[1] = normalized_delta_command[1]
        action[2] = normalized_delta_command[2]
    else: 
        action[0] = msg.pos_z
        action[1] = -msg.pos_x
        action[2] = msg.pos_y
    is_update = True

if __name__ == "__main__":
    rospy.init_node('xarm7env_node', anonymous=True)
    rospy.Subscriber("/vr/right_controller_pose", PosRot, oculus_delta_pose_cb)
    rospy.Subscriber("/vr/joy_command", Joy, joy_cmd_clb)

    rate = rospy.Rate(50)   # 50Hz ctrl
    # register env
    gym.envs.register(
        id='XArm7Env-v1',
        entry_point='xarm7_env:XArm7Env',
    )

    rng = np.random.default_rng(0)
    env = gym.make('XArm7Env-v1')
    print (env.action_space)

    
    while True:
        obs, info = env.reset()
        if info['is_init']:
            break
        print("obs:", obs)
        print("info:", info)

    if info['is_init']:
        while not rospy.is_shutdown():
            observation, reward, terminated, truncated, info = env.step(action=action)
            print("obs: ", observation)
            print("action: ", action)
            print("info: ", info)

            rate.sleep()
            # print("action:", action)