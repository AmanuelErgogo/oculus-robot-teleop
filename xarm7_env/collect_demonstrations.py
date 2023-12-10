# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to collect demonstrations with Isaac Orbit environments."""

"""Launch Isaac Sim Simulator first."""


import argparse

# add argparse arguments
parser = argparse.ArgumentParser("Welcome to Orbit: Omniverse Robotics Environments!")
parser.add_argument("--headless", action="store_true", default=False, help="Force display off at all times.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--device", type=str, default="keyboard", help="Device for interacting with environment")
parser.add_argument("--num_demos", type=int, default=1, help="Number of episodes to store in the dataset.")
parser.add_argument("--filename", type=str, default="hdf_dataset", help="Basename of output file.")
args_cli = parser.parse_args()

"""Rest everything follows."""

import rospy
import contextlib
import os
import time
import json

import xarm7_env
from robomimic_data_collector import RobomimicDataCollector

import gymnasium as gym
import numpy as np

from xarm7_env import XArm7Env
from unity_robotics_demo_msgs.msg import PosRot
from sensor_msgs.msg import Joy

action_type = "absolute"
stop = 0
rec_mode = 1
is_update = False
done_trigger = 0
action = [0.0, 0.0, 0.0, 0.0]
previous_command = np.array([0.0, 0.0, 0.0])

execution_time = {}

def joy_cmd_clb(msg):
    global action, stop, rec_mode
    """
    R_trigger: open/close gripper 
    R_B: Stop
    R_A: Done
    """
    action[3] = msg.axes[2]
    # self.done_trigger = 1 if msg.buttons[2] == 1 else 0
    if msg.buttons[3] == 1:
        stop = 1
    if msg.buttons[0] == 1:
        rec_mode = 1  

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



def main():
    global action, stop, rec_mode
    """Collect demonstrations from the environment using teleop interfaces."""

    rospy.init_node('xarm7env_node', anonymous=True)
    rospy.Subscriber("/vr/right_controller_pose", PosRot, oculus_delta_pose_cb)
    rospy.Subscriber("/vr/joy_command", Joy, joy_cmd_clb)

    rate = rospy.Rate(60)   # 50Hz ctrl

    # register env
    gym.envs.register(
        id='XArm7Env-v1',
        entry_point='xarm7_env:XArm7Env',
    )

    rng = np.random.default_rng(0)
    env = gym.make('XArm7Env-v1')

    # create controller
    

    # specify directory for logging experiments
    log_dir = os.path.join("./logs/robomimic", args_cli.task)

    # create data-collector
    collector_interface = RobomimicDataCollector(
        env_name=args_cli.task,
        directory_path=log_dir,
        filename=args_cli.filename,
        num_demos=args_cli.num_demos,
        flush_freq=1,
    )

    # reset environment
    while True:
        obs, info = env.reset()
        if info['is_init']:
            break
    # obs_dict = env.reset()
    # robomimic only cares about policy observations
    # obs = obs_dict["policy"]
    # reset interfaces
    # teleop_interface.reset()
    collector_interface.reset()

    # simulate environment
    is_new_demo = True
    demo_count = 0
    with contextlib.suppress(KeyboardInterrupt):
        while not collector_interface.is_stopped():

            # get keyboard command
            if is_new_demo:
                user_input = input("Enter to continue demo: Any key")
                is_new_demo = False
                demo_count = demo_count + 1
                t_start = time.monotonic()

            actions = action.copy()  # action updated by callback

            # TODO: Deal with the case when reset is triggered by teleoperation device.
            #   The observations need to be recollected.
            # store signals before stepping
            # -- obs
            # for key, value in obs.items():
            #     collector_interface.add(f"obs/{key}", value)
            # -- actions
            collector_interface.add(f"obs", obs)
            collector_interface.add("actions", actions)
            # perform action on environment
            obs, reward, terminated, truncated, info = env.step(actions)
            # check that simulation is stopped or not
            # if env.unwrapped.sim.is_stopped():
            #     break
            if stop:
                break

            # robomimic only cares about policy observations
            # obs = obs_dict["policy"]
            # store signals from the environment
            # -- next_obs
            # for key, value in obs.items():
            #     collector_interface.add(f"next_obs/{key}", value.cpu().numpy())
            collector_interface.add(f"next_obs", obs)
            # -- rewards
            # collector_interface.add("rewards", rewards)
            # -- dones
            dones = np.array([info["is_success"]])
            # dones = np.array([False])
            collector_interface.add("dones", dones)
            # -- is-success label
            try:
                collector_interface.add("success", np.array([info["is_success"]]))
                # collector_interface.add("success", np.array([False]))
            except KeyError:
                raise RuntimeError(
                    f"Only goal-conditioned environment supported. No attribute named 'is_success' found in {list(info.keys())}."
                )
            # flush data from collector for successful environments
            reset_env_ids = np.nonzero(dones)[0]
            collector_interface.flush(reset_env_ids)

            if dones[0] == True:
                execution_time[f"demo_{demo_count}_min"] = (time.monotonic() - t_start) / 60
                is_new_demo = True
                rec_mode = 0

                # if demo_count == args_cli.num_demos:
                log_dir_exec_time = os.path.join("./logs/robomimic", 'execution_time.json')
                with open(log_dir_exec_time, 'w') as json_file:
                    json.dump(execution_time, json_file)

                # # reset env to start second demo
                # Go back to init pose and press Y key
                max_executions = 300
                counter = 0
                while counter < max_executions:
                    print ("resseting>>>")
                    env.step(action=action)
                    counter += 1
                    if counter >= max_executions:
                        info["is_success"] = False
                        break
                    
                # while True:
                #     print(">> go to reset pos and press x key to continue demoing")
                #     _, _, _, _, info =env.step(action=action)
                #     if info['rec_mode']:
                #         info["is_success"] = False
                #         print(">>>going to next demo")
                #         break
                #     print(info['rec_mode'])
                #     rate.sleep()

            rate.sleep()
                # print(">>Please, put right controller to the strart position and look at the controller.")
                # # rospy.sleep(10)
                # print(">> robot going to reset position!")
# 
                # while True:
                #     if info['is_update']:
                #         obs, info = env.reset()
                #     if info['is_init']:
                #         print(">> continue demoing...")
                #         break
                #     rospy.logdebug("make sure controller pose is being published!")
                


    # close the simulator
    collector_interface.close()
    env.close()
    # simulation_app.close()


if __name__ == "__main__":
    main()