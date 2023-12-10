import rospy
import gym
import math
import numpy as np

from gymnasium.spaces import Box
from gymnasium import spaces
import numpy as np

from unity_robotics_demo_msgs.msg import PosRot
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from robot import XArm7Robot


class XArm7Env(gym.Env):
    def __init__(self):
        super(XArm7Env, self).__init__()

        # Define action and observation spaces
        self.action_space = Box(low=np.array([-1, -1, -1, 0]),
                                       high=np.array([1, 1, 1, 1]),
                                       dtype=np.float32)
        self._observation_space = spaces.Dict({
             'end_effector_pos': Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
             'gripper_status': Box(low=0, high=1, shape=(1,), dtype=np.float32),
             'pick_pos': Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
             'place_pos': Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
         })
        self.observation_space = Box(low=np.array([-np.inf, -np.inf, -np.inf, 0, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf]),
                                     high=np.array([np.inf, np.inf, np.inf, 1, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]),
                                       dtype=np.float32)

        self.pick_position = [0.3804, -0.330, 0.0384]
        self.place_position = [0.4307, 0.332, 0.0374]

        # Initialize xArm7 robot
        self.robot = XArm7Robot()

        current_ee_pose = self.robot.get_end_effector_pose()
        self.pos = [current_ee_pose[0]/1000, current_ee_pose[1]/1000, current_ee_pose[2]/1000]
        self.orn = [1, 0, 0, 0]

        self.reset_pos = [current_ee_pose[0]/1000, current_ee_pose[1]/1000, current_ee_pose[2]/1000]
        self.reset_orn = [1, 0, 0, 0]

        #updated by right_controller_clb
        # self.reset_pos = [current_ee_pose[0]/1000, current_ee_pose[1]/1000, current_ee_pose[2]/1000]
        self.reset_orn = [1, 0, 0, 0]
        self.reset_grip = 0

        # ros
        #rospy.init_node('xarm7env_node', anonymous=True)
        self.joy_sub = rospy.Subscriber("/vr/right_controller_pose", PosRot, self.oculus_delta_pose_cb)
        self.r_control_sub = rospy.Subscriber("/vr/joy_command", Joy, self.joy_cmd_clb)

        self.action_type = "absolute"   # {"relative", "absolute"}
        self.previous_command = np.array([0.0, 0.0, 0.0])
        self.delta_cmd = np.array([0.0, 0.0, 0.0])
        self.pos_scale = np.array([0.1, 0.1, 0.1])

        self.is_init = False
        self.is_update = False
        self.gripper_action = 0
        self.done_trigger = 0
        self.rec_mode = 0

    def reset(self, seed=None):
        # Reset the environment and return initial observation
        # Implement this method based on your specific requirements
        # use position controller to set pose to controller pose: mode 0
        if self.action_type == "absolute":
            if self.is_update:
                rospy.loginfo("Dont't move a controller while the robot reset to controller Pose!")
                self.reset_pos[0] = (self.r_controller_pos[0] + 1) * ((0.7 - 0.02) / 2) + 0.02
                self.reset_pos[1] = ((self.r_controller_pos[1] - (-1)) / (1 - (-1))) * (0.6 - (-0.6)) + (-0.6)
                self.reset_pos[2] = (self.r_controller_pos[2] + 1) * ((0.6 - 0.02) / 2) +  0.02
                euler_ang = euler_from_quaternion([self.reset_orn[0], self.reset_orn[1], self.reset_orn[2], self.reset_orn[3]])
                self.robot.arm.set_position(*[self.reset_pos[0]*1000, self.reset_pos[1]*1000, self.reset_pos[2]*1000, euler_ang[0], euler_ang[1], euler_ang[2]], is_radian=True, wait=True)
                # change mode to mode 1
                self.robot.arm.set_mode(1)   # joint control mode
                self.robot.arm.set_state(state=0)  # moving state
                self.robot.set_gripper_pos(action=0)  # close gripper
                self.is_init = True
                self.is_update = False
                rospy.loginfo("Ready to mimic controller motion!")
            if not self.is_update:
                rospy.loginfo("make sure controller pose is being published!")
        else:
            euler_ang = euler_from_quaternion([self.reset_orn[0], self.reset_orn[1], self.reset_orn[2], self.reset_orn[3]])
            self.robot.arm.set_position(*[self.reset_pos[0]*1000, self.reset_pos[1]*1000, self.reset_pos[2]*1000, euler_ang[0], euler_ang[1], euler_ang[2]], is_radian=True, wait=True)
            # change mode to mode 1
            self.robot.arm.set_mode(1)   # joint control mode
            self.robot.arm.set_state(state=0)  # moving state
            self.robot.set_gripper_pos(action=0)  # close gripper
            self.is_init = True
            rospy.loginfo("Ready to mimic controller motion!")

        observation_dict = {
            'end_effector_pos': self.robot.get_end_effector_pose(is_mm=False)[0:3],
            'gripper_status': np.array([self.robot.get_gripper_status()], dtype=np.float32),
            'pick_pos': np.array(self.pick_position, dtype=np.float32),
            'place_pos': np.array(self.place_position, dtype=np.float32),
        }
        # Convert dictionary values to a NumPy array
        default_gripper_status = np.array([0], dtype=np.float32)
        if np.isnan(observation_dict['gripper_status'])[0]:
            observation_dict['gripper_status'] = default_gripper_status
        # print(f"obs_dict: {observation_dict}")
        observation = np.concatenate([observation_dict[key] for key in self._observation_space.spaces.keys()])
        # observation = np.concatenate([
        #     observation_dict[key].flatten() if isinstance(observation_dict[key], np.ndarray) else np.array([observation_dict[key] if not np.isnan(observation_dict[key]) else default_gripper_status])
        #     for key in self._observation_space.spaces.keys()
        # ])

        reward = 0.0  # You need to define your reward function

        done = False  # You need to define your termination condition

        info = {"is_init": self.is_init, "is_success": False, "is_update": self.is_update}  # Additional information, if needed
        return observation, info

    def step(self, action):
        """
        pos = [x, y, z, gripper_pos, rz]
        rz not considered for now
        """
        if self.action_type == "absolute":
            # process action to pos, rotation z and gripper pos
            pos = action[0:3]
            pos[0] = (action[0] + 1) * ((0.7 - 0.02) / 2) + 0.02
            pos[1] = ((action[1] - (-1)) / (1 - (-1))) * (0.6 - (-0.6)) + (-0.6)
            pos[2] = (action[2] + 1) * ((0.6 - 0.02) / 2) +  0.02
        else: # relative
            # pos = self.robot.get_end_effector_pose(is_mm=False)[0:3] + self.pos_scale * action
            """
            1. scale down
            2. compute desired absolute pos
            """
            pos = (self.robot.get_end_effector_pose(is_mm=False)[0:3] + (self.pos_scale * np.array(action[0:3]))).tolist()
            # pos = self.robot.get_end_effector_pose(is_mm=False)[0:3].tolist()
            print("pos", pos)

        # normalize gripper pos from 0 to 1 to open(800) to close(20)
        if action[3] > 0:
            self.gripper_action = 1
        else:
            self. gripper_action = 0

        # normalize rotation from -1 to 1 to -pi/2 to pi/2
        orn = self.orn

        # compute ik
        joint_positions = self.robot.arm_ik.step(pos=pos, orn=orn)
        # set servo pos
        self.robot.arm.set_servo_angle_j(angles=joint_positions, speed=math.radians(200), is_radian=True)
        self.robot.set_gripper_pos(action=self.gripper_action)
        # step ik
        self.robot.arm_ik.bullet_client.stepSimulation()

        observation_dict = {
            'end_effector_pos': self.robot.get_end_effector_pose(is_mm=False)[0:3],
            'gripper_status': np.array([self.robot.get_gripper_status()], dtype=np.float32),
            'pick_pos': np.array(self.pick_position, dtype=np.float32),
            'place_pos': np.array(self.place_position, dtype=np.float32),
        }
        # Convert dictionary values to a NumPy array
        default_gripper_status = np.array([0], dtype=np.float32)

        if np.isnan(observation_dict['gripper_status'])[0]:
            observation_dict['gripper_status'] = default_gripper_status

        #print(f"obs_dict: {observation_dict}")

        observation = np.concatenate([observation_dict[key] for key in self._observation_space.spaces.keys()])

        done = self.done_trigger # You need to define your termination condition
        reward = 0.0  # You need to define your reward function
        info = {"is_init": self.is_init, "is_success": done, "is_update": self.is_update, "rec_mode": self.rec_mode} 
        terminated = {}
        truncated = {}

        return observation, reward, terminated, truncated, info

    def render(self, mode='human'):
        # Render the environment, if applicable
        pass

    def close(self):
        # Clean up resources, if neededach RL lib
        pass

    def joy_cmd_clb(self, msg):
        """
        R_trigger: open/close gripper 
        R_B: Stop
        R_A: Done
        """
        self.done_trigger = 1 if msg.buttons[2] == 1 else 0
        self.rec_mode = 1 if msg.axes[0] > 0.5 else 0
        # print("Joy clb!")

    def oculus_delta_pose_cb(self, msg):
        self.r_controller_pos = [msg.pos_z, -msg.pos_x, msg.pos_y]

        #if self.action_type == "relative":
        #    """action = delta_pos between -1 and 1
        #    - calculate delta 
        #    - normalize between -1 and 1  to make it similar with agents action
        #    """
        #    if not self.is_update: # first command
        #        self.previous_command = np.array(self.r_controller_pos).copy()
#
        #    delta_command = np.array(self.r_controller_pos) - previous_command
        #    normalized_delta_command = np.clip(delta_command, -1.0, 1.0)
        #    previous_command = np.array(self.r_controller_pos).copy()
#
        #    self.delta_cmd = normalized_delta_command
        
        self.is_update = True
        # print("controller_cb!")