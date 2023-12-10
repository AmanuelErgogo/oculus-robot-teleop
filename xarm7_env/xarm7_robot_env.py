import rospy
import gym
import math
import numpy as np

from gym.spaces import Box
from gym.spaces import space
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
        # self.observation_space = space.Dict({
        #     'end_effector_pos': Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
        #     'gripper_status': Box(low=0, high=1, shape=(1,), dtype=np.float32),
        #     'pick_pos': Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
        #     'place_pos': Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),

        # })
        self.observation_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(10,))

        self.pick_position = [0.3804, -0.330, 0.0384]
        self.place_position = [0.4307, 0.332, 0.0374]

        # Initialize xArm7 robot
        self.robot = XArm7Robot()

        current_ee_pose = self.robot.get_end_effector_pose()
        self.pos = [current_ee_pose[0]/1000, current_ee_pose[1]/1000, current_ee_pose[2]/1000]
        self.orn = [1, 0, 0, 0]

        #updated by right_controller_clb
        self.reset_pos = [current_ee_pose[0]/1000, current_ee_pose[1]/1000, current_ee_pose[2]/1000]
        self.reset_orn = [1, 0, 0, 0]

        self.is_init = False
        self.is_update = False
        self.gripper_action = 0
        self.done_trigger = 0

    def reset(self, seed=None):
        # Reset the environment and return initial observation
        # Implement this method based on your specific requirements
        # use position controller to set pose to controller pose: mode 0
        rospy.loginfo("Dont't move a controller while the robot reset to controller Pose!")
        euler_ang = euler_from_quaternion([self.reset_orn[0], self.reset_orn[1], self.reset_orn[2], self.reset_orn[3]])
        # reset eef to controler pos by position controller
        self.robot.arm.set_position(*[self.reset_pos[0]*1000, self.reset_pos[1]*1000, self.reset_pos[2]*1000, euler_ang[0], euler_ang[1], euler_ang[2]], is_radian=True, wait=True)
        # robot.arm.set_position(*[pos[0]*1000, pos[1]*1000, pos[2]*1000, 3.1415927, 0, 0], is_radian=True, wait=True)
        print("reset_pos: ", self.reset_pos, "reset_orn: ", euler_ang)
        #robot.arm.set_position(*[200, 0, 200, 180, 0, 0], wait=True)

        # change mode to mode 1
        self.robot.arm.set_mode(1)   # joint control mode
        self.robot.arm.set_state(state=0)  # moving state
        self.robot.set_gripper_pos(action=0)  # close gripper

        self.is_init = True

        observation = {
            'end_effector_pos': np.array(self.robot.get_end_effector_pose(is_mm=False)[0:3], dtype=np.float32),
            'gripper_status': np.array(self.robot.get_gripper_status(), dtype=np.float32),
            'pick_pos': np.array(self.pick_position, dtype=np.float32),
            'place_pos': np.array(self.place_position, dtype=np.float32),
        }

        reward = 0.0  # You need to define your reward function

        done = False  # You need to define your termination condition

        info = {}  # Additional information, if needed

        rospy.loginfo("Ready to mimic controller motion!")
        return observation, info

    def step(self, action):
        """
        pos = [x, y, z, gripper_pos, rz]
        rz not considered for now
        """
        # process action to pos, rotation z and gripper pos
        pos = action[0:3]
        pos[0] = (action[0] + 1) * ((0.7 - 0.02) / 2) + 0.02
        pos[1] = ((action[1] - (-1)) / (1 - (-1))) * (0.6 - (-0.6)) + (-0.6)
        pos[2] = (action[2] + 1) * ((0.6 - 0.02) / 2) +  0.02

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

        observation = {
            'end_effector_pose': self.robot.get_end_effector_pose(is_mm=False),
            'gripper_status': self.robot.get_gripper_status(),
            'pick_pos': self.pick_position,
            'place_pos': self.place_position,
        }

        reward = 0.0  # You need to define your reward function

        done = self.done_trigger # You need to define your termination condition

        info = {}  # Additional information, if needed

        return observation, reward, done, info

    def render(self, mode='human'):
        # Render the environment, if applicable
        pass

    def close(self):
        # Clean up resources, if neededach RL lib
        pass
