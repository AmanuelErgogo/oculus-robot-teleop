import tempfile
import gym
import numpy as np
# import gymnasium as gym
from stable_baselines3.common.evaluation import evaluate_policy

from imitation.algorithms import bc
from imitation.algorithms.dagger import SimpleDAggerTrainer
from imitation.policies.serialize import load_policy
from imitation.util.util import make_vec_env

from xarm7_robot_env import XArm7Env

# register env
gym.envs.register(
    id='XArm7Env-v0',
    entry_point='xarm7_robot_env:XArm7Env',
)

rng = np.random.default_rng(0)
env = gym.make('XArm7Env-v0')
print (env.action_space)

bc_trainer = bc.BC(
    observation_space=env.observation_space,
    action_space=env.action_space,
    rng=rng,
)