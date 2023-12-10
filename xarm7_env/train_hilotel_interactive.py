import rospy
import tempfile

import gymnasium as gym
import numpy as np
from stable_baselines3.common import vec_env

from imitation.algorithms import bc, dagger
from imitation.policies import interactive

from xarm7_env import XArm7Env

# env = XArm7Env()

if __name__ == "__main__":
    rospy.init_node('xarm7env_node', anonymous=True)
    # register env
    gym.envs.register(
        id='XArm7Env-v1',
        entry_point='xarm7_env:XArm7Env',
    )

    rng = np.random.default_rng(0)
    env = vec_env.DummyVecEnv([lambda: gym.wrappers.TimeLimit(gym.make('XArm7Env-v1'), 10)])
    env.seed(0)

    # _, _, _, info = env.reset() 
    # while(not info["is_init"]):
    #     _, _, _, info = env.reset()
    #     print("not initialized")
    # print("Initialized!!")

    #env.seed(0)
    expert = interactive.Xarm7InteractivePolicy(env)
 
    bc_trainer = bc.BC(
        observation_space=env.observation_space,
        action_space=env.action_space,
        rng=rng,
    )

    with tempfile.TemporaryDirectory(prefix="dagger_hilotel_v0_") as tmpdir:
        dagger_trainer = dagger.SimpleDAggerTrainer(
            venv=env,
            scratch_dir=tmpdir,
            expert_policy=expert,
            bc_trainer=bc_trainer,
            rng=rng,
        )
        dagger_trainer.train(
            total_timesteps=20,
            rollout_round_min_episodes=1,
            rollout_round_min_timesteps=10,
        )
