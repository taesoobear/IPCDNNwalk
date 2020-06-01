from gym.envs.registration import register
from gym_cartpole.cartpole import *

register(
    id='cartpolelocal-v0',
    entry_point='gym_cartpole:CartPoleEnv',
    max_episode_steps=2000,
)
