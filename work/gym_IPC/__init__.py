from gym.envs.registration import register
from gym_IPC.cartpole import *

register(
    id='IPClocal-v0',
    entry_point='gym_IPC:CartPoleEnv',
    max_episode_steps=2000,
)
