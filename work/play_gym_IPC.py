# sample_python aims to allow seamless integration with lua.
# see examples below
import os
import sys
import pdb # use pdb.set_trace() for debugging
sys.path.append(os.getcwd())
import libmainlib as m   
import luamodule as lua  # see luamodule.py
import numpy as np 

import gym
import gym_IPC
import os
import sys
sys.path.insert(0, os.path.join(os.getcwd(), "pytorchRL"))

# copied from pytorchRL/enjoy.py
import argparse
import os
# workaround to unpickle olf model files
import sys

import numpy as np
import torch

from a2c_ppo_acktr.envs import VecPyTorch, make_vec_envs
from a2c_ppo_acktr.utils import get_render_func, get_vec_normalize

# simply forward events to lua
def onCallback(mid, userdata):
    return

def onFrameChanged(currFrame):
    return

g_globals=None

def frameMove(fElapsedTime):

    global g_globals
    actor_critic, obs, recurrent_hidden_states, masks, args, env, render_func=g_globals
    with torch.no_grad():
        value, action, _, recurrent_hidden_states = actor_critic.act(
            obs, recurrent_hidden_states, masks, deterministic=args.det)

    # Obser reward and next obs
    obs, reward, done, _ = env.step(action)

    masks.fill_(0.0 if done else 1.0)

    if render_func is not None:
        render_func('human')
    return

def handleRendererEvent(ev, button, x,y):
    return 0


sys.path.append('a2c_ppo_acktr')

def main():
    global g_globals
    uiscale=1.5
    m.createMainWin(int((1024+180)*uiscale),int((600+100)*uiscale), int(1024*uiscale), int(600*uiscale),uiscale)
    m.showMainWin()
    m.getPythonWin().loadEmptyScript()


    parser = argparse.ArgumentParser(description='RL')
    parser.add_argument(
        '--seed', type=int, default=1, help='random seed (default: 1)')
    parser.add_argument(
        '--log-interval',
        type=int,
        default=10,
        help='log interval, one log per n updates (default: 10)')
    parser.add_argument(
        '--env-name',
        default='PongNoFrameskip-v4',
        help='environment to train on (default: PongNoFrameskip-v4)')
    parser.add_argument(
        '--load-dir',
        default='./trained_models/',
        help='directory to save agent logs (default: ./trained_models/)')
    parser.add_argument(
        '--non-det',
        action='store_true',
        default=False,
        help='whether to use a non-deterministic policy')
    args = parser.parse_args()

    args.det = not args.non_det

    env = make_vec_envs(
        args.env_name,
        args.seed + 1000,
        1,
        None,
        None,
        device='cpu',
        allow_early_resets=False)

    # Get a render function
    render_func = get_render_func(env)

    # We need to use the same statistics for normalization as used in training
    actor_critic, ob_rms = \
                torch.load(os.path.join(args.load_dir, args.env_name + ".pt"))

    vec_norm = get_vec_normalize(env)
    if vec_norm is not None:
        vec_norm.eval()
        vec_norm.ob_rms = ob_rms

    recurrent_hidden_states = torch.zeros(1,
                                          actor_critic.recurrent_hidden_state_size)
    masks = torch.zeros(1, 1)

    obs = env.reset()

    if render_func is not None:
        render_func('human')


    g_globals=(actor_critic, obs, recurrent_hidden_states, masks, args, env, render_func)

    m.startMainLoop() # this finishes when program finishes

if __name__=="__main__":
    main()
