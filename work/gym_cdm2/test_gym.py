import argparse
import os
# workaround to unpickle olf model files
import sys
import platform
import numpy as np
import torch
import pdb # use pdb.set_trace() for debugging

sys.path.append(os.getcwd())
sys.path.append('gym_cdm2')

#from gym_gang.envs import VecPyTorch, make_vec_envs
from stable_baselines3.common.env_util import make_vec_env
from gym_gang.utils import get_render_func, get_vec_normalize
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
import libmainlib as m
import luamodule as lua  # see luamodule.py
import gymnasium
import gym_cdm2

#_globals={} -> moved to settings.py. this way all the strange bugs related to global variables go away.
import settings


# simply forward UI events to lua
def onCallback(mid, userdata):
    lua.onCallback(mid, userdata)

def onFrameChanged(currFrame):
    lua.onFrameChanged(currFrame)

def frameMove(fElapsedTime):
    lua.frameMove(fElapsedTime)
def handleRendererEvent(ev, button, x,y):
    return lua.handleRendererEvent(ev, button, x,y)

def main():

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
    parser.add_argument(
            '--device',
            default='cpu',
            help='cpu or cuda')
    parser.add_argument(
            '--sep',
            action='store_true',
            default=False,
            help='whether to use a windowed-mode')
    parser.add_argument(
            '--testMode',
            action='store_true',
            default=True,
            help='whether to use a test mode')
    args = parser.parse_args()

    args.det = not args.non_det
    args.device='cpu'
    args.testMode=True

    gym_cdm2.luaEnvGymnasium._createMainWin(args.env_name, 'gym_cdm2/spec/'+args.env_name+'.lua', 'isMainloopInLua=true')
    print('ctor finished')

    print('env start',args.env_name, args.seed)

    #device cpu to cuda
    #env = make_vec_envs( args.env_name, args.seed + 1000, 1, None, None, device=args.device, testMode=True, allow_early_resets=False)
    env = make_vec_env(args.env_name, n_envs=1, env_kwargs={'args':args})
    env = VecNormalize(env, norm_obs=True, norm_reward=False)
    #env = VecPyTorch(env, args.device)
    print('env generated')
    # save necessary variables into a global module (settings)

    settings.args=args
    settings.env=env
    # Get a render function
    render_func = get_render_func(env)

    # We need to use the same statistics for normalization as used in training
    actor_critic, obs_rms = \
                torch.load(os.path.join(args.load_dir, args.env_name + ".pt"))


    print('load finished', args.env_name)
    vec_norm = get_vec_normalize(env)
    if vec_norm is not None:
        vec_norm.obs_rms = obs_rms
        vec_norm.obs_rms_orig = obs_rms

    recurrent_hidden_states = torch.zeros(1, actor_critic.recurrent_hidden_state_size)
    masks = torch.zeros(1, 1)

    print('reset started')
    obs = env.reset()
    print('reset finished', obs)

    # save into global variables (settings module)
    settings.ac=actor_critic
    settings.rh=recurrent_hidden_states
    settings.obs=obs
    settings.masks=masks
    settings.reward=0

    #만약 루프를 lua가 아닌 python에서 돌려면 아래 uncomment
    #while True:
    #    envStep() 
    #    if render_func is not None:
    #        render_func()
    #pdb.set_trace()
    m.startMainLoop() # this finishes when program finishes

def envStep():
    masks=settings.masks
    actor_critic=settings.ac
    _args=settings.args
    obs=settings.obs
    env=settings.env
    recurrent_hidden_states=settings.rh

    #print('before', recurrent_hidden_states, obs)
    with torch.no_grad():
        value, action, _, recurrent_hidden_states = actor_critic.act(
            torch.from_numpy(obs.astype(np.float32)), recurrent_hidden_states, masks, deterministic=_args.det)

    #print('after', recurrent_hidden_states, action)
    # Obser reward and next obs
    obs, reward, done, _ = env.step(action)

    settings.rh=recurrent_hidden_states
    settings.obs=obs
    settings.reward+=reward
    if done:
        settings.reward=0
    lua.F('printReward', float(settings.reward))

    if done:
        return False

    masks.fill_(0.0 if done else 1.0)

    # 그리는 것은 루아에서.
    #if render_func is not None:
    #    render_func()
    return True

if __name__=="__main__":
    main()
