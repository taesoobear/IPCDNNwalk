import copy
import glob
import os
import time
from collections import deque
import pdb
import gym
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import sys
sys.path.append(os.getcwd())
from gym_gang import algo, utils
from gym_gang.algo import gail
from gym_gang.arguments import get_args
from gym_gang.envs import make_vec_envs
from gym_gang.model import Policy
from gym_gang.storage import RolloutStorage
from gym_gang.evaluation import evaluate

import settings
settings.useConsole=True

import luamodule as lua  # see luamodule.py
from tensorboardX import SummaryWriter
import gym_cdm2

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
    args = get_args()
    args.algo='ppo'
    args.num_steps=1024
    args.use_gae=True
    #learning rate
    args.lr=1e-4
    #nn update 시 너무 많이 하면 안되서 하는거 
    #args.clip_param=0.2
    args.clip_param=0.2
    #ciritic networ update시 0.5 or 1
    args.value_loss_coef=0.5
    args.num_mini_batch=256
    args.log_interval=1
    args.use_linear_lr_decay=True
    #action 랜덤성 주는 것(0.01)
    args.entropy_coef=0.01
    args.ppo_epoch=10
    args.num_env_steps=200000000
    args.gamma=0.995
    args.lamda=0.95
    args.time_save_path=args.env_name+'-time'
    args.frame_save_path=args.env_name+'-frame'
    args.best_save_path=args.env_name+'-best'
    #new method
    args.testMode=False
    args.cuda=False # taesoo

    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)

    writer = SummaryWriter(comment="ppo"+"\nenv_name " + args.env_name + "\nbatch size "+str(args.num_steps)+"\nmini batch size "+ str(args.num_mini_batch)
           +"\nlearning rate "+ str(args.lr) +"\nclip "+ str(args.clip_param))
    #writer = None # to disable summary writing

    if args.cuda and torch.cuda.is_available() and args.cuda_deterministic:
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True

    log_dir = os.path.expanduser(args.log_dir)
    eval_log_dir = log_dir + "_eval"
    utils.cleanup_log_dir(log_dir)
    utils.cleanup_log_dir(eval_log_dir)

    torch.set_num_threads(1)
    device = torch.device("cuda:0" if args.cuda else "cpu")
    #device="cpu"
    envs = make_vec_envs(args.env_name, args.seed, args.num_processes,
                         args.gamma, args.log_dir, device,args.testMode, False)
    actor_critic = Policy(
        envs.observation_space.shape,
        envs.action_space,
        base_kwargs={'recurrent': args.recurrent_policy})
    actor_critic.to(device)
    if args.algo == 'a2c':
        agent = algo.A2C_ACKTR(
            actor_critic,
            args.value_loss_coef,
            args.entropy_coef,
            lr=args.lr,
            eps=args.eps,
            alpha=args.alpha,
            max_grad_norm=args.max_grad_norm)
    elif args.algo == 'ppo':
        agent = algo.PPO(
            actor_critic,
            args.clip_param,
            args.ppo_epoch,
            args.num_mini_batch,
            args.value_loss_coef,
            args.entropy_coef,
            lr=args.lr,
            eps=args.eps,
            max_grad_norm=args.max_grad_norm)
    elif args.algo == 'acktr':
        agent = algo.A2C_ACKTR(
            actor_critic, args.value_loss_coef, args.entropy_coef, acktr=True)

    if args.gail:
        assert len(envs.observation_space.shape) == 1
        discr = gail.Discriminator(
            envs.observation_space.shape[0] + envs.action_space.shape[0], 100,
            device)
        file_name = os.path.join(
            args.gail_experts_dir, "trajs_{}.pt".format(
                args.env_name.split('-')[0].lower()))

        gail_train_loader = torch.utils.data.DataLoader(
            gail.ExpertDataset(
                file_name, num_trajectories=4, subsample_frequency=20),
            batch_size=args.gail_batch_size,
            shuffle=True,
            drop_last=True)

    rollouts = RolloutStorage(args.num_steps, args.num_processes,
                              envs.observation_space.shape, envs.action_space,
                              actor_critic.recurrent_hidden_state_size)
    obs = envs.reset()
    rollouts.obs[0].copy_(obs)
    rollouts.to(device)

    episode_rewards = deque(maxlen=10)

    start = time.time()
    time_interval=0
    best_reward=0
    current_reward=0

    num_updates = int(
        args.num_env_steps) // args.num_steps // args.num_processes
    for j in range(num_updates):
        if args.use_linear_lr_decay:
            # decrease learning rate linearly
            utils.update_linear_schedule(
                agent.optimizer, j, num_updates,
                agent.optimizer.lr if args.algo == "acktr" else args.lr)

        #envs.venv.updateIterJ(j)

        for step in range(args.num_steps):
            # Sample actions
            with torch.no_grad():
                value, action, action_log_prob, recurrent_hidden_states = actor_critic.act(
                    rollouts.obs[step], rollouts.recurrent_hidden_states[step],
                    rollouts.masks[step])
            obs, reward, done, infos = envs.step(action)
            #for kk in range(args.num_processes):
            #    if done[kk]==True:
            #        print('episode end')
            #        break;
            for info in infos:
                if 'episode' in info.keys():
                    episode_rewards.append(info['episode']['r'])
                    current_reward=info['episode']['r']

            # If done then clean the history of observations.
            masks = torch.FloatTensor(
                [[0.0] if done_ else [1.0] for done_ in done])
            bad_masks = torch.FloatTensor(
                [[0.0] if 'bad_transition' in info.keys() else [1.0]
                 for info in infos])
            rollouts.insert(obs, recurrent_hidden_states, action,
                            action_log_prob, value, reward, masks, bad_masks)
        with torch.no_grad():
            next_value = actor_critic.get_value(
                rollouts.obs[-1], rollouts.recurrent_hidden_states[-1],
                rollouts.masks[-1]).detach()

        if args.gail:
            if j >= 10:
                envs.venv.eval()

            gail_epoch = args.gail_epoch
            if j < 10:
                gail_epoch = 100  # Warm up
            for _ in range(gail_epoch):
                discr.update(gail_train_loader, rollouts,
                             utils.get_vec_normalize(envs)._obfilt)

            for step in range(args.num_steps):
                rollouts.rewards[step] = discr.predict_reward(
                    rollouts.obs[step], rollouts.actions[step], args.gamma,
                    rollouts.masks[step])

        rollouts.compute_returns(next_value, args.use_gae, args.gamma,
                                 args.gae_lambda, args.use_proper_time_limits)

        value_loss, action_loss, dist_entropy = agent.update(rollouts)

        rollouts.after_update()
        if (j % args.save_interval == 0
                or j == num_updates - 1) and args.save_dir != "":
            save_path = os.path.join(args.save_dir, args.algo)
            try:
                os.makedirs(save_path)
            except OSError:
                pass

            torch.save([
                actor_critic,
                getattr(utils.get_vec_normalize(envs), 'ob_rms', None)
            ], os.path.join(save_path, args.env_name + ".pt"))
        #3600 = 1 minute
        if writer != None and (time.time()-start)%3600*30>time_interval:
            time_interval+=1
            save_path = os.path.join(args.save_dir, args.algo, args.time_save_path)
            try:
                os.makedirs(save_path)
            except OSError:
                pass

            torch.save([
                actor_critic,
                getattr(utils.get_vec_normalize(envs), 'ob_rms', None)
            ], os.path.join(save_path, args.env_name +str(time_interval) +".pt"))

        if writer !=None and current_reward>best_reward:
            best_reward=current_reward
            save_path = os.path.join(args.save_dir, args.algo, args.best_save_path)
            try:
                os.makedirs(save_path)
            except OSError:
                pass

            torch.save([
                actor_critic,
                getattr(utils.get_vec_normalize(envs), 'ob_rms', None)
            ], os.path.join(save_path, args.env_name +"("+str(best_reward) +")"+".pt"))
            print('best case episode update', best_reward)


        if j % args.log_interval == 0 and len(episode_rewards) > 1:
            total_num_steps = (j + 1) * args.num_processes * args.num_steps
            end = time.time()

            mean = np.mean(episode_rewards)
            if writer!=None :
                writer.add_scalar('steps', total_num_steps/(end-start), j)
                writer.add_scalar('mean_reward_update', mean, j)
                writer.add_scalar('mean_reward_step', mean, total_num_steps)
                writer.add_scalar('median_reward', np.median(episode_rewards), j)
                writer.add_scalar('max_reward', np.max(episode_rewards), j)
                writer.add_scalar('min_reward', np.min(episode_rewards), j)
                writer.add_scalar('dist_entropy', dist_entropy, j)
                writer.add_scalar('value_loss', value_loss, j)
                writer.add_scalar('action_loss', action_loss, j)

            print(
                "Updates {}, num timesteps {}, FPS {} \n Last {} training episodes: mean/median reward {:.1f}/{:.1f}, min/max reward {:.1f}/{:.1f}\n"
                .format(j, total_num_steps,
                        int(total_num_steps / (end - start)),
                        len(episode_rewards), np.mean(episode_rewards),
                        np.median(episode_rewards), np.min(episode_rewards),
                        np.max(episode_rewards), dist_entropy, value_loss,
                        action_loss))

        if (args.eval_interval is not None and len(episode_rewards) > 1
                and j % args.eval_interval == 0):
            ob_rms = utils.get_vec_normalize(envs).ob_rms
            evaluate(actor_critic, ob_rms, args.env_name, args.seed,
                     args.num_processes, eval_log_dir, device)


if __name__ == "__main__":
    main()
