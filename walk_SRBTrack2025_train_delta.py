import os

# Keep each worker single-threaded for NumPy/SciPy/Clarabel.
# Otherwise multi subprocesses can oversubscribe CPU threads badly.
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import torch
import torch.nn
from env.SRBEnv5_mocap_list_window_training import SRBEnv
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.vec_env import VecNormalize
import time
from env.Policy_prior_posterior_MoE_window_training import SRBPolicy
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
from collections import defaultdict
from myppo import MyPPO as PPO


class RewardLoggingCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(RewardLoggingCallback, self).__init__(verbose)
        self.episode_rewards = defaultdict(list)  # 用于累积当前 episode 内的奖励
        self.current_episode_rewards = defaultdict(float)  # 用于累积单个 episode 的奖励
        self.episode_step_count = 0  # 用于计算每个 episode 的步骤数

    def _on_step(self) -> bool:
        # 在每个 step 后收集奖励信息
        for info in self.locals['infos']:
            for key in ['reward_survive', 'reward_contact',
                        'reward_end_effector', 'reward_delta_pos', 'reward_delta_ori', 'reward_pos', 'reward_ori',
                        'reward_total']:
                if key in info['reward_info']:
                    self.current_episode_rewards[key] += info['reward_info'][key]
        self.episode_step_count += 1

        # 把时间步写入 policy
        self.model.policy.num_timesteps = self.model.num_timesteps
        self.model.policy.total_timesteps = self.model._total_timesteps
        return True

    def _on_rollout_end(self) -> None:
        """
        在 rollout 结束时计算平均奖励，并确保这些奖励显示在 Stable-Baselines3 的 `rollout/` 统计下。
        """
        # # 获取所有完成的episode信息
        # ep_info = self.model.ep_info_buffer
        # ep_lengths = [info['l'] for info in ep_info]  # 获取所有episode的长度
        # ep_lengths_mean = np.mean(np.array(ep_lengths))
        # 获取并行环境数量
        num_envs = self.training_env.num_envs
        mean_rewards = {}
        # 计算每个 episode 的平均奖励
        for key, total_reward in self.current_episode_rewards.items():
            mean_rewards[key] = total_reward / (num_envs*self.episode_step_count) if self.episode_step_count > 0 else 0.0

            # 记录到 SB3 的 rollout 统计中
            self.logger.record(f"rollout/{key}_mean", mean_rewards[key])

        # 重置奖励数据以便下一个 episode 使用
        self.current_episode_rewards.clear()
        self.episode_step_count = 0


if __name__ == '__main__':

    t1 = time.time()
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    n_envs = 32
    mocap_path_0 = "motiondata_mujoco_refined/walk1_subject5.txt"
    mocap_path_1 = "motiondata_mujoco_refined/run1_subject5.txt"
    mocap_path_2 = "motiondata_mujoco_refined/jumps1_subject1.txt"
    mocap_path_3 = "motiondata_mujoco_refined/jumps1_subject5.txt"
    mocap_path_4 = "motiondata_mujoco_refined/sprint1_subject2.txt"
    mocap_path_5 = "motiondata_mujoco_refined_mirrored/walk1_subject5_mirrored.txt"
    mocap_path_6 = "motiondata_mujoco_refined_mirrored/run1_subject5_mirrored.txt"
    mocap_path_7 = "motiondata_mujoco_refined_mirrored/jumps1_subject1_mirrored.txt"
    mocap_path_8 = "motiondata_mujoco_refined_mirrored/jumps1_subject5_mirrored.txt"
    mocap_path_9 = "motiondata_mujoco_refined_mirrored/sprint1_subject2_mirrored.txt"
    mocap_path_10 = "motiondata_mujoco_refined/stand1.txt"
    mocap_path_list = [mocap_path_0, mocap_path_1, mocap_path_2, mocap_path_5, mocap_path_6, mocap_path_7,
                       mocap_path_10, mocap_path_10, mocap_path_10]

    env_kwargs = {
        "mocap_path_list": mocap_path_list
    }

    env = make_vec_env(
        SRBEnv,
        n_envs=n_envs,
        vec_env_cls=SubprocVecEnv,
        env_kwargs=env_kwargs,
        vec_env_kwargs={"start_method": "fork"},
    )

    # 使用自定义回调记录奖励信息
    reward_callback = RewardLoggingCallback()

    tensorboard_log = './tensorboard_logs/AE_window_test/'
    model_saved_name = "AE_window_test_0"


    def exponential_then_constant_schedule(progress_remaining: float) -> float:
        """
        progress_remaining: 1.0 -> 0.0
        在 progress_remaining 从 1.0 -> threshold 时，指数下降
        到达 threshold 后，学习率保持 lr_min 不再变化
        """
        lr_start = 1e-4  # 初始学习率
        lr_min = 1e-5  # 最小学习率（不再下降的值）
        threshold_progress = 0.375  # 衰减截止点，比如最后20%保持不变

        # 衰减系数，lr_end 指的是 threshold_progress 时的值
        decay_rate = lr_min / lr_start

        # progress_remaining 大于 threshold，指数衰减
        if progress_remaining > threshold_progress:
            # 重新映射到 (1.0 -> threshold_progress) 区间
            ratio = (progress_remaining - threshold_progress) / (1.0 - threshold_progress)
            lr = lr_start * (decay_rate ** (1 - ratio))
        else:
            # 保持最小学习率
            lr = lr_min

        return lr


    model = PPO(SRBPolicy, env, verbose=1, n_steps=1024, gamma=0.995, batch_size=512, learning_rate=exponential_then_constant_schedule, ent_coef=0.001,
                tensorboard_log=tensorboard_log, device="cuda", policy_kwargs={"prior_dim": 25, "posterior_dim": 125},)
    # check where the model is, cpu or gpu
    print(next(model.policy.parameters()).device)

    try:
        model.learn(total_timesteps=int(4e7), callback=reward_callback)

        model.save(model_saved_name)
        print("---------- Model saved ----------")
    except KeyboardInterrupt:
        model.save(model_saved_name)
        print("---------- Interupted model saved ----------")
    except Exception as e:
        print(e)
        model.save(model_saved_name)
        print("---------- Error model saved ----------")

    t2 = time.time()
    print(t2-t1)
