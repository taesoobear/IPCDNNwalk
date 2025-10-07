import torch
import torch as th
import torch.nn as nn
import torch.nn.functional as F
from stable_baselines3.common.policies import ActorCriticPolicy
from gymnasium import spaces
from typing import Tuple, Optional

from stable_baselines3.common.type_aliases import Schedule, PyTorchObs

import numpy as np
from opt_einsum import contract
import math

res = []
res_1 = []
res_2 = []
def reparameterize(mu, logvar):
    std = torch.exp(0.5 * logvar)
    eps = torch.randn_like(mu)
    return mu + eps * std


def sigma2logvar(sigma):
    logvar = 2*math.log(sigma)
    return torch.tensor(logvar)


def batch_axis_angle_to_rotation_matrix(axis_angles: torch.Tensor) -> torch.Tensor:
    """
    Convert a batch of axis-angle vectors to rotation matrices.

    Args:
        axis_angles: Tensor of shape [batch_size, 3]

    Returns:
        rotation_matrices: Tensor of shape [batch_size, 3, 3]
    """
    batch_size = axis_angles.shape[0]

    # Compute rotation angles (norm of each axis-angle vector)
    angles = torch.norm(axis_angles, dim=1, keepdim=True)  # [batch_size, 1]

    # Avoid division by zero by setting a small epsilon
    eps = 1e-8

    # Normalize axis vectors, avoid NaNs when angle is zero
    axes = axis_angles / (angles + eps)  # [batch_size, 3]

    # Compute sine and cosine of angles
    sin = torch.sin(angles).unsqueeze(-1)  # [batch_size, 1, 1]
    cos = torch.cos(angles).unsqueeze(-1)  # [batch_size, 1, 1]

    # Create skew-symmetric cross-product matrices K for each axis
    zero = torch.zeros(batch_size, 1, device=axis_angles.device)

    kx = axes[:, 0].unsqueeze(1)  # [batch_size, 1]
    ky = axes[:, 1].unsqueeze(1)
    kz = axes[:, 2].unsqueeze(1)

    K = torch.stack([
        torch.cat([zero, -kz, ky], dim=1),
        torch.cat([kz, zero, -kx], dim=1),
        torch.cat([-ky, kx, zero], dim=1)
    ], dim=1)  # [batch_size, 3, 3]

    # Compute rotation matrix: I + sin(angle) * K + (1 - cos(angle)) * K @ K
    I = torch.eye(3, device=axis_angles.device).unsqueeze(0)  # [1, 3, 3]

    K_dot_K = torch.bmm(K, K)

    R = I + sin * K + (1 - cos) * K_dot_K  # [batch_size, 3, 3]

    # For angles near zero, fallback to identity matrix
    mask = (angles.squeeze(-1) < eps).float().unsqueeze(-1).unsqueeze(-1)  # [batch_size, 1, 1]
    R = R * (1 - mask) + I * mask

    return R


def convert_matrix_to_6d(rot_matrix):
    """
    将3x3旋转矩阵转换为6D表示
    :param rot_matrix: shape (..., 3, 3), 旋转矩阵
    :return: shape (..., 6), 6D旋转表示
    """
    rot_6d = torch.cat([rot_matrix[..., 0], rot_matrix[..., 1]], dim=-1)
    return rot_6d


class CustomFeatureExtractor(nn.Module):
    """
    自定义特征抽取层
    输入：obs（观察空间）
    输出：共享的特征
    """

    def __init__(self, obs_dim):
        super(CustomFeatureExtractor, self).__init__()

        self.net = nn.Sequential(
            nn.Linear(obs_dim, obs_dim),
            nn.Tanh(),
        )

    def forward(self, obs):
        return self.net(obs)


class CustomNetwork(nn.Module):
    """
    自定义策略网络层
    输入：共享特征
    输出：连续动作均值、离散动作 logits、值函数
    """
    def __init__(self, feature_dim, hidden_dim=256):
        super(CustomNetwork, self).__init__()

        self.latent_dim_pi = 32
        self.latent_dim_vf = 32

        self.policy_net = nn.Sequential(
            torch.nn.Linear(feature_dim, hidden_dim),
            torch.nn.Tanh(),
            torch.nn.Linear(hidden_dim, hidden_dim),
            torch.nn.Tanh(),
            torch.nn.Linear(hidden_dim, self.latent_dim_pi),
            torch.nn.Tanh(),
        )
        self.value_net = nn.Sequential(
            torch.nn.Linear(feature_dim, hidden_dim),
            torch.nn.Tanh(),
            torch.nn.Linear(hidden_dim, hidden_dim),
            torch.nn.Tanh(),
            torch.nn.Linear(hidden_dim, self.latent_dim_vf),
            torch.nn.Tanh(),
        )

    def forward(self, features):
        return self.forward_actor(features), self.forward_critic(features)

    def forward_actor(self, features):
        return self.policy_net(features)

    def forward_critic(self, features):
        return self.value_net(features)


class GatingMixedDecoder(nn.Module):
    def __init__(self, state_size, action_size, hidden_dim=128, latent_size=32, sigma_pi=0.05):
        super(GatingMixedDecoder, self).__init__()

        hidden_size = hidden_dim
        num_experts = 6
        num_layer = 3
        input_size = latent_size + state_size
        inter_size = latent_size + hidden_size
        self.sigma = sigma_pi
        self.activation = nn.ELU()

        # 创建专家网络层
        self.decoder_layers = []
        for i in range(num_layer + 1):
            layer = (
                nn.Parameter(torch.empty(num_experts, inter_size if i != 0 else input_size, hidden_size if i != num_layer else action_size)),
                nn.Parameter(torch.empty(num_experts, hidden_size if i != num_layer else action_size)),
                self.activation if i < num_layer else None
            )
            self.decoder_layers.append(layer)

        # 初始化权重和标准差
        for index, (weight, bias, _) in enumerate(self.decoder_layers):
            index = str(index)
            stdv = 1.0 / math.sqrt(weight.size(1))
            weight.data.uniform_(-3*stdv, 3*stdv)
            bias.data.uniform_(-stdv, stdv)
            self.register_parameter("w" + index, weight)
            self.register_parameter("b" + index, bias)

        # 门控网络
        gate_hsize = 32
        self.gate = nn.Sequential(
            nn.Linear(input_size, gate_hsize),
            nn.ELU(),
            nn.Linear(gate_hsize, gate_hsize),
            nn.ELU(),
            nn.Linear(gate_hsize, num_experts)
        )

    def forward(self, state, latent):
        input_combined = torch.cat((state, latent), dim=-1)
        coefficients = F.softmax(self.gate(input_combined), dim=-1)
        layer_out = state
        for (weight, bias, activation) in self.decoder_layers:
            input = latent if layer_out is None else torch.cat((layer_out, latent), dim=-1)

            # 根据 coefficients 的维度判断是否有 batch_size
            if coefficients.dim() == 1:
                # input = F.layer_norm(input, input.shape[:])
                # 推断模式（没有 batch_size）
                mixed_bias = contract('e, ek->k', coefficients, bias)
                mixed_input = contract('e, j, ejk->k', coefficients, input, weight)
            else:
                # 训练模式（有 batch_size）
                # input = F.layer_norm(input, input.shape[1:])
                mixed_bias = contract('be, ek->bk', coefficients, bias)
                mixed_input = contract('be, bj, ejk->bk', coefficients, input, weight)
            out = mixed_input + mixed_bias
            layer_out = activation(out) if activation is not None else out
        layer_out = reparameterize(layer_out, sigma2logvar(self.sigma))
        return layer_out


_last_contact_prob=[None, None]
class SRBPolicy(ActorCriticPolicy):
    """
    自定义策略:
      - 连续动作: continuous_dim
      - 离散动作: 4个可选 (one-hot形式)
      - 离散概率: softmax(logits)
      - 输出动作的顺序:
        actions = [continuous_actions, discrete_one_hot]

    Custom policy:
    - Continuous action: continuous_dim
    - Discrete action: 4 options (one-hot form)
    - Discrete probability: softmax(logits)
    - Sequence of output actions:
        actions = [continuous_actions, discrete_one_hot]
    """

    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Space,
        lr_schedule: callable,
        prior_dim: int,
        posterior_dim: int,
        *args,
        **kwargs
    ):
        # 1. **先**定义子类特有的属性
        self.continuous_dim = action_space.shape[0] - 4  # 18 + 4 = 22
        self.discrete_dim = 4  # 离散动作数量
        self.hidden_dim = 128
        self.prior_dim = prior_dim
        self.posterior_dim = posterior_dim
        self.called_num = 0
        self.num_timesteps = 0.0
        self.total_timesteps = 0.0
        # 2. 调用父类构造函数
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            *args,
            **kwargs,
        )

    def _build(self, lr_schedule: Schedule) -> None:
        self._build_mlp_extractor()

        latent_dim_pi = self.mlp_extractor.latent_dim_pi
        latent_dim_vf = self.mlp_extractor.latent_dim_vf

        self.decoder = GatingMixedDecoder(self.prior_dim, self.hidden_dim)
        # self.decoder = LatentConcatDecoder(latent_dim_pi, self.prior_dim, self.hidden_dim)

        # 连续动作头/ Continuous action head
        self.continuous_action_net_mocap = nn.Sequential(
            nn.Linear(self.hidden_dim, self.hidden_dim),
            nn.Tanh(),
            nn.Linear(self.hidden_dim, self.continuous_dim)
        )
        self.continuous_action_net_delta = nn.Sequential(
            nn.Linear(self.hidden_dim, self.hidden_dim),
            nn.Tanh(),
            nn.Linear(self.hidden_dim, 6)
        )
        # 遍历网络层，做初始化
        for m in self.continuous_action_net_delta.modules():
            if isinstance(m, nn.Linear):
                if m.out_features == 6:
                    # 输出层：小权重 + 零偏置
                    nn.init.normal_(m.weight, mean=0.0, std=1e-3)
                    nn.init.constant_(m.bias, 0.0)
                else:
                    # 隐藏层权重正常初始化
                    nn.init.kaiming_uniform_(m.weight, a=math.sqrt(5))
                    nn.init.constant_(m.bias, 0.0)

        # 创建可训练的 log_std 用于连续部分/ Create trainable log_std for continuous parts
        self.log_std = nn.Parameter(torch.full((self.continuous_dim,), -2.5))  # -0.5
        # 离散动作头/ Discrete action head
        self.discrete_action_net = nn.Sequential(
            nn.Linear(self.hidden_dim, self.hidden_dim),
            nn.ReLU(),
            nn.Linear(self.hidden_dim, self.discrete_dim)
        )
        # 值函数头/ Value function header
        self.value_net = nn.Linear(latent_dim_vf, 1)
        self.ortho_init = False

        # # freeze decoder and action head
        # for param in self.mlp_extractor.policy_net.parameters():
        #     param.requires_grad = False
        # for param in self.decoder.parameters():
        #     param.requires_grad = False
        # for param in self.continuous_action_net_mocap.parameters():
        #     param.requires_grad = False
        # for param in self.continuous_action_net_delta.parameters():
        #     param.requires_grad = False
        # for param in self.discrete_action_net.parameters():
        #     param.requires_grad = False
        # self.log_std.requires_grad = False

        # Setup optimizer with initial learning rate
        self.optimizer = self.optimizer_class(self.parameters(), lr=lr_schedule(1), **self.optimizer_kwargs)  # type: ignore[call-arg]

    def _build_mlp_extractor(self) -> None:
        self.mlp_extractor = CustomNetwork(self.prior_dim+self.posterior_dim, hidden_dim=256)

    def forward(self, obs: th.Tensor, deterministic: bool = False) -> Tuple[th.Tensor, th.Tensor, th.Tensor]:
        prior_obs = obs[..., :self.prior_dim]

        latent_pi, latent_vf = self.mlp_extractor(obs)
        latent_pi = self.decoder(prior_obs, latent_pi)

        # 离散部分/ Discrete part
        discrete_logits = self.discrete_action_net(latent_pi)
        discrete_dist = torch.distributions.Categorical(logits=discrete_logits)

        if deterministic:
            #discrete_actions = torch.argmax(discrete_dist.probs, dim=-1)
            # convert joint probability to independent probability through marginalization
            discrete_actions_L= discrete_dist.probs[:, 0::2].sum(dim=-1)<discrete_dist.probs[:, 1::2].sum(dim=-1)
            discrete_actions_R= discrete_dist.probs[:, 0:2].sum(dim=-1)<discrete_dist.probs[:, 2:].sum(dim=-1)

            #import work.rendermodule as RE
            #RE.output('contactPredL', discrete_actions_L, discrete_dist.probs[:, 0::2].sum(dim=-1),discrete_dist.probs[:, 1::2].sum(dim=-1))
            #RE.output('contactPredR', discrete_actions_R,  discrete_dist.probs[:, 0:2].sum(dim=-1),discrete_dist.probs[:, 2:].sum(dim=-1))

            discrete_actions=discrete_actions_R*2+discrete_actions_L

            global _last_contact_prob
            if discrete_dist.probs.shape[0]==1:
                # test_mode
                _last_contact_prob[0]=discrete_dist.probs[:, 1::2].sum(dim=-1)[0].item()
                _last_contact_prob[1]=discrete_dist.probs[:, 2:].sum(dim=-1)[0].item()
        else:
            discrete_actions = discrete_dist.sample()

        discrete_one_hot = F.one_hot(discrete_actions, num_classes=self.discrete_dim).float()
        discrete_log_prob = discrete_dist.log_prob(discrete_actions)

        # 连续部分/ Continuous part
        continuous_mean = self.continuous_action_net_mocap(latent_pi)
        delta = self.continuous_action_net_delta(latent_pi)
        # print(continuous_mean)
        # print(delta)
        increment = torch.zeros_like(continuous_mean)
        increment[..., [0, 1, 2, 3, 16, 17]] = delta
        continuous_mean = continuous_mean + 0.1*increment

        continuous_std = self.log_std.exp()
        continuous_dist = torch.distributions.Normal(continuous_mean, continuous_std)

        if deterministic:
            continuous_actions = continuous_mean
        else:
            continuous_actions = continuous_dist.rsample()

        continuous_log_prob = continuous_dist.log_prob(continuous_actions).sum(dim=-1)

        # 拼起来: 前 continuous_dim, 后 discrete_dim/ Connect: pre-continuous_dim, post-discrete_dim
        actions = torch.cat([continuous_actions, discrete_one_hot], dim=-1)
        log_prob = continuous_log_prob + discrete_log_prob

        # critic
        values = self.value_net(latent_vf).flatten()

        return actions, values, log_prob

    def evaluate_actions(self, obs: torch.Tensor, actions: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        训练时, SB3会再调用 policy.evaluate_actions(...) 来计算:
          - values
          - log_prob
          - entropy
        给定: obs, actions(从buffer拿到)
        During training, SB3 calls policy.evaluate_actions(...) again. To calculate:
            - values
            - log_prob
            - entropy
        Given: obs, actions(from buffer)
        """
        latent_pi_, latent_vf = self.mlp_extractor(obs)
        prior_obs = obs[..., :self.prior_dim]
        latent_pi = self.decoder(prior_obs, latent_pi_)

        # 拆分出 连续动作 & 离散 one-hot & discrete_probs/ Separate out continuous action & discrete one-hot & discrete_probs
        continuous_actions = actions[..., :self.continuous_dim]
        discrete_one_hot = actions[..., self.continuous_dim:self.continuous_dim + self.discrete_dim]

        discrete_actions = torch.argmax(discrete_one_hot, dim=-1)

        # 连续分布/ Continuous distribution
        continuous_mean = self.continuous_action_net_mocap(latent_pi)
        ori_continuous_mean = continuous_mean.clone()
        delta = self.continuous_action_net_delta(latent_pi)

        increment = torch.zeros_like(continuous_mean)
        increment[..., [0, 1, 2, 3, 16, 17]] = delta
        continuous_mean = continuous_mean + 0.1*increment


        # 离散分布/ Discrete distribution
        discrete_logits = self.discrete_action_net(latent_pi)
        discrete_dist = torch.distributions.Categorical(logits=discrete_logits)
        discrete_log_prob = discrete_dist.log_prob(discrete_actions)
        discrete_entropy = discrete_dist.entropy()

        # supervised loss
        weight_start_timestep = 0  # int(1e7)
        weight_end_timestep = int(2e7)
        if self.num_timesteps >= weight_start_timestep and self.num_timesteps <= weight_end_timestep:
            weight_ratio = (self.num_timesteps-weight_start_timestep)/(weight_end_timestep-weight_start_timestep)
        elif self.num_timesteps < weight_start_timestep:
            weight_ratio = 0.0
        else:
            weight_ratio = 1.0
        # weight_ratio = 0

        # ce_loss = F.cross_entropy(categorical_logits, indices.detach())

        next_feet_contact_target = obs[..., 46:50]
        target = next_feet_contact_target.argmax(dim=-1)
        next_feet_ce_loss = F.cross_entropy(discrete_logits, target)
        ce_loss = next_feet_ce_loss*0.15  # (0.01 + 0.19*weight_ratio)

        action_foot_xy = ori_continuous_mean[..., [0, 1, 2, 3, 16, 17]]
        obs_foot_xy = obs[..., [38, 39, 42, 43, 41, 45]]

        ce_loss = ce_loss+((action_foot_xy - obs_foot_xy) ** 2).mean()*50

        continuous_std = self.log_std.exp()
        continuous_dist = torch.distributions.Normal(continuous_mean, continuous_std)
        continuous_log_prob = continuous_dist.log_prob(continuous_actions).sum(dim=-1)
        continuous_entropy = continuous_dist.entropy().sum(dim=-1)

        # recularization term
        regularization_loss = (delta**2).mean()
        ce_loss = ce_loss+regularization_loss*0.015  # (0.1+0.1*weight_ratio)

        # total log_prob & entropy
        log_prob = continuous_log_prob + discrete_log_prob
        entropy = continuous_entropy + discrete_entropy

        # 价值
        values = self.value_net(latent_vf).flatten()

        return values, log_prob, entropy, ce_loss

    def _predict(self, observation: torch.Tensor, deterministic: bool = False) -> torch.Tensor:
        """
        当SB3在 rollout 时需要执行动作，会调用此函数.
        This function is called when SB3 needs to perform an action during rollout.
        """
        actions, _, _ = self.forward(observation, deterministic=deterministic)
        return actions
