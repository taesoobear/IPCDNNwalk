import torch
import torch.nn as nn
import torch.nn.functional as F
import math
from opt_einsum import contract


def reparameterize(mu, logvar):
    std = torch.exp(0.5 * logvar)
    eps = torch.randn_like(mu)
    return mu + eps * std


def sigma2logvar(sigma):
    logvar = 2*math.log(sigma)
    return torch.tensor(logvar)


class PriorEncoder(nn.Module):
    def __init__(self, state_dim, hidden_dim=256, latent_dim=32, sigma_p=0.3):
        super(PriorEncoder, self).__init__()
        self.sigma = sigma_p
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc_mu = nn.Linear(hidden_dim, latent_dim)
        self.ELU = nn.ELU()

    def forward(self, state):
        h = self.ELU(self.fc1(state))
        h = self.ELU(self.fc2(h))
        mu_p = self.fc_mu(h)
        return mu_p


class PosteriorEncoder(nn.Module):
    def __init__(self, state_dim, full_body_obs_dim, hidden_dim=256, latent_dim=32, sigma_q=0.3):
        super(PosteriorEncoder, self).__init__()
        self.sigma = sigma_q
        self.fc1 = nn.Linear(state_dim+full_body_obs_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc_mu = nn.Linear(hidden_dim, latent_dim)
        self.ELU = nn.ELU()

    def forward(self, state, next_observation):
        x = torch.cat([state, next_observation], dim=-1)  # concatenate input and condition
        h = self.ELU(self.fc1(x))
        h = self.ELU(self.fc2(h))
        mu_q = self.fc_mu(h)
        return mu_q


class FullbodyVAEEncoder(nn.Module):
    def __init__(self, state_dim, full_body_obs_dim, hidden_dim=256, latent_dim=32*3, sigma_p=0.3):
        super(FullbodyVAEEncoder, self).__init__()
        self.prior_encoder = PriorEncoder(state_dim, hidden_dim, latent_dim)
        self.posterior_encoder = PosteriorEncoder(state_dim, full_body_obs_dim, hidden_dim, latent_dim)
        self.sigma_p = sigma_p

    def forward(self, current_observation, full_body_observation):
        """
        前向传播，包括后验分布和重参数化采样 z
        :param current_observation: 当前的观测
        :param full_body_observation: 下一时刻的观测
        :return: 重参数化采样得到的 z_t
        """
        # 获取先验均值 mu_p
        mu_p = self.prior_encoder(current_observation)

        # 获取后验残差 mu_q
        mu_q = self.posterior_encoder(current_observation, full_body_observation)

        # 后验均值 mu_hat_q
        mu_hat_q = mu_p + mu_q

        # 使用重参数化技巧生成 z
        z = reparameterize(mu_hat_q, sigma2logvar(self.sigma_p))

        return z, mu_q

    def kl_loss(self, mu_q):
        kl_loss = torch.norm(mu_q, p=2, dim=-1) ** 2 / (2 * self.sigma_p ** 2)
        return torch.mean(kl_loss)


class FullbodyVAEDecoder(nn.Module):
    def __init__(self, state_size, output_size, hidden_dim=256, latent_size=32*3, sigma_pi=0.05):
        super(FullbodyVAEDecoder, self).__init__()
        hidden_size = hidden_dim

        self.fc1 = nn.Linear(state_size + latent_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size + latent_size, hidden_size)
        # self.fc3 = nn.Linear(hidden_size + latent_size, hidden_size)
        self.fc_output = nn.Linear(hidden_size + latent_size, output_size)

        self.ElU = nn.ELU()
        self.sigma = sigma_pi

    def forward(self, state, latent_code):
        x = torch.cat([state, latent_code], dim=-1)  # concatenate input and condition
        h = self.ElU(self.fc1(x))
        h = self.ElU(self.fc2(torch.cat([h, latent_code], dim=-1)))
        # h = self.ElU(self.fc3(torch.cat([h, latent_code], dim=-1)))
        output = self.fc_output(torch.cat([h, latent_code], dim=-1))

        return output
