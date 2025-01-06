import os

current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
os.sys.path.append(parent_dir)

import torch
import torch.nn as nn
import torch.nn.functional as F

from gym_gang.controller import init, DiagGaussian
import pdb

init_r_ = lambda m: init(
    m,
    nn.init.orthogonal_,
    lambda x: nn.init.constant_(x, 0),
    nn.init.calculate_gain("relu"),
)
init_s_ = lambda m: init(
    m,
    nn.init.orthogonal_,
    lambda x: nn.init.constant_(x, 0),
    nn.init.calculate_gain("sigmoid"),
)
init_t_ = lambda m: init(
    m,
    nn.init.orthogonal_,
    lambda x: nn.init.constant_(x, 0),
    nn.init.calculate_gain("tanh"),
)

#controller (
class Controller(nn.Module):
    def __init__(self,observation_dim,action_dim):
        super().__init__()
        
        self.observation_dim = observation_dim[0]
        self.action_dim = action_dim.shape[0]

        h_size = 64
        self.actor = nn.Sequential(
            init_t_(nn.Linear(self.observation_dim, h_size)),
            nn.Tanh(),
            init_t_(nn.Linear(h_size, h_size)),
            nn.Tanh(),
            init_t_(nn.Linear(h_size, h_size)),
            nn.Tanh(),
            init_t_(nn.Linear(h_size, self.action_dim)),
            nn.Tanh(),
        )

    def forward(self, x):
        return self.actor(x)

#Policy
class Policy(nn.Module):
    def __init__(self, controller):
        super().__init__()
        self.actor = controller
        self.dist = DiagGaussian(controller.action_dim, controller.action_dim)

        init_s_ = lambda m: init(
            m,
            nn.init.orthogonal_,
            lambda x: nn.init.constant_(x, 0),
            nn.init.calculate_gain("sigmoid"),
        )
        init_r_ = lambda m: init(
            m,
            nn.init.orthogonal_,
            lambda x: nn.init.constant_(x, 0),
            nn.init.calculate_gain("relu"),
        )

        h_size = 64
        self.critic = nn.Sequential(
            init_t_(nn.Linear(controller.observation_dim, h_size)),
            nn.Tanh(),
            init_t_(nn.Linear(h_size, h_size)),
            nn.Tanh(),
            init_t_(nn.Linear(h_size, h_size)),
            nn.Tanh(),
            init_t_(nn.Linear(h_size, 1)),
        )
        self.state_size = 1

    def forward(self, inputs):
        raise NotImplementedError

    def act(self, inputs, deterministic=False):
        action = self.actor(inputs)
        dist = self.dist(action)

        if deterministic:
            action = dist.mode()
        else:
            action = dist.sample()
            action.clamp_(-1.0, 1.0)

        action_log_probs = dist.log_probs(action)
        value = self.critic(inputs)

        return value, action, action_log_probs

    def get_value(self, inputs):
        value = self.critic(inputs)
        return value

    def evaluate_actions(self, inputs, action):
        value = self.critic(inputs)
        mode = self.actor(inputs)
        dist = self.dist(mode)

        action_log_probs = dist.log_probs(action)
        dist_entropy = dist.entropy().mean()

        return value, action_log_probs, dist_entropy
