import argparse
import os
# workaround to unpickle olf model files
import sys
import platform
import numpy as np
import torch
import pdb # use pdb.set_trace() for debugging

sys.path.append(os.getcwd())
sys.path.append('gym_cdm')

import work.settings as settings

class EnvNormalize:
    """
    A non-vectorized wrapper by taesoo that normalizes the observations
    """
    def __init__(self, ob_rms, ret=True, clipob=10., cliprew=10., gamma=0.99, epsilon=1e-8 ):
        self.ob_rms = ob_rms
        self.clipob = clipob
        self.gamma = gamma
        self.epsilon = epsilon


    def process(self, obs):
        if self.ob_rms:
            obs = np.clip((obs - self.ob_rms.mean) / np.sqrt(self.ob_rms.var + self.epsilon), -self.clipob, self.clipob)
            return torch.from_numpy(obs).float()
        else:
            return obs

def loadAgent(spec_id, filename):
    if not (hasattr(settings, 'agents')):
        settings.agents={}

    ac, ob_rms= torch.load(filename, weights_only=False)
    vec_norm =EnvNormalize(ob_rms);
    agent={
            'actor_critic': ac, 
            'ob_rms': ob_rms,
            'vec_norm':vec_norm,
            }
    settings.agents[spec_id]=agent


def getAction(spec_id, obs, taction):
    agent=settings.agents[spec_id]
    actor_critic=agent['actor_critic']
    obs=agent['vec_norm'].process(obs.ref())
    base=actor_critic.base
    with torch.no_grad():
        actor_features=base.actor(obs)
        action=actor_critic.dist.fc_mean(actor_features)
        action=action.cpu().numpy().astype(np.float64)
    taction.setSize(len(action))
    taction.ref()[:]=action


