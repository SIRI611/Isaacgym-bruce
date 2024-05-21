from time import time
import numpy as np
import os

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil

import torch
# from torch.tensor import Tensor
from typing import Tuple, Dict

from legged_gym.envs import LeggedRobot
from legged_gym import LEGGED_GYM_ROOT_DIR
from .bruce_config import BRUCECfg

class BRUCE(LeggedRobot):
    cfg : BRUCECfg

    # def check_termination(self):
    #     """ Check if environments need to be reset
    #     """
    #     self.reset_buf = torch.any(torch.norm(self.contact_forces[:, self.termination_contact_indices, :], dim=-1) > 1.,
    #                                dim=1)
    #     self.time_out_buf = self.episode_length_buf > self.cfg.env.max_episode_length  # no terminal reward for time-outs
    #     self.reset_buf |= self.time_out_buf
    #     if self.cfg.rewards.use_terminal_body_height:
    #         self.body_height_buf = torch.mean(self.root_states[:, 2].unsqueeze(1) - self.measured_heights, dim=1) \
    #                                < self.cfg.rewards.terminal_body_height
    #         self.reset_buf = torch.logical_or(self.body_height_buf, self.reset_buf)
  