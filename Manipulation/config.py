import numpy as np

# dataset parameters
pred_horizon = 16
obs_horizon = 2
action_horizon = 8

# network parameters
# ResNet18 has output dim of 512
vision_feature_dim = 512
# agent_pos is 12 dimensional
lowdim_obs_dim = 12
# observation feature has 512 + 12 dims in total per step
obs_dim = vision_feature_dim + lowdim_obs_dim
# agent_action is 12 dimensional
action_dim = 12

stats = {
        'min': np.array([0.000] * 12),
        'max': np.array([0.020] * 12)}
