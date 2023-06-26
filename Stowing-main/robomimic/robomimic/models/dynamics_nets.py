"""
Contains torch Modules for policy networks. These networks take an
observation dictionary as input (and possibly additional conditioning,
such as subgoal or goal dictionaries) and produce pred_state predictions,
samples, or distributions as outputs. Note that pred_states
are assumed to lie in [-1, 1], and most networks will have a final
tanh activation to help ensure this range.
"""
import textwrap
import numpy as np
from collections import OrderedDict

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.distributions as D

import robomimic.utils.tensor_utils as TensorUtils
from robomimic.models.base_nets import Module
from robomimic.models.obs_nets import MIMO_MLP, RNN_MIMO_MLP
from robomimic.models.vae_nets import VAE
from robomimic.models.distributions import TanhWrappedDistribution


class DynamicsNetwork(MIMO_MLP):
    """
    A basic policy network that predicts pred_obs from observations.
    Can optionally be goal conditioned on future observations.
    """
    def __init__(
        self,
        obs_shapes,
        pred_obs_dim,
        mlp_layer_dims,
        goal_shapes=None,
        encoder_kwargs=None,
    ):
        """
        Args:
            obs_shapes (OrderedDict): a dictionary that maps observation keys to
                expected shapes for observations.

            pred_dim (int): dimension of pred_ob space.

            mlp_layer_dims ([int]): sequence of integers for the MLP hidden layers sizes.

            goal_shapes (OrderedDict): a dictionary that maps observation keys to
                expected shapes for goal observations.

            encoder_kwargs (dict or None): If None, results in default encoder_kwargs being applied. Otherwise, should
                be nested dictionary containing relevant per-observation key information for encoder networks.
                Should be of form:

                obs_modality1: dict
                    feature_dimension: int
                    core_class: str
                    core_kwargs: dict
                        ...
                        ...
                    obs_randomizer_class: str
                    obs_randomizer_kwargs: dict
                        ...
                        ...
                obs_modality2: dict
                    ...
        """
        assert isinstance(obs_shapes, OrderedDict)
        self.obs_shapes = obs_shapes
        self.pred_obs_dim = pred_obs_dim

        # set up different observation groups for @MIMO_MLP
        observation_group_shapes = OrderedDict()
        observation_group_shapes["obs"] = OrderedDict(self.obs_shapes)

        self._is_goal_conditioned = False
        if goal_shapes is not None and len(goal_shapes) > 0:
            assert isinstance(goal_shapes, OrderedDict)
            self._is_goal_conditioned = True
            self.goal_shapes = OrderedDict(goal_shapes)
            observation_group_shapes["goal"] = OrderedDict(self.goal_shapes)
        else:
            self.goal_shapes = OrderedDict()

        output_shapes = self._get_output_shapes()
        super(DynamicsNetwork, self).__init__(
            input_obs_group_shapes=observation_group_shapes,
            output_shapes=output_shapes,
            layer_dims=mlp_layer_dims,
            encoder_kwargs=encoder_kwargs,
        )

    def _get_output_shapes(self):
        """
        Allow subclasses to re-define outputs from @MIMO_MLP, since we won't
        always directly predict pred_obs, but may instead predict the parameters
        of a pred_state distribution.
        """
        return OrderedDict(pred_obs=self.pred_obs_dim,)

    def output_shape(self, input_shape=None):
        return [self.pred_obs_dim]

    def forward(self, obs_dict, goal_dict=None):
        pred_obs = super(DynamicsNetwork, self).forward(obs=obs_dict, goal=goal_dict)["pred_obs"]
        # apply tanh squashing to ensure pred_obs are in [-1, 1]
        return torch.tanh(pred_obs)

    def _to_string(self):
        """Info to pretty print."""
        return "pred_obs_dim={}".format(self.pred_obs_dim)


