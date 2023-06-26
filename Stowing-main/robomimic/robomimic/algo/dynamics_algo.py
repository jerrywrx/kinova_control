"""
Implementation of 
"""
from collections import OrderedDict
from copy import deepcopy

import torch
import torch.nn as nn
import torch.nn.functional as F

import robomimic.models.base_nets as BaseNets
import robomimic.models.obs_nets as ObsNets
import robomimic.models.policy_nets as PolicyNets
import robomimic.models.vae_nets as VAENets
import robomimic.models.dynamics_nets as DynamicsNets
import robomimic.utils.loss_utils as LossUtils
import robomimic.utils.tensor_utils as TensorUtils
import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.obs_utils as ObsUtils

from robomimic.algo import register_algo_factory_func, PolicyAlgo


@register_algo_factory_func("dynamics_algo")
def algo_config_to_class(algo_config):
    """
    Maps algo config to the DYNA algo class to instantiate, along with additional algo kwargs.

    Args:
        algo_config (Config instance): algo config

    Returns:
        algo_class: subclass of Algo
        algo_kwargs (dict): dictionary of additional kwargs to pass to algorithm
    """

    # note: we need the check below because some configs import DYNAConfig and exclude
    # some of these options

    return DYNA, {}

class DYNA(PolicyAlgo):
    """
    Normal Dynamics training.
    """
    def __init__(
        self,
        algo_config,
        obs_config,
        global_config,
        obs_key_shapes,
        ac_dim,
        device
    ):
        """
        Args:
            algo_config (Config object): instance of Config corresponding to the algo section
                of the config

            obs_config (Config object): instance of Config corresponding to the observation
                section of the config

            global_config (Config object): global training config

            obs_key_shapes (OrderedDict): dictionary that maps observation keys to shapes

            ac_dim (int): dimension of action space

            device (torch.Device): where the algo should live (i.e. cpu, gpu)
        """
        self.optim_params = deepcopy(algo_config.optim_params)
        self.algo_config = algo_config
        self.obs_config = obs_config
        self.global_config = global_config

        self.ac_dim = ac_dim
        (_, self.next_obs_shape), = obs_key_shapes.items()
        
        self.device = device

        self.obs_key_shapes = obs_key_shapes
        # add ac_dim to input shape
        self.obs_key_shapes['actions'] = [self.ac_dim]

        self.nets = nn.ModuleDict()
        self._create_shapes(obs_config.modalities, obs_key_shapes)
        self._create_networks()
        self._create_optimizers()
        assert isinstance(self.nets, nn.ModuleDict)

    def _create_shapes(self, obs_keys, obs_key_shapes):
        """
        Create obs_shapes, goal_shapes, and subgoal_shapes dictionaries, to make it
        easy for this algorithm object to keep track of observation key shapes. Each dictionary
        maps observation key to shape.

        Args:
            obs_keys (dict): dict of required observation keys for this training run (usually
                specified by the obs config), e.g., {"obs": ["rgb", "proprio"], "goal": ["proprio"]}
            obs_key_shapes (dict): dict of observation key shapes, e.g., {"rgb": [3, 224, 224]}
        """
        # determine shapes
        self.obs_shapes = OrderedDict()
        self.goal_shapes = OrderedDict()
        self.subgoal_shapes = OrderedDict()

        # We check across all modality groups (obs, goal, subgoal), and see if the inputted observation key exists
        # across all modalitie specified in the config. If so, we store its corresponding shape internally
        for k in obs_key_shapes:
            if "obs" in self.obs_config.modalities and k in [obs_key for modality in self.obs_config.modalities.obs.values() for obs_key in modality]:
                self.obs_shapes[k] = obs_key_shapes[k]
            elif k == "actions":
                self.obs_shapes[k] = obs_key_shapes[k]

    def _create_networks(self):
        """
        Creates networks and places them into @self.nets.
        """
        self.nets = nn.ModuleDict()
        self.nets["policy"] = DynamicsNets.DynamicsNetwork(
            obs_shapes=self.obs_shapes,
            goal_shapes=self.goal_shapes,
            pred_obs_dim=self.next_obs_shape,
            mlp_layer_dims=self.algo_config.actor_layer_dims,
            encoder_kwargs=ObsUtils.obs_encoder_kwargs_from_config(self.obs_config.encoder),
        )
        self.nets = self.nets.float().to(self.device)

    def process_batch_for_training(self, batch):
        """
        Processes input batch from a data loader to filter out
        relevant information and prepare the batch for training.

        Args:
            batch (dict): dictionary with torch.Tensors sampled
                from a data loader

        Returns:
            input_batch (dict): processed and filtered batch that
                will be used for training 
        """
        input_batch = dict()
        input_batch["obs"] = {k: batch["obs"][k][:, 0, :] for k in batch["obs"]}
        input_batch["obs"]["actions"] = batch["actions"][:, 0, :]
        input_batch["goal_obs"] = batch.get("goal_obs", None) # goals may not be present
        input_batch["actions"] = batch["actions"][:, 0, :]
        input_batch["next_obs"] = {k: batch["next_obs"][k][:, 0, :] for k in batch["next_obs"]}
        return TensorUtils.to_device(TensorUtils.to_float(input_batch), self.device)

    def train_on_batch(self, batch, epoch, validate=False):
        """
        Training on a single batch of data.

        Args:
            batch (dict): dictionary with torch.Tensors sampled
                from a data loader and filtered by @process_batch_for_training

            epoch (int): epoch number - required by some Algos that need
                to perform staged training and early stopping

            validate (bool): if True, don't perform any learning updates.

        Returns:
            info (dict): dictionary of relevant inputs, outputs, and losses
                that might be relevant for logging
        """
        with TorchUtils.maybe_no_grad(no_grad=validate):
            info = super(DYNA, self).train_on_batch(batch, epoch, validate=validate)
            predictions = self._forward_training(batch)
            losses = self._compute_losses(predictions, batch)

            info["predictions"] = TensorUtils.detach(predictions)
            info["losses"] = TensorUtils.detach(losses)

            if not validate:
                step_info = self._train_step(losses)
                info.update(step_info)

        return info

    def _forward_training(self, batch):
        """
        Internal helper function for DYNA algo class. Compute forward pass
        and return network outputs in @predictions dict.

        Args:
            batch (dict): dictionary with torch.Tensors sampled
                from a data loader and filtered by @process_batch_for_training

        Returns:
            predictions (dict): dictionary containing network outputs
        """
        predictions = OrderedDict()
        pred_obs = self.nets["policy"](obs_dict=batch["obs"], goal_dict=batch["goal_obs"])
        predictions["pred_obs"] = pred_obs
        return predictions

    def _compute_losses(self, predictions, batch):
        """
        Internal helper function for DYNA algo class. Compute losses based on
        network outputs in @predictions dict, using reference labels in @batch.

        Args:
            predictions (dict): dictionary containing network outputs, from @_forward_training
            batch (dict): dictionary with torch.Tensors sampled
                from a data loader and filtered by @process_batch_for_training

        Returns:
            losses (dict): dictionary of losses computed over the batch
        """
        losses = OrderedDict()
        obs_target = list(batch["next_obs"].values())[0]
        pred_obs =  predictions["pred_obs"]
        losses["l2_loss"] = nn.MSELoss()(pred_obs, obs_target)
        losses["l1_loss"] = nn.SmoothL1Loss()(pred_obs, obs_target)
        # cosine direction loss on eef delta position
        losses["cos_loss"] = LossUtils.cosine_loss(pred_obs[..., :3], obs_target[..., :3])

        action_losses = [
            self.algo_config.loss.l2_weight * losses["l2_loss"],
            self.algo_config.loss.l1_weight * losses["l1_loss"],
            self.algo_config.loss.cos_weight * losses["cos_loss"],
        ]
        action_loss = sum(action_losses)
        losses["action_loss"] = action_loss
        return losses

    def _train_step(self, losses):
        """
        Internal helper function for DYNA algo class. Perform backpropagation on the
        loss tensors in @losses to update networks.

        Args:
            losses (dict): dictionary of losses computed over the batch, from @_compute_losses
        """

        # gradient step
        info = OrderedDict()
        policy_grad_norms = TorchUtils.backprop_for_loss(
            net=self.nets["policy"],
            optim=self.optimizers["policy"],
            loss=losses["action_loss"],
        )
        info["policy_grad_norms"] = policy_grad_norms
        return info

    def log_info(self, info):
        """
        Process info dictionary from @train_on_batch to summarize
        information to pass to tensorboard for logging.

        Args:
            info (dict): dictionary of info

        Returns:
            loss_log (dict): name -> summary statistic
        """
        log = super(DYNA, self).log_info(info)
        log["Loss"] = info["losses"]["action_loss"].item()
        if "l2_loss" in info["losses"]:
            log["L2_Loss"] = info["losses"]["l2_loss"].item()
        if "l1_loss" in info["losses"]:
            log["L1_Loss"] = info["losses"]["l1_loss"].item()
        if "cos_loss" in info["losses"]:
            log["Cosine_Loss"] = info["losses"]["cos_loss"].item()
        if "policy_grad_norms" in info:
            log["Policy_Grad_Norms"] = info["policy_grad_norms"]
        return log

    def get_action(self, obs_dict, goal_dict=None):
        """
        Get policy action outputs.

        Args:
            obs_dict (dict): current observation
            goal_dict (dict): (optional) goal

        Returns:
            action (torch.Tensor): action tensor
        """
        assert not self.nets.training
        return self.nets["policy"](obs_dict, goal_dict=goal_dict)

