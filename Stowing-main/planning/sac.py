import glob
import numpy as np
# import open3d as o3d as o3d
import os
import pickle
import re
import readchar
import subprocess
import sys
import torch
import yaml

torch.set_printoptions(sci_mode=False)

from control_utils import *
from datetime import datetime
# from std_msgs.msg import UInt8
from tool import *
from config.config import gen_args
from utils.data_utils import *
from utils.loss import *
from utils.visualize import *


import robosuite as suite
from robosuite import load_controller_config
from robosuite.wrappers import DataCollectionWrapper, VisualizationWrapper
from robosuite.utils.input_utils import input2action
from config.config import gen_args

from robomimic.envs.env_robosuite import EnvRobosuite
from robosuite.wrappers import DataCollectionWrapper, VisualizationWrapper, KeyframeCollectionWrapper

from perception.sample import *
from skills.skill_controller import SkillController
from skills.skills import *

from control import get_test_name, create_env, get_state_from_env

import threading
import time
from scipy.spatial.transform import Rotation

from stable_baselines3 import SAC
from stable_baselines3.sac.policies import MlpPolicy
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from gym import spaces
from robosuite.wrappers import GymWrapper

from robosuite.wrappers import Wrapper
from gym.core import Env


class StowGymWrapper(Wrapper, Env):
    """
    Initializes the Gym wrapper. Mimics many of the required functionalities of the Wrapper class
    found in the gym.core module

    Args:
        env (MujocoEnv): The environment to wrap.
        keys (None or list of str): If provided, each observation will
            consist of concatenated keys from the wrapped environment's
            observation dictionary. Defaults to proprio-state and object-state.

    Raises:
        AssertionError: [Object observations must be enabled if no keys]
    """

    def __init__(self, env, static_pcd, env_config, args, keys=None ):
        # Run super method
        super().__init__(env=env)
        # Create name for gym
        robots = "".join([type(robot.robot_model).__name__ for robot in self.env.robots])
        self.name = robots + "_" + type(self.env).__name__

        # Get reward range
        self.reward_range = (0, self.env.reward_scale)

        if keys is None:
            keys = []
            # Add object obs if requested
            if self.env.use_object_obs:
                keys += ["object-state"]
            # Add image obs if requested
            # if self.env.use_camera_obs:
            #     keys += [f"{cam_name}_image" for cam_name in self.env.camera_names]
            # Iterate over all robots to add to state
            for idx in range(len(self.env.robots)):
                keys += ["robot{}_proprio-state".format(idx)]
        self.keys = keys

        # Gym specific attributes
        self.env.spec = None
        self.metadata = None

        # set up observation and action spaces
        obs = self.env.reset()
        self.modality_dims = {key: obs[key].shape for key in self.keys}
        flat_ob = self._flatten_obs(obs)
        self.obs_dim = flat_ob.size
        high = np.inf * np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low=low, high=high)
        low, high = self.env.action_spec
        self.action_space = spaces.Box(low=low, high=high)
        
        self.static_pcd = static_pcd
        self.env_config = env_config
        self.args = args
        self.max_steps = 1000  # Set the horizon length here
        self.current_step = 0  # Define the current step here

    def _flatten_obs(self, obs_dict, verbose=False):
        """
        Filters keys of interest out and concatenate the information.

        Args:
            obs_dict (OrderedDict): ordered dictionary of observations
            verbose (bool): Whether to print out to console as observation keys are processed

        Returns:
            np.array: observations flattened into a 1d array
        """
        ob_lst = []
        for key in self.keys:
            if key in obs_dict:
                if verbose:
                    print("adding key: {}".format(key))
                ob_lst.append(np.array(obs_dict[key]).flatten())
        return np.concatenate(ob_lst)

    def reset(self):
        """
        Extends env reset method to return flattened observation instead of normal OrderedDict.

        Returns:
            np.array: Flattened environment observation space after reset occurs
        """
        self.current_step = 0  # Reset the current step here

        ob_dict = self.env.reset()
        return self._flatten_obs(ob_dict)

    def step(self, action):
        """
        Extends vanilla step() function call to return flattened observation instead of normal OrderedDict.

        Args:
            action (np.array): Action to take in environment

        Returns:
            4-tuple:

                - (np.array) flattened observations from the environment
                - (float) reward from the environment
                - (bool) whether the current episode is completed or not
                - (dict) misc information
        """
        ob_dict, reward, done, info = self.env.step(action)

        reward, info = self.compute_reward(ob_dict, self.static_pcd, self.env_config)
        self.current_step += 1  # Increment the current step here

        done = False
        if self.current_step >= self.max_steps:
            done = True

        return self._flatten_obs(ob_dict), reward, done, info

    def seed(self, seed=None):
        """
        Utility function to set numpy seed

        Args:
            seed (None or int): If specified, numpy seed to set

        Raises:
            TypeError: [Seed must be integer]
        """
        # Seed the generator
        if seed is not None:
            try:
                np.random.seed(seed)
            except:
                TypeError("Seed must be an integer type!")

    def compute_reward(self, obs, static_pcd, env_config):
        """
        Dummy function to be compatible with gym interface that simply returns environment reward

        Args:
            achieved_goal: [NOT USED]
            desired_goal: [NOT USED]
            info: [NOT USED]

        Returns:
            float: environment reward
        """
        # Dummy args used to mimic Wrapper interface
        h5_files = sorted(glob.glob(os.path.join(self.args.target_ep_dir, '*.h5')))
        target_data = load_data(self.args.data_names, h5_files[-1])
        target_shape = target_data[0]
        state_cur_dict = get_state_from_env(self.args, obs, static_pcd, env_config)
        info = {'mse': mse(state_cur_dict['tensor'].cpu().numpy().squeeze(), target_shape.squeeze(), pkg='numpy'),
                'chamfer': chamfer(state_cur_dict['tensor'].cpu().numpy().squeeze(), target_shape.squeeze(), pkg='numpy'),
                'emd': emd(state_cur_dict['tensor'].cpu().numpy().squeeze(), target_shape.squeeze(), pkg='numpy'),}
        return -mse(state_cur_dict['tensor'].cpu().numpy().squeeze(), target_shape.squeeze(), pkg='numpy'), info



def main():
    args = gen_args()

    cd = os.path.dirname(os.path.realpath(sys.argv[0]))

    rollout_root = os.path.join(cd, '..', 'dump', 'rl', 
        args.target_shape_name)
    os.system('mkdir -p ' + rollout_root)

    for dir in ['states', 'raw_data']:
        os.system('mkdir -p ' + os.path.join(rollout_root, dir))
    sac_out = f'sac_{args.target_shape_name}'
    
    original_stdout = sys.stdout  # Save a reference to the original standard output
    
    log_file = open(rollout_root+f'{sac_out}.txt', 'w')
    sys.stdout = log_file

    args.target_ep_dir = os.path.join(args.target_path, args.target_shape_name)

    env, static_pcd, env_config = create_env(args)
    env = StowGymWrapper(env, static_pcd, env_config, args)

    num_layers = 2
    num_hidden_units = 256
    learning_rate = 0.001
    buffer_size = int(1e6)
    discount_factor = 0.99
    policy_kwargs = dict(activation_fn=torch.nn.ReLU,
                        net_arch=[num_hidden_units]*num_layers)

    model = SAC(MlpPolicy,
            env,
            verbose=1,
            policy_kwargs=policy_kwargs,
            learning_rate=learning_rate,
            buffer_size=buffer_size,
            gamma=discount_factor,
            )

    model.learn(total_timesteps=50000)
    model_name = rollout_root + sac_out
    model.save(model_name)
    # model = SAC.load(model_name, env=env_id)
    # Load trained model
    model = SAC.load(model_name, env=env)

    final_rewards = []
    num_episodes = 10
    infos = []
    for _ in range(num_episodes):
        obs = env.reset()
        done = False
        while not done:
            action, _states = model.predict(obs)
            obs, reward, done, info = env.step(action)
        infos.append(info)
    mean_mse = np.mean([info['mse'] for info in infos])
    mean_chamfer = np.mean([info['chamfer'] for info in infos])
    mean_emd = np.mean([info['emd'] for info in infos])
    
    print(f'Final mean mse: {mean_mse}, mean chamfer: {mean_chamfer}, mean emd: {mean_emd}')
    
    log_file.close()
    sys.stdout = original_stdout  # Restore standard output to its original value

if __name__ == '__main__':
    main()
