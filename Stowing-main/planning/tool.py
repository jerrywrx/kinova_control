import copy
import numpy as np
import torch

from model_based_planner import ModelBasedPlanner
from timeit import default_timer as timer
from config.config import update_dy_args
from utils.visualize import *


class Tool(object):
    def __init__(self, args, skill_name, planner_type, tool_model_path_dict=None, env_config=None):
        self.name = skill_name
        self.planner_type = planner_type

        if 'gnn' in planner_type:
            tool_args = copy.deepcopy(args)
            tool_args_dict = np.load(f'{tool_model_path_dict[skill_name]}/args.npy', allow_pickle=True).item()
            tool_args = update_dy_args(tool_args, tool_args_dict)
            tool_args.env = skill_name
            self.planner = ModelBasedPlanner(tool_args,skill_name, model_path=f'{tool_model_path_dict[skill_name]}/net_best.pth', env_config=env_config)
        elif 'sim' in planner_type:
            tool_args = copy.deepcopy(args)
            tool_args.env = skill_name
            self.planner = ModelBasedPlanner(tool_args)
        else:
            raise NotImplementedError


    def rollout(self, state_cur_dict, target_shape, target_dir, rollout_path, max_n_actions, rs_loss_threshold=float('inf')):
        if 'sim' in self.planner_type:
            state_cur = state_cur_dict['dense']
        else:
            state_cur = state_cur_dict['tensor']
        obs = state_cur_dict['obs']
        attr_cur = state_cur_dict['attr']
        # state_demo = np.stack(state_demo_dict['sparse'])
        
        with torch.no_grad():
            start = timer()
            param_seq = self.planner.plan(state_cur, attr_cur, obs, target_shape, target_dir, rollout_path, max_n_actions, rs_loss_threshold=rs_loss_threshold)
            end = timer()
            print(f"{self.name.upper()}: \n{param_seq}")
            print(f"PLANNING TIME: {end - start}")
            
            # Write the planning time to a file
            time_path = os.path.join(rollout_path, 'planning_time.txt')
            if os.path.exists(time_path):
                with open(time_path, 'r') as f:
                    planning_time = float(f.read())
            else:
                planning_time = 0.0

            with open(time_path, 'w') as f:
                f.write(str(planning_time + end - start))
                
            # Evaluate the plan and get the resulting state sequence and information dictionary
            # We are trying to evaluate the final state of the plan
            state_seq, info_dict = self.planner.eval_soln(param_seq, state_cur, attr_cur, obs, target_shape, target_dir)
            print(f"{self.name.upper()} LOSS: {info_dict['loss'][-1]} ")

        return param_seq, state_seq, info_dict
