import copy
import itertools
import numpy as np
import os
import pickle
import subprocess
import sys
import torch
import torch.nn.functional as F
import yaml

torch.set_printoptions(sci_mode=False)

from control_utils import *
from datetime import datetime
from dynamics.model import ChamferLoss, EarthMoverLoss, HausdorffLoss
from dynamics.gnn import GNN
from perception.pcd_utils import *
from tqdm import tqdm
from utils.data_utils import *
# from utils.Density_aware_Chamfer_Distance.utils_v2.model_utils import calc_dcd
from utils.loss import *
from utils.visualize import *
from skills.skill_controller import SkillController


class ModelBasedPlanner(object):
    def __init__(self, args, skill_name, model_path=None, env_config=None):
        self.args = args
        self.skill_name = skill_name
        self.skill_controller = SkillController(args, args.skill_config)
        
        self.env_config = env_config
        self.chamfer_loss = ChamferLoss(args.loss_ord)
        self.emd_loss = EarthMoverLoss(args.loss_ord)
        self.h_loss = HausdorffLoss(args.loss_ord)

        if 'sim' in args.planner_type:
            from dynamics.sim import MPM
            self.model = MPM(args)
        else:
            self.model = GNN(args, model_path)
        self.act_len = 15

        self.device = args.device
        self.batch_size = args.control_batch_size
        
        self.param_dim = SkillController.SKILL_MAPS[self.skill_name].get_param_dim()
        # self.RS_sample_size = args.RS_sample_size
        self.RS_elite_size_per_act = args.RS_elite_size_per_act
        self.CEM_sample_size = args.CEM_sample_size
        self.CEM_elite_size = args.CEM_elite_size
        self.CEM_sample_iter = args.CEM_sample_iter
        self.CEM_decay_factor = args.CEM_decay_factor

        self.CEM_param_var = [1e-1] * self.param_dim
        self.sample_ratio = {1: (3, 3, 3, 3, 3)}
        
        # The MPPI_temperature parameter controls the sharpness of the weighting distribution. 
        # A smaller temperature gives higher weight to the lower cost sequences.
        self.MPPI_temperature = args.MPPI_temperature
        self.MPPI_sample_size = args.MPPI_sample_size
        self.num_MPC_steps = args.num_MPPI_steps
        self.MPPI_noise_std = args.MPPI_noise_std

        if args.debug:
            # self.RS_sample_size = 8
            self.batch_size = 1
            self.RS_elite_size_per_act = 1
            self.CEM_sample_size = 8
            self.CEM_elite_size = 2
            self.CEM_sample_iter = 2
            self.CEM_decay_factor = 0.5
            self.sample_ratio = {1: (2, 1, 1, 1, 1)}


    def plan(self, state_cur, attr_cur, obs, target_shape, target_dir, rollout_path, max_n_actions, rs_loss_threshold=float('inf')):
        self.rollout_path = rollout_path
        self.center = torch.mean(state_cur.squeeze(), dim=0).cpu()

        if self.args.debug: max_n_actions = 1

        
        self.skill_controller.reset_to_skill(self.skill_name)

        self.skill_dim = self.skill_controller._cur_skill.get_param_dim()

        self.param_bounds = get_param_bounds(self.skill_dim)
        
        # Perform random search
        param_seq_pool, loss_seq_pool, state_seq_pool= self.random_search(
            state_cur, attr_cur, target_shape, target_dir, max_n_actions, obs)

        # If loss is too high, return the first sequence in the pool
        if loss_seq_pool[0] > rs_loss_threshold:
            return param_seq_pool[0]

        # Choose optimization algorithm and optimize the parameter sequence
        if self.args.optim_algo == 'RS':
            sort_ind = torch.argsort(loss_seq_pool)
            # print(f"Selected idx: {sort_ind[0]} with loss {loss_seq_pool[sort_ind[0]]}")
            param_seq_opt = param_seq_pool[sort_ind[0]]
            # action_seq_opt = action_seq_pool[sort_ind[0]]

        elif self.args.optim_algo == 'CEM':
            param_seq_CEM, loss_seq_CEM = self.optimize_CEM(
                (param_seq_pool, loss_seq_pool, state_seq_pool), state_cur, attr_cur, target_shape, obs, target_dir)
            sort_ind = torch.argsort(loss_seq_CEM)
            # print(f"Selected idx: {sort_ind[0]} with loss {loss_seq_CEM[sort_ind[0]]}")
            param_seq_opt = param_seq_CEM[sort_ind[0]]

        elif self.args.optim_algo == "GD":
            with torch.set_grad_enabled(True):
                param_seq_opt = self.optimize_GD(param_seq_pool, state_cur, attr_cur, target_shape, obs)

        elif self.args.optim_algo == "MPPI":
            param_seq_MPPI, loss_seq_MPPI = self.optimize_MPPI(
                (param_seq_pool, loss_seq_pool, state_seq_pool), state_cur, attr_cur, target_shape, obs, target_dir)
            sort_ind = torch.argsort(loss_seq_MPPI)
            param_seq_opt = param_seq_MPPI[sort_ind[0]]

        elif self.args.optim_algo == "CEM_BEFORE_GD":
            param_seq_CEM, loss_seq_CEM, = self.optimize_CEM(
                (param_seq_pool, loss_seq_pool, state_seq_pool), state_cur, target_shape)
            with torch.set_grad_enabled(True):
                param_seq_opt = self.optimize_GD(param_seq_pool, state_cur, attr_cur, target_shape, obs)

        elif self.args.optim_algo == "CEM_GUIDED_GD":
            param_seq_CEM, loss_seq_CEM, = self.optimize_CEM(
                (param_seq_pool, loss_seq_pool, state_seq_pool), state_cur, target_shape)
            with torch.set_grad_enabled(True):
                param_seq_opt = self.optimize_GD(param_seq_pool, state_cur, attr_cur, target_shape, obs, guide=param_seq_CEM)

        else:
            raise NotImplementedError

        return param_seq_opt


    def rollout(self, param_seqs, state_cur, attr_cur, target_shape, obs):
        '''
        args:
            param_seqs: (N, T, skill_dim)
            state_cur: (1, N, state_dim)
            attr_cur: (N, attr_dim)
            target_shape: (N, state_dim)
        '''
        B = self.batch_size
        
        # init_pose_seqs is the initial pose of the gripper
        # act_seqs is the movement of the gripper
        self.skill_controller._cur_skill.update_param(param_seqs, obs, self.env_config)
        keyposes = self.skill_controller._cur_skill.get_keyposes()

        action_his = param_seqs_to_act_seqs(self.args, param_seqs, keyposes, obs)

        n_batch = int(np.ceil(param_seqs.shape[0] / B))
        state_seqs = []
        loss_seqs = []
        batch_iterator = tqdm(range(n_batch)) if n_batch > 2 else range(n_batch)
        for i in batch_iterator:
            start = B * i
            end = min(B * (i + 1), param_seqs.shape[0])
            state_seq, _, _, pred_transformations  = self.model.rollout(state_cur.squeeze(0), action_his[start: end], attr_cur)
            # state_seq is of shape [batch, n_step, n_particles_object, state_dim=3]
            loss_seq = self.evaluate_traj(state_seq, target_shape, self.args.control_loss_type)
            state_seqs.append(state_seq)
            loss_seqs.append(loss_seq)
            
        # Concatenate loss sequences and state sequences along batch dimension
        return torch.cat(loss_seqs, dim=0), torch.cat(state_seqs, dim=0)


    def evaluate_traj(
        self,
        state_seqs,     # [n_sample, n_steps, n_particles, state_dim]
        target_shape,
        loss_type
    ):
        '''
        args:
            state_seqs: (n_sample, n_steps, n_particles, state_dim)
        '''
        loss_seqs = []
        for i in range(state_seqs.shape[0]):
            n_actions = int(np.ceil(state_seqs.shape[1] / self.act_len))
            loss_seq = []
            for j in range(n_actions):
                state_idx = min(state_seqs.shape[1] - 1, (j + 1) * self.act_len - 1)
                if 'sim' in self.args.planner_type:
                    dense_pcd = o3d.geometry.PointCloud()
                    dense_pcd.points = o3d.utility.Vector3dVector(state_seqs[i, state_idx].cpu().numpy())

                    surf_mesh = alpha_shape_mesh_reconstruct(dense_pcd, alpha=0.02, visualize=False)
                    surf_pcd = o3d.geometry.TriangleMesh.sample_points_poisson_disk(surf_mesh, self.args.n_particles)
                    # visualize_o3d([surf_mesh], title='surf_pcd')

                    state_cur = torch.tensor(np.asarray(surf_pcd.points), device=self.args.device, dtype=torch.float32).unsqueeze(0)
                else:
                    state_cur = state_seqs[i, state_idx].unsqueeze(0)

                state_goal = torch.tensor(target_shape['sparse'], device=self.args.device, dtype=torch.float32).unsqueeze(0)

                state_cur_norm, state_goal_norm = normalize_state(self.args, state_cur[:, :self.args.n_particles], state_goal[:, :self.args.n_particles], pkg='torch')

                if loss_type == "emd":
                    loss = self.emd_loss(state_cur_norm, state_goal_norm)
                elif loss_type == "chamfer":
                    loss = self.chamfer_loss(state_cur_norm, state_goal_norm)
                elif loss_type == "chamfer_emd":
                    loss = 0
                    loss += self.args.emd_weight * self.emd_loss(state_cur_norm, state_goal_norm)
                    loss += self.args.chamfer_weight * self.chamfer_loss(state_cur_norm, state_goal_norm)
                elif loss_type == "L1_pos":
                    loss = F.l1_loss(state_cur_norm, state_goal_norm)
                elif loss_type == "mse":
                    loss = F.mse_loss(state_cur_norm, state_goal_norm)

                elif loss_type == "iou": # complement of IOU
                    state_cur_upsample = upsample(state_cur[0], visualize=False)
                    loss = 1 - iou(state_cur_upsample, target_shape['dense'], voxel_size=0.003, visualize=False)
                    loss = torch.tensor([loss], dtype=torch.float32)
                elif loss_type == "soft_iou":
                    loss = 1 - soft_iou(state_cur_norm, state_goal_norm, size=8, pkg='torch', soft=True)
                else:
                    raise NotImplementedError

                loss_seq.append(loss)

            loss_seqs.append(torch.stack(loss_seq))

        loss_seqs = torch.stack(loss_seqs)
        
        return loss_seqs


    def eval_soln(self, param_seq, state_cur, attr_cur, obs, target_shape, target_dir):
        # init_pose_seqs, act_seqs = param_seqs_to_init_poses_act(self.args, param_seq.unsqueeze(0), self.keyposes, obs)
        action_his = param_seqs_to_act_seqs(self.args, param_seq.unsqueeze(0), self.keyposes, obs)


        if 'sim' in self.args.planner_type:
            state_seq, _, _, pred_transformations = self.model.rollout(state_cur, action_his, param_seq[:, 1].unsqueeze(0), 
                os.path.join(self.rollout_path, 'anim'))
        else:
            # if len(state_cur.shape) == 3:
            #     state_cur = state_cur.squeeze(0)
            
            # state_cur.shape = [1, n_particles, state_dim]
            # state_seq.shape = [1, n_steps, n_particles, state_dim]
            state_seq, _, _, pred_transformations = self.model.rollout(state_cur, action_his, attr_cur)
            
        # state_seq_wshape = add_tool_positions(self.args, state_seq.squeeze(0).cpu().numpy(), 
        #     init_pose_seqs[0].cpu().numpy(), act_seqs[0].cpu().numpy())
        state_seq_wshape = np.concatenate([state_seq.squeeze(0).cpu().numpy(), action_his.squeeze(0)[1:].cpu().numpy()], axis=1)
        if self.args.animate_control:
            self.render_state(state_seq_wshape, target_shape)
        loss_gnn = self.evaluate_traj(state_seq, target_shape, self.args.control_loss_type)[0][-1].item()
        # chamfer_loss = self.evaluate_traj(state_seq, target_shape, 'chamfer')[0][-1].item()
        # emd_loss = self.evaluate_traj(state_seq, target_shape, 'emd')[0][-1].item()
        # iou_loss = self.evaluate_traj(state_seq, target_shape, 'iou')[0][-1].item()

        title = f'{self.args.env.upper()}: '
        for i in range(0, len(param_seq), 2):
            if i > 0: title += '\n'
            title += f'{[[round(x.item(), 3) for x in y] for y in param_seq[i:i+2]]}'
        title += f' -> {round(loss_gnn, 4)}'

        pkl_path = os.path.join(self.rollout_path, 'anim_args', 
            f'{self.args.env}_anim_{datetime.now().strftime("%b-%d-%H:%M:%S")}_args.pkl')
        
        state_cur_np = state_cur.cpu().numpy()
        target_state = target_shape['sparse'][None,:] # np.concatenate((state_demo, target_shape['sparse'][None,:]), axis=0)
        state_pred_seq = np.concatenate((state_cur_np, state_seq_wshape), axis=0)
        with open(pkl_path, 'wb') as f:
            pickle.dump({'args': self.args, 'col_titles': ['Ground Truth', title], 'state_seqs': [target_state, state_pred_seq], 'data_path':target_dir, 
                         'img_st_idx':target_shape['subgoal_st_idx'], 'only_goal_img':True}, f)
        p = subprocess.Popen(['python', 'utils/visualize.py', pkl_path], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

        # render_anim(self.args, [title], [state_seq_wshape], target=target_shape['sparse'], 
        #     attn_mask_pred=[attn_mask_pred], rels_pred=[rels_pred],
        #     path=os.path.join(self.rollout_path, 'anim', f'{anim_name}.mp4'))

        with open(os.path.join(self.rollout_path, 'param_seqs', f'{self.args.env}_param_seq.yml'), 'w') as f:
            yaml.dump({self.args.env: param_seq.tolist()}, f, default_flow_style=True)

        info_dict = {
            'loss': [loss_gnn], 
            'subprocess': [p], 
        }

        return state_seq_wshape, info_dict


    def random_search(
        self,
        state_cur,
        attr_cur,
        target_shape,
        target_dir,
        max_n_actions,
        obs
    ):
        def grid_sample(n_actions):
            param_sample_list = [
                np.linspace(*self.param_bounds[i], 
                            num=self.sample_ratio[n_actions][i], 
                            endpoint=True)
                for i in range(self.param_bounds.shape[0])
            ]

            param_grid = np.meshgrid(*param_sample_list, indexing='ij')
            param_samples = np.column_stack([x.flat for x in param_grid])

            if n_actions == 1:
                return np.expand_dims(param_samples, axis=1)
            else:
                return np.array(list(itertools.product(param_samples, repeat=n_actions)))

        param_seqs = None
        for _ in range(max_n_actions):
            param_seq_sample = grid_sample(1)
            if param_seqs is None:
                param_seqs = param_seq_sample
            else:
                np.random.shuffle(param_seq_sample)
                param_seqs = np.concatenate((param_seqs, param_seq_sample), axis=1)

        # param_seqs: numpy before and torch after
        param_seqs = torch.tensor(param_seqs, dtype=torch.float32, device=self.device)
        
        self.skill_controller._cur_skill.update_param(param_seqs, obs, self.env_config)
        self.keyposes = self.skill_controller._cur_skill.get_keyposes()
        
        # loss_seq_list: [n_samples, max_n_actions]
        # state_seq_list: [n_samples, n_actions, N, state_dim]
        # act_seqs: [n_samples, n_actions, 1, act_dim]
        loss_seq_list, state_seq_list = self.rollout(param_seqs, state_cur, attr_cur, target_shape, obs)
        best_loss_ind = torch.argsort(loss_seq_list, dim=0)[:self.RS_elite_size_per_act]
        
        loss_opt, param_seq_opt, state_seq_opt, act_seq_opt = [], [], [], []
        for i in range(max_n_actions):
            indices = best_loss_ind[:, i]
            loss_opt.extend(loss_seq_list[indices, i])
            param_seq_opt.extend(param_seqs[indices, :i+1])
            state_seq_opt.extend(state_seq_list[indices, :(i+1)*self.act_len])
            # act_seq_opt.extend(act_seqs[indices, :(i+1)*self.act_len])
            
        loss_opt = torch.stack(loss_opt)
        # sort_ind = torch.argsort(loss_opt)
        # loss_opt = loss_opt[sort_ind[:max_n_actions]]
        # param_seq_opt = [param_seq_opt[x] for x in sort_ind[:max_n_actions]]
        # state_seq_opt = [state_seq_opt[x] for x in sort_ind[:max_n_actions]]
        # action_seq_opt = [act_seq_opt[x] for x in sort_ind[:max_n_actions]]
        if self.args.animate_control:
            self.render_selections('RS', obs, state_cur, param_seq_opt, loss_opt, state_seq_opt, target_shape, target_dir)
        return param_seq_opt, loss_opt, state_seq_opt


    def optimize_CEM(    
        self,
        soln_pool,
        state_cur,
        attr_cur, 
        target_shape, 
        obs,
        target_dir,
        first_iter_threshold=5e-4,
        plateau_threshold=3e-4,
        plateau_max_iter=5
    ):
        # https://github.com/martius-lab/iCEM

        param_seq_opt, loss_opt, state_seq_opt = [], [], []
        traj_loss_dict = {}
        for i, (param_seq_cur, loss_cur, state_seq_cur) in enumerate(zip(*soln_pool)):
            n_actions = param_seq_cur.shape[0]
            best_loss, best_param_seq, best_state_seq = loss_cur, param_seq_cur, state_seq_cur
            
            plateau_iter, traj_loss_list = 0, []
            param_seq_mean = param_seq_cur.cpu().numpy().flatten()
            param_seq_var = np.tile(self.CEM_param_var, n_actions)
            sample_size = self.CEM_sample_size * n_actions
            interrupt = ''
            for j in range(self.CEM_sample_iter):
                param_seq_samples = np.random.multivariate_normal(mean=param_seq_mean, 
                    cov=np.diag(param_seq_var), size=sample_size)

                param_seq_CEM = torch.tensor(param_seq_samples, dtype=torch.float32).view(sample_size, n_actions, -1)
                param_seq_CEM = torch.clamp(param_seq_CEM, min=self.param_bounds[:, 0], max=self.param_bounds[:, 1])
                loss_list, state_seq_list = self.rollout(param_seq_CEM.to(self.args.device), state_cur, attr_cur, target_shape, obs) 
                
                loss_list = loss_list[:, -1]

                sort_ind = torch.argsort(loss_list)
                traj_loss_list.append(loss_list[sort_ind[0]].item())

                param_seq_CEM = param_seq_CEM.reshape(sample_size, -1)
                param_seq_CEM_elite = param_seq_CEM[sort_ind[:self.CEM_elite_size].cpu()].numpy()
                param_seq_mean = np.mean(param_seq_CEM_elite, axis=0)
                param_seq_var = np.var(param_seq_CEM_elite, axis=0)

                sample_size = max(2 * self.CEM_elite_size, int(self.CEM_decay_factor * sample_size))

                # check the improvement of absolute scale
                if j == 0 and i != 0 and best_loss - loss_list[sort_ind[0]] < first_iter_threshold:
                    interrupt = f"---> Stop criterion 1: not the best RS Traj and < {first_iter_threshold} improvement in the first iteration!"

                if best_loss - loss_list[sort_ind[0]] < plateau_threshold:
                    plateau_iter += 1
                else:
                    plateau_iter = 0

                # break after not improving for 3 iterations
                if len(loss_opt) > 0 and best_loss > min(loss_opt):
                    if plateau_iter >= 0.5 * plateau_max_iter:
                        interrupt = f"---> Stop criterion 2.a: not currently the best traj and improving " + \
                            f"less than {plateau_threshold} for {int(np.ceil(0.5 * plateau_max_iter))} iterations!"
                else:
                    if plateau_iter >= plateau_max_iter:
                        interrupt = f"---> Stop criterion 2.b: currently the best traj and improving " + \
                            f"less than {plateau_threshold} for {plateau_max_iter} iterations!"

                if loss_list[sort_ind[0]] < best_loss:
                    best_loss = loss_list[sort_ind[0]]
                    best_state_seq = state_seq_list[sort_ind[0]]
                    best_param_seq = param_seq_CEM[sort_ind[0]]
                    
                if len(interrupt) > 0: break

            if len(interrupt) > 0: print(interrupt)
            
            print(f'From: {[round(x.item(), 3) for x in param_seq_cur.flatten()]} -> {round(loss_cur.item(), 4)}\n' + 
                f'To: {[round(x.item(), 3) for x in best_param_seq.flatten()]} -> {round(best_loss.item(), 4)}')
            
            param_seq_opt.append(best_param_seq.reshape(n_actions, -1).cpu())
            loss_opt.append(best_loss)
            state_seq_opt.append(best_state_seq)

            traj_loss_dict[f'Traj {i}'] = traj_loss_list

        loss_opt = torch.tensor(loss_opt, dtype=torch.float32)

        plot_eval_loss('Control CEM Loss', traj_loss_dict, 
            path=os.path.join(self.rollout_path, 'optim_plots', f"{self.args.env}_CEM_loss"))

        self.render_selections('CEM', obs, state_cur, param_seq_opt, loss_opt, state_seq_opt, target_shape, target_dir)

        return param_seq_opt, loss_opt

    def optimize_MPPI(self, soln_pool, state_cur, attr_cur, target_shape, obs, target_dir, num_MPC_steps=10):
        param_seq_opt, loss_opt, state_seq_opt = [], [], []
        traj_loss_dict = {}

        for i, (param_seq_cur, loss_cur, state_seq_cur) in enumerate(zip(*soln_pool)):
            n_actions = param_seq_cur.shape[0]
            best_loss, best_param_seq, best_state_seq = loss_cur, param_seq_cur, state_seq_cur

            # Initialize mean action sequence with the current solution
            mean_param_seq = param_seq_cur.clone()

            # MPC loop
            for mpc_step in range(self.num_MPC_steps):
                # Add exploration noise
                exploration_noise = torch.normal(mean=0., std=self.MPPI_noise_std, size=(self.MPPI_sample_size, n_actions, self.param_dim)).to(self.args.device)
                param_seq_samples = mean_param_seq + exploration_noise

                # Ensure the samples are within the action bounds
                param_seq_samples = torch.clamp(param_seq_samples, min=self.param_bounds[:, 0].to(self.args.device), max=self.param_bounds[:, 1].to(self.args.device))

                # Rollout with the sampled action sequences and calculate the cost for each sequence
                cost_list, state_seq_list = self.rollout(param_seq_samples, state_cur, attr_cur, target_shape, obs) 
                cost_list = cost_list[:, -1]

                # Check for NaNs or Infs in cost_list
                if torch.any(torch.isnan(cost_list)) or torch.any(torch.isinf(cost_list)):
                    raise ValueError("NaN or Inf found in cost_list during MPPI optimization.")
                
                # Calculate weights for each sampled sequence based on the costs
                weights = torch.exp(-1.0/self.MPPI_temperature * (cost_list - torch.min(cost_list)))
                weights = weights / torch.sum(weights)  # normalize the weights

                # Check for NaNs or Infs in weights
                if torch.any(torch.isnan(weights)) or torch.any(torch.isinf(weights)):
                    raise ValueError("NaN or Inf found in weights during MPPI optimization.")

                # Update the action sequence with the weighted average of the sampled sequences
                mean_param_seq = torch.sum(weights.view(-1, 1, 1) * param_seq_samples, dim=0)

            best_param_seq = mean_param_seq
            best_loss, best_state_seq = self.rollout(best_param_seq.unsqueeze(0).to(self.args.device), state_cur, attr_cur, target_shape, obs)

            param_seq_opt.append(best_param_seq.cpu())
            loss_opt.append(best_loss[0])
            state_seq_opt.append(best_state_seq.squeeze(0))

            traj_loss_dict[f'Traj {i}'] = [best_loss.item()]  # Save loss history for plotting

        loss_opt = torch.stack(loss_opt)

        plot_eval_loss('Control MPPI Loss', traj_loss_dict, 
            path=os.path.join(self.rollout_path, 'optim_plots', f"{self.args.env}_MPPI_loss"))

        self.render_selections('MPPI', obs, state_cur, param_seq_opt, loss_opt, state_seq_opt, target_shape, target_dir)

        return param_seq_opt, loss_opt
    def render_selections(self, name, obs, state_cur, param_seq_opt, loss_opt, state_seq_opt, target_shape, target_dir):
        # import pdb; pdb.set_trace()
        # init_pose_seqs, act_seqs = param_seqs_to_init_poses_act(self.args, torch.stack(param_seq_opt), self.keyposes, obs)
        self.skill_controller._cur_skill.update_param(torch.cat(param_seq_opt).unsqueeze(1).to(self.args.device), obs, self.env_config)
        keyposes = self.skill_controller._cur_skill.get_keyposes()

        action_his = param_seqs_to_act_seqs(self.args, torch.stack(param_seq_opt), keyposes, obs)

        state_cur_np = state_cur.cpu().numpy()

        col_titles = []
        state_seq_wshape_list = []
        col_titles.append(f'Target')
        state_seq_wshape_list.append(target_shape['sparse'][None,:]) # np.concatenate([state_demo, target_shape['sparse'][None,:]], axis=0)

        for i in range(len(param_seq_opt)):
            title = f'{name} {i}: ' 
            for j in range(0, param_seq_opt[i].shape[0], 2):
                if j > 0: title += '\n'
                title += f'{[[round(x.item(), 3) for x in y] for y in param_seq_opt[i][j:j+2]]}'
            title += f' -> {round(loss_opt[i].item(), 4)}'
            col_titles.append(title)
            
            # state_seq_wshape = add_tool_positions(self.args, state_seq_opt[i].cpu().numpy(), 
            #     init_pose_seqs[i].cpu().numpy(), act_seqs[i].cpu().numpy())
            state_seq_wshape = np.concatenate([state_seq_opt[i].cpu().numpy(), action_his[i,1:].cpu().numpy()], axis=1)
            state_seq_wshape_list.append(np.concatenate([state_cur_np, state_seq_wshape], axis=0))

        print(f"{name} best loss seqs: {loss_opt}")
        print(f"{name} best param seqs: {param_seq_opt}")
            
        pkl_path = os.path.join(self.rollout_path, 'anim_args', 
            f'{self.skill_name}_anim_{name}_{datetime.now().strftime("%b-%d-%H:%M:%S")}_args.pkl')
        with open(pkl_path, 'wb') as f:
            pickle.dump({'args': self.args, 'col_titles': col_titles, 'state_seqs': state_seq_wshape_list, 'data_path':target_dir, 
                         'img_st_idx':target_shape['subgoal_st_idx'], 'only_goal_img':True}, f)
        subprocess.Popen(['python', 'utils/visualize.py', pkl_path], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

        # render_anim(self.args, row_titles, state_seq_wshape_list, target=target_shape['sparse'], 
        #     path=os.path.join(self.rollout_path, 'anim', f'{self.args.env}_anim_{name}.mp4'))


    def render_state(self, state_cur, target_shape, state_pred=None, pred_err=0, frame_suffix='model'):
        # if 'surf' in self.args.tool_type:
        #     state_goal = torch.tensor(target_shape['surf'], device=self.args.device, dtype=torch.float32).unsqueeze(0)
        # else:
        
        # state_cur.shape = (n_steps, N, state_dim)
        state_cur = state_cur[-1][None,:]
        state_cur = torch.tensor(state_cur, device=self.args.device, dtype=torch.float32)
        state_goal = torch.tensor(target_shape['sparse'], device=self.args.device, dtype=torch.float32).unsqueeze(0)

        state_cur_norm, state_goal_norm = normalize_state(self.args, state_cur, state_goal, pkg='torch')

        if frame_suffix == 'before' and state_pred is not None:
            state_pred_norm = (state_pred - torch.mean(state_pred, dim=1)) / torch.std(state_pred, dim=1)
            render_frames(self.args, [f'State', f'State Pred={round(pred_err.item(), 6)}', 'State Normalized', 'State Pred Normalized'], 
                [state_cur.cpu(), state_pred.cpu(), state_cur_norm.cpu(), state_pred_norm.cpu()], 
                axis_off=False, focus=[True, True, True, True], 
                target=[state_goal.cpu()[0], state_goal.cpu()[0], state_goal_norm.cpu()[0], state_goal_norm.cpu()[0]], 
                path=os.path.join(self.rollout_path, 'states'), 
                name=f"{self.args.env}_state_{frame_suffix}.png")
        else:
            render_frames(self.args, [f'Target', 'State'], 
                [state_goal.cpu().numpy(), state_cur.cpu().numpy()], axis_off=False, focus=True, 
                path=os.path.join(self.rollout_path, 'states'), 
                name=f"{self.skill_name}_state_{frame_suffix}.png")


    def optimize_GD(
        self,
        param_seq_pool,
        state_cur,
        attr_cur, 
        target_shape,
        obs,
        guide=None
    ):
        from torchmin import minimize


        param_seq_opt = []
        loss_list_dict = {}
        loss_opt_min = float('inf')
        best_idx = 0
        for i in range(len(param_seq_pool)):
            print(f"Trajectory {i+1}/{len(param_seq_pool)}:")

            param_seq_cand = param_seq_pool[i].detach().requires_grad_().to(state_cur.device)

            # loss, state_seq= self.rollout(param_seq_cand.contiguous().unsqueeze(0), state_cur, attr_cur, target_shape, obs)
            # get_dot = register_hooks(loss)
            # loss.backward()
            # dot = get_dot()
            # dot.render('tmp')
            
            # Compute the loss
            # import torchviz
            # loss, _, _ = self.rollout(param_seq_cand.contiguous().unsqueeze(0), state_cur, attr_cur, target_shape, obs)
            # loss = loss[:, -1]
            # # Compute the gradients
            # loss.backward()

            # # Visualize the gradient graph
            # dot = torchviz.make_dot(loss, params=dict(param_seq_cand=param_seq_cand))
            # dot.view(filename=f"gradient_graph_traj_{i}")

            loss_list = []
            def loss_func(param_seq):
                nonlocal loss_list
                # print(f"Params: {param_seq_opt}")
                # param_seq_clamped = torch.clamp(param_seq, min=self.param_bounds[:, 0], max=self.param_bounds[:, 1])
                loss, _ = self.rollout(param_seq.unsqueeze(0), state_cur, attr_cur, target_shape, obs)


                if guide is not None:
                    loss += 0.2 * torch.linalg.norm(param_seq - guide[i])
                loss_list.append(loss[:, -1].item())
                return loss[:, -1]
            
            # def closure():
            #     nonlocal loss_list
            #     optimizer.zero_grad()
            #     loss = loss_func(param_seq_cand)
            #     loss.backward()
            #     loss_list.append(loss.item())
            #     return loss

            # from torch.optim import LBFGS
            # optimizer = LBFGS([param_seq_cand.contiguous()], lr=1e-2, line_search_fn='strong_wolfe', max_iter=100)
            # res = optimizer.step(closure)

            res = minimize(
                loss_func, param_seq_cand.contiguous(), 
                method='l-bfgs', 
                options=dict(lr=1e-1, line_search='strong-wolfe'),
                disp=2,
            )

            print(res)
            # param_seq_clamped = torch.clamp(
            #     res.x, min=self.param_bounds[:, 0], max=self.param_bounds[:, 1])
            param_seq_opt.append(res.x)
            loss_opt = res.fun

            loss_list_dict[f'Traj {i}'] = loss_list
            
            if loss_opt.item() < loss_opt_min:
                loss_opt_min = loss_opt.item()
                best_idx = i

        plot_eval_loss('Control GD Loss', loss_list_dict, 
            path=os.path.join(self.rollout_path, 'optim_plots', f"{self.args.env}_GD_loss"))

        return param_seq_opt[best_idx].cpu()

# from graphviz import Digraph
# import torch
# from torch.autograd import Variable, Function

# def iter_graph(root, callback):
#     queue = [root]
#     seen = set()
#     while queue:
#         fn = queue.pop()
#         if fn in seen:
#             continue
#         seen.add(fn)
#         for next_fn, _ in fn.next_functions:
#             if next_fn is not None:
#                 queue.append(next_fn)
#         callback(fn)

# def register_hooks(var):
#     fn_dict = {}
#     def hook_cb(fn):
#         def register_grad(grad_input, grad_output):
#             fn_dict[fn] = grad_input
#         fn.register_hook(register_grad)
#     iter_graph(var.grad_fn, hook_cb)

#     def is_bad_grad(grad_output):
#         if grad_output is None:
#                 return True
#         grad_output = grad_output.data
#         return grad_output.ne(grad_output).any() or grad_output.gt(1e6).any()

#     def make_dot():
#         node_attr = dict(style='filled',
#                         shape='box',
#                         align='left',
#                         fontsize='12',
#                         ranksep='0.1',
#                         height='0.2')
#         dot = Digraph(node_attr=node_attr, graph_attr=dict(size="12,12"))

#         def size_to_str(size):
#             return '('+(', ').join(map(str, size))+')'

#         def build_graph(fn):
#             if hasattr(fn, 'variable'):  # if GradAccumulator
#                 u = fn.variable
#                 node_name = 'Variable\n ' + size_to_str(u.size())
#                 dot.node(str(id(u)), node_name, fillcolor='lightblue')
#             else:
#                 grad_inputs = fn_dict.get(fn, None)  # Use the get method to handle missing keys
#                 if grad_inputs is not None:
#                     fillcolor = 'white'
#                     if any(is_bad_grad(gi) for gi in grad_inputs):
#                         fillcolor = 'red'
#                     dot.node(str(id(fn)), str(type(fn).__name__), fillcolor=fillcolor)
#             for next_fn, _ in fn.next_functions:
#                 if next_fn is not None:
#                     next_id = id(getattr(next_fn, 'variable', next_fn))
#                     dot.edge(str(next_id), str(id(fn)))
#         iter_graph(var.grad_fn, build_graph)

#         return dot

#     return make_dot
