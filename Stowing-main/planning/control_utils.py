import numpy as np
import torch

from datetime import datetime
from utils.data_utils import *
from pytorch3d.transforms import *
from transforms3d.quaternions import *
from utils.utils3d import *
import utils.robot_utils as robot_utils


def normalize_state(args, state_cur, state_goal, pkg='numpy'):
    if len(state_cur.shape) == 2:
        dim = 0 
    elif len(state_cur.shape) == 3:
        dim = 1
    else:
        raise NotImplementedError

    if pkg == 'numpy':
        mean_p = np.mean(state_cur, axis=dim)
        std_p = np.std(state_cur, axis=dim)
        state_cur_norm = state_cur - mean_p
        
        mean_goal = np.mean(state_goal, axis=dim)
        std_goal = np.std(state_goal, axis=dim)
        # if 'alphabet' in args.target_shape_name:    
        #     # state_goal_norm = (state_goal - mean_goal) / std_goal
        #     if dim == 0:
        #         std_goal_new = np.concatenate((std_p[:2], std_goal[2:]))
        #     else:
        #         std_goal_new = np.concatenate((std_p[:, :2], std_goal[:, 2:]), axis=1)
        
        #     state_goal_norm = (state_goal - mean_goal) / std_goal_new
        # else:
        state_goal_norm = state_goal - mean_goal
    else:
        mean_p = torch.mean(state_cur, dim=dim)
        std_p = torch.std(state_cur, dim=dim)
        state_cur_norm = state_cur - mean_p
        
        mean_goal = torch.mean(state_goal, dim=dim)
        std_goal = torch.std(state_goal, dim=dim)
        # if 'alphabet' in args.target_shape_name:    
        #     # state_goal_norm = (state_goal - mean_goal) / std_goal
        #     if dim == 0:
        #         std_goal_new = torch.cat((std_p[:2], std_goal[2:]))
        #     else:
        #         std_goal_new = torch.cat((std_p[:, :2], std_goal[:, 2:]), dim=1)
        
        #     state_goal_norm = (state_goal - mean_goal) / std_goal_new
        # else:
        state_goal_norm = state_goal - mean_goal

    return state_cur_norm, state_goal_norm


def get_param_bounds(skill_dim):
    param_bounds = np.repeat(np.array([-1, 1])[np.newaxis, :], skill_dim, axis=0)

    # r = min(max_bounds[:2] - min_bounds[:2]) / 2
    # param_bounds.append([-0.01, 0.04]) # r
    # param_bounds.append(tool_params["theta_range"])
    # param_bounds.append(tool_params["phi1_range"])    
    # param_bounds.append(tool_params["phi2_range"])
    # param_bounds.append([tool_params["grip_min"], min(2*r, 0.08)])

    return torch.FloatTensor(param_bounds)


def param_seqs_to_init_poses_act(args, param_seqs, keyposes, obs):
    
    B = param_seqs.shape[0] * param_seqs.shape[1]
    pose_seq_all = []
    for poses in keyposes: # TODO: this can be vectorized
        ee_pos_B = poses['pos'].view(-1, 3)
        ee_quat_B = poses['quat'].view(-1, 4)
        pose_seq = []
        for i in range(B):
            ee_pos = ee_pos_B[i]
            ee_quat = ee_quat_B[i]
            gripper_width = torch.tensor(robot_utils.get_gripper_width(args, obs)).to(param_seqs.device).float()
            tool_repr = create_gripper_pcd(args, ee_pos, ee_quat, gripper_width=gripper_width, n_particles=args.n_particles_per_object['gripper'])
            if args.evenly_spaced:
                tool_repr_tensor = tool_repr if isinstance(tool_repr, torch.Tensor) else torch.from_numpy(tool_repr).float()
                # tool_repr_tensor.requires_grad = param_seqs.requires_grad
                pose_seq.append(tool_repr_tensor)
            else:
                pose_seq_tensor = torch.from_numpy(np.asarray(tool_repr.points).float())
                # pose_seq_tensor.requires_grad = param_seqs.requires_grad
                pose_seq.append(pose_seq_tensor)
        pose_seq = torch.stack(pose_seq).view(param_seqs.shape[0], param_seqs.shape[1], -1, 3)
        pose_seq_all.append(pose_seq)

    init_pose_seqs = pose_seq_all[0]

    act_seq_list = []
    # Iterate over each state in the state sequence
    for i in range(1, len(pose_seq_all)):
        action = []
        tool_idx = 0
        # Iterate over each tool in the state
        for j in range(len(args.tool_dim)):
            tool_dim = args.tool_dim[j]
            state_diff = pose_seq_all[i][...,tool_idx,:] - pose_seq_all[i-1][...,tool_idx,:] # TODO: check whether to use first or use average
            action.append(torch.nn.functional.pad(state_diff, (0, 3)))
            tool_idx += tool_dim

        act_seq_list.append(torch.cat(action, dim=action[0].dim()-1))

    act_seqs = torch.stack(act_seq_list, dim=1).view(param_seqs.shape[0], param_seqs.shape[1], -1, 12)


    return init_pose_seqs, act_seqs

def param_seqs_to_act_seqs(args, param_seqs, keyposes, obs):
    
    B = param_seqs.shape[0] * param_seqs.shape[1]
    pose_seq_all = []
    for poses in keyposes: # TODO: this can be vectorized
        ee_pos_B = poses['pos'].view(-1, 3)
        ee_quat_B = poses['quat'].view(-1, 4)
        pose_seq = []
        for i in range(B):
            ee_pos = ee_pos_B[i]
            ee_quat = ee_quat_B[i]
            gripper_width = torch.tensor(robot_utils.get_gripper_width(args, obs)).to(param_seqs.device).float()
            tool_repr = create_gripper_pcd(args, ee_pos, ee_quat, gripper_width=gripper_width, n_particles=args.n_particles_per_object['gripper'])
            if args.evenly_spaced:
                tool_repr_tensor = tool_repr if isinstance(tool_repr, torch.Tensor) else torch.from_numpy(tool_repr).float()
                # tool_repr_tensor.requires_grad = param_seqs.requires_grad
                pose_seq.append(tool_repr_tensor)
            else:
                pose_seq_tensor = torch.from_numpy(np.asarray(tool_repr.points).float())
                # pose_seq_tensor.requires_grad = param_seqs.requires_grad
                pose_seq.append(pose_seq_tensor)
        pose_seq = torch.stack(pose_seq).view(param_seqs.shape[0], param_seqs.shape[1], -1, 3)
        pose_seq_all.append(pose_seq.squeeze(1))

    return torch.stack(pose_seq_all, dim=1)

def keypose_to_tool_repr(args, poses, obs):
    ee_pos = poses['pos']
    ee_quat = poses['quat']
    gripper_width = np.array(robot_utils.get_gripper_width(args, obs))
    tool_repr = create_gripper_pcd(args, ee_pos, ee_quat, gripper_width=gripper_width, n_particles=args.n_particles_per_object['gripper'])
    return tool_repr if  args.evenly_spaced else np.asarray(tool_repr.points)

    
def params_to_init_pose(args, center, plan_params, param_seq, is_3d=False):
    ee_fingertip_T_mat = torch.FloatTensor(args.ee_fingertip_T_mat)

    init_pose_seq = []
    for params in param_seq:
        r, theta, phi1, phi2, grip_width = params

        center_x, center_y, center_z = center
        if is_3d:
            pos_x = center_x + (-r * torch.sin(phi1) + args.tool_center_z * torch.sin(phi2)) * 0.0
            pos_y = center_y + (-r * torch.sin(phi1) + args.tool_center_z * torch.sin(phi2)) * -1.0
            pos_z = center_z + r * torch.cos(phi1) + args.tool_center_z * torch.cos(phi2)

            ee_quat = qmult([0, 1, 0, 0], qmult(axangle2quat([0, 0, 1], 0 - np.pi / 4), 
                axangle2quat([1, 1, 0], phi2)))
        else:
            pos_x = center_x + (r * torch.sin(phi1) + args.tool_center_z * 0.0) * torch.sin(theta)
            pos_y = center_y + (r * torch.sin(phi1) + args.tool_center_z * 0.0) * torch.cos(theta)
            pos_z = center_z + r * torch.cos(phi1) + args.tool_center_z * 1.0

            ee_quat = qmult([0, 1, 0, 0], axangle2quat([0, 0, 1], theta - np.pi / 4))

        fingermid_pos = torch.FloatTensor([pos_x, pos_y, pos_z])
        ee_rot = quaternion_to_matrix(torch.FloatTensor(ee_quat))
        fingertip_mat = ee_rot @ ee_fingertip_T_mat[:3, :3]

        fingertip_T_list = []
        for k in range(len(args.tool_dim[args.env])):
            offset = torch.FloatTensor([0, (2 * k - 1) * (plan_params["init_grip"] / 2), 0])
            fingertip_pos = fingertip_mat @ offset + fingermid_pos
            fingertip_T = torch.cat((torch.cat((fingertip_mat, fingertip_pos.unsqueeze(1)), dim=1), 
                torch.FloatTensor([[0, 0, 0, 1]])), dim=0)
            fingertip_T_list.append(fingertip_T)

        fingertip_T_batch = torch.stack(fingertip_T_list)
        # print(fingertip_T_batch)
        tool_repr = get_tool_repr(args, fingertip_T_batch, pkg='torch')
        init_pose_seq.append(tool_repr)

    return torch.stack(init_pose_seq)


def param_seqs_to_actions(tool_params, param_seqs, keyposes, obs, step=1, is_3d=False):
    device = param_seqs.device
    B = param_seqs.shape[0] * param_seqs.shape[1]
    ps = param_seqs.view(B, -1)

    zero_pad = torch.zeros((B, 3), dtype=torch.float32, device=device)
    act_seq_list = []
    grip_rate = ((tool_params['init_grip'] - ps[:, -1]) / 2) / (tool_params['act_len'] / step)
    for _ in range(0, tool_params['act_len'], step):
        if is_3d:
            x = grip_rate * np.sin(np.pi / 2) 
            y = grip_rate * np.cos(np.pi / 2)
        else:
            x = grip_rate * torch.cos(ps[:, 1])
            y = -grip_rate * torch.sin(ps[:, 1])
        gripper_l_act = torch.stack((x, y, torch.zeros(B, dtype=torch.float32, device=device)), dim=-1)
        gripper_r_act = 0 - gripper_l_act
        act = torch.cat((gripper_l_act, zero_pad, gripper_r_act, zero_pad), dim=-1)
        act_seq_list.append(act)

    act_seqs = torch.stack(act_seq_list, dim=1).view(param_seqs.shape[0], param_seqs.shape[1], -1, 12)

    return act_seqs


def params_to_actions(tool_params, param_seq, step=1, is_3d=False):
    zero_pad = torch.zeros(3)
    act_seq = []
    for params in param_seq:
        actions = []
        r, theta, phi1, phi2, grip_width = params
        grip_rate = ((tool_params['init_grip'] - grip_width) / 2) / (tool_params['act_len'] / step)
        for _ in range(0, tool_params['act_len'], step):
            if is_3d:
                x = grip_rate * np.sin(np.pi / 2) 
                y = grip_rate * np.cos(np.pi / 2)
            else:
                x = grip_rate * torch.cos(theta)
                y = -grip_rate * torch.sin(theta)
            gripper_l_act = torch.cat([x.unsqueeze(0), y.unsqueeze(0), torch.zeros(1)])
            gripper_r_act = 0 - gripper_l_act
            act = torch.cat((gripper_l_act, zero_pad, gripper_r_act, zero_pad))
            actions.append(act)

        act_seq.append(torch.stack(actions))

    return torch.stack(act_seq)


def init_pose_to_params(init_pose_seq):
    # import pdb; pdb.set_trace()
    if not torch.is_tensor(init_pose_seq):
        init_pose_seq = torch.FloatTensor(init_pose_seq)

    mid_point = (init_pose_seq.shape[1] - 1) // 2
    mid_point_seq = (init_pose_seq[:, mid_point, :3] + init_pose_seq[:, mid_point, 7:10]) / 2

    rot_seq = torch.atan2(init_pose_seq[:, mid_point, 2] - mid_point_seq[:, 2], \
        init_pose_seq[:, mid_point, 0] - mid_point_seq[:, 0])

    a = init_pose_seq[:, 0, :3] - init_pose_seq[:, -1, :3]
    b = torch.FloatTensor([[0.0, 1.0, 0.0]]).expand(init_pose_seq.shape[0], -1)
    z_angle_seq = torch.acos((a * b).sum(dim=1) / (a.pow(2).sum(dim=1).pow(0.5) * b.pow(2).sum(dim=1).pow(0.5)))

    pi = torch.full(rot_seq.shape, np.pi)
    rot_seq = pi - rot_seq
    z_angle_seq = pi - z_angle_seq
    
    return mid_point_seq, rot_seq, z_angle_seq
