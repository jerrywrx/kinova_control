
import glob
import numpy as np
import os
from collections import OrderedDict
import torch.nn.functional as F

import torch

from torch.utils.data import Dataset
from utils.data_utils import *
from perception.sample import pcd_dis_wall


class SkillClassificationDataset(Dataset):
    def __init__(self, args, phase):
        self.args = args
        self.phase = phase
        self.data_dir_dic = {}
        # self.stat_path = os.path.join('data/stats.h5')
        self.skill_dataset_len = {}
        self.state_data_dic = {}

        for skill_name in args.active_skill_list:
            dataset_len = 0
            self.data_dir = os.path.join(f'data/{args.data_type}/stow_{args.skill_name}', phase)
            self.data_dir_dic[skill_name] = self.data_dir

            vid_path_list = sorted(glob.glob(os.path.join(self.data_dir, '*')))
            if phase == 'train':
                vid_path_list = vid_path_list[:int(args.train_set_ratio * len(vid_path_list))]

            n_frames_min = float('inf')
            state_data_list = []
            # self.load_stats()
            for gt_vid_path in vid_path_list:
                frame_start = 0
                n_frames = len(glob.glob(os.path.join(gt_vid_path, '*.h5')))
                n_frames_min = min(n_frames, n_frames_min)
                gt_state_list = []
                
                for i in [0, n_frames - 1]:

                    frame_data = load_data(args.data_names, f"{gt_vid_path}/{i:03d}.h5")
                    obj_gt_state, obj_id, attr, pcd_dis_wall, _, tool_repr = frame_data[:6]
                    gt_state = torch.from_numpy(obj_gt_state[:self.args.n_particles+self.args.floor_dim] ).to(args.device).float()
                        
                    gt_state_list.append(gt_state)
                    # gt_action_list.append(torch.from_numpy(tool_repr).to(args.device).float())
                    
                    # gt_action_list.append(obj_gt_state)
                dataset_len += 1

                state_data_list.append(torch.cat(gt_state_list, dim=0))
                
            self.state_data_dic[skill_name] = state_data_list
            self.skill_dataset_len[skill_name] = dataset_len
        print(f"{phase} -> number of paired sequences: {self.skill_dataset_len}")


    def __len__(self):
        # Each data point consists of a sequence
        return sum(self.skill_dataset_len.values())

    # @profile
    def __getitem__(self, idx):
        idx_curr = idx
        for i, ((key, lst), length) in enumerate(zip(self.state_data_dic.items(), self.skill_dataset_len.values())):
            if idx_curr < length:
                return lst[idx_curr], i
            idx_curr -= length

