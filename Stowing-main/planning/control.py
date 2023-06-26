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

import argparse
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


import threading
import time
from scipy.spatial.transform import Rotation

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2, Session_pb2
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
import planning.kinova_utilities as kinova_utilities
from planning.kinova_gen3 import move_to_desired_pose, get_feedback, example_move_to_home_position
from realsense_video import VideoRecorder

command_feedback = 0
def command_fb_callback(msg):
    global command_feedback
    if msg.data > 0:
        command_feedback = msg.data


def get_test_name(args):
    test_name = ['control']
    if args.close_loop:
        test_name.append('close')
    else:
        test_name.append('open')

    if len(args.active_skill_list) == 1:
        if 'rm=1' in args.tool_model_dict[args.tool_type]:
            test_name.append('rm')
        if 'attn=1' in args.tool_model_dict[args.tool_type]:
            test_name.append('attn')

    test_name.append(f'max={args.max_n_actions}')
    test_name.append(args.optim_algo)
    if 'CEM' in args.optim_algo and not args.debug:
        test_name.append(f'{args.CEM_sample_size}')
        test_name.append(f'{args.CEM_decay_factor}')
    test_name.append(args.control_loss_type)
    
    if args.debug: test_name.append('debug')

    test_name.append(datetime.now().strftime("%b-%d-%H:%M:%S"))

    return '_'.join(test_name)


def calculate_cubeA_position(obs, insert_config, env_config):
    cubeA_pos = obs['robot0_eef_pos'].copy()
    grasp_offset = insert_config['grasp_offset'].copy()
    cubeA_pos[0] = cubeA_pos[0] - grasp_offset[0] + env_config['cubeA_size'][0]/2
    cubeA_pos[2] -= grasp_offset[1]
    return cubeA_pos, np.array([ -0.7071068, 0, 0, 0.7071068 ])

def get_state_file(args, path, subgoal_num):
    target_shape_name = int(args.target_shape_name)
    return os.path.join(path, f'ep_{target_shape_name:03d}', f'{subgoal_num:03d}_state.npz')

def get_real_obs(args, env, path, obj_in_gp=False):
    obs = get_feedback(env['base'],env['base_cyclic'])

    save_path = path #os.path.join(path, 'states')
    env_config_path = os.path.join(args.target_ep_dir, "env_config.json")
    env_config = load_json(env_config_path)
    subgoal_num = 0
    import planning.scripts.make_target_real as make_target_real
    
    from planning.scripts import make_target_real
    target_real_args = argparse.Namespace(
        ep_num=int(args.target_shape_name),
        subgoal_num=subgoal_num,
        env_config_path=env_config_path,
        save_state_path=save_path,
        control=True
    )
    make_target_real.main(target_real_args)

    # print(f"save_path: {save_path}")
    # vision_command = f"conda activate sam && \
    #     python /home/haonan/Projects/Stowing/planning/scripts/make_target_real.py \
    #         --ep_num {int(args.target_shape_name)} \
    #         --subgoal_num {subgoal_num} \
    #         --env_config_path {env_config_path} \
    #         --save_state_path {save_path}"
    # # subprocess.check_output(vision_command, shell='/bin/bash')
    # os.system(f'source ~/miniconda3/etc/profile.d/conda.sh;{vision_command}')
    # os.system(vision_command)
    # print(f"vision_command: {vision_command}")
    state_file = get_state_file(args, path, subgoal_num)
    dic = np.load(state_file, allow_pickle=True)  
    frame_obs = dic["obs"].item()
    obs.update(frame_obs) 

    if obj_in_gp:
        obs['cubeA_pos'], obs['cubeA_quat'] = calculate_cubeA_position(obs, args.skill_config['insert_config'], env_config)

    return obs

def get_state_from_env(args, obs, static_pcd, env_config, env=None):
    h5_data = preprocess_raw_pcd(args, obs, static_pcd, env_config)

    state_cur_dict = {
        'tensor': torch.tensor(h5_data[0], 
            device=args.device, dtype=torch.float32).unsqueeze(0),
        'obs': h5_data[args.data_names.index('obs')],
        'attr': torch.tensor(h5_data[args.data_names.index('attr')], device=args.device, dtype=torch.float32)
    }

    return state_cur_dict


class MPController(object):
    def __init__(self, cd, args, rollout_root, static_pcd, env_config):
        self.cd = cd
        self.args = args
        # self.args.show_src_img = False
        self.rollout_root = rollout_root
        self.tool = {}
        self.static_pcd = static_pcd
        self.env_config = env_config
        self.sub_goals = []
        self.sub_goal_idx = 0
        self.skill_controller = SkillController(args, args.skill_config)
        self.target_dir = self.args.target_ep_dir
        if self.args.real_world:
            self.recorder = VideoRecorder(self.rollout_root+'/planned.avi')

        self.get_target_shape()
        self.load_tool()


    def get_target_shape(self):

        target_shape = {}
        
        # target_frame_path = os.path.join(self.target_dir, f'001.h5')
        # target_data = load_data(self.args.data_names, target_frame_path)
        
        h5_files = sorted(glob.glob(os.path.join(self.args.target_ep_dir, '*.h5')))
        for h5_path in h5_files:
            if h5_path == h5_files[0]:
                continue
            # frame_id = int(h5_path.split('/')[-1].split('.')[0])
            target_data = load_data(self.args.data_names, h5_path)
            self.sub_goals.append(target_data[0])
            
        # We only care about the dynamic objects
        target_shape['sparse'] = self.sub_goals[self.sub_goal_idx]
        # target_shape['num_keyframes'] = SkillController.SKILL_MAPS[self.args.active_skill_list[0]].num_keyframes()
        target_shape['subgoal_st_idx'] = self.sub_goal_idx

        self.target_shape = target_shape

    def load_tool(self):
        # with open('config/plan_params.yml', 'r') as f:
        #     plan_params = yaml.load(f, Loader=yaml.FullLoader)

        with open('config/model_map.yml', 'r') as f:
            model_dict = yaml.load(f, Loader=yaml.FullLoader)

        if 'sim' in self.args.planner_type:
            model_path_list = None
        else:
            tool_model_names = model_dict[self.args.planner_type]
            if isinstance(tool_model_names, dict):
                model_path_dict = {}
                for skill_name, model_dir_name in tool_model_names.items():
                    model_path_dict[skill_name] = os.path.join(self.cd, '..', 'dump/dynamics/dump_{}'.format(skill_name), model_dir_name)
            else:
                model_path_list = [os.path.join(self.cd, '..', 'models', self.args.planner_type, tool_model_names)]
        for skill_name in self.args.active_skill_list:
            self.tool[skill_name] = Tool(self.args, skill_name, self.args.planner_type, model_path_dict, self.env_config)





    def control(self, env=None):
        if self.args.real_world:
            self.recorder.start_recording()

        if self.args.close_loop:
            if self.args.real_world:
                obs = get_real_obs(self.args, env, self.rollout_root)
            else:
                obs = env.reset()
        
                zero_action = np.array(
                    [-0.027, -0.048, 0.964, -2.22110601,  2.22110601,  0., -1])

                for _ in range(3):
                    obs, _, _, _ = env.step(zero_action)
    

            state_init_dict = get_state_from_env(self.args, obs, self.static_pcd, self.env_config)
        else:
            target_dir = os.path.join(self.cd, '..', 'target_shapes', self.args.target_shape_name)

            name = '000'

            sparse_init_state_path = os.path.join(target_dir, f'{name}.h5')
            sparse_init_state_data = load_data(self.args.data_names, sparse_init_state_path)

            state_init_dict = {
                'tensor': torch.tensor(sparse_init_state_data[0][:, :], 
                    device=self.args.device, dtype=torch.float32).unsqueeze(0),
                'obs': sparse_init_state_data[self.args.data_names.index('obs')],
                'attr': torch.tensor(sparse_init_state_data[self.args.data_names.index('attr')], device=self.args.device, dtype=torch.float32)
            }

        for dir in ['param_seqs', 'anim', 'anim_args', 'states', 'optim_plots', 'raw_data', 'image']:
            os.system('mkdir -p ' + os.path.join(self.rollout_root, dir))

        # plan the actions given the tool
        param_seq_all, state_seq, info_dict, obs, state_cur_dict = self.plan(state_init_dict, self.rollout_root, obs, env)


        if self.args.real_world:
            self.recorder.stop_recording()

        print(f"{'#'*27} MPC SUMMARY {'#'*28}")
        print(param_seq_all)
        
        # Get current state
        if self.args.close_loop:
            try:
                state_cur_dict = get_state_from_env(self.args, obs, self.static_pcd, self.env_config)
                state_cur = state_cur_dict['tensor'].squeeze().cpu().numpy()
            except:
                state_cur = state_cur_dict['tensor'].squeeze().cpu().numpy()
        else:
            state_cur = state_seq[-1, :self.args.n_particles]
            
        # Normalize the current state and the goal state
        state_cur_norm = state_cur - np.mean(state_cur, axis=0)
        state_goal = self.target_shape['sparse']
        state_goal_norm = state_goal - np.mean(state_goal, axis=0)
        
        # Calculate the final chamfer distance between the current and goal states
        dist_final_cd = chamfer(state_cur_norm, state_goal_norm, pkg='numpy')
        dist_final_cmd = emd(state_cur_norm, state_goal_norm, pkg='numpy')
        dist_final_mse = mse(state_cur_norm, state_goal_norm, pkg='numpy')
        print(f'FINAL chamfer distance: {dist_final_cd}, Earth Mover Distance: {dist_final_cmd}, MSE: {dist_final_mse}')
        
        # Print the total planning time
        with open(os.path.join(self.rollout_root, 'planning_time.txt'), 'r') as f:
            print(f'TOTAL planning time (s): {f.read()}')
            
        # Write the parameter sequence to a YAML file
        with open(os.path.join(self.rollout_root, f'MPC_param_seq.yml'), 'w') as f:
            yaml.dump(param_seq_all, f, default_flow_style=True)

        for p in info_dict['subprocess']:
            p.communicate()
            
        # Concatenate the individual animation videos into one video
        anim_list_path = os.path.join(self.rollout_root, f'anim_list.txt')
        with open(anim_list_path, 'w') as f:
            anim_path_list = sorted(glob.glob(os.path.join(self.rollout_root, 'anim', '*.mp4')))
            for anim_path in anim_path_list:
                anim_name = os.path.basename(anim_path)
                if not 'RS' in anim_name and not 'CEM' in anim_name \
                    and not 'GD' in anim_name and not 'sim' in anim_name:
                    f.write(f"file '{anim_path}'\n")

        mpc_anim_path = os.path.join(self.rollout_root, f'MPC_anim.mp4')
        subprocess.run(['ffmpeg', '-f', 'concat', '-safe', '0', '-i', anim_list_path, '-c', 'copy', mpc_anim_path], 
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)


    def execute(self, cur_skill, param_seq, obs, env):
        print(f"Continue executing skill: {cur_skill} from optimized param sequence.")
        self.skill_controller.reset_param(param_seq, obs, self.env_config)
        if self.args.real_world:
            st_state = self.skill_controller._cur_skill.__class__.KEY_STATES[0]
        else:
            st_state = self.skill_controller._cur_skill.__class__.ALL_KEY_STATES[0]

        self.skill_controller._cur_skill._state = st_state
        self.skill_controller._cur_skill.subgoal_pose = self.skill_controller._cur_skill.subgoal_poses[st_state]
        real_state_idx = 0
        while True:
            if self.args.real_world:
                subgoal_state = self.skill_controller._cur_skill.__class__.KEY_STATES[real_state_idx]
                real_state_idx += 1
                st_pose = self.skill_controller._cur_skill.subgoal_poses[subgoal_state]
                ll_action = self.skill_controller._cur_skill.action_dict_to_array(st_pose)
                move_to_desired_pose(env['base'], env['base_cyclic'], ll_action)
                self.skill_controller._cur_skill._state = subgoal_state
                next_obs = get_feedback(env['base'],env['base_cyclic'])
                if self.skill_controller._cur_skill._state == self.skill_controller._cur_skill.__class__.KEY_STATES[-1]:
                    break
            else:
                ll_action = self.skill_controller.step_ll_action(obs)
                next_obs, _, _, _ = env.step(ll_action)
                self.video_writer.append_data(next_obs[f"cam_1_image"])
                if not obj_within_box(self.args.crop_bound, next_obs, self.env_config):
                    # self.video_writer.close()
                    print(f'Skill: {cur_skill} moved the object out of the workspace!')
                    break
                obs = copy.deepcopy(next_obs)

                if self.skill_controller.done():
                    break
        if self.args.real_world:
            obs = get_real_obs(self.args, env, self.rollout_root)

        return obs

    def get_st_pose_for_skill(self, cur_skill, obs, env):
        print(f"Start executing skill: {cur_skill}.")
        self.skill_controller.reset_to_skill(cur_skill)
        hl_param = self.skill_controller.random_hl_param(obs, self.env_config)
        if self.args.real_world:
            st_state = self.skill_controller._cur_skill.__class__.ALL_KEY_STATES[0]
        else:
            st_state = self.skill_controller._cur_skill.__class__.KEY_STATES[0]

        real_state_idx = 0
        while True:
            
            if self.args.real_world:
                # Can be accelerated
                subgoal_state = self.skill_controller._cur_skill.__class__.STATES[real_state_idx]
                real_state_idx += 1
                st_pose = self.skill_controller._cur_skill.subgoal_poses[subgoal_state]
                ll_action = self.skill_controller._cur_skill.action_dict_to_array(st_pose)
            
                move_to_desired_pose(env['base'], env['base_cyclic'], ll_action)
                self.skill_controller._cur_skill._state = subgoal_state
                next_obs = get_feedback(env['base'],env['base_cyclic'])

            else:
                ll_action = self.skill_controller.step_ll_action(obs)

                next_obs, _, done, _ = env.step(ll_action)
            if not self.args.real_world:
                self.video_writer.append_data(next_obs[f"cam_1_image"])

            # cur_ee_pose = {
            #     'cur_pos': next_obs['robot0_eef_pos'], 'cur_quat': next_obs['robot0_eef_quat']}
            # if self.skill_controller._cur_skill.reached_subgoal(cur_ee_pose, self.skill_controller._cur_skill.subgoal_poses[st_state]):
            # if self.skill_controller.is_keyframe_reached() != None:
            if self.skill_controller._cur_skill._state == st_state:
                break

        if self.args.real_world :
            next_obs = get_real_obs(self.args, env, self.rollout_root, obj_in_gp=True if cur_skill == 'insert' else False)
        
        return next_obs

        
    def plan(self, state_cur_dict, rollout_path, obs, env, max_n_actions=1, pred_err_bar=0.015):
        global command_feedback
        best_param_seq_list = []
        if not self.args.real_world:
            self.video_writer = imageio.get_writer(
                f'{rollout_path}/planned.mp4', fps=20)

            self.video_writer.append_data(obs[f"cam_1_image"])
        if self.args.close_loop:
            for cur_skill in self.args.active_skill_list:
                
                self.target_shape['sparse'] = self.sub_goals[self.sub_goal_idx]
                # state_demo_dict = {'sparse': self.sub_goals[self.sub_goal_idx-SkillController.SKILL_MAPS[cur_skill].num_keyframes()+1: self.sub_goal_idx]}
                st_skill_obs = self.get_st_pose_for_skill(cur_skill, obs, env)
                # try:
                state_cur_dict = get_state_from_env(self.args, st_skill_obs, self.static_pcd, self.env_config)
                # except: 
                #     print(f'Failed to execute the skill: {cur_skill}!')
                #     self.video_writer.close()
                #     break

                best_param_seq, best_state_seq, best_info_dict = self.tool[cur_skill].rollout(
                    state_cur_dict, self.target_shape, self.target_dir, rollout_path, self.args.max_n_actions
                )


                act_len = best_state_seq.shape[0] // best_param_seq.shape[0]
                act_start = 0
                act_end = 1
                param_seq_todo = best_param_seq[act_start:act_end].cpu().numpy()
                param_seq_pred = param_seq_todo
                state_seq_pred = best_state_seq[:act_len][:self.args.n_particles]
                state_pred_tensor = torch.tensor(state_seq_pred[-1], device=self.args.device, 
                    dtype=torch.float32).unsqueeze(0)
                info_dict_pred = best_info_dict

                # loss_dict = {'Chamfer': [], 'EMD': [], 'IOU': []}
                
                cur_obs = self.execute(cur_skill, param_seq_todo, st_skill_obs, env)

                obs = copy.deepcopy(cur_obs)
                
                state_cur_dict = get_state_from_env(self.args, obs, self.static_pcd, self.env_config)
                
                pred_err = mse(state_cur_dict['tensor'].squeeze(), state_pred_tensor.squeeze(), pkg='torch')
                
                execution_err = mse(state_cur_dict['tensor'].squeeze(), torch.from_numpy(self.target_shape['sparse'].squeeze()).float().to(self.args.device), pkg='torch')

                print(f"The prediction error is {pred_err}!")
                print(f"The execution error is {execution_err}!")
                # chamfer_loss, emd_loss = self.eval_state(state_cur_dict['tensor'].cpu(), 
                #     step, best_target_idx, state_pred=state_pred_tensor.cpu(), pred_err=pred_err)
                
                self.sub_goal_idx += 1 # SkillController.SKILL_MAPS[cur_skill].num_keyframes()-1
                self.target_shape['subgoal_st_idx'] = self.sub_goal_idx

            best_param_seq_list.append(param_seq_pred) 
            best_state_seq = state_seq_pred 
            best_info_dict = info_dict_pred
        else:
            best_param_seq, best_state_seq, best_info_dict = self.tool['push'].rollout(
                state_cur_dict, self.target_shape, self.target_dir, rollout_path, self.args.max_n_actions
            )

            state_cur_dict['tensor'] = torch.tensor(best_state_seq[-1][:self.args.n_particles], 
                device=self.args.device, dtype=torch.float32).unsqueeze(0)
        if not self.args.real_world: 
            self.video_writer.close()
        
        if self.args.real_world:
            # Clean up manually, as __exit__ would have done
            env['device_connection'].__exit__(None, None, None)

        return best_param_seq_list, best_state_seq, best_info_dict, obs, state_cur_dict

def create_env(args):
    env_config_path = os.path.join(args.target_ep_dir, "env_config.json")
    env_config = load_json(env_config_path)
    if args.env_config is None:
        args.env_config = env_config
    static_pcd = create_static_pcd(args, env_config)

    if args.real_world:
        # from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2, Session_pb2
        # from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
        # from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
        # import planning.kinova_utilities as kinova_utilities
        # from planning.kinova_gen3 import move_to_desired_pose, get_feedback

        # Parse arguments
        kinova_args = kinova_utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        device_connection = kinova_utilities.DeviceConnection.createTcpConnection(kinova_args)

        # Manually do what __enter__ would have done
        router = device_connection.__enter__()
        
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        example_move_to_home_position(base)
        env = {'base': base, 'base_cyclic': base_cyclic, 'device_connection': device_connection}
    
    else:
        xml_path = os.path.join(args.target_ep_dir, "model.xml")
        env = suite.make(env_name='Stow',
                        has_renderer=False,
                        has_offscreen_renderer=True,
                        use_camera_obs=True,
                        # render_camera="agentview",
                        ignore_done=True,
                        control_freq=20,
                        args=args,
                        **args.sim_kwargs,
                        )
        env.reset()

        with open(xml_path, "r") as f:
            xml = f.read()
            # xml = env.edit_model_xml(f.read())
            env.reset_from_xml_string(xml)
    
        env.sim.reset()


        init_state = np.load(os.path.join(args.target_ep_dir, "000_state.npz"), allow_pickle=True)["states"][0]
    
        # load the initial state
        env.sim.set_state_from_flattened(init_state)

        env.sim.forward()

    return env, static_pcd, env_config

def main():
    args = gen_args()
    # args.data_names = ['positions', 'shape_quats', 'scene_params']

    # if args.close_loop:
        # import rospy
        # from perception.get_visual_feedback import ros_bag_to_pcd
        # from rospy.numpy_msg import numpy_msg
        # from rospy_tutorials.msg import Floats
        # from std_msgs.msg import UInt8, String

    cd = os.path.dirname(os.path.realpath(sys.argv[0]))
 
    rollout_root = os.path.join(cd, '..', 'dump', 'control' if not args.real_world else 'control_real', 
        args.target_shape_name, get_test_name(args))
    os.system('mkdir -p ' + rollout_root)

    for dir in ['states', 'raw_data', f'ep_{args.target_shape_name}', os.path.join('states', args.target_shape_name)]:
        os.system('mkdir -p ' + os.path.join(rollout_root, dir))
    tee = Tee(os.path.join(rollout_root, 'control.txt'), 'w')

    args.target_ep_dir = os.path.join(args.target_path, args.target_shape_name)

    env, static_pcd, env_config = create_env(args)
    
    mpcontroller = MPController(cd, args, rollout_root, static_pcd, env_config)

    # if args.close_loop:

    #     mpcontroller.param_seq_pub = rospy.Publisher('/param_seq', numpy_msg(Floats), queue_size=10)
    #     mpcontroller.command_pub = rospy.Publisher('/command', String, queue_size=10)
    #     mpcontroller.ros_data_path_pub = rospy.Publisher('/raw_data_path', String, queue_size=10)
    #     rospy.Subscriber('/command_feedback', UInt8, command_fb_callback)
    
    mpcontroller.control(env)


if __name__ == '__main__':
    main()
