import os, glob
from config.config import gen_args
import utils.data_utils
from utils.data_utils import *
from perception.sample import preprocess_raw_pcd, create_static_pcd, save_data_step
import shutil
import pdb

def main():
    args = gen_args()
    demo_ep_dir = os.path.join(args.demo_path, "real", 'ep_001')
    if not os.path.exists(demo_ep_dir):
        print('The demo directory does not exist')

    last_ep_path = sorted(glob.glob(os.path.join(args.target_path, '*') ))

    if not last_ep_path:
        target_ep_dir = os.path.join(args.target_path, '000')
    else:
        last_ep_path = last_ep_path[-1]
        target_ep_dir = last_ep_path[:-3] + '{:03d}'.format(int(last_ep_path[-3:]) + 1)
        
    
    shutil.copytree(demo_ep_dir, target_ep_dir)
    
    ep_num = int(target_ep_dir[-3:])

    # store model xml as an attribute
    env_config_path = os.path.join(target_ep_dir, "env_config.json")

    state_paths = os.path.join(target_ep_dir, "*_state.npz")
    
    h5_files = glob.glob(os.path.join(target_ep_dir, '*.h5'))

    env_config = load_json(env_config_path)

    static_pcd = create_static_pcd(args, env_config)

    if not h5_files:

        i = 0
        for state_file in sorted(glob.glob(state_paths)):
            dic = np.load(state_file, allow_pickle=True)
            
            if 'real' in demo_ep_dir:
                frame_obs = dic["obs"].item()
            else:
                frame_obs = dic["obs"][0]
                
                

            h5_data = preprocess_raw_pcd(
                args, frame_obs, static_pcd, env_config)
            dummy_tool = np.zeros((args.gripper_dim, args.state_dim))
            h5_data.append(dummy_tool)

            save_data_step(args, h5_data, frame_obs, args.target_path, ep_num, i)
            i += 1
            
        # Delete the last state. This is because when the DataCollector wrapper
        # recorded the states and actions, the states were recorded AFTER playing that action,
        # so we end up with an extra state at the end.





if __name__ == "__main__":
    main()
