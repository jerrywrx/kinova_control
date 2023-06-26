import os.path
from perception_pipeline import *
from oak_camera import *
import os 
import argparse

def main(args):
    capture()
    ep_num = args.ep_num # 0
    subgoal_num = args.subgoal_num # 2
    # demo_path = os.path.join() # f'/home/haonan/Projects/Stowing/target_shapes/real/{ep_num:03d}'
    config_path = args.env_config_path
    save_path = os.path.join(args.save_state_path, f'ep_{ep_num:03d}', f'{subgoal_num:03d}_state.npz')
    side_view_gt_img_list = [
        '/home/haonan/Projects/stowing_perception/gt_image/front_cropped_5.png',
        '/home/haonan/Projects/stowing_perception/gt_image/front_cropped_1.png',
        '/home/haonan/Projects/stowing_perception/gt_image/front_cropped_12.png',
        '/home/haonan/Projects/stowing_perception/gt_image/front_cropped_11.png'
    ]
    top_gt_img = '/home/haonan/Projects/stowing_perception/gt_image/top_cropped_11.png'
    perception = perception_wrapper(config_path, save_path, side_view_gt_img_list, top_gt_img, visualize=False, control=args.control)
    perception.pose_estimation(subgoal_num)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Collect the state.')
    parser.add_argument('--ep_num', type=int, default=1)
    parser.add_argument('--subgoal_num', type=int, choices=[0, 1 ,2, 3], default=3)
    parser.add_argument('--env_config_path', type=str, default=f'/home/haonan/Projects/Stowing/dump/demos/real')
    parser.add_argument('--save_state_path', type=str, default=f'/home/haonan/Projects/Stowing/dump/demos/real')
    parser.add_argument('--control', type=bool, default=False)

    args = parser.parse_args()
    if 'env_config.json' not in args.env_config_path:
        args.env_config_path = os.path.join(args.env_config_path, f'ep_{args.ep_num:03d}','env_config.json')

    main(args)
