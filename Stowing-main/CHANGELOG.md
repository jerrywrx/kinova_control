# Change Log
All notable changes to this project will be documented in this file.
 
The format is based on [Keep a Changelog](http://keepachangelog.com/).

## 2023-05-010
### Modified
- Adjust the parameter bounds for different env_confi
- Add an option to construct edges between static objects and dynamics objects
- change the action to keyposes in sampling, training, and evaluation
- visualize edges (it is able to show the edge at the first frame)


## 2023-05-06
### Modified
- Correct gripper motion
- Increase edge radius

## 2023-05-06
### Modified
- Fix the bug in dynamics prediction in tool touching case
- Fix the tool movement code in eval code


## 2023-05-05
### Added
- A new visualization for processed point cloud from the raw observation

## 2023-05-02
### Added
- Implement the gradient descent method to optimize the trajectory

## 2023-05-02
### Modified
- Removed images in obs dictionary to accelerate loading speed
- Fix the make_target.py function to convert human demonstration to target shape

## 2023-05-01
### Modified
- Fix the problem in EGL rendering backend


## 2023-04-28
### Modified
- Modify the render_anim function to align the camera image with object movement prediction


## 2023-04-28
### Added
- Enable control code to generate a video to show actual control from trajectory optimization
- Added script to patch the target_shapes


## 2023-04-22
### Modified
- Add python code to collect keyframe in a demonstration


## 2023-04-22
### Modified
- Vary the number of instance in data sampling
- Vary the shelf length


## 2023-04-11
### Modified
- Better visualization for eval code
### Added
- Random shooting

## 2023-04-11
### Added
- Add inserting and sweeping skill
- Write a script for automatically cleaning the invalid perception data
### Modified
- Increase the friction of the robotiq 85 gripper

## 2023-03-27
### Added
- Add physical dynamics modelling

## 2023-03-21
### Added
- parallelizd the sampling code

### Fixed
- Used the previous robotiq-85 xml file to correct end effector frame and make absolute action work (https://github.com/ARISE-Initiative/robosuite/pull/204)

## 2023-03-15
### Added
- Added gripper pcd

## 2023-03-13
### Added
- Added shelf pcd and table pcd

## 2023-03-12
### Added
- Created box pcd from position, quaternion, and size

## 2023-03-09
### Added
- Computed the oriented bounding box based on the PCA of the convex hull. Generated an approximation to the minimal bounding box.


## 2023-03-06
### Modified
- Clean the code for merging pcd

## 2023-03-04
### Modified
- Point cloud cropping and outliners removal

## 2023-03-03
### Modified
- Merged point cloud from multiple cameras in sampling

## 2023-03-02
### Added
- Different color for different objects

## 2023-02-27
### Added
- Added skill controller and push skill
- Added data sampling

## 2023-02-19
### Added
- Added 4 more cameras in the simulator 

## 2023-02-15
### Added
- Added segementation by getting some code from robosuite-1.3 and upgrading mujoco-py 

## 2023-02-08
### Added
- Included point cloud downsampling into _default_obs_processor of PointCloudModality using farthest distance

## 2023-02-07
### Added
- Added a randomizer to inject gussian noise for low_dim input
- Added a randomizer to randomly drop points, scale points, and shift points for point cloud input

## 2023-02-06
### Fixed
- Fix vendor id, product id, accessible keys to support spacemouse compact

## 2023-01-31
### Added
- Added PointNet++ as point cloud encoder backbone
- Loaded pretrianed weights for PointNet++, and disabled all layers gradient expect the last layer in Pytorch?

## 2023-01-22
### Added
- Add robomimic
- Add depth image collection in robomimic
- Added a new modality for [Point Clouds](https://github.com/vaibhavsaxena11/robomimic/pull/1), called pcd in robomimic 


## 2023-01-22
### Changed
- Transfer robosuite from 1.4 to 1.2 for imitation learning

## 2023-01-15
### Added
- Added a shelf_overlap_check to make sure valid placement of objects on the shelf

## 2023-01-14
### Added
- Added check_contact between objects and shelf_bottom_collision to check whether the object is placed on the shelf

## 2023-01-11
### Added
- Added prior rotation matrix in OSC controller
- Added keyboard support for prior rotation configuration

## 2023-01-10
### Added
- Reduced the simulation solver timestep from the default value of 0.002 to 0.001 and set the “noslip iterations” to 20 to improve simulation quality on contacts. (Same trick used in [Learning to Grasp the Ungraspable with Emergent Extrinsic Dexterity](https://github.com/Wenxuan-Zhou/ungraspable))


## 2023-01-08
### Changed 
- Set up multi-view cameras for human data collection in [opencv_render.py](https://github.com/haonan16/Stowing/blob/main/robosuite/robosuite/utils/opencv_renderer.py) and [collect_human_demonstrations.py](https://github.com/haonan16/Stowing/blob/main/robosuite/robosuite/scripts/collect_human_demonstrations.py)

## 2023-12-31
### Changed
- Changed the shelf size in [stowing_arena.xml](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/models/assets/arenas) and [stow.py](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/environments/manipulation/)



## 2023-12-27
### Changed
- Added a box for grasping in [stow.py](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/environments/manipulation/)
- Rewrote the _setup_observables and _check_success method in [stow.py](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/environments/manipulation/)

## 2023-12-23
### Changed
- Added opencv-python==4.2.0.32 as the pip requirements.txt
### Fixed
- Modified _load_model method in [stow.py](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/environments/manipulation/) to make box is sampled on the shelf.


## 2023-12-21
  
Here we would have the update steps for boxes and shelf in mujoco simulator.
 
### Added
- Created robosuite.robosuite.models.assets.arenas.[stowing_arena.xml](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/models/assets/arenas) as mujoco arena environment
- Added a [jupyter notebook](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/models/assets/arenas) to visualize the simulation as mujoco arena environment. However, mediapy.show_video cannot be loaded.
- Python binding for [stowing_arena.py](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/models/arenas) from bins_arena.py.
- A minimalistic example of how to interact with an environment in [demon_run_env.py](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/demos)
- The robosuite environment for stowing in [stow.py](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/environments/manipulation/)
### Changed
 - Add a table, remove the visual legs of shelf, and reduce the size of shelf base in z-axis to 0.01 meter in [stowing_arena.xml](https://github.com/haonan16/Stowing/tree/main/robosuite/robosuite/models/assets/arenas) .

 
