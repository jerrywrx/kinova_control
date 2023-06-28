# Kinova Control
A repository that contains experiments using the Kinova Gen3 robot

## Running Spacemouse-Kinova Control
1. Launch Kortex driver\
`source ~/spa/catkin_workspace/devel/setup.bash && roslaunch kortex_driver kortex_driver.launch gripper:="robotiq_2f_85"`

3. Launch Spacemouse driver\
`source ~/spa/catkin_workspace/devel/setup.bash && rosrun kortex_driver spacemouse_node.py`

5. Launch Spacemouse-Kinova Control script\
`source ~/spa/catkin_workspace/devel/setup.bash && rosrun kortex_examples spaceMouse_kinova.py`
