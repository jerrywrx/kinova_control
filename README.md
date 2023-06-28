# Kinova Control
A repository that contains experiments using the Kinova Gen3 robot

## Running Spacemouse-Kinova Control
1. Launch Kortex driver

`source catkin_workspace/devel/setup.bash && roslaunch kortex_driver kortex_driver.launch gripper:="robotiq_2f_85"`

2. Launch Spacemouse driver
 
`source catkin_workspace/devel/setup.bash && rosrun kortex_driver spacemouse_node.py`

3. Launch Spacemouse-Kinova Control script

`source catkin_workspace/devel/setup.bash && rosrun kortex_examples spaceMouse_kinova.py`

## Running Waypoints Save-and-Replay
```
cd catkin_workspace/src/ros_kortex/kortex_examples/src/save_and_replay
mkdir recordings
```
### Save waypoints
You can save waypoints automatically or manually

- Save waypoints automatically (define the recording period in seconds):

`python save_waypoints_auto.py`

- Save waypoints manually (save waypoint by pressing Enter key):

`python save_waypoints_manual.py`

### Replay waypoints
`python replay_waypoints.py <filename>`

Replace `<filename>` with the name of the recorded yaml file. The script will replay the latest yaml file in the recordings folder if no argument is entered.
