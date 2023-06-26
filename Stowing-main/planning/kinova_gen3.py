from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2, Session_pb2
import threading
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
import time
import pdb
from scipy.spatial.transform import Rotation
import numpy as np

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    def check(notification, e = e):
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def populateCartesianCoordinate(waypointInformation):
    
    waypoint = Base_pb2.CartesianWaypoint()  
    waypoint.pose.x = waypointInformation[0]
    waypoint.pose.y = waypointInformation[1]
    waypoint.pose.z = waypointInformation[2]
    waypoint.blending_radius = waypointInformation[3]
    waypoint.pose.theta_x = waypointInformation[4]
    waypoint.pose.theta_y = waypointInformation[5]
    waypoint.pose.theta_z = waypointInformation[6] 
    waypoint.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    
    return waypoint


def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def move_to_desired_pose(base, base_cyclic, pose):
    x, y, z, theta_x, theta_y, theta_z, gripper = pose

    # Construct action
    action = Base_pb2.Action()
    action.name = "Move to desired position"
    action.application_data = ""
    
    # Set target pose
    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = x
    cartesian_pose.y = y
    cartesian_pose.z = z
    cartesian_pose.theta_x = theta_x
    cartesian_pose.theta_y = theta_y
    cartesian_pose.theta_z = theta_z


    # Set gripper command
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    finger.finger_identifier = 1
    finger.value = gripper

    # Start action
    print("Executing action")
    base.ExecuteAction(action)

    # Send gripper command
    base.SendGripperCommand(gripper_command)

    # Wait for action to finish
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(check_for_end_or_abort(e), Base_pb2.NotificationOptions())
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def get_feedback(base, base_cyclic):
    # Refresh feedback
    feedback = base_cyclic.RefreshFeedback()

    # Get the end effector pose
    pose = feedback.base

    # Get the gripper state
    gripper_feedback = feedback.interconnect.gripper_feedback
    motor = gripper_feedback.motor[0]

    obs = {
        'robot0_eef_pos': [pose.tool_pose_x, pose.tool_pose_y, pose.tool_pose_z],
        'robot0_eef_ori': [pose.tool_pose_theta_x, pose.tool_pose_theta_y, pose.tool_pose_theta_z],
        'robot0_eef_quat': Rotation.from_euler('xyz', [pose.tool_pose_theta_x, pose.tool_pose_theta_y, pose.tool_pose_theta_z], degrees=True).as_quat(),
        'robot0_gripper_mpos': [motor.position],
    }



    return obs

def get_feedback(base, base_cyclic):
    # Refresh feedback
    feedback = base_cyclic.RefreshFeedback()

    # Get the end effector pose
    pose = feedback.base

    # Get the gripper state
    gripper_feedback = feedback.interconnect.gripper_feedback
    motor = gripper_feedback.motor[0]

    obs = {
        'robot0_eef_pos': [pose.tool_pose_x, pose.tool_pose_y, pose.tool_pose_z],
        'robot0_eef_ori': [pose.tool_pose_theta_x, pose.tool_pose_theta_y, pose.tool_pose_theta_z],
        'robot0_eef_quat': Rotation.from_euler('xyz', [pose.tool_pose_theta_x, pose.tool_pose_theta_y, pose.tool_pose_theta_z], degrees=True).as_quat(),
        'robot0_gripper_mpos': [motor.position],
    }



    return obs

def demo_control():
    import planning.kinova_utilities as kinova_utilities

    # Parse arguments
    kinova_args = kinova_utilities.parseConnectionArguments()

    # Create connection to the device and get the router
    with kinova_utilities.DeviceConnection.createTcpConnection(kinova_args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        
        # # Create an instance of the CartesianSpeed class
        # cartesian_speed = Base_pb2.CartesianSpeed

        # # Set the desired speeds
        # cartesian_speed.translation = 1  # meters per second
        # cartesian_speed.orientation = 30.0  # degrees per second

        # Define a series of desired poses and gripper states
        push_quat = [1, 0, 0., 0.]
        push_angle = Rotation.from_quat(push_quat).as_euler('xyz', degrees=True)
        push_pose = (0.4, 0.05, 0.25, *push_angle, 0)
        
        sweep_pose0 = (0.64, 0, 0.25, *Rotation.from_quat([0.707, 0,  0.707,  0]).as_euler('xyz', degrees=True), 0)
        sweep_pose1 = (0.82, 0, 0.25, *Rotation.from_quat([0.707, 0,  0.707,  0]).as_euler('xyz', degrees=True), 0)
        
        
        #{'robot0_eef_pos': [0.2559052109718323, -0.407727986574173, -0.0009112982079386711], 
        # 'robot0_eef_ori': [15.392931938171387, -94.22189331054688, 163.69529724121094], 
        # 'robot0_eef_quat': array([ 0.73166045, -0.01273553,  0.68154865, -0.00149081]), 
        # 'robot0_gripper_mpos': [99.12281036376953]}
        poses = [
            # push_pose,
            [ 0.43274727, -0.36915424,  0.2  ,  180, -90, 0,  0],
            [ 0.5       ,  -0.2,  0.36434424, 90.        ,  0.        ,90.        ,  1.        ],
            [2.33182214e-01, -4.14088204e-01,  1.12844242e-01,  180, -90, 0,  0],
            [0.31735326, -0.39917169,  0.45, 90, -90, 90.,  0.        ],
            [0.31735326, -0.39917169,  0.55, 90, -90, 90.,  1.        ],
            [0.31735326, -0.39917169,  0.1, 90, -90, 90, 1],
            [0.75      , 0.14083833, 0.20868639, 90, -90, 90, 1],
            [0.75      , 0.14083833, 0.345304 , 90, -90, 90, 1],
            [0.5       , 0.14083833, 0.345304, 90, -90, 90, 1],
            [0.5       , 0.14083833, 0.17868639, 90, -90, 90, 1],
            [0.5       , 0.14083833, 0.17868639, 90, -90, 90, 1],
            [0.5       , 0.018     , 0.17868639, 90, -90, 90, 1],
            [0.5       , 0.018     , 0.17868639, 90, -90, 90, 1],
            sweep_pose0,
            (0.82, 0.0, 0.03, 90, -90, 90, 0),   # sweep pose
            (0.8, 0.1, 0.4, 80.0, 10.0, 80.0, 0),  # Pose 2
            (0.7, 0.0, 0.5, 90.0, 0.0, 90.0, 1),   # Pose 1
            # Add more poses as needed
        ]
        
        # Execute each pose in the series
        for pose in poses:
            obs = get_feedback(base, base_cyclic)
            pdb.set_trace()
            move_to_desired_pose(base, base_cyclic, pose)

def demo_listen():
    import planning.kinova_utilities as kinova_utilities

    # Parse arguments
    kinova_args = kinova_utilities.parseConnectionArguments()

    # Create connection to the device and get the router
    with kinova_utilities.DeviceConnection.createTcpConnection(kinova_args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        
        # # Create an instance of the CartesianSpeed class
        # cartesian_speed = Base_pb2.CartesianSpeed

        # # Set the desired speeds
        # cartesian_speed.translation = 1  # meters per second
        # cartesian_speed.orientation = 30.0  # degrees per second
        while True:
            obs = get_feedback(base, base_cyclic)
            print(obs)
            time.sleep(1)


def demo_tcp():
    import planning.kinova_utilities as kinova_utilities

    # Parse arguments
    kinova_args = kinova_utilities.parseConnectionArguments()

    # Create connection to the device and get the router
    device_connection = kinova_utilities.DeviceConnection.createTcpConnection(kinova_args)

    # Manually do what __enter__ would have done
    router = device_connection.__enter__()
    
    # Create required services
    base = BaseClient(router)
    base_cyclic = BaseCyclicClient(router)
    
    # Rest of your code here

    push_quat = [1, 0, 0., 0.]
    push_angle = Rotation.from_quat(push_quat).as_euler('xyz', degrees=True)
    push_pose = np.array((0.4, 0.05, 0, *push_angle, 0))
    
    sweep_pose0 = np.array((0.64, 0, 0.03, *Rotation.from_quat([0.707, 0,  0.707,  0]).as_euler('xyz', degrees=True), 0))
    sweep_pose1 = (0.82, 0, 0.03, *Rotation.from_quat([0.707, 0,  0.707,  0]).as_euler('xyz', degrees=True), 0)
    
    #{'robot0_eef_pos': [0.2559052109718323, -0.407727986574173, -0.0009112982079386711], 
    # 'robot0_eef_ori': [15.392931938171387, -94.22189331054688, 163.69529724121094], 
    # 'robot0_eef_quat': array([ 0.73166045, -0.01273553,  0.68154865, -0.00149081]), 
    # 'robot0_gripper_mpos': [99.12281036376953]}
    poses = [
        [ 3.17469937e-01, -3.99760930e-01,  2.30000000e-02,  1.80000000e+02, -9.00000000e+01,  0.00000000e+00,  1.00000000e+00],
        [3.17469937e-01, -3.99760930e-01,  1.00000000e-01,  1.80000000e+02, -9.00000000e+01,  0.00000000e+00,  1.00000000e+00],
        [0.75      ,  0.14083833,  0.20868639, 90.        ,  0.        ,  90.        ,  1.        ],
        [0.75      ,  0.14083833,  0.345304  , 90.        ,  0.        , 90.        ,  1.        ],
        push_pose,
        sweep_pose0,
        (0.82, 0.0, 0.03, 90, -90, 90, 0),   # sweep pose
        (0.8, 0.1, 0.4, 80.0, 10.0, 80.0, 0),  # Pose 2
        (0.7, 0.0, 0.5, 90.0, 0.0, 90.0, 1),   # Pose 1
        # Add more poses as needed
    ]

    # Execute each pose in the series
    for pose in poses:
        obs = get_feedback(base, base_cyclic)
        pdb.set_trace()
        move_to_desired_pose(base, base_cyclic, pose)

    # Clean up manually, as __exit__ would have done
    device_connection.__exit__(None, None, None)

if __name__ == "__main__":

    exit(demo_listen())
