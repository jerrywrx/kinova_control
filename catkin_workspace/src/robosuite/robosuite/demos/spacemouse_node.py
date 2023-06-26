import argparse
import numpy as np
import rospy

# def spacemouse_talker():
#     pub = rospy.Publisher('dx', String, queue_size=10)
#     rospy.init_node('spacemouse_state', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--environment", type=str, default="Lift")
    parser.add_argument("--robots", nargs="+", type=str, default="Panda", help="Which robot(s) to use in the env")
    parser.add_argument(
        "--config", type=str, default="single-arm-opposed", help="Specified environment configuration if necessary"
    )
    parser.add_argument("--arm", type=str, default="right", help="Which arm to control (eg bimanual) 'right' or 'left'")
    parser.add_argument("--switch-on-grasp", action="store_true", help="Switch gripper control on gripper action")
    parser.add_argument("--toggle-camera-on-grasp", action="store_true", help="Switch camera angle on gripper action")
    parser.add_argument("--controller", type=str, default="osc", help="Choice of controller. Can be 'ik' or 'osc'")
    parser.add_argument("--device", type=str, default="keyboard")
    parser.add_argument("--pos-sensitivity", type=float, default=1.0, help="How much to scale position user inputs")
    parser.add_argument("--rot-sensitivity", type=float, default=1.0, help="How much to scale rotation user inputs")
    args = parser.parse_args()

    # Import controller config for EE IK or OSC (pos/ori)
    if args.controller == "ik":
        controller_name = "IK_POSE"
    elif args.controller == "osc":
        controller_name = "OSC_POSE"
    else:
        print("Error: Unsupported controller specified. Must be either 'ik' or 'osc'!")
        raise ValueError

    # initialize device
    from robosuite.devices import SpaceMouse
    device = SpaceMouse(pos_sensitivity=args.pos_sensitivity, rot_sensitivity=args.rot_sensitivity)

    while True:

        # try:
        #     spacemouse_talker()
        # except rospy.ROSInterruptException:
        #     pass

        # Initialize variables that should the maintained between resets
        last_grasp = 0

        # Initialize device control
        device.start_control()

        while True:
            # Set active robot

            # Get current spacemouse state
            state = device.get_controller_state()

            dpos, rotation, raw_drotation, grasp, reset = (
            state["dpos"],
            state["rotation"],
            state["raw_drotation"],
            state["grasp"],
            state["reset"],
            )

            print("dpos")
            print(dpos)
            # print("rotation")
            # print(rotation)
            # print("raw_drotation")
            # print(raw_drotation)
