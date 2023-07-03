#!/usr/bin/env python

import yaml
import rospy
from datetime import datetime

from std_msgs.msg import Float64
from kortex_driver.msg import BaseCyclic_Feedback

class WaypointsRecorder:
    def __init__(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0

        self.d_linear_x = 0.0
        self.d_linear_y = 0.0
        self.d_linear_z = 0.0
        self.d_angular_x = 0.0
        self.d_angular_y = 0.0
        self.d_angular_z = 0.0

        self.waypoint_count = 1

        curr_time = datetime.now().strftime("%d-%b-%Y-%H:%M:%S.%f")
        self.file_name = "recordings/" + str(curr_time) + ".yaml"

        # Initialize the YAML file with an empty list
        with open(self.file_name, 'w') as file:
            yaml.dump([], file)

        # subscribe to data including joint angles, joint velocities, and tool pose
        rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, self.tool_pose_callback)

        # Subscribe to spacemouse linear velocity commands
        rospy.Subscriber("spacemouse_state/d_linear_x", Float64, self.spacemouse_linear_x_callback)
        rospy.Subscriber("spacemouse_state/d_linear_y", Float64, self.spacemouse_linear_y_callback)
        rospy.Subscriber("spacemouse_state/d_linear_z", Float64, self.spacemouse_linear_z_callback)

        # Subscribe to spacemouse angular velocity commands
        rospy.Subscriber("spacemouse_state/d_angular_x", Float64, self.spacemouse_angular_x_callback)
        rospy.Subscriber("spacemouse_state/d_angular_y", Float64, self.spacemouse_angular_y_callback)
        rospy.Subscriber("spacemouse_state/d_angular_z", Float64, self.spacemouse_angular_z_callback)

        rospy.Subscriber("spacemouse_state/grasp", Float64, self.spacemouse_grasp_callback)

    def tool_pose_callback(self, msg):
        self.linear_x = msg.base.tool_pose_x
        self.linear_y = msg.base.tool_pose_y
        self.linear_z = msg.base.tool_pose_z
        self.angular_x = msg.base.tool_pose_theta_x
        self.angular_y = msg.base.tool_pose_theta_y
        self.angular_z = msg.base.tool_pose_theta_z

    def spacemouse_linear_x_callback(self, msg):
        self.d_linear_x = -msg.data * 40

    def spacemouse_linear_y_callback(self, msg):
        self.d_linear_y = -msg.data * 40
        
    def spacemouse_linear_z_callback(self, msg):
        self.d_linear_z = msg.data * 40

    def spacemouse_angular_x_callback(self, msg):
        self.d_angular_x = -msg.data * 80

    def spacemouse_angular_y_callback(self, msg):
        self.d_angular_y = -msg.data * 80

    def spacemouse_angular_z_callback(self, msg):
        self.d_angular_z = -msg.data * 80

    def spacemouse_grasp_callback(self, msg):
        self.grasp = msg.data

    def save_to_yaml(self):
        curr_time = datetime.now().strftime("%d-%b-%Y-%H:%M:%S.%f")

        data = {
            "waypoint" : self.waypoint_count,
            "timestamp" : curr_time,
            "linear_x" : self.linear_x,
            "linear_y" : self.linear_y,
            "linear_z" : self.linear_z,
            "angular_x" : self.angular_x,
            "angular_y" : self.angular_y,
            "angular_z" : self.angular_z,
            "d_linear_x" : self.d_linear_x,
            "d_linear_y" : self.d_linear_y,
            "d_linear_z" : self.d_linear_z,
            "d_angular_x" : self.d_angular_x,
            "d_angular_y" : self.d_angular_y,
            "d_angular_z" : self.d_angular_z
        }

        with open(self.file_name, 'a') as file:
            file.write('---\n')
            yaml.dump(data, file, default_flow_style=False)

        print(f"Data appended to '{self.file_name}'")

    def run(self):
        key = None
        while key != 'q':
            key = input("Press Enter to record Waypoint " + str(self.waypoint_count) + " or 'q' to end recording ")
            self.save_to_yaml()
            self.waypoint_count += 1

if __name__ == "__main__":
    rospy.init_node("waypoints_recorder")
    recorder = WaypointsRecorder()
    recorder.run()
