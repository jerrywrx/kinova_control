#!/usr/bin/env python

import yaml
import rospy
from datetime import datetime

from std_msgs.msg import Float64
from kortex_driver.msg import BaseCyclic_Feedback

class WaypointsRecorder:
    def __init__(self):
        rospy.init_node("waypoints_recorder")

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0

        # subscribe to data including joint angles, joint velocities, and tool pose
        rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, self.tool_pose_callback)

    def tool_pose_callback(self, msg):
        self.linear_x = msg.base.tool_pose_x
        self.linear_y = msg.base.tool_pose_y
        self.linear_z = msg.base.tool_pose_z
        self.angular_x = msg.base.tool_pose_theta_x
        self.angular_y = msg.base.tool_pose_theta_y
        self.angular_z = msg.base.tool_pose_theta_z

        print(msg.base.tool_twist_linear_x)

    def save_file(self):
        time_now = datetime.now().strftime("%d-%b-%Y-%H:%M:%S.%f")
        file_path = "recordings/" + str(time_now) + ".yaml"



        # Save the data to a YAML file
        with open(file_path, 'w') as file:
            yaml.dump(data, file)

        print(f"Data saved to '{file_path}'.")

data = {
    'timestamp1': [
        'x position',
        'y position',
        'z position',
        'yaw',
        'roll',
        'pitch',
        'joint angles',
        'joint velocities'
    ]
}


if __name__ == "__main__":
    recorder = WaypointsRecorder()


    rospy.spin()
