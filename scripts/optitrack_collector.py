#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from your_package_name.msg import CreateObject  # Import the CreateObject message

class OptitrackCollector:
    def __init__(self):
        # Global dictionary to store measured values
        self.pose_dict = {
            'position_x': [],
            'position_y': [],
            'position_z': [],
            'orientation_x': [],
            'orientation_y': [],
            'orientation_z': [],
            'orientation_w': []
        }

        # Global flag to enable/disable collection
        self.collection_enabled = False

        # Initialize the ROS node
        rospy.init_node('pose_collector')

        self.rate = rospy.Rate(10)

        # Subscribe to the geometry_msgs/Pose topic
        rospy.Subscriber('your_pose_topic', Pose, self.pose_callback)

        # Subscribe to the enable_collection flag topic
        rospy.Subscriber('/enable_collection', Bool, self.enable_collection_callback)

    def pose_callback(self, msg):
        if self.collection_enabled:
            # Append the measured values to the corresponding lists in the pose_dict
            self.pose_dict['position_x'].append(msg.position.x)
            self.pose_dict['position_y'].append(msg.position.y)
            self.pose_dict['position_z'].append(msg.position.z)
            self.pose_dict['orientation_x'].append(msg.orientation.x)
            self.pose_dict['orientation_y'].append(msg.orientation.y)
            self.pose_dict['orientation_z'].append(msg.orientation.z)
            self.pose_dict['orientation_w'].append(msg.orientation.w)

    def enable_collection_callback(self, msg):
        self.collection_enabled = msg.data

    def create_object_callback(self, msg):
        # Extract the information from the CreateObject message
        origin_pose = msg.origin
        primitive_type = msg.primitive_type.data

        # Find the minimum and maximum values of the position measurements
        min_position_x = min(self.pose_dict['position_x'])
        max_position_x = max(self.pose_dict['position_x'])
        min_position_y = min(self.pose_dict['position_y'])
        max_position_y = max(self.pose_dict['position_y'])
        min_position_z = min(self.pose_dict['position_z'])
        max_position_z = max(self.pose_dict['position_z'])

        # Print the minimum and maximum values
        print("Minimum Position (X, Y, Z): ({}, {}, {})".format(min_position_x, min_position_y, min_position_z))
        print("Maximum Position (X, Y, Z): ({}, {}, {})".format(max_position_x, max_position_y, max_position_z))

        # Process the received data as needed
        # ...

    def run(self):
        # Run the node until it is shutdown
        while not rospy.is_shutdown():
            self.rate.sleep()

        # Print the collected measurements
        print(self.pose_dict)

if __name__ == '__main__':
    collector = OptitrackCollector()
    collector.run()
