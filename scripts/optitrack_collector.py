#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from hri_optitrack.msg import CreateObject  # Import the CreateObject message
from moveit_msgs.msg import CollisionObject  # Import the CollisionObject message
from shape_msgs.msg import SolidPrimitive  # Import the SolidPrimitive message


class OptitrackCollector:
    def __init__(self):
        # Global dictionary to store measured values
        self.pose_dict = self.init_pose_dict()

        # Global flag to enable/disable collection
        self.collection_enabled = False

        # Initialize the ROS node
        rospy.init_node('optitrack_pose_collector')

        self.rate = rospy.Rate(10)

       # Initialize all the subscribers
        self._init_subscribers()
        self.optitrack_det_sub_name = "pose_topic"
        self.enable_collection_sub_name = "/kalipen/joy"
        self.collision_object_sub_name = "/collision_object_topic"
        self.create_object_sub_name = "/create_object"

    def _init_subscribers(self):

        # Subscribe to the geometry_msgs/Pose topic
        rospy.Subscriber(self.optitrack_det_sub_name, Pose, self.pose_callback)

        # Subscribe to the enable_collection flag topic
        rospy.Subscriber(self.enable_collection_sub_name, Joy, self.enable_collection_callback)

        # Subscribe to the moveit_msgs/CollisionObject topic
        rospy.Subscriber(self.collision_object_sub_name, CollisionObject, self.collision_object_callback)

        # Subscribe to the /create_object topic
        rospy.Subscriber(self.create_object_sub_name, CreateObject, self.create_object_callback)

    def init_pose_dict(self): 
        return {'position_x': [], 'position_y': [], 'position_z': []}

    def pose_callback(self, msg):
        if self.collection_enabled:
            self.pose_dict = self.init_pose_dict()
            # Append the measured values to the corresponding lists in the pose_dict
            self.pose_dict['position_x'].append(msg.position.x)
            self.pose_dict['position_y'].append(msg.position.y)
            self.pose_dict['position_z'].append(msg.position.z)
            # TODO: Add orientation
            

    def enable_collection_callback(self, msg):
        if msg.buttons[0] == 1: 
            self.collection_enabled = True
        if msg.buttons[0] == 0: 
            self.collection_enabled = False

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
        origin_x = (min_position_x + max_position_x)/2
        origin_y = (min_position_y + max_position_y)/2
        origin_z = (min_position_z + max_position_z)/2

        # Print the minimum and maximum values
        rospy.logdebug("Minimum Position (X, Y, Z): ({}, {}, {})".format(min_position_x, min_position_y, min_position_z))
        rospy.logdebug("Maximum Position (X, Y, Z): ({}, {}, {})".format(max_position_x, max_position_y, max_position_z))
        rospy.logdebug("Origin of the collision primitive is: (X, Y, Z) {}{}{}".format(origin_x, origin_y, origin_z))
        # Process the received data as needed
        # ...

    def run(self):
        # Run the node until it is shutdown
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    collector = OptitrackCollector()
    collector.run()
