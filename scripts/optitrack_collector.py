#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from hri_optitrack.msg import CreateObject  # Import the CreateObject message
from moveit_msgs.msg import CollisionObject  # Import the CollisionObject message
from shape_msgs.msg import SolidPrimitive  # Import the SolidPrimitive message
from sensor_msgs.msg import Joy


class OptitrackCollector:
    def __init__(self):
        # Global dictionary to store measured values
        self.pose_dict = self.init_pose_dict()

        # Global flag to enable/disable collection
        self.collection_enabled = False
        self.collection_completed = False
        self.kalipen_reciv = True

        # Initialize the ROS node
        rospy.init_node('optitrack_pose_collector', log_level=rospy.DEBUG)

        self.rate = rospy.Rate(30)

       # Initialize all the subscribers
        self.optitrack_det_sub_name = "/vrpn_client_node/INST_POINT/pose"
        self.enable_collection_sub_name = "/kalipen/joy"
        self.collision_object_sub_name = "/collision_object_topic"
        self.create_object_sub_name = "/create_object"
        self._init_subscribers()

    def _init_subscribers(self):

        # Subscribe to the geometry_msgs/Pose topic
        rospy.Subscriber(self.optitrack_det_sub_name, PoseStamped, self.pose_callback)

        # Subscribe to the enable_collection flag topic
        rospy.Subscriber(self.enable_collection_sub_name, Joy, self.enable_collection_callback)

        # Subscribe to the moveit_msgs/CollisionObject topic
        #rospy.Subscriber(self.collision_object_sub_name, CollisionObject, self.collision_object_callback)

        # Subscribe to the /create_object topic
        #rospy.Subscriber(self.create_object_sub_name, CreateObject, self.create_object_callback)

    def init_pose_dict(self): 
        return {'position_x': [], 'position_y': [], 'position_z': []}

    def pose_callback(self, msg):
        if self.collection_enabled and self.kalipen_reciv:
            # Append the measured values to the corresponding lists in the pose_dict
            self.pose_dict['position_x'].append(msg.pose.position.x)
            self.pose_dict['position_y'].append(msg.pose.position.y)
            self.pose_dict['position_z'].append(msg.pose.position.z)
            # TODO: Add orientation
            if self.collection_completed:
                self.create_object(self.pose_dict)
            

    def enable_collection_callback(self, msg):

        self.kalipen_reciv = True
        rospy.logdebug("Recieved ros_kalipen msg")
        if msg.buttons[0] == 1: 
            self.collection_enabled = True
        elif msg.buttons[0] == 0: 
            self.collection_completed = True

    def create_object(self, pose_dict):

        # Find the minimum and maximum values of the position measurements
        min_position_x = round(min(pose_dict['position_x']), 4)
        max_position_x = round(max(pose_dict['position_x']), 4)
        min_position_y = round(min(pose_dict['position_y']), 4)
        max_position_y = round(max(pose_dict['position_y']), 4)
        min_position_z = round(min(pose_dict['position_z']), 4)
        max_position_z = round(max(pose_dict['position_z']), 4)
        origin_x = round((min_position_x + max_position_x) / 2, 3)
        origin_y = round((min_position_y + max_position_y) / 2, 3)
        origin_z = round((min_position_z + max_position_z) / 2, 3)

        # Print the minimum and maximum values
        rospy.logdebug("Minimum Position (X, Y, Z): ({}, {}, {})".format(min_position_x, min_position_y, min_position_z))
        rospy.logdebug("Maximum Position (X, Y, Z): ({}, {}, {})".format(max_position_x, max_position_y, max_position_z))
        rospy.logdebug("Origin of the collision primitive is: (X, Y, Z) {} {} {}".format(origin_x, origin_y, origin_z))
        rospy.logdebug("Dimensions of the (X, Y, Z): ")
        # Process the received data as needed
        # Call required service for object creation 
        #origin_pose = msg.origin
        #primitive_type = msg.primitive_type.data
        rospy.logdebug("Calling create obstacle function: {} {} {}".format(origin_x, origin_y, origin_z))
        self.collection_completed = False
        self.collection_enabled = False



    def run(self):
        # Run the node until it is shutdown
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    collector = OptitrackCollector()
    collector.run()
