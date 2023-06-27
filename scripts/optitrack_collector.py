#!/usr/bin/env python

import sys 
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool
from hri_optitrack.msg import CreateObject  # Import the CreateObject message
from moveit_msgs.msg import CollisionObject, PlanningScene  # Import the CollisionObject message
from shape_msgs.msg import SolidPrimitive  # Import the SolidPrimitive message
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class OptitrackCollector:
    def __init__(self):

        # TODO: 
        # Find transform between robot base and optitrack 
        # Transform found object to base and optitrack 
        # I don't deal with transformations :) 

        # Global dictionary to store measured values
        self.pose_dict = self.init_pose_dict()

        # Global flag to enable/disable collection
        self.collection_enabled = False
        self.collection_completed = False
        self.kalipen_reciv = False
        self.num_added_objects = 0

        # Initialize the ROS node
        rospy.init_node('optitrack_pose_collector', log_level=rospy.DEBUG)

        self.rate = rospy.Rate(30)

       # Initialize all the subscribers
        self.optitrack_det_sub_name = "/kdno/INST_POINT"
        self.enable_collection_sub_name = "/kalipen/joy"
        self.collision_object_sub_name = "/collision_object_topic"
        self.create_object_sub_name = "/create_object"
        self.planning_scene_sub_name = "/planning_scene"
        self._init_publishers()
        self._init_subscribers()
        self._init_services()

        #self.MODE = "points"
        self.MODE = "plane"

        # Init moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "manipulator"
        #self.robot = moveit_commander.RobotCommander(group_name)
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.scene = moveit_commander.PlanningSceneInterface() # --> ns? 
    
    #### Initializers ####
    def _init_subscribers(self):

        # Subscribe to the geometry_msgs/Pose topic
        rospy.Subscriber(self.optitrack_det_sub_name, PoseStamped, self.pose_callback)

        # Subscribe to the enable_collection flag topic
        rospy.Subscriber(self.enable_collection_sub_name, Joy, self.enable_collection_callback)

        rospy.Subscriber(self.planning_scene_sub_name, PlanningScene, self.planning_scene_callback)

        # Subscribe to the moveit_msgs/CollisionObject topic
        #rospy.Subscriber(self.collision_object_sub_name, CollisionObject, self.collision_object_callback)

        # Subscribe to the /create_object topic
        #rospy.Subscriber(self.create_object_sub_name, CreateObject, self.create_object_callback)

    def _init_publishers(self): 
        self.co_pub = rospy.Publisher("/collision_object", CollisionObject, queue_size=1)

    def _init_services(self): 

        self.remove_obj_srv = rospy.Service('remove_objects', Trigger, self.remove_objects_srv_handler)

    def init_pose_dict(self): 
        return {'position_x': [], 'position_y': [], 'position_z': []}
    

    def planning_scene_callback(self, msg): 

        print(msg)

    #### Topic callbacks ####
    def pose_callback(self, msg):
        if self.collection_enabled and self.kalipen_reciv:
            # Append the measured values to the corresponding lists in the pose_dict
            self.pose_dict['position_x'].append(msg.pose.position.x)
            self.pose_dict['position_y'].append(msg.pose.position.y)
            self.pose_dict['position_z'].append(msg.pose.position.z)
            # TODO: Add orientation
            if self.collection_completed:
                if self.MODE == "plane":
                    self.create_object_plane(self.pose_dict)
                if self.MODE == "point": 
                    self.create_plane_points(self.pose_dict)
            

    def enable_collection_callback(self, msg):

        self.kalipen_reciv = True
        rospy.logdebug("Recieved ros_kalipen msg")
        if msg.buttons[0] == 1: 
            self.collection_enabled = True
        elif msg.buttons[0] == 0: 
            self.collection_completed = True

    #### Service handlers ####
    def remove_objects_srv_handler(self, req):
        # Process the request and generate a response
        response = TriggerResponse()
        # ... process the request and populate the response

        return response

    #### Helper methods ####
    def create_object_plane(self, pose_dict):

        # Find the minimum and maximum values of the position measurements
        min_x = round(min(pose_dict['position_x']), 4)
        max_x = round(max(pose_dict['position_x']), 4)
        min_y = round(min(pose_dict['position_y']), 4)
        max_y = round(max(pose_dict['position_y']), 4)
        min_z = round(min(pose_dict['position_z']), 4)
        max_z = round(max(pose_dict['position_z']), 4)
        origin = Vector3()
        origin.x = round((min_x + max_x) / 2, 3)
        origin.y = round((min_y + max_y) / 2, 3)
        origin.z = round((min_z + max_z) / 2, 3)

        # Print the minimum and maximum values
        rospy.logdebug("Minimum Position (X, Y, Z): ({}, {}, {})".format(min_x, min_y, min_z))
        rospy.logdebug("Maximum Position (X, Y, Z): ({}, {}, {})".format(max_x, max_y, max_z))
        rospy.logdebug("Origin of the collision primitive is: (X, Y, Z) {} {} {}".format(origin.x, origin.y, origin.z))
        rospy.logdebug("Dimensions of the (X, Y, Z): ")
        # Process the received data as needed
        # Call required service for object creation 
        #origin_pose = msg.origin
        #primitive_type = msg.primitive_type.data
        rospy.logdebug("Calling create obstacle function: {} {} {}".format(origin.x, origin.y, origin.z))

        l, w, h = self.calculate_box_dim((min_x, min_y, min_z), (max_x, max_y, max_z))

        self.add_object_to_planning_scene(origin, (l, w, h))


    def calculate_box_dim(self, min_coords, max_coords):
        """
        Calculates the dimensions of a 3D box based on the coordinates
        of its minimum and maximum vertices.

        Args:
            min_coords (tuple or list): Tuple or list containing the (x, y, z) coordinates of the minimum vertex.
            max_coords (tuple or list): Tuple or list containing the (x, y, z) coordinates of the maximum vertex.

        Returns:
            tuple: A tuple containing the dimensions (length, width, height) of the 3D box.
        """
        length = max_coords[0] - min_coords[0]
        width = max_coords[1] - min_coords[1]
        height = max_coords[2] - min_coords[2]
        return length, width, height

    def make_box(self, name, pose, size): 
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives = [box]
        co.primitive_poses = [pose.pose]
        return co 


    def add_object_to_planning_scene(self, origin, size): 

        box_pose = PoseStamped()
        box_pose.header.frame_id = "world_opti"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = origin.x
        box_pose.pose.position.y = origin.y
        box_pose.pose.position.z = origin.z       
        box_name = "obstacle{}".format(self.num_added_objects+1)
        self.num_added_objects += 1
        self.scene.add_box(box_name, box_pose, size)
        #co_ = self.make_box(box_name, box_pose, size)
        #self.co_pub.publish(co_)


    def create_plane_points(self, p1, p2, p3): 

        # TODO: Specify rectangle by 3 points 
        pass 

    #### Execution method ####
    def run(self):
        # Run the node until it is shutdown
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.collection_completed:
                attached_objects = self.scene.get_attached_objects()
                objects = self.scene.get_objects()
                rospy.loginfo("Attached objects are: {}".format(attached_objects))
                rospy.loginfo("Objects are: {}".format(objects))
                self.collection_completed = False
                self.collection_enabled = False

if __name__ == '__main__':
    collector = OptitrackCollector()
    collector.run()
