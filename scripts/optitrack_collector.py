#!/usr/bin/env python

import sys 
import rospy
import copy
import moveit_commander
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool
from hri_optitrack.msg import CreateObject  # Import the CreateObject message
from moveit_msgs.msg import CollisionObject, PlanningScene  # Import the CollisionObject message
from shape_msgs.msg import SolidPrimitive  # Import the SolidPrimitive message
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker
from calc_utils import get_RotX, get_RotY, get_RotZ
from scipy.spatial.transform import Rotation as R

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
        self.pose_reciv = False
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

        self.MODE = "points"
        #self.MODE = "plane"
        # Click counter
        self.click = 0
        self.A, self.B, self.C = np.array([]), np.array([]), np.array([])

        # Init moveit_commander
        self.moveit = False
        if self.moveit:
            moveit_commander.roscpp_initialize(sys.argv)
            group_name = "manipulator"
            #self.robot = moveit_commander.RobotCommander(group_name)
            self.group = moveit_commander.MoveGroupCommander(group_name)
            self.scene = moveit_commander.PlanningSceneInterface() # --> ns? 


        self.v1, self.v2, self.v3 = Marker(), Marker(), Marker()


        # TODO: Add visualization: 

    
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
        self.pose_reciv = True
        if self.collection_enabled and self.kalipen_reciv:
            # Append the measured values to the corresponding lists in the pose_dict
            self.pose_dict['position_x'].append(msg.pose.position.x)
            self.pose_dict['position_y'].append(msg.pose.position.y)
            self.pose_dict['position_z'].append(msg.pose.position.z)
            # TODO: Add orientation
            if self.collection_completed:
                if self.MODE == "plane":
                    origin, (l, w, h) = self.create_object_plane(self.pose_dict)
                    self.add_box_to_planning_scene(origin, (l, w, h))
                if self.MODE == "points": 
                    self.create_plane_points(self.pose_dict)
                    if self.click == 4:
                        # Plane representations 
                        # https://ocw.mit.edu/ans7870/18/18.013a/textbook/HTML/chapter05/section04.html
                        # Should add origin: 
                        # origin, (l, w, h) = self.create_plane_points(self.pose_dict)
                        self.add_plane_to_planning_scene(origin, n, (l, w))
                if self.MODE == "ransac": 
                    origin, (l, w, h) = self.create_ransac_plane(self.pose_dict)
                    #TODO: Add O3D RANSAC 
            

    def enable_collection_callback(self, msg):

        self.kalipen_reciv = True
        rospy.logdebug("Recieved ros_kalipen msg")
        if msg.buttons[0] == 1: 
            self.collection_enabled = True
        elif msg.buttons[0] == 0: 
            self.collection_completed = True
            self.click += 1

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

        return origin, (l, w, h)
    
    def create_plane_points(self, pose_dict): 
        """
        Creates a plane from three points.

        Args:
            pose_dict (dict): Dictionary containing the pose data from the Optitrack system.

        Returns:
            origin (Vector3): The origin of the plane.
            (l, w, h) (tuple): The dimensions of the plane.
        """
                
        if self.click == 1:
            self.B = np.array([np.average(pose_dict['position_x']), np.average(pose_dict['position_y']), np.average(pose_dict['position_z'])])
            self.o = copy.deepcopy(self.B)
            rospy.logdebug_once("First click, first point: {}".format(self.B))
        if self.click == 2:
            self.A = np.array([np.average(pose_dict['position_x']), np.average(pose_dict['position_y']), np.average(pose_dict['position_z'])])
            self.a = copy.deepcopy(self.A)
            rospy.logdebug_once("Second click, second point: {}".format(self.A))
        if self.click == 3: 
            self.C = np.array([np.average(pose_dict['position_x']), np.average(pose_dict['position_y']), np.average(pose_dict['position_z'])])
            self.b = copy.deepcopy(self.C)
            rospy.logdebug_once("Third click, third point: {}".format(self.C))


        width = self.get_euclidean(self.o, self.a)
        length = self.get_euclidean(self.o, self.b)

        self.v1_ = self.a - self.o
        self.v2_ = self.b - self.o

        if self.visualize: 
            self.v1 = self.create_vector_marker(self.o, self.a)
            self.v1_unit = self.get_unit_vector(self.v1_)
            self.v1_quat = self.get_quaternion(self.v1_unit)
            self.v2 = self.create_vector_marker(self.o, self.b)
            self.v2_unit = self.get_unit_vector(self.v2_)
            self.v2_quat = self.get_quaternion(self.v2_unit)


        # TODO: Return origin and normal of the plane 
        # How to find origin of the plane (orientation of the plane)

    def get_plane_eq(self): 
        # Get the normal of the plane
        p_x = self.create_normalized_vector(self.o, self.a)
        p_y = self.create_normalized_vector(self.o, self.b)
        p_z = self.create_normalized_vector(np.cross(p_x, p_y))
        T_plane = np.matrix([p_x.T, p_y.T, p_z.T, self.o.T])
        print(T_plane)

        # Get the normal of the plane
        #n = np.cross(self.B - self.A, self.C - self.B)
        # Get the equation of the plane ax + by + cz + d = 0
        #d = np.dot(-n, self.A)
        return n, d
    
    def get_unit_vector(self, vector): 
        return vector / np.linalg.norm(vector)
    
    def create_cs(self, vector): 
        a = vector 
        b = self.matmul(get_RotZ(-90), a)
        c = self.matmul(a, b)
        R_ = np.matrix([a, b, c])
        r = R.from_matrix(R_)
        return r.as_quat()
    
    def create_vector_marker(self, origin, quaternion):
        marker = Marker()
        marker.header.frame_id = "n_thorax"
        marker.header.stamp = rospy.Time().now()
        marker.ns = "arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = origin.x
        marker.pose.position.y = origin.y
        marker.pose.position.z = origin.z
        # How to transform x,y,z values to the orientation 
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        return marker

    def normalize_vector(self, vector):
        """
        Normalizes a vector to unit length.

        Args:
            vector (list or tuple): List or tuple representing the vector.

        Returns:
            list: A list representing the normalized vector.
        """
        magnitude = np.sqrt(sum(component ** 2 for component in vector))
        normalized_vector = np.array([component / magnitude for component in vector])

        return normalized_vector
    
    def create_normalized_vector(self, point1, point2):
        v = point2 - point1
        v = self.normalize_vector(v)
        return v


    def create_ransac_plane(self, pose_dict):
        pass

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
        width  = max_coords[1] - min_coords[1]
        height = max_coords[2] - min_coords[2]
        return length, width, height

    def add_box_to_planning_scene(self, origin, size): 

        box_pose = PoseStamped()
        box_pose.header.frame_id = "world_opti"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = origin.x
        box_pose.pose.position.y = origin.y
        box_pose.pose.position.z = origin.z       
        box_name = "obstacle{}".format(self.num_added_objects+1)
        self.num_added_objects += 1
        self.scene.add_box(box_name, box_pose, size)

    def get_euclidean(self, p1, p2):
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)


    def add_plane_to_planning_scene(self, n, d): 
        # TODO: 
        #  Create box for planning scene
        pass

    #### Execution method ####
    def run(self):
        # Run the node until it is shutdown
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.collection_completed and self.MODE == "plane":
                attached_objects = self.scene.get_attached_objects()
                objects = self.scene.get_objects()
                rospy.loginfo("Attached objects are: {}".format(attached_objects))
                rospy.loginfo("Objects are: {}".format(objects))
                self.collection_completed = False
                self.collection_enabled = False

            if self.collection_completed and self.MODE == "points": 
                if self.click == 4: 
                    n, d = self.get_plane_eq()
                    nn = self.normalize_vector(n)
                    rospy.loginfo_once("Normal is: {}".format(n))
                    rospy.loginfo_once("Normalized vector is: {}".format(nn))
                    rospy.loginfo_once("d is: {}".format(d))
                    rospy.loginfo_once("Adding defined ")
                    self.collection_completed = False
                    self.collection_enabled = False


if __name__ == '__main__':
    collector = OptitrackCollector()
    collector.run()
