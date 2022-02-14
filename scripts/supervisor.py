#!/usr/bin/env python3

from enum import Enum

import rospy
from asl_turtlebot.msg import DetectedObject
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker
from utils.utils import wrapToPi
import tf
import numpy as np

class Mode(Enum):
    """State machine modes. Feel free to change."""
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6


class SupervisorParams:

    def __init__(self, verbose=False):
        # If sim is True (i.e. using gazebo), we want to subscribe to
        # /gazebo/model_states. Otherwise, we will use a TF lookup.
        self.use_gazebo = rospy.get_param("sim")

        # How is nav_cmd being decided -- human manually setting it, or rviz
        self.rviz = rospy.get_param("rviz")

        # If using gmapping, we will have a map frame. Otherwise, it will be odom frame.
        self.mapping = rospy.get_param("map")

        # Threshold at which we consider the robot at a location
        self.pos_eps = rospy.get_param("~pos_eps", 0.2)
        self.theta_eps = rospy.get_param("~theta_eps", 0.5)

        # Time to stop at a stop sign
        self.stop_time = rospy.get_param("~stop_time", 3.)

        # Minimum distance from a stop sign to obey it
        self.stop_min_dist = rospy.get_param("~stop_min_dist", 0.5)

        # Time taken to cross an intersection
        self.crossing_time = rospy.get_param("~crossing_time", 3.)

        if verbose:
            print("SupervisorParams:")
            print("    use_gazebo = {}".format(self.use_gazebo))
            print("    rviz = {}".format(self.rviz))
            print("    mapping = {}".format(self.mapping))
            print("    pos_eps, theta_eps = {}, {}".format(self.pos_eps, self.theta_eps))
            print("    stop_time, stop_min_dist, crossing_time = {}, {}, {}".format(self.stop_time, self.stop_min_dist, self.crossing_time))


class Supervisor:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.params = SupervisorParams(verbose=True)

        # Current state
        self.x = 0
        self.y = 0
        self.theta = 0

        # Goal state
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        # Current mode
        self.mode = Mode.IDLE
        self.prev_mode = None  # For printing purposes
        
        # Marker Count
        self.marker_count = 0

        ########## PUBLISHERS ##########

        # Command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)

        # Command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.objects_pub = rospy.Publisher('/detected_objects', String)
        
        self.fire_pub = rospy.Publisher("/fire",String)
        
        self.marker_pub = rospy.Publisher("/object_marker", Marker, queue_size=10)
        
        # Final Project
        self.queue = [] #list of goal positions in sequential order
        #self.object_locations = {'tv':(3.2, 0.5, np.pi), 'airplane': (2.5, 0.5, np.pi), 'stop_sign': (0.25, 1.65, 0)}
        self.object_locations = dict() #Store object locations as dictionary
        self.object_distances = dict() #Store corresponding distances
        
        self.reached_goal = False # general flage for reaching goal
        self.explored  = False
        self.picked_up = False

        ########## SUBSCRIBERS ##########

        # Stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.detected_object_callback)
        #rospy.Subscriber('/detector/sports_ball', DetectedObject, self.detected_object_callback)
        rospy.Subscriber('/detector/cup', DetectedObject, self.detected_object_callback)
        rospy.Subscriber("/detector/fire_hydrant",DetectedObject, self.detected_object_callback)
        #rospy.Subscriber("/detector/umbrella",DetectedObject, self.detected_object_callback)
        #rospy.Subscriber("/detector/airplane",DetectedObject, self.detected_object_callback)
        
        # High-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)

        rospy.Subscriber('/rescue_items', String, self.rescue_items_callback)

        # If using gazebo, we have access to perfect state
        if self.params.use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.trans_listener = tf.TransformListener()

        # If using rviz, we can subscribe to nav goal click
        if self.params.rviz:
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        else:
            self.x_g, self.y_g, self.theta_g = 1.5, -4., 0.
            self.mode = Mode.NAV
        

    ########## SUBSCRIBER CALLBACKS ##########
    
    def rescue_items_callback(self, msg):
    	print(f"Received items to rescue: {msg.data}")
    	objects = msg.data.split()
    	for obj in objects:
    	    self.queue.append(self.object_locations[obj])
    	self.explored = True
        

    def detected_object_callback(self,data):

        if data.confidence > 0.5 and data.distance < 0.5:
            x_o=self.x
            y_o=self.y
            
            if(data.name == "stop_sign" and self.mode == Mode.NAV and data.confidence > 0.5):
                print("STOP SIGN DISTANCE, Theta", data.distance, data.thetaleft,data.thetaright)
                # distance of the stop sign
                dist = data.distance
                # if close enough and in nav mode, stop
                if dist > 0 and dist < self.params.stop_min_dist and self.mode == Mode.NAV:
                    self.init_stop_sign()
                        
            if(data.name == "fire_hydrant" and self.mode == Mode.NAV and data.confidence > 0.5):
                self.fire_pub.publish("FIRE FIRE FIRE!!!")

            if data.name not in self.object_locations.keys():
                self.object_locations[data.name] = (x_o, y_o, self.theta)
                self.objects_pub.publish(String(data.name))
                self.object_distances[data.name] = data.distance
                print("object location:",self.x,self.y,self.theta,data.name)
               
                object_marker = Marker()
                object_marker.header.frame_id = "map"
                object_marker.header.stamp = rospy.Time()
                object_marker.id = self.marker_count
                self.marker_count += 1
                object_marker.type = 2 # sphere
    		
                object_marker.pose.position.x = x_o 
                object_marker.pose.position.y = y_o 
                object_marker.pose.position.z = 0
                object_marker.color.b = 1.0
                object_marker.color.g = 0.0
                object_marker.color.r = 0.0
                object_marker.color.a = 1.0
                object_marker.scale.x = 0.2
                object_marker.scale.y = 0.2
                object_marker.scale.z = 0.2
                self.marker_pub.publish(object_marker)
            elif data.distance < self.object_distances[data.name]:
                print('Updating object location: ', data.name)
                self.object_locations[data.name] = (x_o, y_o, self.theta)
                self.object_distances[data.name] = data.distance


    def gazebo_callback(self, msg):
        if "turtlebot3_burger" not in msg.name:
            return

        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if self.params.mapping else "/odom"
        print("Rviz command received!")

        try:
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (nav_pose_origin.pose.orientation.x,
                          nav_pose_origin.pose.orientation.y,
                          nav_pose_origin.pose.orientation.z,
                          nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < self.params.stop_min_dist and self.mode == Mode.NAV:
            self.init_stop_sign()


    ########## STATE MACHINE ACTIONS ##########

    ########## Code starts here ##########
    # Feel free to change the code here. You may or may not find these functions
    # useful. There is no single "correct implementation".

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.nav_goal_publisher.publish(nav_g_msg)
        
    def nav_to_home(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = 3.2
        nav_g_msg.y = 1.6
        nav_g_msg.theta = 0
        print("Going Home")

        self.nav_goal_publisher.publish(nav_g_msg)
        if self.close_to(self.x_g, self.y_g, self.theta_g):
        	self.mode = Mode.IDLE
    

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self, x, y, theta):
        """ checks if the robot is at a pose within some threshold """
        
        return abs(x - self.x) < self.params.pos_eps and \
               abs(y - self.y) < self.params.pos_eps and \
               abs(wrapToPi(theta - self.theta)) < self.params.theta_eps

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return self.mode == Mode.STOP and \
               rospy.get_rostime() - self.stop_sign_start > rospy.Duration.from_sec(self.params.stop_time)

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return self.mode == Mode.CROSS and \
               rospy.get_rostime() - self.cross_start > rospy.Duration.from_sec(self.params.crossing_time)
               
    

    ########## Code ends here ##########


    ########## STATE MACHINE LOOP ##########

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if not self.params.use_gazebo:
            try:
                origin_frame = "/map" if self.params.mapping else "/odom"
                translation, rotation = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x, self.y = translation[0], translation[1]
                self.theta = tf.transformations.euler_from_quaternion(rotation)[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # logs the current mode
        if self.prev_mode != self.mode:
            rospy.loginfo("Current mode: %s", self.mode)
            self.prev_mode = self.mode


        ########## Code starts here ##########
        # TODO: Currently the state machine will just go to the pose without stopping
        #       at the stop sign.
        
        
        if self.mode == Mode.IDLE:
            # Send zero velocity
            self.stay_idle()
            if self.explored and not self.picked_up:
                self.mode = Mode.NAV
                self.x_g, self.y_g, self.theta_g = self.queue.pop(0)

        elif self.mode == Mode.POSE:
            # Moving towards a desired pose
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # At a stop sign
            self.stay_idle()
            if self.has_stopped():
                self.init_crossing()

        elif self.mode == Mode.CROSS:
            # Crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                if not self.explored:
                    self.nav_to_pose()
                else:
                    # Rescue
                    print("Picking up Item")
                    rescue_start = rospy.get_rostime()
                    while(rospy.get_rostime() - rescue_start) < rospy.Duration.from_sec(3): #Waiting for Rescue 
                    	self.stay_idle()
                    	
                    if len(self.queue) > 0:
                        print("Navigating to next item")
                        self.x_g, self.y_g, self.theta_g = self.queue.pop(0)
                        self.nav_to_pose()
                        
                    else:
                        print("Picked up all items")
                        self.picked_up = True
                        self.nav_to_home()
            else:
            	self.nav_to_pose()
        else:
            raise Exception("This mode is not supported: {}".format(str(self.mode)))

        ############ Code ends here ############

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
    
