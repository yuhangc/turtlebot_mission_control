#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String, Bool
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
import pdb

def pose_to_xyth(pose):
    th = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w))[2]
    return [pose.position.x, pose.position.y, th]


class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.trans_listener = tf.TransformListener()
        self.trans_broad = tf.TransformBroadcaster()

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"
        rospy.Subscriber('/mission', Int32MultiArray, self.mission_callback)    # mission order
        rospy.Subscriber('/mode', Bool, self.mode_callback)
        rospy.Subscriber('/turtlebot_controller/goal_reached', Bool, self.nav_callback)

        self.nav_goal_pub = rospy.Publisher('/turtlebot_controller/nav_goal', Float32MultiArray, queue_size=10)
        self.ctrl_mode_pub = rospy.Publisher('/turtlebot_controller/ctrl_mode', String, queue_size=10)
        self.vel_goal_pub = rospy.Publisher('/turtlebot_controller/velocity_goal', Float32MultiArray, queue_size=10)
        self.success_pub = rospy.Publisher('/success', Bool, queue_size=10)

        self.waypoint_locations = {}    # dictionary that caches the most updated locations of each mission waypoint
        self.waypoint_offset = PoseStamped()
        self.waypoint_offset.pose.position.z = .4    # waypoint is located 40cm in front of the AprilTag, facing it
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.waypoint_offset.pose.orientation.x = quat[0]
        self.waypoint_offset.pose.orientation.y = quat[1]
        self.waypoint_offset.pose.orientation.z = quat[2]
        self.waypoint_offset.pose.orientation.w = quat[3]
        self.mission = []
        self.auto_mode = False
        self.waypoints_found = []
        self.num_waypoints_found = 0
        self.nav_goal = Float32MultiArray()
        self.nav_goal.data = np.array([0,0,0])
        self.total_tags = 0
        self.tags_visited = 0

    # Select mission mode
    def mode_callback(self, mode):
        if (mode.data and not self.auto_mode): # if entering auto mode
            self.auto_mode = True
            self.init_auto_nav()



    def init_auto_nav(self):
        print "In auto mode"
        if not (self.num_waypoints_found == self.total_tags):
            print "Warning: not all tag locations are known"
        # init nav
        if self.mission[0] in self.waypoint_locations:
            self.nav_goal.data = pose_to_xyth(self.waypoint_locations[self.mission[self.tags_visited]].pose)

        else:
            print "First tag not found"

        # turn on the controller
        ctrl_mode = String()
        ctrl_mode.data = 'closed'
        self.ctrl_mode_pub.publish(ctrl_mode)

    # Subscribe to mission order
    def mission_callback(self, mission):
        self.mission = mission.data
        if not self.waypoints_found: # init
            self.total_tags = len(self.mission)
            self.waypoints_found = [False]*self.total_tags

    # Choose destination from map    
    def rviz_goal_callback(self, msg):
        if not self.auto_mode:
            self.nav_goal.data = pose_to_xyth(msg.pose)    # example usage of the function pose_to_xyth (defined above)
            # turn on the controller
            ctrl_mode = String()
            ctrl_mode.data = 'closed'
            self.ctrl_mode_pub.publish(ctrl_mode)
            
            # publish nav_goal
            self.nav_goal_pub.publish(self.nav_goal)


    # Create waypoints to visit, based on tag locations
    def update_waypoints(self):
        for tag_number in self.mission:
            try:
                self.waypoint_offset.header.frame_id = "/tag_{0}".format(tag_number)
                self.waypoint_locations[tag_number] = self.trans_listener.transformPose("/map", self.waypoint_offset)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def check_waypoints_found(self):
        if self.mission:  # if mission is published
            waypoints_found_temp = [False]*len(self.mission)
            changes = False #if any new waypoints found since last update
            for i in range(self.total_tags): # loop through all mission goals
                waypoints_found_temp[i] = (self.mission[i] in self.waypoint_locations)
                if not (waypoints_found_temp[i] == self.waypoints_found[i]): # if waypoint is found for the first time
                    print "Found Tag", self.mission[i]
                    self.num_waypoints_found += 1
                    self.waypoints_found[i] = waypoints_found_temp[i]
                    if self.num_waypoints_found == self.total_tags:
                        print "All required tags found"
                        print "Switching to auto mode"
                        self.auto_mode = True
                        self.init_auto_nav()
                    else:
                        print "There are", (self.total_tags-self.num_waypoints_found), "tag(s) still not found"
            

    def nav_callback(self, near):
        if near.data and self.auto_mode:
            self.tags_visited += 1
            print "next tag number: ", self.tags_visited

            if self.tags_visited >= self.total_tags:
                # FIXME self.nav_goal.data = np.array([0,0,0])
                ctrl_mode = String()
                ctrl_mode.data = 'open'
                self.ctrl_mode_pub.publish(ctrl_mode)

                # Publish success
                msg = Bool()
                msg.data = True
                self.success_pub.publish(msg)
                print "Success"

                vel_goal = Float32MultiArray()
                vel_goal.data = [0, 0]
                self.vel_goal_pub.publish(vel_goal)
            else:
                self.nav_goal.data = pose_to_xyth(self.waypoint_locations[self.mission[self.tags_visited]].pose)
            self.nav_goal_pub.publish(self.nav_goal)
            

    def run(self):
        rate = rospy.Rate(1) # 1 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            self.update_waypoints()
            self.check_waypoints_found()

            
            if self.auto_mode:
                self.nav_goal_pub.publish(self.nav_goal)

            # FILL ME IN!

            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
