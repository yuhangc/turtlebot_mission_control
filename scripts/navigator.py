#!/usr/bin/env python

# FILL ME IN!

# !/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
import matplotlib.pyplot as plt
import tf
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from astar import AStar, StochOccupancyGrid2D, StochOccupancyGrid2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Navigator:
    def __init__(self):
        rospy.init_node('navigator', anonymous=True)

        self.plan_resolution = 0.25
        self.plan_horizon = 15

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0, 0]
        self.map_probs = []
        self.occupancy = None

        self.nav_sp = None
        self.nav_sp_changed = False
        self.nav_sp_changed_thresh = 0.1  # to be changed
        self.waypoint_reached = False
        self.nav_sp_reached = False

        self.path = None
        self.counter = 0

        self.flag = 0

        # desired average navigation speed
        self.vel_nav = 0.20
        self.t_last_waypoint = 0.0
        self.t_expire = 0.0

        self.trans_listener = tf.TransformListener()

        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.map_md_callback)
        rospy.Subscriber("/turtlebot_controller/nav_goal", Float32MultiArray, self.nav_sp_callback)
        rospy.Subscriber("/turtlebot_controller/ctrl_feedback", Bool, self.ctrl_callback)

        self.pose_sp_pub = rospy.Publisher('/turtlebot_controller/position_goal', Float32MultiArray, queue_size=10)
        self.nav_path_pub = rospy.Publisher('/turtlebot_controller/path_goal', Path, queue_size=10)
        self.goal_reached_pub = rospy.Publisher('/turtlebot_controller/goal_reached', Bool, queue_size=10)
        self.tag_ahead_pub = rospy.Publisher('/turtlebot_controller/tag_ahead', Bool, queue_size=10)

    def map_md_callback(self, msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x, msg.origin.position.y)

    def map_callback(self, msg):
        self.map_probs = msg.data
        if self.map_width > 0 and self.map_height > 0 and len(self.map_probs) > 0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  int(self.plan_resolution / self.map_resolution * 2.25),
                                                  self.map_probs,
                                                  thresh=0.5)

    def nav_sp_callback(self, msg):
        if self.nav_sp:
            nav_sp_new = (msg.data[0], msg.data[1], msg.data[2])
            nav_sp_diff = np.array(nav_sp_new) - np.array(self.nav_sp)
            if np.linalg.norm(nav_sp_diff) > self.nav_sp_changed_thresh:
                self.nav_sp = nav_sp_new
                self.nav_sp_changed = True
                self.send_pose_sp()
            else:
                self.nav_sp_changed = False
        else:
            self.nav_sp = (msg.data[0], msg.data[1], msg.data[2])
            self.nav_sp_changed = True
            self.send_pose_sp()

    def ctrl_callback(self, msg):
        if msg.data:
            self.waypoint_reached = True
        else:
            self.waypoint_reached = False

    def path_clear(self):
        for state in self.path[1:-1]:
            if not self.occupancy.is_free(state):
                rospy.logerr("(%f, %f) is not free!", state[0], state[1])
                return False

        return True

    def send_waypoint(self):
        tag_ahead = Bool()
        if self.counter >= len(self.path):
            pose_sp = self.nav_sp
            tag_ahead.data = True
        else:
            if self.counter == len(self.path) - 1:

                # th_g = np.arctan2(self.nav_sp[1] - self.path[self.counter][1],
                #                   self.nav_sp[0] - self.path[self.counter][0])
                th_g = self.nav_sp[2]
                self.t_expire = 50.0

                tag_ahead.data = False
            else:
                tag_ahead.data = False
                th_g = np.arctan2(self.path[self.counter+1][1] - self.path[self.counter][1],
                                  self.path[self.counter+1][0] - self.path[self.counter][0])
                path_diff = np.array(self.path[self.counter+1]) - np.array(self.path[self.counter])
                self.t_expire = np.linalg.norm(path_diff) / self.vel_nav

                # allow for longer time for the first waypoint
                if self.counter == 0:
                    self.t_expire *= 5.0

            pose_sp = (self.path[self.counter][0], self.path[self.counter][1], th_g)

        msg = Float32MultiArray()
        msg.data = pose_sp
        self.pose_sp_pub.publish(msg)
        self.tag_ahead_pub.publish(tag_ahead)
        # reset the timer
        self.t_last_waypoint = rospy.get_time()

    def round_to_grid(self, x, y):
        x_grid = round(x / self.plan_resolution) * self.plan_resolution
        y_grid = round(y / self.plan_resolution) * self.plan_resolution
        return x_grid, y_grid

    def re_plan(self, robot_translation, robot_rotation):
        state_min = (-int(round(self.plan_horizon)), -int(round(self.plan_horizon)))
        state_max = (int(round(self.plan_horizon)), int(round(self.plan_horizon)))
        x_init = self.round_to_grid(robot_translation[0], robot_translation[1])
        x_goal = self.round_to_grid(self.nav_sp[0], self.nav_sp[1])

        # plan using astar
        astar = AStar(state_min, state_max, x_init, x_goal, self.occupancy, self.plan_resolution)

        rospy.loginfo("Computing navigation plan")
        if astar.solve():

            # a naive path follower we could use
            self.path = astar.path
            self.counter = 0
            self.send_waypoint()

            path_msg = Path()
            path_msg.header.frame_id = 'map'
            for state in astar.path:
                rospy.logerr("path (%f, %f)", state[0], state[1])
                pose_st = PoseStamped()
                pose_st.pose.position.x = state[0]
                pose_st.pose.position.y = state[1]
                pose_st.header.frame_id = 'map'
                path_msg.poses.append(pose_st)
            self.nav_path_pub.publish(path_msg)
            rospy.logerr("Have a solution!")

            self.nav_sp_reached = False

        else:
            rospy.logwarn("Could not find path")

    def send_pose_sp(self):
        try:
            (robot_translation, robot_rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint",
                                                                                      rospy.Time(0))
            self.has_robot_location = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            robot_translation = (0, 0, 0)
            robot_rotation = (0, 0, 0, 1)
            self.has_robot_location = False

        if self.occupancy and self.has_robot_location and self.nav_sp:
            if self.nav_sp_changed:
                rospy.logerr("nav goal changed!")
                self.re_plan(robot_translation, robot_rotation)
                msg = Bool()
                msg.data = False
                self.goal_reached_pub.publish(msg)
                self.nav_sp_changed = False
            else:
                # nav goal not changed
                # check if close enough to the goal - tune the distance here
                if self.counter < len(self.path):
                    # check if the path is clear
                    if not self.path_clear():
                        # replan if path is not clear
                        rospy.logerr("path not clear!")
                        self.re_plan(robot_translation, robot_rotation)

                    dt = rospy.get_time() - self.t_last_waypoint
                    if self.waypoint_reached or dt > self.t_expire:
                        rospy.logerr("move to next waypoint!")
                        self.counter += 1
                        self.send_waypoint()

                        self.waypoint_reached = False
                    else:
                        pass
                # initialize the path planner
                else:
                    # publish to message
                    # control directly to the goal
                    if self.waypoint_reached and not self.nav_sp_reached:
                        msg = Bool()
                        msg.data = True
                        self.goal_reached_pub.publish(msg)
                        self.nav_sp_reached = True
                    else:
                        self.send_waypoint()

    def run(self):
        rate = rospy.Rate(5)  # 3 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            self.send_pose_sp()
            rate.sleep()

if __name__ == '__main__':
    nav = Navigator()
    nav.run()
