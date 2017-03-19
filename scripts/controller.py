#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
import tf

class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        rospy.Subscriber('/turtlebot_controller/position_goal', Float32MultiArray, self.pos_goal_callback)
        rospy.Subscriber('/turtlebot_controller/velocity_goal', Float32MultiArray, self.vel_goal_callback)
        rospy.Subscriber('/turtlebot_controller/ctrl_mode', String, self.mode_callback)

        self.vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.reached_pub = rospy.Publisher('/turtlebot_controller/ctrl_feedback', Bool, queue_size=10)

        # a transformation listener
        self.trans_listener = tf.TransformListener()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.x_g = 1.0
        self.y_g = 1.0
        self.th_g = 0.0

        self.vel_g = np.array([0.0, 0.0])
        self.near_goal = False

        # threshold for singularity check
        self.alpha_thresh = 1e-3

        # control mode can be:
        # 'none' - no control
        # 'closed' - closed-loop control
        # 'open' - open-loop control
        self.ctrl_mode = 'none'

        # threshold for checking if goal reached
        self.goal_reached_thresh = 0.1
        self.theta_goal_thresh = 0.05

    def pos_goal_callback(self, msg):
        # update x_g, y_g, th_g
        self.x_g = msg.data[0]
        self.y_g = msg.data[1]
        self.th_g = msg.data[2]

    def vel_goal_callback(self, vel_msg):
        self.vel_g = vel_msg.data

    def mode_callback(self, mode_msg):
        self.ctrl_mode = mode_msg.data

    @staticmethod
    def wrapToPi(a):
        if isinstance(a, list):  # backwards compatibility for lists (distinct from np.array)
            return [(x + np.pi) % (2 * np.pi) - np.pi for x in a]
        return (a + np.pi) % (2 * np.pi) - np.pi

    def get_ctrl_output(self):
        # if in velocity control mode, just return the goal velocity
        if self.ctrl_mode == 'open':
            cmd_vel =Twist()
            cmd_vel.linear.x = self.vel_g[0]
            cmd_vel.angular.y = self.vel_g[1]

            goal_reached = Bool()
            goal_reached.data = False

            return cmd_vel, goal_reached

        try:
            # update position information
            (translation, rotation) = self.trans_listener.lookupTransform("/map",
                                                                          "/base_footprint", rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            self.theta = tf.transformations.euler_from_quaternion(rotation)[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Cannot localize robot!")

        # use self.x self.y and self.theta to compute the right control input here
        # compute rho and alpha, beta
        x_g = self.x_g
        y_g = self.y_g
        th_g = self.th_g

        # Distance to goal
        rho = np.sqrt((x_g - self.x) ** 2 + (y_g - self.y) ** 2)



        # Direction to goal
        ang = np.arctan2(y_g - self.y, x_g - self.x)

        # Difference between heading and ang
        alpha = self.wrapToPi(ang - self.theta)

        # Difference between heading and th_g
        beta = self.wrapToPi(th_g - self.theta)

        # check if near goal
        # Only reset if double the threshold from goal. Adds robustness to map jumping
        if not self.near_goal:
            self.near_goal = rho < self.goal_reached_thresh
        else:
            self.near_goal = 0.5*rho < self.goal_reached_thresh


        if not self.near_goal:
            V = 0.5-np.absolute(np.arctan(alpha*5.0)/(np.pi)) - max(0, -0.5/0.2*rho + 0.5)
            om = np.arctan(alpha*2.5)/(np.pi/2.0)
        else:
            V = 0
            om = np.arctan(beta*2.5)/(np.pi/2.0)


        # Apply saturation limits
        V = np.sign(V) * min(0.3, np.abs(V))
        om = np.sign(om) * min(1, np.abs(om))

        cmd = Twist()
        cmd.linear.x = V
        cmd.angular.z = om

        # check if goal is reached
        goal_reached = Bool()
        near_theta = np.absolute(self.theta - th_g) < self.theta_goal_thresh
        goal_reached.data = self.near_goal and near_theta
        #if np.linalg.norm(rho) < self.goal_reached_thresh and (np.absolute(self.theta - th_g) < self.theta_goal_thresh):
        #    goal_reached.data = True
        #else:
        #    goal_reached.data = False

        return cmd, goal_reached

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if self.ctrl_mode != 'none':
                ctrl_output, goal_reached = self.get_ctrl_output()
                self.vel_pub.publish(ctrl_output)
                self.reached_pub.publish(goal_reached)

            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
