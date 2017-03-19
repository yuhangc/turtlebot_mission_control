#!/usr/bin/env python

#

import rospy
from std_msgs.msg import Bool

if __name__ == '__main__':

    auto_mode = Bool()
    auto_mode.data = False
    pub = rospy.Publisher('/mode', Bool, queue_size=10)
    rospy.init_node('mode_publisher', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(auto_mode)
        rate.sleep()
