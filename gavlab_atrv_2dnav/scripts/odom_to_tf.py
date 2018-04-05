#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf
from math import radians

rospy.init_node('odom_to_tf')

pub = None

def callback(msg):

    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     "base_footprint",
                     "odom")

def main():
    global pub
    rospy.init_node('odom_to_tf')
    
    rospy.Subscriber("/atrv_node/odom", Odometry, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
