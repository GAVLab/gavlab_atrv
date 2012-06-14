#!/usr/bin/env python

import roslib; roslib.load_manifest('gavlab_atrv_2dnav')

import rospy
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler as qfe
from tf.transformations import euler_from_quaternion as efq
from math import radians

pub = None

def callback(msg):
    global pub
    # r, p, y = efq((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
    # q = qfe(r, p, -y)
    # msg.pose.pose.orientation.x = q[0]
    # msg.pose.pose.orientation.y = q[1]
    # msg.pose.pose.orientation.z = q[2]
    # msg.pose.pose.orientation.w = q[3]
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     "base_footprint",
                     "odom")
    # br.sendTransform((0, 0, 0),
    #                  qfe(radians(180), 0, 0),
    #                  msg.header.stamp,
    #                  "base_footprint",
    #                  "base_footprint_inv")
    # msg.header.frame_id = "odom"
    # pub.publish(msg)

def main():
    global pub
    rospy.init_node('odom_to_tf')
    
    pub = rospy.Publisher("odom_corrected", Odometry)
    rospy.Subscriber("/atrv_node/odom", Odometry, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
