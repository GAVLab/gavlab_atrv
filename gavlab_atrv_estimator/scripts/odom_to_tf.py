#!/usr/bin/env python

import roslib; roslib.load_manifest('gavlab_atrv_estimator')

import rospy
from nav_msgs.msg import Odometry
import tf

pub = None

def callback(msg):
    global pub
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     "base_footprint",
                     "odom")
    msg.header.frame_id="odom"
    pub.publish(msg)

def main():
    global pub
    rospy.init_node('odom2tf')
    
    pub = rospy.Publisher("odom_corrected",Odometry)

    rospy.Subscriber("/atrv_node/odom", Odometry, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
