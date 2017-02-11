#!/usr/bin/env python
# This script is intended to hold onto the latest asus point cloud messages
# and publish once the SLAM map has updated. This is to prevent distortion
# of the octomap.
import rospy

from sensor_msgs.msg import PointCloud2
# from nav_msgs.msg import OccupancyGrid
from tf.msg import tfMessage

global cur_pc_msg,is_populated

is_populated = False

def pc_callback(msg):
    global cur_pc_msg,is_populated
    is_populated = True
    cur_pc_msg = msg

# def map_callback(msg):
#     global cur_pc_msg,is_populated
#     if is_populated:
#         pub.publish(cur_pc_msg)

def tf_callback(msg):
    global cur_pc_msg,is_populated
    for trans in msg.transforms:
        if trans.header.frame_id=='map' and trans.child_frame_id=='odom' and is_populated:
            pub.publish(cur_pc_msg)

if __name__ == '__main__':
    rospy.init_node('point_cloud_sync')
    pub = rospy.Publisher('/asus/map_synced_points', PointCloud2,queue_size=1000)
    # rospy.Subscriber('/map',OccupancyGrid,map_callback, queue_size=1)
    rospy.Subscriber('/tf',tfMessage,tf_callback, queue_size=1000)
    rospy.Subscriber('/asus/depth_registered/points',PointCloud2,pc_callback, queue_size=1)

    rospy.spin()