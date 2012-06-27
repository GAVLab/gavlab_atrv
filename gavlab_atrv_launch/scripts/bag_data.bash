#!/bin/bash

rosbag record --split --size=1024 -b 2048 -o $1 /asus/depth_registered/points /asus/rgb/image_color/compressed /atrv_node/cmd_vel /atrv_node/front/encoders /atrv_node/odom /atrv_node/rear/encoders /odom_corrected /scan /tf /xbow440_node/imu/data
