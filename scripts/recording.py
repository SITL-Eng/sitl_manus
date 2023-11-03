#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from ros_numpy.point_cloud2 import array_to_pointcloud2
from geometry_msgs.msg import Transform
from sitl_manus.msg import glove
import math
import numpy as np
from std_msgs.msg import Float64,Bool

import rospy
import rosbag
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener

def callback1(msg):
    bag1.write('transform_tracker', msg)

def callback2(msg):
    bag2.write('transform_psm2', msg)

if __name__ == '__main__':
    rospy.init_node('transform_recorder')

    bag1 = rosbag.Bag('transform_tracker.bag', 'w')
    bag2 = rosbag.Bag('transform_psm2.bag', 'w')

    #tf_listener = TransformListener()

    rospy.Subscriber('/vive_tracker_transform', TransformStamped, callback1)
    rospy.Subscriber('/PSM2/custom/local/setpoint_cp', TransformStamped, callback2)

    rospy.spin()

    bag1.close()
    bag2.close()
