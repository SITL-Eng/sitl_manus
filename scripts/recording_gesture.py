#!/usr/bin/env python3

import rospy
import rosbag
from sitl_manus.msg import glove
from geometry_msgs.msg import TransformStamped, PoseStamped
from sitl_dvrk.msg import BoolStamped,Float64Stamped
import message_filters
from sensor_msgs.msg import JointState


class RECORD_HAND():
    def __init__(self):
        # Open the bag file in write mode
        self.bag = rosbag.Bag("/home/phd-leonardo-sitl/Desktop/PhdLeo/dataset_glove/ICRA_2025/hand_recog/gesture_test/none_test/none_test.bag", "w")
        
        # self.bag = rosbag.Bag("/home/phd-leonardo-sitl/Desktop/PhdLeo/dataset_glove/rf_v2/rf7.bag", "w")
        # self.bag = rosbag.Bag("/home/phd-leonardo-sitl/Desktop/PhdLeo/dataset_glove/clutch_v2/clutch7.bag", "w")
    
    def callback(self,cp):
        
        self.bag.write("/manus/cp",cp)
            


if __name__ == '__main__':
    rospy.init_node("record_gesture")

    app = RECORD_HAND()

    rospy.loginfo("START RECORDIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIING")

    rospy.Subscriber("/manus/cp",glove,app.callback)


    try:
        rospy.spin()
    finally:
        print("Saving bag")

        app.bag.close()