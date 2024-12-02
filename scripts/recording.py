#!/usr/bin/env python3

import rospy
import rosbag
from sitl_manus.msg import glove
from geometry_msgs.msg import TransformStamped, PoseStamped
from sitl_dvrk.msg import BoolStamped,Float64Stamped
import message_filters
from sensor_msgs.msg import JointState


class RECORD_HAND_PSM():
    def __init__(self,topic_names):
        # Open the bag file in write mode
        self.bag = rosbag.Bag("/home/phd-leonardo-sitl/Desktop/PhdLeo/dataset_glove/ICRA_2025/icra_video_3.bag", "w")
        self.topics = topic_names
    
    def callback(self,*msgs):
        for i, msg in enumerate(msgs):
            self.bag.write(self.topics[i], msg, msg.header.stamp)
            


if __name__ == '__main__':
    rospy.init_node("transform_recorder")


    topic_names = [
        
        "/tracker_current_pos_tf",
        "/tracker_current_raw_data",
        "/PSM2/custom/local/setpoint_cp",
        "/glove/left/fist",
        "/glove/left/angle",
        "/PSM2/jaw/setpoint_js",
        "glove/left/on_off",
        'glove/left/raw_angle'
        # "/manus/cp"
        # "/MTML/local/setpoint_cp",
    ]
    app = RECORD_HAND_PSM(topic_names)

    rospy.loginfo("START RECORDIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIING")
    msgs = [
        
        message_filters.Subscriber(topic_names[0] ,TransformStamped),
        message_filters.Subscriber(topic_names[1] ,TransformStamped),
        message_filters.Subscriber(topic_names[2] ,PoseStamped),
        message_filters.Subscriber(topic_names[3] ,BoolStamped),
        message_filters.Subscriber(topic_names[4] ,Float64Stamped),
        message_filters.Subscriber(topic_names[5] ,JointState),
        message_filters.Subscriber(topic_names[6] ,BoolStamped),
        message_filters.Subscriber(topic_names[7] ,Float64Stamped)
        # message_filters.Subscriber(topic_names[8] ,glove)
        
        # message_filters.Subscriber(topic_names[6] ,TransformStamped),
    ]
    ts = message_filters.ApproximateTimeSynchronizer(msgs, queue_size=10, slop=0.01)

    ts.registerCallback(app.callback)
    try:
        rospy.spin()
    finally:
        print("Saving bag")
        app.bag.close()




