#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import PointCloud2
from ros_numpy.point_cloud2 import array_to_pointcloud2
from geometry_msgs.msg import Transform
from sitl_manus.msg import glove
import math
import numpy as np
from std_msgs.msg import Float64,Bool
from sitl_dvrk.msg import BoolStamped
import pandas as pd
import joblib
import time
import traceback


def boolstampedmsg(data, t):
    msg = BoolStamped()
    msg.header.stamp = t
    msg.data = data
    return msg


class PUB_GLOVE_PCL():
    def __init__(self):
        
        self.num_joints = 21
        self.bool_monopolar = False
        self.array_voting = np.zeros((7,6))
        self.pub_monopolar = rospy.Publisher('pedals/write/monopolar', BoolStamped, queue_size=10)
 
        self.data_list = []


    def extract_translation_rotation(self, tf_msg):
        return [tf_msg.translation.x, tf_msg.translation.y, tf_msg.translation.z,
                tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z, tf_msg.rotation.w]


    def callback(self, glove_msg):
        # Create a dictionary to store the data for each joint
        data = {}

        # Update the dictionary with translation and rotation values
        for i in range(1, self.num_joints + 1):

            transform_name = f'transform{i}'
            pose = self.extract_translation_rotation(getattr(glove_msg, transform_name))
            data.update({f'{transform_name}.translation.x': pose[0],
                         f'{transform_name}.translation.y': pose[1],
                         f'{transform_name}.translation.z': pose[2],
                         f'{transform_name}.rotation.x': pose[3],
                         f'{transform_name}.rotation.y': pose[4],
                         f'{transform_name}.rotation.z': pose[5],
                         f'{transform_name}.rotation.w': pose[6]})
            
        # Append the data to the list
        self.data_list.append(data)


    def gesture_case(self, gesture):
            """Sets the appropriate element in array_voting to 1 based on gesture."""
            gesture_mapping = {
                'none': 0,
                'monopolar': 1
            }
            self.array_voting[2, :] = 0  
            self.array_voting[2, gesture_mapping[gesture]] = 1


    def pushing_values(self):
            """Shifts values in array_voting."""
            self.array_voting = np.roll(self.array_voting, 1, axis=0)  # Roll values efficiently


    def voting(self):
            """Updates boolean variables based on voting results."""
            max_sum_col_index = np.argmax(self.array_voting.sum(axis=0))
            self.bool_monopolar = max_sum_col_index == 1


if __name__ == "__main__":
    voting_num = 30
    rospy.init_node('gesture_recog_right')
    app = PUB_GLOVE_PCL()

    rospy.Subscriber("/manus/cp_right", glove, app.callback)

    loaded_classifier = joblib.load('/home/phd-leonardo-sitl/Desktop/PhdLeo/dataset_glove/gesture_recog_model_lgbm_r.joblib')
    rate = rospy.Rate(200)  
    count = 0
    prediction_times = []
    try:
        while not rospy.is_shutdown():
            if app.data_list:
                
                app.df = pd.DataFrame(app.data_list)
                
                #VOTING
                start_time = time.time()
                y_pred = loaded_classifier.predict(app.df)

                if count <voting_num:
                    end_time=time.time()
                    prediction_speed= end_time-start_time
                    prediction_times.append(prediction_speed)
                    count = count+1
                
                elif count == voting_num:
                    mean_prediction_time = np.round(np.mean(prediction_times)*voting_num, 2)
                    std_prediction_time = np.round(np.std(prediction_times)*voting_num, 2)
 
                    count = count+1

                app.gesture_case(y_pred[0])
                    
                app.pushing_values()
                app.voting()
                app.pub_monopolar.publish(boolstampedmsg(app.bool_monopolar, rospy.Time.now()))
                
                app.data_list = []  
            rate.sleep()
    except:
        traceback.print_exc()