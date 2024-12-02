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

        self.bool_fist = False
        self.bool_vibration = False
        self.bool_on_off = False
        self.bool_on_off_v2 = False
        self.bool_dist =  False
        self.bool_dist_real =  False
        self.bool_four = False
        self.bool_pinky = False
        self.bool_thumbs_up = False
        self.first = False
        self.start = False
        self.pub_fist = rospy.Publisher('glove/left/fist', BoolStamped, queue_size=10)
        self.pub_angle_bool = rospy.Publisher('glove/left/angle_bool', Bool, queue_size=10) #VIBRATION
        self.pub_on_off = rospy.Publisher('glove/left/on_off', BoolStamped, queue_size=10)
        self.pub_real = rospy.Publisher('glove/left/real', BoolStamped, queue_size=10)
        self.array_voting = np.zeros((20,3)) 

        # self.pub_fist = rospy.Publisher('glove/left/fist', Bool, queue_size=10)
        # self.pub_angle_bool = rospy.Publisher('glove/left/angle_bool', Bool, queue_size=10)
        # self.pub_on_off = rospy.Publisher('glove/left/on_off', Bool, queue_size=10)
        # self.pub_real = rospy.Publisher('glove/left/real', Bool, queue_size=10)

        self.clock= 0
        self.clock_real= 0
        self.clock_vibration = 0
 
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
            


            gesture_mapping = {
                'clutch': 0,
                # 'four': 1,
                'ring': 1,
                # 'pinky': 3,
                # 'thumb': 4,
                'none': 2
            }

            
            self.array_voting[2, :] = 0  
            self.array_voting[2, gesture_mapping[gesture]] = 1


    def pushing_values(self):
            """Shifts values in array_voting."""
            self.array_voting = np.roll(self.array_voting, 1, axis=0)  # Roll values efficiently

    def voting(self):
            """Updates boolean variables based on voting results."""
            max_sum_col_index = np.argmax(self.array_voting.sum(axis=0))
           
            self.bool_fist = max_sum_col_index == 0
            # self.bool_four = max_sum_col_index == 1
            self.bool_on_off = max_sum_col_index == 1
            # self.bool_pinky = max_sum_col_index == 3
            # self.bool_thumbs_up = max_sum_col_index == 4


            # if self.bool_on_off:
            #     print("on_off")
            # elif self.bool_fist:
            #     print("clutch")
            # elif  max_sum_col_index == 2:
            #     print("none")


    

        
    def on_off(self):
        #ON OFF
        if self.bool_on_off_v2 is not None and self.bool_on_off and (self.bool_on_off and self.bool_on_off_v2) and (self.bool_fist is not True):
            
            if (self.clock  >= 140):
                
                if self.first == False:
                    
                    self.bool_dist = not self.bool_dist
                    self.first = True
                    self.start = True
                    print("STARTIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIING")
                     
            self.clock = self.clock+1
                
        else:
            
            self.clock = 0
            self.first = False
    
        self.bool_on_off_v2 = self.bool_on_off

    def perform_double_vibration(self):
        if self.clock_vibration>1 and self.clock_vibration<60:
            self.pub_angle_bool.publish(True)


        elif self.clock_vibration>60 and self.clock_vibration<120:
            self.pub_angle_bool.publish(False)
    # def draw_prediction(self):



    def feedback(self):
        #FEEDBACK
        if self.start:
            # print("starting")
            self.clock_vibration = self.clock_vibration+1
            self.perform_double_vibration()
            
            if self.clock_vibration>180:
                self.start = False
                self.clock_vibration = 0
                self.bool_dist_real = False

        self.pub_on_off.publish(boolstampedmsg(self.bool_dist, rospy.Time.now()))
        self.pub_real.publish(boolstampedmsg(self.bool_dist_real, rospy.Time.now()))


if __name__ == "__main__":

    rospy.init_node('gesture_on_off')
    app = PUB_GLOVE_PCL()
    
    rospy.Subscriber("/manus/cp", glove, app.callback)
    
    # loaded_classifier = joblib.load('/home/phd-leonardo-sitl/Desktop/PhdLeo/dataset_glove/gesture_recog_model_lgbm_augm.joblib')
    loaded_classifier = joblib.load('/home/phd-leonardo-sitl/Desktop/PhdLeo/dataset_glove/gesture_recog_model_lgbm_best.joblib')
    
    rate = rospy.Rate(200)  
    count = 0
    prediction_times = []
    
    try:
        while not rospy.is_shutdown():
            
            if app.data_list:
                 
                app.df = pd.DataFrame(app.data_list)
                app.df = app.df.drop(columns=[col for col in app.df.columns if 'translation' in col]) #TO WORK ONLY WITH QUAT
                # app.df = app.create_relative_df(app.df)
                
                #VOTING
                start_time = time.time()

                y_proba = loaded_classifier.predict_proba(app.df) 
                y_pred = loaded_classifier.predict(app.df)
                # print(y_proba)
                print(y_pred)
                
                # print(y_pred)
                if count <1000:
                    end_time=time.time()
                    prediction_speed= end_time-start_time
                    prediction_times.append(prediction_speed)
                    count = count+1
                
                elif count == 1000:
                    mean_prediction_time = np.round(np.mean(prediction_times)*1000, 2)
                    std_prediction_time = np.round(np.std(prediction_times)*1000, 2)
                    print("###########################RESULTS##########################################")
                    print(mean_prediction_time,std_prediction_time)
                    print("############################################################################")
                    count = count+1

                app.gesture_case(y_pred[0])
                    
                app.pushing_values()
                app.voting()

                # print(y_pred[0])
                # print(app.array_voting)
                
                app.pub_fist.publish(boolstampedmsg(app.bool_fist, rospy.Time.now()))

                if app.bool_fist:
                    app.pub_angle_bool.publish(True)
                    



                else:
                    app.pub_angle_bool.publish(False)

               

                app.on_off()
                app.feedback()

                app.bool_fist = False
                app.bool_four = False
                app.bool_on_off = False
                app.bool_pinky = False

                app.data_list = []  
            rate.sleep()
    except:
        traceback.print_exc()