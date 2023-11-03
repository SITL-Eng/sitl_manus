#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from ros_numpy.point_cloud2 import array_to_pointcloud2
from geometry_msgs.msg import Transform
from sitl_manus.msg import glove
import math
import numpy as np
from std_msgs.msg import Float64,Bool

class PUB_GLOVE_PCL():

    def __init__(self):
        self.num_joints = 21
        self.points = np.zeros((self.num_joints,3))
        self.distance = None
        self.last_distance_update_time = rospy.Time.now()
        self.eps= 0.02
        self.bool_dist =  False
        self.bool_fist = False
        self.pub_on_off = rospy.Publisher('glove/left/on_off', Bool, queue_size=10)
        self.pub_flag = rospy.Publisher('glove/left/angle_bool', Bool, queue_size=10)
        self.rate = 100
        
        self.clock= 0
        self.clock_vibration = 0

        self.first = False
        self.start = False

        self.vibration =  False



    def __del__(self):
        rospy.loginfo("destructing PUB_GLOVE_PCL...")



    def calculate_distance(self,x1, y1, z1, x2, y2, z2):
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        return distance



    def fist_callback(self,fist):
        self.bool_fist = fist



    def extract_translation(self,tf_msg):
        out = np.zeros((3,))
        out[0] = tf_msg.translation.x
        out[1] = tf_msg.translation.y
        out[2] = tf_msg.translation.z

        return out
    


    def perform_double_vibration(self):

        if self.clock_vibration>1 and self.clock_vibration<60:
            self.pub_flag.publish(True)

        elif self.clock_vibration>60 and self.clock_vibration<120:
            self.pub_flag.publish(False)
 



    def callback(self,glove_msg):

        self.points[0,:] = self.extract_translation(glove_msg.transform1)
        self.points[1,:] = self.extract_translation(glove_msg.transform2)
        self.points[2,:] = self.extract_translation(glove_msg.transform3)
        self.points[3,:] = self.extract_translation(glove_msg.transform4)
        self.points[4,:] = self.extract_translation(glove_msg.transform5)
        self.points[5,:] = self.extract_translation(glove_msg.transform6)
        self.points[6,:] = self.extract_translation(glove_msg.transform7)
        self.points[7,:] = self.extract_translation(glove_msg.transform8)
        self.points[8,:] = self.extract_translation(glove_msg.transform9)

        self.points[9,:] = self.extract_translation(glove_msg.transform10)
        self.points[10,:] = self.extract_translation(glove_msg.transform11)
        self.points[11,:] = self.extract_translation(glove_msg.transform12)
        self.points[12,:] = self.extract_translation(glove_msg.transform13)
        self.points[13,:] = self.extract_translation(glove_msg.transform14)
        self.points[14,:] = self.extract_translation(glove_msg.transform15)
        self.points[15,:] = self.extract_translation(glove_msg.transform16)
        self.points[16,:] = self.extract_translation(glove_msg.transform17)
        self.points[17,:] = self.extract_translation(glove_msg.transform18)
        self.points[18,:] = self.extract_translation(glove_msg.transform19)
        self.points[19,:] = self.extract_translation(glove_msg.transform20)
        self.points[20,:] = self.extract_translation(glove_msg.transform21)

        new_distance = self.calculate_distance(self.points[4,0], self.points[4,1], self.points[4,2],
                                           self.points[12,0], self.points[12,1], self.points[12,2])

        

        if self.distance is not None and (new_distance<= 0.08) and (abs(new_distance - self.distance) <= 0.02) and (self.bool_fist.data is not True):
            
            if (self.clock  >= 120):
                
                if self.first == False:
                    
                    self.bool_dist = not self.bool_dist
                    self.first = True
                    self.start = True
            self.clock = self.clock+1
                
        else:
            
            self.clock = 0
            self.first = False

        self.distance = new_distance

        if self.start:
            
            self.clock_vibration = self.clock_vibration+1
            self.perform_double_vibration()
            
            if self.clock_vibration>180:
                self.start = False
                self.clock_vibration = 0


        self.pub_on_off.publish(self.bool_dist)
        

       
if __name__ == "__main__":
    rospy.init_node("gesture_on_off")
    app = PUB_GLOVE_PCL()

    rospy.Subscriber("/manus/cp", glove, app.callback)
    rospy.Subscriber("glove/left/fist", Bool, app.fist_callback)

    rate = rospy.Rate(app.rate)  # Create a rate object with the desired rate (100 Hz)
    while not rospy.is_shutdown():
        rate.sleep()

 