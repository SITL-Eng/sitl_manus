#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Transform
from sitl_manus.msg import glove
from scipy.spatial.transform import Rotation as R
import numpy as np
from utils import tf_utils, ik_utils, dvrk_utils
from geometry_msgs.msg import TransformStamped,Vector3,Quaternion
np.set_printoptions(suppress=True)
import math




class GLOVE_READER():
    def __init__(self):
               


        self.num_joints = 21
        self.rotation = np.zeros((self.num_joints,4))
        self.translation= np.zeros((self.num_joints,3))
        self.pub_glove_pcl = rospy.Publisher("/manus/pcl_right",PointCloud2,queue_size=10)

        self.rate = rospy.Rate(1)



    def __del__(self):
        rospy.loginfo("destructing PUB_GLOVE_PCL...")

    def extract_translation(self,tf_msg):
        out = np.zeros((3,))
        out[0] = tf_msg.translation.x
        out[1] = tf_msg.translation.y
        out[2] = tf_msg.translation.z
        return out
    
    def extract_rotation(self,tf_msg):
        out = np.zeros((4,))
        out[0] = tf_msg.rotation.x
        out[1] = tf_msg.rotation.y
        out[2] = tf_msg.rotation.z
        out[3] = tf_msg.rotation.w
        return out
    
    def rescale_value(self,original_value, max_old, min_old,min_new,max_new):

        rescaled_value = ((original_value - min_old) * (max_new - min_new)) / (max_old - min_old) + min_new

        if rescaled_value > max_new:
            rescaled_value = max_new

        elif rescaled_value < min_new:
            rescaled_value = min_new

        return rescaled_value

    def callback(self,glove_msg):

        self.translation[0,:] = self.extract_translation(glove_msg.transform1)
        self.translation[1,:] = self.extract_translation(glove_msg.transform2)
        self.translation[2,:] = self.extract_translation(glove_msg.transform3)
        self.translation[3,:] = self.extract_translation(glove_msg.transform4)
        self.translation[4,:] = self.extract_translation(glove_msg.transform5)
        self.translation[5,:] = self.extract_translation(glove_msg.transform6)
        self.translation[6,:] = self.extract_translation(glove_msg.transform7)
        self.translation[7,:] = self.extract_translation(glove_msg.transform8)
        self.translation[8,:] = self.extract_translation(glove_msg.transform9)
        self.translation[9,:] = self.extract_translation(glove_msg.transform10)
        self.translation[10,:] = self.extract_translation(glove_msg.transform11)
        self.translation[11,:] = self.extract_translation(glove_msg.transform12)
        self.translation[12,:] = self.extract_translation(glove_msg.transform13)
        self.translation[13,:] = self.extract_translation(glove_msg.transform14)
        self.translation[14,:] = self.extract_translation(glove_msg.transform15)
        self.translation[15,:] = self.extract_translation(glove_msg.transform16)
        self.translation[16,:] = self.extract_translation(glove_msg.transform17)
        self.translation[17,:] = self.extract_translation(glove_msg.transform18)
        self.translation[18,:] = self.extract_translation(glove_msg.transform19)
        self.translation[19,:] = self.extract_translation(glove_msg.transform20)
        self.translation[20,:] = self.extract_translation(glove_msg.transform21)

        self.rotation[0,:] = self.extract_rotation(glove_msg.transform1)
        self.rotation[1,:] = self.extract_rotation(glove_msg.transform2)
        self.rotation[2,:] = self.extract_rotation(glove_msg.transform3)
        self.rotation[3,:] = self.extract_rotation(glove_msg.transform4)
        self.rotation[4,:] = self.extract_rotation(glove_msg.transform5)
        self.rotation[5,:] = self.extract_rotation(glove_msg.transform6)
        self.rotation[6,:] = self.extract_rotation(glove_msg.transform7)
        self.rotation[7,:] = self.extract_rotation(glove_msg.transform8)
        self.rotation[8,:] = self.extract_rotation(glove_msg.transform9)
        self.rotation[9,:] = self.extract_rotation(glove_msg.transform10)
        self.rotation[10,:] = self.extract_rotation(glove_msg.transform11)
        self.rotation[11,:] = self.extract_rotation(glove_msg.transform12)
        self.rotation[12,:] = self.extract_rotation(glove_msg.transform13)
        self.rotation[13,:] = self.extract_rotation(glove_msg.transform14)
        self.rotation[14,:] = self.extract_rotation(glove_msg.transform15)
        self.rotation[15,:] = self.extract_rotation(glove_msg.transform16)
        self.rotation[16,:] = self.extract_rotation(glove_msg.transform17)
        self.rotation[17,:] = self.extract_rotation(glove_msg.transform18)
        self.rotation[18,:] = self.extract_rotation(glove_msg.transform19)
        self.rotation[19,:] = self.extract_rotation(glove_msg.transform20)
        self.rotation[20,:] = self.extract_rotation(glove_msg.transform21)


if __name__ == "__main__":

    rospy.init_node("glove_reader_right")
    app = GLOVE_READER()
    rospy.Subscriber("/manus/cp_right",glove,app.callback)

    try:
        while not rospy.is_shutdown():
            index_tip = app.rotation[4,:]
            
            try: 
                r1 = R.from_quat(np.array([index_tip[0], index_tip[1], index_tip[2], index_tip[3]]))
                r1_euler=r1.as_euler('xyz',degrees=True)
            

                rroll = app.rescale_value(r1_euler[2],-180, 180,app.params["joint4_min"],app.params["joint4_max"])
                rpitch = app.rescale_value(r1_euler[0],-180,180,app.params["joint5_min"],app.params["joint5_max"])
                ryaw = app.rescale_value(r1_euler[1],-180,180,app.params["joint6_min"],app.params["joint6_max"])

                print([rroll,rpitch,ryaw])

 

            except: 
                r1_euler= None

        

            

    except Exception as e:
        rospy.loginfo(e)
    finally:
        del app


    rospy.spin()