#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from ros_numpy.point_cloud2 import array_to_pointcloud2
from geometry_msgs.msg import TransformStamped
from sitl_manus.msg import glove
import math
import numpy as np
from std_msgs.msg import Float64,Float64MultiArray
from sitl_dvrk.msg import BoolStamped, Float64Stamped
from pyquaternion import Quaternion

class PUB_GLOVE_PCL():

    def __init__(self):

        self.num_joints = 21
        self.index_tip =  Float64MultiArray()
        self.index_tip.data = np.zeros((7,))

        self.index_base_pose =  Float64MultiArray()
        self.index_base_pose.data = np.zeros((7,))

        self.index_base = np.zeros((4,))
        self.rel_quat_base_index =  Float64MultiArray()
        self.rel_quat_base_index.data = np.zeros((4,))

        
        self.distance = None
        self.angle_norm = None
        self.pub_angle = rospy.Publisher('glove/right/angle', Float64Stamped, queue_size=10)
        self.pub_raw_angle = rospy.Publisher('glove/right/raw_angle', Float64Stamped, queue_size=10)
        
        self.points = np.zeros((self.num_joints,3))
        
        self.pub_glove_pcl = rospy.Publisher("/manus/pcl_right",PointCloud2,queue_size=10)
        
        self.flag = False

        self.pub_index_tip = rospy.Publisher('index_tip_pose_right', Float64MultiArray, queue_size=10)
        self.pub_index_base = rospy.Publisher('index_base_pose_right', Float64MultiArray, queue_size=10)

        self.pub_index_relq = rospy.Publisher('index_tip_relq_right', Float64MultiArray, queue_size=10)

        # self.pub_flag = rospy.Publisher('glove/left/fist', BoolStamped, queue_size=10)
        # self.pub_angle_bool = rospy.Publisher('glove/left/angle_bool', Bool, queue_size=10) #TO MAKE THE GLOVE VIBRATE!!!!!!!!



    def __del__(self):
        rospy.loginfo("destructing PUB_GLOVE_PCL...")

    def extract_translation(self,tf_msg):

        out = np.zeros((3,))
        out[0] = tf_msg.translation.x
        out[1] = tf_msg.translation.y
        out[2] = tf_msg.translation.z

        return out
    
    def extract_pose_index_tip(self,tf_msg):

        self.index_tip.data[0] = tf_msg.translation.x
        self.index_tip.data[1] = tf_msg.translation.y
        self.index_tip.data[2] = tf_msg.translation.z
        self.index_tip.data[3] = tf_msg.rotation.x
        self.index_tip.data[4] = tf_msg.rotation.y
        self.index_tip.data[5] = tf_msg.rotation.z
        self.index_tip.data[6] = tf_msg.rotation.w
    
    def extract_pose_index_base(self,tf_msg):

        self.index_base_pose.data[0] = tf_msg.translation.x
        self.index_base_pose.data[1] = tf_msg.translation.y
        self.index_base_pose.data[2] = tf_msg.translation.z
        self.index_base_pose.data[3] = tf_msg.rotation.x
        self.index_base_pose.data[4] = tf_msg.rotation.y
        self.index_base_pose.data[5] = tf_msg.rotation.z
        self.index_base_pose.data[6] = tf_msg.rotation.w
    
    def extract_rotation_index_base(self,tf_msg):
       
        self.index_base[0] = tf_msg.rotation.x
        self.index_base[1] = tf_msg.rotation.y
        self.index_base[2] = tf_msg.rotation.z
        self.index_base[3] = tf_msg.rotation.w
        # print(f"index_base[0]: {self.index_base[0]:.4f} index_base[1]: {self.index_base[1]:.4f} index_base[2]: {self.index_base[2]:.4f} index_base[3]: {self.index_base[3]:.4f}")


    def extract_rotation_and_print(self,tf_msg):
        out= np.zeros((4,))
        out[0] = tf_msg.rotation.x
        out[1] = tf_msg.rotation.y
        out[2] = tf_msg.rotation.z
        out[3] = tf_msg.rotation.w
        self.quaternion_to_euler(out[0],out[1],out[2],out[3])

    def calculate_distance(self,x1, y1, z1, x2, y2, z2):
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

        return distance

    def rescale_value(self, original_value, max_old, min_old,min_new,max_new):

            rescaled_value = ((original_value - min_old) * (max_new - min_new)) / (max_old - min_old) + min_new

            if rescaled_value > max_new:
                rescaled_value = max_new

            elif rescaled_value < min_new:
                rescaled_value = min_new

            return rescaled_value
    
    def compute_rel(self, q1,q2):   
            q_base = Quaternion(x=q2[0],y=q2[1],z=q2[2],w=q2[3])
            q_tip = Quaternion(x=q1[0],y=q1[1],z=q1[2],w=q1[3])
            q_base_inv = q_base.inverse
            q_relative = q_base_inv * q_tip
   
            # self.quaternion_to_euler(q_relative.x,q_relative.y,q_relative.z,q_relative.w)
            return q_relative
  
    def quaternion_to_euler(self, x, y, z, w):
        q = Quaternion(w, x, y, z)
        euler_angles = q.yaw_pitch_roll
        euler_angles_degrees = np.degrees(euler_angles)
        print("Euler angles (degrees):", euler_angles_degrees)

    def callback(self,glove_msg):

        self.extract_rotation_index_base(glove_msg.transform1)

        #thumb
        self.points[1,:] = self.extract_translation(glove_msg.transform2)
        self.points[2,:] = self.extract_translation(glove_msg.transform3)
        self.points[3,:] = self.extract_translation(glove_msg.transform4)
        self.points[4,:] = self.extract_translation(glove_msg.transform5)

        #index
        self.points[5,:] = self.extract_translation(glove_msg.transform6) 
        self.points[6,:] = self.extract_translation(glove_msg.transform7)
        self.extract_pose_index_base(glove_msg.transform7)
        self.points[7,:] = self.extract_translation(glove_msg.transform8)
        self.points[8,:] = self.extract_translation(glove_msg.transform9)
        self.extract_pose_index_tip(glove_msg.transform9)
        
        
        # computing relative quat                   #w is last      
        q_rel = self.compute_rel(self.index_base,self.index_tip.data[2:-1])
        self.rel_quat_base_index.data = [q_rel.x, q_rel.y, q_rel.z, q_rel.w]

        #major
        self.points[9,:] = self.extract_translation(glove_msg.transform10)
        self.points[10,:] = self.extract_translation(glove_msg.transform11)
        self.points[11,:] = self.extract_translation(glove_msg.transform12)
        self.points[12,:] = self.extract_translation(glove_msg.transform13)
        
        #rf
        self.points[13,:] = self.extract_translation(glove_msg.transform14)
        self.points[14,:] = self.extract_translation(glove_msg.transform15)
        self.points[15,:] = self.extract_translation(glove_msg.transform16)
        self.points[16,:] = self.extract_translation(glove_msg.transform17)
        
        #pinky
        self.points[17,:] = self.extract_translation(glove_msg.transform18)
        self.points[18,:] = self.extract_translation(glove_msg.transform19)
        self.points[19,:] = self.extract_translation(glove_msg.transform20)
        self.points[20,:] = self.extract_translation(glove_msg.transform21)

        self.distance=self.calculate_distance(self.points[4,0],self.points[4,1],self.points[4,2],
                                self.points[8,0],self.points[8,1],self.points[8,2]) 
        self.angle_norm = self.rescale_value(self.distance, 0.15, 0.01, -5, 85)

        data = np.zeros(self.num_joints, dtype=[('x', np.float32),('y', np.float32),('z', np.float32)])

        data['x'] = self.points[:,0]
        data['y'] = self.points[:,1]
        data['z'] = self.points[:,2]

        pcl_msg = array_to_pointcloud2(data,frame_id="base_pcl",stamp=rospy.Time.now())
        self.pub_glove_pcl.publish(pcl_msg)
        
        self.pub_index_tip.publish(self.index_tip)
        self.pub_index_base.publish(self.index_base_pose)
        # print(self.index_base_pose)
        self.pub_index_relq.publish(self.rel_quat_base_index)

        

        if self.angle_norm is not None:
            msg = Float64Stamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = self.angle_norm
            self.pub_angle.publish(msg)

            msg.data = self.distance

            self.pub_raw_angle.publish(msg)

if __name__ == "__main__":

    rospy.init_node("pub_glove_pcl_right")
    app = PUB_GLOVE_PCL()
    rospy.Subscriber("/manus/cp_right",glove,app.callback)

    rospy.spin()