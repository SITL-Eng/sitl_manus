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
        
        self.distance = None
        self.angle_norm = None
        self.pub_angle = rospy.Publisher('glove/left/angle', Float64, queue_size=10)
        
        self.points = np.zeros((self.num_joints,3))
        self.pub_glove_pcl = rospy.Publisher("/manus/pcl",PointCloud2,queue_size=10)
        
        self.flag = False
        self.pub_flag = rospy.Publisher('glove/left/fist', Bool, queue_size=10)
        self.pub_angle_bool = rospy.Publisher('glove/left/angle_bool', Bool, queue_size=10) #TO MAKE THE GLOVE VIBRATE!!!!!!!!

        


    def __del__(self):
        rospy.loginfo("destructing PUB_GLOVE_PCL...")



    def extract_translation(self,tf_msg):
        out = np.zeros((3,))
        out[0] = tf_msg.translation.x
        out[1] = tf_msg.translation.y
        out[2] = tf_msg.translation.z

        return out
    


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
    


    def fist(self):
        # mcp_index=self.points[5,:]        # index=self.points[8,:]
        # mcp_major=self.points[9,:]        # major=self.points[12,:]
        # mcp_ring=self.points[13,:]        # ring=self.points[16,:]
        # mcp_pinky=self.points[17,:]       # pinky=self.points[20,:]

        d1=self.calculate_distance(self.points[5,0],self.points[5,1],self.points[5,2],
                                self.points[8,0],self.points[8,1],self.points[8,2])
        
        d2=self.calculate_distance(self.points[9,0],self.points[9,1],self.points[9,2],
                                self.points[12,0],self.points[12,1],self.points[12,2])
        d3=self.calculate_distance(self.points[13,0],self.points[13,1],self.points[13,2],
                                self.points[16,0],self.points[16,1],self.points[16,2])
        
        d4=self.calculate_distance(self.points[16,0],self.points[16,1],self.points[16,2],
                                self.points[20,0],self.points[20,1],self.points[20,2])
        
        # Format and print the variables with three decimal places
        formatted_d1 = "{:.3f}".format(d1)
        formatted_d2 = "{:.3f}".format(d2)
        formatted_d3 = "{:.3f}".format(d3)
        formatted_d4 = "{:.3f}".format(d4)

        print(formatted_d1,formatted_d2,formatted_d3,formatted_d4)

        flag = sum(1 for var in [d1, d2, d3, d4] if var < 0.07) >= 4

        if flag:
            print("Clutch activated")
            self.pub_angle_bool.publish(True)
        else:
            self.pub_angle_bool.publish(False)

        return flag



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

        self.distance=self.calculate_distance(self.points[4,0],self.points[4,1],self.points[4,2],
                                self.points[8,0],self.points[8,1],self.points[8,2]) 
        self.angle_norm = self.rescale_value(self.distance, 0.19, 0.05,-5,55)

        data = np.zeros(self.num_joints, dtype=[('x', np.float32),('y', np.float32),('z', np.float32)])

        data['x'] = self.points[:,0]
        data['y'] = self.points[:,1]
        data['z'] = self.points[:,2]

        pcl_msg = array_to_pointcloud2(data,frame_id="base_pcl",stamp=rospy.Time.now())
        self.pub_glove_pcl.publish(pcl_msg)
        
        self.flag = self.fist()
        self.pub_flag.publish(self.flag)

        if self.angle_norm is not None:
            self.pub_angle.publish(self.angle_norm)

    

if __name__ == "__main__":

    rospy.init_node("pub_glove_pcl")
    app = PUB_GLOVE_PCL()
    rospy.Subscriber("/manus/cp",glove,app.callback)

    rospy.spin()