#!/usr/bin/env python3

import tf
import math
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from tracker import triad_openvr
from pyquaternion import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from sitl_dvrk.msg import BoolStamped 

from utils import tf_utils

class node_tracker():


    def __init__(self):

        # Initialize ROS node
        rospy.init_node('vive_tracker_publisher')

        # Create a publisher
        self.pub = rospy.Publisher('vive_tracker_transform', TransformStamped, queue_size=10)
        self.pub_base = rospy.Publisher('base_tracker', TransformStamped, queue_size=10) 
        self.pub_initpos = rospy.Publisher('tracker_initial_position', Vector3, queue_size=10)
        
        self.pub_tracker_current_pos = rospy.Publisher('tracker_current_pos_tf', TransformStamped, queue_size=10)
        self.pub_tracker_current_raw_data = rospy.Publisher('tracker_current_raw_data', TransformStamped, queue_size=10)

        self.pub_base_pcl = rospy.Publisher('transform_base_pcl', TransformStamped, queue_size=10)


        self.vMsg=Vector3()

        self.v = triad_openvr.triad_openvr()
        self.v.print_discovered_objects()
        self.R = rospy.Rate(200)

        self.br = tf.TransformBroadcaster()

        

        g_rot = tf_utils.cv2vecs2g(
            np.array([1,0,0])*math.radians(-90), np.array([0,0,0])
        ).dot(
            tf_utils.cv2vecs2g(
                np.array([0,0,1])*math.radians(-90), np.array([0,0,0])
            )
        )
        self.g_rot_inv = tf_utils.ginv(g_rot)
        

        g_base = tf_utils.cv2vecs2g(
            np.array([0,0,1])*math.radians(180), np.array([2.5,-0.8,1.0])
        )

        self.g_rot2 = tf_utils.cv2vecs2g(
            np.array([0,1,0])*math.radians(90), np.array([0,0,0])
        ).dot(
            tf_utils.cv2vecs2g(
                np.array([1,0,0])*math.radians(90), np.array([0,0,0])
            )
        )
        self.g_init = self.g_rot_inv.dot(self.get_pose()).dot(self.g_rot2)
        self.base_tf_msg = tf_utils.g2tfstamped(g_base, rospy.Time.now(), "map", "vive_tracker_base")
        
        self.g_base_inv = tf_utils.ginv(g_base)

        # base->tracker
        self.transform_msg = TransformStamped()
        self.transform_msg.child_frame_id = "vive_tracker"
        self.transform_msg.header.frame_id = "vive_tracker_base"

        # tracker -> base for the pcl
        self.base_pcl = TransformStamped()
        self.base_pcl.child_frame_id = "base_pcl"
        self.base_pcl.header.frame_id = "vive_tracker"
        

        self.base_pcl.transform.translation.x = 0
        self.base_pcl.transform.translation.y = 0
        self.base_pcl.transform.translation.z = 0

        self.base_pcl.transform.rotation.x    = 0
        self.base_pcl.transform.rotation.y    = 0
        self.base_pcl.transform.rotation.z    = -0.7071
        self.base_pcl.transform.rotation.w    = 0.7071

        #tracker->mapping for the input
        self.input_transform = TransformStamped()
        self.input_transform.child_frame_id = "tracker_input"
        self.input_transform.header.frame_id = "vive_tracker"

        self.quat= np.array([1,0,0,0])
        self.quat2=np.array([0.5,0.5,-0.5,0.5])
        self.flag = False
        self.bool_dist_real = False

    def get_pose(self):
        while True:
            data = self.v.devices["tracker_1"].get_pose_quaternion()
            if data is None:
                continue
            quat = np.array([data[4], data[5], data[6], data[3]])
            quat /= np.linalg.norm(quat)
            rvec = tf_utils.quat2rvec(quat)
            return tf_utils.cv2vecs2g(rvec, np.array(data[0:3]))

    def rotate_quat(self,w,x,y,z,qw,qx,qy,qz):
        original_quat = Quaternion(w=w, x=x, y=y, z=z)
        rotation_quat = Quaternion(w=qw, x=qx, y=qy, z=qz)
        resulting_quat = (original_quat * rotation_quat).normalised 

        return resulting_quat.x, resulting_quat.y, resulting_quat.z, resulting_quat.w
    
    
    def relative_rotation(self,wc,xc,yc,zc,wcnew,xcnew,ycnew,zcnew):

        qc = Quaternion(w=wc, x=xc, y=yc, z=zc)
        qcnew = Quaternion(w=wcnew, x=xcnew, y=ycnew, z=zcnew)

        # Invert the first quaternion (conjugate of a normalized quaternion = inv)
        qc_inv = qc.conjugate

        # Compute the relative rotation quaternion
        qrel = qc_inv * qcnew

        # Normalize the result
        qrel = qrel.normalised

        return qrel.x, qrel.y, qrel.z, qrel.w

    
    def saturate(self,val,max,min):
            if val>max:
                val=max
            elif val<min:
                val =min

            return val
    
    def callback(self,fist_bool):
        self.flag = fist_bool.data
    
    def callback_real(self,bool_dist_real):
        self.bool_dist_real = bool_dist_real.data



if __name__ == '__main__':  

    app=node_tracker()
    initialdata = app.v.devices["tracker_1"].get_pose_quaternion()
    initialdata2 = app.v.devices["tracker_2"].get_pose_quaternion()
    
    offset=initialdata[1]
    
    #we save the first data of the tracker 
    app.vMsg.x=initialdata[1]
    app.vMsg.y=initialdata[2]
    app.vMsg.z=initialdata[0]
    
    #IF YOU CHANGE IT HERE CHANGE IT ALSO IN PCL_WORKSPACE_V2.PY
    size_hand_workspace = 0.25

    rospy.Subscriber("glove/left/fist", BoolStamped, app.callback)
    rospy.Subscriber("glove/left/real", BoolStamped, app.callback_real)

    memorized_x = 0
    memorized_y = 0
    memorized_z = 0
    memorized_rtx = 0
    memorized_rty = 0
    memorized_rtz = 0
    memorized_rtw = 1

    memorized_x_c = 0
    memorized_y_c = 0
    memorized_z_c = 0
    memorized_x_rc = 0
    memorized_y_rc = 0
    memorized_z_rc = 0
    memorized_w_rc = 1
    
    base_offset_x = 0 
    base_offset_y = 0 
    base_offset_z = 0 

    last_rotation_x = 0
    last_rotation_y = 0
    last_rotation_z = 0
    last_rotation_w = 1

    previous_flag = False
    first = True
    reset = False

    # Get a list of device names
    device_names = app.v.devices.keys()

    # Print the names of all discovered devices
    print("Discovered devices:")
    for name in device_names:
        print(name)

    while not rospy.is_shutdown():
        
        pose_data = app.get_pose()
        if pose_data is None:
            continue
        
        t = rospy.Time.now()

        app.base_tf_msg.header.stamp = t
        app.transform_msg.header.stamp = t
        app.input_transform.header.stamp = t
        app.base_pcl.header.stamp = t

        g_offset = app.g_rot_inv.dot(app.get_pose()).dot(app.g_rot2)
        g_offset[:3,3] = g_offset[:3,3] - app.g_init[:3,3]
        

        tracker_current = tf_utils.g2tfstamped(g_offset, t, "vive_tracker_base", "vive_tracker_current")  
        tracker_current_raw_data = tf_utils.g2tfstamped(g_offset, t, "vive_tracker_base", "vive_tracker_raw_sensor")  
        
    
        #clutch
        if (app.flag != previous_flag) and (previous_flag == True):
            
            #THIS AND THE THREE LINE BELOW NEEDS TO BE COHERENT
            base_offset_x = app.base_tf_msg.transform.translation.x
            base_offset_y = app.base_tf_msg.transform.translation.y
            base_offset_z = app.base_tf_msg.transform.translation.z

            #alignement 
            last_rotation_x=tracker_current.transform.rotation.x
            last_rotation_y=tracker_current.transform.rotation.y
            last_rotation_z=tracker_current.transform.rotation.z
            last_rotation_w=tracker_current.transform.rotation.w

        if app.flag == True:
            

            g_offset = app.g_rot_inv.dot(app.get_pose()).dot(app.g_rot2) #CARE CARE CARE CARE CARE
            T_tracker= g_offset[:3,3] - app.g_init[:3,3]

            #"THREE LINES BELOW"  x=pose_data[0]
            app.base_tf_msg.transform.translation.x = T_tracker[0]
            app.base_tf_msg.transform.translation.y = T_tracker[1]
            app.base_tf_msg.transform.translation.z = T_tracker[2]
            

            #we apply the last saved translation before clutching
            tracker_current.transform.translation.x = memorized_x
            tracker_current.transform.translation.y = memorized_y
            tracker_current.transform.translation.z = memorized_z

            tracker_current.transform.rotation.x = memorized_rtx 
            tracker_current.transform.rotation.y = memorized_rty 
            tracker_current.transform.rotation.z = memorized_rtz 
            tracker_current.transform.rotation.w = memorized_rtw 

            memorized_x_c = memorized_x
            memorized_y_c = memorized_y
            memorized_z_c = memorized_z
            
            #in the notes: qmemorized
            memorized_x_rc = memorized_rtx 
            memorized_y_rc = memorized_rty 
            memorized_z_rc = memorized_rtz 
            memorized_w_rc = memorized_rtw

            reset = False
            
        else:

            g_offset = app.g_rot_inv.dot(app.get_pose()).dot(app.g_rot2)
            g_offset[:3,3] = g_offset[:3,3] - app.g_init[:3,3]

            tracker_current.transform.translation.x = g_offset[0,3] + memorized_x_c - base_offset_x 
            tracker_current.transform.translation.y = g_offset[1,3] + memorized_y_c - base_offset_y
            tracker_current.transform.translation.z = g_offset[2,3] + memorized_z_c - base_offset_z

            #memorizing for the clutch
            memorized_x = tracker_current.transform.translation.x 
            memorized_y = tracker_current.transform.translation.y 
            memorized_z = tracker_current.transform.translation.z 
            
            if reset==False:

            
                quaternion = Quaternion(matrix=g_offset[0:3,0:3]).normalised
                rtx=quaternion.x
                rty=quaternion.y
                rtz=quaternion.z
                rtw=quaternion.w

                rtx,rty,rtz,rtw = app.relative_rotation(wc=last_rotation_w,xc=last_rotation_x,yc=last_rotation_y,zc=last_rotation_z,
                                                    xcnew=rtx,ycnew=rty,zcnew=rtz,wcnew=rtw)
                rtx,rty,rtz,rtw = app.rotate_quat(w=memorized_w_rc, x=memorized_x_rc, y=memorized_y_rc, z=memorized_z_rc,
                                                qw=rtw,qx=rtx,qy=rty,qz=rtz)
                
                
            #in the notes qcnew
            memorized_rtx = rtx
            memorized_rty = rty
            memorized_rtz = rtz
            memorized_rtw = rtw

            tracker_current.transform.rotation.x = memorized_rtx
            tracker_current.transform.rotation.y = memorized_rty
            tracker_current.transform.rotation.z = memorized_rtz
            tracker_current.transform.rotation.w = memorized_rtw

        previous_flag =  app.flag
        
        if app.bool_dist_real==True:
            reset=True
    

        # Publish the Transform message
        app.pub.publish(app.transform_msg)
        app.pub_tracker_current_raw_data.publish(tracker_current_raw_data)
        app.pub_tracker_current_pos.publish(tracker_current)
        app.br.sendTransformMessage(app.transform_msg)

        # else:
        #     print("Lost Tracker...")
        app.pub_base.publish(app.base_tf_msg)
        app.br.sendTransformMessage(app.base_tf_msg)
        app.br.sendTransformMessage(tracker_current)
        app.R.sleep()



