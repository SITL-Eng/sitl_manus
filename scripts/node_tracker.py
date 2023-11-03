#!/usr/bin/env python3

import tf
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from tracker import triad_openvr
from pyquaternion import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

class node_tracker():


    def __init__(self):

        # Initialize ROS node
        rospy.init_node('vive_tracker_publisher')

        # Create a publisher
        self.pub = rospy.Publisher('vive_tracker_transform', TransformStamped, queue_size=10)
        self.pub_base = rospy.Publisher('base_tracker', TransformStamped, queue_size=10) 
        self.pub_initpos = rospy.Publisher('tracker_initial_position', Vector3, queue_size=10) 
        self.pub_base_pcl = rospy.Publisher('transform_base_pcl', TransformStamped, queue_size=10)

        self.vMsg=Vector3()

        self.v = triad_openvr.triad_openvr()
        self.v.print_discovered_objects()
        self.R = rospy.Rate(100)

        self.br = tf.TransformBroadcaster()

        self.base_tf_msg = TransformStamped()
        self.base_tf_msg.child_frame_id = "vive_tracker_base"
        self.base_tf_msg.header.frame_id = "map"

        self.base_tf_msg.transform.translation.x = 0
        self.base_tf_msg.transform.translation.y = 0
        self.base_tf_msg.transform.translation.z = 0

        self.base_tf_msg.transform.rotation.x    = -0.5
        self.base_tf_msg.transform.rotation.y    = -0.5
        self.base_tf_msg.transform.rotation.z    = -0.5
        self.base_tf_msg.transform.rotation.w    =  0.5

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

    def rotate_quat(self,w,x,y,z,qw,qx,qy,qz):
        
        original_quat = Quaternion(w=w, x=x, y=y, z=z)
        rotation_quat = Quaternion(w=qw, x=qx, y=qy, z=qz)
        resulting_quat = (original_quat * rotation_quat).normalised 
        

        return resulting_quat.x, resulting_quat.y, resulting_quat.z, resulting_quat.w

    def callback(self,fist_bool):

        self.flag = fist_bool.data

if __name__ == '__main__':  

    app=node_tracker()
    initialdata = app.v.devices["tracker_1"].get_pose_quaternion()
    offset=initialdata[1]
    
    app.vMsg.x=initialdata[1]
    app.vMsg.y=initialdata[2]
    app.vMsg.z=initialdata[0]
    print(app.vMsg.z)
    rospy.Subscriber("glove/left/fist", Bool, app.callback)

    memorized_x = 0
    memorized_y = 0
    memorized_z = 0
    memorized_x_c = 0
    memorized_y_c = 0
    memorized_z_c = 0
    base_offset_x = 0 
    base_offset_y = 0 
    base_offset_z = 0 
    previous_flag = False
    first = True

    while not rospy.is_shutdown():
        
        pose_data = app.v.devices["tracker_1"].get_pose_quaternion()
        
        t = rospy.Time.now()

        app.base_tf_msg.header.stamp = t
        app.transform_msg.header.stamp = t
        app.input_transform.header.stamp = t
        app.base_pcl.header.stamp = t
        
        if pose_data is not None:
    
            #clutch
            if (app.flag != previous_flag) and (previous_flag == True):

                base_offset_x = app.base_tf_msg.transform.translation.z
                base_offset_y = app.base_tf_msg.transform.translation.y
                base_offset_z = app.base_tf_msg.transform.translation.x
                

            if app.flag == True:
                
                print("Cluthching")
                app.base_tf_msg.transform.translation.x = pose_data[2]
                app.base_tf_msg.transform.translation.y = pose_data[0]
                app.base_tf_msg.transform.translation.z = pose_data[1]

                # base->tracker
                rtx,rty,rtz,rtw=app.rotate_quat(w=app.base_tf_msg.transform.rotation.w, x=app.base_tf_msg.transform.rotation.x, 
                                                y=app.base_tf_msg.transform.rotation.y, z=app.base_tf_msg.transform.rotation.z,
                                                qw=pose_data[3],qx=pose_data[4],qy=pose_data[5],qz=pose_data[6])
                quat_test=[0.7071,0.7071,0,0]

                rtx,rty,rtz,rtw=app.rotate_quat(w=rtw, x=rtx, 
                                                y=rty, z=rtz,
                                                qw=quat_test[0],qx=quat_test[1],qy=quat_test[2],qz=quat_test[3])
                quat_test=[0.7071,0,0,-0.7071]

                rtx,rty,rtz,rtw=app.rotate_quat(w=rtw, x=rtx, 
                                                y=rty, z=rtz,
                                                qw=quat_test[0],qx=quat_test[1],qy=quat_test[2],qz=quat_test[3])
                
                #we apply the last saved translation before clutching
                app.transform_msg.transform.translation.x = memorized_x
                app.transform_msg.transform.translation.y = memorized_y
                app.transform_msg.transform.translation.z = memorized_z

                #FOR WE INVERTED X AND Y HERE
                app.transform_msg.transform.rotation.x=rty
                app.transform_msg.transform.rotation.y=-rtx
                app.transform_msg.transform.rotation.z=rtz
                app.transform_msg.transform.rotation.w=rtw

                memorized_x_c = memorized_x
                memorized_y_c = memorized_y
                memorized_z_c = memorized_z


            else: 
                # print("MC loop",memorized_x_c)
                # base->tracker
                rtx,rty,rtz,rtw=app.rotate_quat(w=app.base_tf_msg.transform.rotation.w, x=app.base_tf_msg.transform.rotation.x, 
                                                y=app.base_tf_msg.transform.rotation.y, z=app.base_tf_msg.transform.rotation.z,
                                                qw=pose_data[3],qx=pose_data[4],qy=pose_data[5],qz=pose_data[6])
                
                quat_test=[0.7071,0.7071,0,0]

                rtx,rty,rtz,rtw=app.rotate_quat(w=rtw, x=rtx, 
                                                y=rty, z=rtz,
                                                qw=quat_test[0],qx=quat_test[1],qy=quat_test[2],qz=quat_test[3])
                
                quat_test=[0.7071,0,0,-0.7071]

                rtx,rty,rtz,rtw=app.rotate_quat(w=rtw, x=rtx, 
                                                y=rty, z=rtz,
                                                qw=quat_test[0],qx=quat_test[1],qy=quat_test[2],qz=quat_test[3])
                
                app.transform_msg.transform.translation.x = pose_data[1]-base_offset_x+memorized_x_c
                app.transform_msg.transform.translation.y = pose_data[2]-base_offset_z+memorized_y_c
                app.transform_msg.transform.translation.z = pose_data[0]-base_offset_y+memorized_z_c
                
                #memorizing for the clutch
                memorized_x = app.transform_msg.transform.translation.x 
                memorized_y = app.transform_msg.transform.translation.y 
                memorized_z = app.transform_msg.transform.translation.z 

                # print(memorized_x)
                #FOR WE INVERTED X AND Y HERE
                app.transform_msg.transform.rotation.x=rty
                app.transform_msg.transform.rotation.y=-rtx
                app.transform_msg.transform.rotation.z=rtz
                app.transform_msg.transform.rotation.w=rtw

            previous_flag =  app.flag
            

            #tracker->mapping for the input
            rtx2,rty2,rtz2,rtw2=app.rotate_quat(w=rtw, x=rtx, y=rty, z=rtz,
                                        qw=app.quat2[0],qx=app.quat2[1],qy=app.quat2[2],qz=app.quat2[3])
            
            app.input_transform.transform.translation.x = pose_data[1] 
            app.input_transform.transform.translation.y = pose_data[2] 
            app.input_transform.transform.translation.z = pose_data[0] 

            # print("x: ",pose_data[1]," y: ", pose_data[2]," z: ", pose_data[0])

            app.input_transform.transform.rotation.x=rtx2
            app.input_transform.transform.rotation.y=rty2
            app.input_transform.transform.rotation.z=rtz2
            app.input_transform.transform.rotation.w=rtw2
            
            # Publish the Transform message
            app.pub.publish(app.transform_msg)
            app.pub_base.publish(app.base_tf_msg)
            app.br.sendTransformMessage(app.base_tf_msg)
            app.br.sendTransformMessage(app.transform_msg)
            app.br.sendTransformMessage(app.base_pcl)
            app.pub_initpos.publish(app.vMsg)
            app.pub_base_pcl.publish(app.base_pcl)

        else:
            print("Lost Tracker...")

        app.R.sleep()



