#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped,PoseStamped
import numpy as np
import struct
import tf
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Vector3
from ros_numpy import point_cloud2
from std_msgs.msg import Bool
from sitl_dvrk.msg import BoolStamped
from pyquaternion import Quaternion

class workspace():

    def __init__(self):
        
        rospy.init_node('cube_point_cloud_publisher_r')
        
        self.min_val_old = [0, 0, 0]
        self.max_val_old = [0,0,0]

        self.BIT_MOVE_16 = 2**16
        self.BIT_MOVE_8  = 2**8

        self.cube_points = None
        self.is_first_iteration = True

        self.x = None
        self.y = None
        self.z = None
        self.rx = None
        self.ry = None
        self.rz = None
        self.rw = None

        # Create a publisher
        self.pub = rospy.Publisher('glove_workspace_tf_r', TransformStamped, queue_size=10)
        self.R = rospy.Rate(100)

        self.br = tf.TransformBroadcaster()        
        self.base_gloveinput = TransformStamped()
        self.base_gloveinput.child_frame_id = "psm1_base_flipped"
        self.base_gloveinput.header.frame_id = "psm1_base"

        self.base_gloveinput.transform.translation.x = 0       
        self.base_gloveinput.transform.translation.y = 0
        self.base_gloveinput.transform.translation.z = 0
        
        self.base_gloveinput.transform.rotation.x=0
        self.base_gloveinput.transform.rotation.y=-0.7071
        self.base_gloveinput.transform.rotation.z=0
        self.base_gloveinput.transform.rotation.w=0.7071

        self.br = tf.TransformBroadcaster()        
        self.input = TransformStamped()
        self.input.child_frame_id = "next_translation_r"
        self.input.header.frame_id = "psm1_base_flipped"

        self.input.transform.rotation.x = 0.5
        self.input.transform.rotation.y = -0.5
        self.input.transform.rotation.z = 0.5
        self.input.transform.rotation.w = 0.5

        #clutch
        self.flag = False

        #cube    
        self.points = []
        self.first_point = None
        self.last_point = None
        self.min_val = None
        self.max_val = None
        self.cloud = None
    
    def pt_array_3d_to_pcl(self,pt_array_3d,color=(255,0,0)):
        data = np.zeros(
            pt_array_3d.shape[0],
            dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('rgb', np.uint32)
            ]
        )

        data['x'] = pt_array_3d[:,0]
        data['y'] = pt_array_3d[:,1]
        data['z'] = pt_array_3d[:,2]
        color = np.repeat(np.array([np.asarray(color)]),pt_array_3d.shape[0],axis=0)
        rgb_data = color[:,0]*self.BIT_MOVE_16 + color[:,1]*self.BIT_MOVE_8 + color[:,2]
        rgb_data = rgb_data.astype(np.uint32)
        data['rgb'] = rgb_data

        return data
    


    def create_cube_point_cloud(self):

        psm2_cp = rospy.wait_for_message("/PSM1/custom/local/setpoint_cp",PoseStamped)

        # Define the size of the cube
        cube_size = 0.07 

    # Define the offset values (change these values as needed)
        offset_x = psm2_cp.pose.position.x  # Example offset for x
        offset_y = psm2_cp.pose.position.y  # Example offset for y
        offset_z = psm2_cp.pose.position.z  # Example offset for z

        # Calculate the start and end points for x, y, and z to center the cube at (0, 0, 0)
        start = -cube_size / 2.0
        end = cube_size / 2.0

        x = np.linspace(start, end, num=2)
        y = np.linspace(start, end, num=2)
        z = np.linspace(start, end, num=2)

        points = []

        for xi in x:
            for yi in y:
                for zi in z:
                    # Adjust the coordinates by the offset
                    points.append([xi + offset_x, yi + offset_y, zi + offset_z])

        points = np.array(points)
        xyzrgb = self.pt_array_3d_to_pcl(points)
        cube_pcl = point_cloud2.array_to_pointcloud2(xyzrgb,frame_id="psm1_base")

        return cube_pcl


    def rescale_value(self, original_values, min_old_values, max_old_values, min_new_values, max_new_values):
        rescaled_values = []
        for i in range(len(original_values)):
            rescaled_value = ((float(original_values[i]) - min_old_values[i]) * (max_new_values[i] - min_new_values[i])) / (max_old_values[i] - min_old_values[i]) + min_new_values[i]
            rescaled_values.append(rescaled_value)

        return rescaled_values
    

    def callback(self,tracker):
        
        
        self.x = tracker.transform.translation.x
        self.y = tracker.transform.translation.y
        self.z = tracker.transform.translation.z

        self.rx = tracker.transform.rotation.x
        self.ry = tracker.transform.rotation.y
        self.rz = tracker.transform.rotation.z
        self.rw = tracker.transform.rotation.w

        
    def get_min_max(self):
        while not rospy.is_shutdown():
            init_pos = rospy.wait_for_message('tracker_current_pos_tf_r',TransformStamped)
            if init_pos is not None:

                #IF YOU CHANGE THIS VALUE CHANGE IT ALSO IN NODE_TRACKER.PY
                size_hand_workspace = 0.15

                self.min_val_old[0] = init_pos.transform.translation.x-size_hand_workspace
                self.max_val_old[0] = init_pos.transform.translation.x+size_hand_workspace

                self.min_val_old[1] = init_pos.transform.translation.y-size_hand_workspace
                self.max_val_old[1] = init_pos.transform.translation.y+size_hand_workspace

                self.min_val_old[2] = init_pos.transform.translation.z-size_hand_workspace
                self.max_val_old[2] = init_pos.transform.translation.z+size_hand_workspace
                break
        

    def exctracting_boundaries(self):

        min_x = min_y = min_z = float('inf')
        max_x = max_y = max_z = float('-inf')

        for point in self.points:
            x, y, z = point
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            min_z = min(min_z, z)
            max_x = max(max_x, x)
            max_y = max(max_y, y)
            max_z = max(max_z, z)

        min_val=[min_x,min_y,min_z]
        max_val=[max_x,max_y,max_z]
        
        return min_val,max_val


    def rotate_quat(self,w,x,y,z,qw,qx,qy,qz):
            original_quat = Quaternion(w=w, x=x, y=y, z=z)
            rotation_quat = Quaternion(w=qw, x=qx, y=qy, z=qz)
            resulting_quat = (original_quat * rotation_quat).normalised 
           
            return resulting_quat.x, resulting_quat.y, resulting_quat.z, resulting_quat.w
    

    def callback_clutch(self,fist_bool):
        self.flag = fist_bool.data


def main():
    app=workspace()
    
    pub = rospy.Publisher('/cube_point_cloud_r', PointCloud2, queue_size=10)
    pub_tf = rospy.Publisher('/GLOVE_INPUT/T_r', TransformStamped, queue_size=10)

    rate = rospy.Rate(100) 
    app.cloud = app.create_cube_point_cloud()
    rospy.Subscriber('tracker_current_pos_tf_r',TransformStamped,app.callback)
    rospy.Subscriber("glove/left/fist", BoolStamped, app.callback_clutch)

    app.points = list(pc2.read_points(app.cloud, field_names=("x", "y", "z"), skip_nans=True))
    app.min_val,app.max_val=app.exctracting_boundaries()

    app.get_min_max()

    while not rospy.is_shutdown():

        t = rospy.Time.now()

        app.base_gloveinput.header.stamp = t
        app.input.header.stamp = t

        app.br.sendTransformMessage(app.base_gloveinput)

        try:
            coord = [app.x, app.y, app.z]

            temp = "\nmin hand  {} \nmax hand  {} \nmin robot {} \nmax robot {}".format(
                app.min_val_old, app.max_val_old, app.min_val, app.max_val
            )

            rospy.loginfo_once(temp)
            x, y, z = app.rescale_value(coord, app.min_val_old, app.max_val_old, app.min_val, app.max_val)
            
            app.input.transform.translation.x = x  
            app.input.transform.translation.y = y  #this because left become right, right become left
            app.input.transform.translation.z = z  
            
            app.br.sendTransformMessage(app.input)

        except Exception as e:
            print(e)

        app.cloud.header.stamp = rospy.Time.now()

        pub.publish(app.cloud)
        pub_tf.publish(app.input)
        rate.sleep()

if __name__ == '__main__':
    main()
