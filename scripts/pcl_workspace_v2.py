#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
import numpy as np
import struct
import tf
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Vector3
from ros_numpy import point_cloud2

from pyquaternion import Quaternion

class workspace():

    def __init__(self):
        
        rospy.init_node('cube_point_cloud_publisher')
        
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
        self.pub = rospy.Publisher('glove_workspace_tf', TransformStamped, queue_size=10)
        self.R = rospy.Rate(100)

        self.br = tf.TransformBroadcaster()        
        self.base_gloveinput = TransformStamped()
        self.base_gloveinput.child_frame_id = "psm2_base_flipped"
        self.base_gloveinput.header.frame_id = "psm2_base"

        self.base_gloveinput.transform.translation.x = 0       
        self.base_gloveinput.transform.translation.y = 0
        self.base_gloveinput.transform.translation.z = 0
        
        self.base_gloveinput.transform.rotation.x=0
        self.base_gloveinput.transform.rotation.y=-0.7071
        self.base_gloveinput.transform.rotation.z=0
        self.base_gloveinput.transform.rotation.w=0.7071

        self.br = tf.TransformBroadcaster()        
        self.input = TransformStamped()
        self.input.child_frame_id = "next_translation"
        self.input.header.frame_id = "psm2_base_flipped"


        self.input.transform.rotation.x = 0.5
        self.input.transform.rotation.y = -0.5
        self.input.transform.rotation.z = 0.5
        self.input.transform.rotation.w = 0.5


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

        psm2_cp = rospy.wait_for_message("/PSM2/custom/local/setpoint_cp",TransformStamped)

        # Define the size of the cube
        cube_size = 0.05  

    # Define the offset values (change these values as needed)
        offset_x = psm2_cp.transform.translation.x  # Example offset for x
        offset_y = psm2_cp.transform.translation.y  # Example offset for y
        offset_z = psm2_cp.transform.translation.z  # Example offset for z

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
        cube_pcl = point_cloud2.array_to_pointcloud2(xyzrgb,frame_id="psm2_base")

        return cube_pcl


    


    def rescale_value(self,original_value, min_old, max_old,min_new,max_new):

            

            rescaled_value = ((float(original_value) - min_old) * (max_new - min_new)) / (max_old - min_old) + min_new

            # if rescaled_value > max_new:
            #     rescaled_value = max_new

            # elif rescaled_value < min_new:
            #     rescaled_value = min_new

            return rescaled_value
    
    def callback(self,tracker):
        
        self.x = tracker.transform.translation.x
        self.y = tracker.transform.translation.y
        self.z = tracker.transform.translation.z
        
        self.rx = tracker.transform.rotation.x
        self.ry = tracker.transform.rotation.y
        self.rz = tracker.transform.rotation.z
        self.rw = tracker.transform.rotation.w
    
    def callback_workspace(self, init_pos):
        if init_pos is not None:

            size_hand_workspace = 0.5 
            # size_hand_workspacex=0.8


            self.min_val_old[0] = init_pos.x#-size_hand_workspace
            self.max_val_old[0] = init_pos.x+2*size_hand_workspace

            self.min_val_old[1] = init_pos.y-size_hand_workspace
            self.max_val_old[1] = init_pos.y+size_hand_workspace

            self.min_val_old[2] = init_pos.z-size_hand_workspace
            self.max_val_old[2] = init_pos.z+size_hand_workspace
            
        else: 
            pass

        
    def exctracting_boundaries(self,first_point,last_point):

        # Extract the minimum and maximum values of x, y, and z
        min_val = [
            min(first_point[0], last_point[0]),
            min(first_point[1], last_point[1]),
            min(first_point[2], last_point[2])
        ]

        max_val = [
            max(first_point[0], last_point[0]),
            max(first_point[1], last_point[1]),
            max(first_point[2], last_point[2])
        ]

        return min_val,max_val

    def rotate_quat(self,w,x,y,z,qw,qx,qy,qz):
           
            original_quat = Quaternion(w=w, x=x, y=y, z=z)
            rotation_quat = Quaternion(w=qw, x=qx, y=qy, z=qz)
            resulting_quat = (original_quat * rotation_quat).normalised 
           
            return resulting_quat.x, resulting_quat.y, resulting_quat.z, resulting_quat.w

def main():

    app=workspace()
    
    pub = rospy.Publisher('/cube_point_cloud', PointCloud2, queue_size=10)
    pub_tf = rospy.Publisher('/GLOVE_INPUT/T', TransformStamped, queue_size=10)

    rate = rospy.Rate(100) 
    cloud = app.create_cube_point_cloud()

    rospy.Subscriber('vive_tracker_transform',TransformStamped,app.callback)
    rospy.Subscriber('tracker_initial_position',Vector3,app.callback_workspace)

    points = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
    first_point = points[0]
    last_point = points[-1]
    
    min_val,max_val=app.exctracting_boundaries(first_point,last_point)
    print(min_val,max_val)

    while not rospy.is_shutdown():

        t = rospy.Time.now()

        app.base_gloveinput.header.stamp = t
        app.input.header.stamp = t

        app.br.sendTransformMessage(app.base_gloveinput)

        
        try:

            x= app.rescale_value(app.x,app.min_val_old[0],app.max_val_old[0],min_val[0],max_val[0])
            y= app.rescale_value(app.y,app.min_val_old[1],app.max_val_old[1],min_val[1],max_val[1])
            z= app.rescale_value(app.z,app.min_val_old[2],app.max_val_old[2],min_val[2],max_val[2])
            
            app.input.transform.translation.x = x-(min_val[0]+max_val[0])/2-0.04
            app.input.transform.translation.y = y-(min_val[1]+max_val[1])/2+0.01
            app.input.transform.translation.z = z+0.78
            
            
            app.br.sendTransformMessage(app.input)

        except:
            print("error")

        cloud.header.stamp = rospy.Time.now()

        pub.publish(cloud)
        pub_tf.publish(app.input)
        rate.sleep()

if __name__ == '__main__':
    main()
