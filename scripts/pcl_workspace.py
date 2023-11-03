#!/usr/bin/env python

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np
import struct

cube_points = None
is_first_iteration = True

def create_cube_point_cloud():
    
    # Define the size of the cube
    cube_size = 0.2  # 1 meter
    point_step = 0.1  # Spacing between points

   # Define the offset values (change these values as needed)
    offset_x = 0.05  # Example offset for x
    offset_y = 0.05  # Example offset for y
    offset_z = 0.05  # Example offset for z

    # Calculate the start and end points for x, y, and z to center the cube at (0, 0, 0)
    start = -cube_size / 2.0
    end = cube_size / 2.0

    x = np.arange(start, end, point_step)
    y = np.arange(start, end, point_step)
    z = np.arange(start, end, point_step)

    points = []

    for xi in x:
        for yi in y:
            for zi in z:
                # Adjust the coordinates by the offset
                points.append([xi + offset_x, yi + offset_y, zi + offset_z])

    # Convert points to PointCloud2 format
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'psm2_tip'

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.is_dense = False
    cloud.is_bigendian = False
    cloud.point_step = 12  # 3 * float32
    cloud.row_step = cloud.point_step * cloud.width
    cloud.fields = fields

    buffer = []

    for point in points:
        for value in point:
            buffer.append(struct.pack('f', value))

    buffer = b''.join(buffer)  # Join with bytes object instead of string
    cloud.data = buffer

    # Set the flag to False since it's no longer the first iteration
    is_first_iteration = False
    cube_points=cloud

    return cloud



def main():
  
    rospy.init_node('cube_point_cloud_publisher')

    pub = rospy.Publisher('/cube_point_cloud', PointCloud2, queue_size=10)

    rate = rospy.Rate(100)  # 1 Hz

    while not rospy.is_shutdown():
        cloud = create_cube_point_cloud()
        pub.publish(cloud)
        rate.sleep()

if __name__ == '__main__':
    main()
