#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np

def talker():

    pub1 = rospy.Publisher('pc1', PointCloud2, queue_size=5)
    pub2 = rospy.Publisher('pc2', PointCloud2, queue_size=5)
    rospy.init_node('pointcloud_publisher_node', anonymous=True)
    rate = rospy.Rate(10)

    

    while not rospy.is_shutdown():
        points1 = np.random.random(size=300000).reshape(100000,3)
        points2 = np.random.random(size=300000).reshape(100000,3)

        msg1 = PointCloud2()
        msg1.header.stamp = rospy.Time().now()
        msg1.header.frame_id = "lidar_frame"

        msg2 = PointCloud2()
        msg2.header.stamp = rospy.Time().now()
        msg2.header.frame_id = "depth_frame"

        if len(points1.shape) == 3:
            msg1.height = points1.shape[1]
            msg1.width = points1.shape[0]
        else:
            msg1.height = 1
            msg1.width = len(points1)

        if len(points2.shape) == 3:
            msg2.height = points2.shape[1]
            msg2.width = points2.shape[0]
        else:
            msg2.height = 1
            msg2.width = len(points2)

        msg1.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg1.is_bigendian = False
        msg1.point_step = 12
        msg1.row_step = msg1.point_step * points1.shape[0]
        msg1.is_dense = False
        msg1.data = np.asarray(points1, np.float32).tostring()

        msg2.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg2.is_bigendian = False
        msg2.point_step = 12
        msg2.row_step = msg2.point_step * points2.shape[0]
        msg2.is_dense = False
        msg2.data = np.asarray(points2, np.float32).tostring()

        pub1.publish(msg1)
        pub2.publish(msg2)
        print("published...")
        rate.sleep()
        

if __name__ == '__main__':     

    talker()