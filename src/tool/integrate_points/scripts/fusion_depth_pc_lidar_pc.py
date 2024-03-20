#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
import message_filters
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
import numpy as np
import time

rgb = np.array((255 << 16) | (255 << 8) | (255 << 0),dtype=np.uint32)
rgb.dtype = np.float32

RING_MAP = [62 ,83 ,84 ,1 ,86 ,88 ,2 ,4 ,5 ,7 ,8 ,9 ,11 ,12 ,13 ,14 ,15 ,16 ,18 ,19 ,20 ,21 ,22 ,24 ,25 ,26 ,27 ,29 ,30 ,31 ,33 ,34 ,35 ,37 ,38 ,39 ,41 ,42 ,44 ,46 ,48 ,50 ,52 ,54 ,56 ,58 ,60 ,63 ,65 ,67 ,69 ,71 ,73 ,75 ,77 ,79 ,81 ,89 ,91 ,93 ,95 ,97 ,99 ,101 ,103 ,105 ,107 ,109 ,111 ,113 ,115 ,117 ,119 ,121 ,123 ,125 ,127 ,128 ,126 ,124 ,122 ,120 ,118 ,116 ,114 ,112 ,110 ,108 ,106 ,104 ,102 ,100 ,98 ,96 ,94 ,92 ,90 ,82 ,80 ,78 ,76 ,74 ,72 ,70 ,68 ,66 ,64 ,61 ,59 ,57 ,55 ,53 ,51 ,49 ,47 ,45 ,43 ,40 ,36 ,32 ,28 ,23 ,17 ,10 ,6 ,3 ,87 ,85 ]




def callback(lidar_pc, depth_left_pc, depth_right_pc):
    #print("callback")
    #start = time.time()
    #深度点云坐标到激光雷达坐标的转换矩阵的转置
    T_left = np.array([0.4226184,0,0.9063077999999999953,0,-0.30997549999999998226,-0.9396926,0.144544,0,0.85165069999999995166,-0.34202019999999996824,-0.3971314,0,0.0988199999999999812,0.045970000000000029184,-0.10292999999999998608,1]).reshape(4,4)                                #需修改（转换矩阵的转置）
    T_right = np.array([0.4226184,0,0.9063077999999999953,0,0.30997549999999998226,-0.9396926,-0.144544,0,0.85165069999999995166,0.34202019999999996824,-0.3971314,0,0.0988199999999999812,-0.045970000000000029184,-0.10292999999999998608,1]).reshape(4,4)                                #需修改（转换矩阵的转置）

    #定义话题发布者
    pub = rospy.Publisher('depth_lidar_fusion_pc2', PointCloud2, queue_size=5)                  #融合点云的话题名
    rate = rospy.Rate(10)                                                                        #发布频率（hz）

    #获取激光雷达数据
    assert isinstance(lidar_pc, PointCloud2)
    # lidar_gen = point_cloud2.read_points(lidar_pc, field_names=("x", "y", "z","intensity","ring","time"), skip_nans=True)
    lidar_gen = point_cloud2.read_points(lidar_pc, field_names=("x", "y", "z","intensity"), skip_nans=True)

    lidar_data = np.array(list(lidar_gen))

    #为激光雷达数据添加rgb信息
    allrgb = np.full((len(lidar_data),1),rgb)


    #allone = np.ones((len(lidar_data),1))
    
    #XYZI
    alloneUINT16 = np.ones((len(lidar_data),1)).astype(np.uint16)
    time_rslidar = np.full((len(lidar_data),1),float(lidar_pc.header.stamp.secs+lidar_pc.header.stamp.nsecs/1000000000))
    lidar_data_full = np.rec.fromarrays((lidar_data[:,0].astype(np.float32), lidar_data[:,1].astype(np.float32), lidar_data[:,2].astype(np.float32), allrgb[:,0].astype(np.float32), lidar_data[:,3].astype(np.uint8), alloneUINT16[:,0], time_rslidar[:,0].astype(np.float64)), names=('x', 'y', 'z', 'rgb', 'intensity', 'ring', 'time'))
    
    #lidar_data_full = np.rec.fromarrays((lidar_data[:,0].astype(np.float32), lidar_data[:,1].astype(np.float32), lidar_data[:,2].astype(np.float32), allrgb[:,0].astype(np.float32), lidar_data[:,3].astype(np.float32), lidar_data[:,4].astype(np.uint16), lidar_data[:,5].astype(np.float32)), names=('x', 'y', 'z', 'rgb', 'intensity', 'ring', 'time'))
    #lidar_data_full = np.c_[lidar_data[:,0:3],allrgb,lidar_data[:,3:6]]


    '''
    #XYZIRT
    ringUINT16 = lidar_data[:,4].astype(np.uint16)
    lidar_data_full = np.rec.fromarrays((lidar_data[:,0].astype(np.float32), lidar_data[:,1].astype(np.float32), lidar_data[:,2].astype(np.float32), allrgb[:,0].astype(np.float32), lidar_data[:,3].astype(np.uint8), lidar_data[:,4].astype(np.uint16), lidar_data[:,5].astype(np.float64)), names=('x', 'y', 'z', 'rgb', 'intensity', 'ring', 'time'))
    '''

    
    #获取深度相机数据
    assert isinstance(depth_left_pc, PointCloud2)
    depth_left_gen = point_cloud2.read_points(depth_left_pc, field_names=("x", "y", "z","rgb"), skip_nans=True)
    depth_left_data = np.array(list(depth_left_gen))

    depth_left_data = np.delete(depth_left_data, np.where(depth_left_data[:,2] > 4), axis = 0)

    assert isinstance(depth_right_pc, PointCloud2)
    depth_right_gen = point_cloud2.read_points(depth_right_pc, field_names=("x", "y", "z","rgb"), skip_nans=True)
    depth_right_data = np.array(list(depth_right_gen))

    depth_right_data = np.delete(depth_right_data, np.where(depth_right_data[:,2] > 4), axis = 0)


    #深度相机数据坐标转换到激光雷达坐标系下
    allone = np.ones((len(depth_left_data),1))
    depth_left_data_one = np.c_[depth_left_data[:,0:3],allone]
    depth_left_data_trans = np.matmul(depth_left_data_one, T_left)

    allone = np.ones((len(depth_right_data),1))
    depth_right_data_one = np.c_[depth_right_data[:,0:3],allone]
    depth_right_data_trans = np.matmul(depth_right_data_one, T_right)


    #计算线数
    d_left = np.linalg.norm(depth_left_data_trans[:,0:3], 2, axis=1)
    #d_left = d_left.reshape(len(depth_left_data_trans),1)
    pitch_left = np.arcsin(depth_left_data_trans[:,2] / d_left) # arcsin(z, depth)
    ring_left_index = (np.around((pitch_left + 6.39/180.0*np.pi) / (10.3/180.0*np.pi) * 103)).astype(int)
    ring_left_index[ring_left_index>102] = 102
    ring_left_index[ring_left_index<0] = 0
    ring_left = ring_left_index
    for i in range(len(ring_left_index)):
        if ring_left_index[i] >= 0 and ring_left_index[i] <= 102:
            ring_left[i] = RING_MAP[ring_left_index[i]] - 1
    print("max_left:",pitch_left.max()/np.pi*180.0,ring_left.max())
    print("min_left:",pitch_left.min()/np.pi*180.0, ring_left.min())
    ring_left.astype(np.uint16)
    #ring_left.reshape(depth_left_data_trans.shape[0],1)



    d_right = np.linalg.norm(depth_right_data_trans[:,0:3], 2, axis=1)
    #d_right = d_right.reshape(len(depth_right_data_trans),1)
    pitch_right = np.arcsin(depth_right_data_trans[:,2] / d_right) # arcsin(z, depth)
    ring_right_index = (np.around((pitch_right + 6.39/180.0*np.pi) / (10.3/180.0*np.pi) * 103)).astype(int)
    ring_right_index[ring_right_index>102] = 102
    ring_right_index[ring_right_index<0] = 0
    ring_right = ring_right_index
    for i in range(len(ring_right_index)):
        if ring_right_index[i] >= 0 and ring_right_index[i] <= 102:
            ring_right[i] = RING_MAP[ring_right_index[i] + 13] - 1
    print("max_right:",pitch_right.max()/np.pi*180.0)
    print("min_right:",pitch_right.min()/np.pi*180.0)
    ring_right.astype(np.uint16)
    #ring_right.reshape(depth_right_data_trans.shape[0],1)
    




    #深度相机添加rgb、intensity、ring、time数据
    allone = np.ones((len(depth_left_data),1))
    # alloneUINT16 = np.ones((len(depth_left_data),1)).astype(np.uint16)
    time_left = np.full((len(depth_left_data),1),float(depth_left_pc.header.stamp.secs + depth_left_pc.header.stamp.nsecs/1000000000))
    #depth_left_data_full = np.c_[depth_left_data_trans[:,0:3],depth_left_data[:,3],allone,allone,time_left]
    depth_left_data_full = np.rec.fromarrays((depth_left_data_trans[:,0].astype(np.float32), depth_left_data_trans[:,1].astype(np.float32), depth_left_data_trans[:,2].astype(np.float32), depth_left_data[:,3].astype(np.float32),allone[:,0].astype(np.uint8),ring_left.astype(np.uint16),time_left[:,0].astype(np.float64)), names=('x', 'y', 'z', 'rgb', 'intensity', 'ring', 'time'))

    allone = np.ones((len(depth_right_data),1))
    # alloneUINT16 = np.ones((len(depth_right_data),1)).astype(np.uint16)
    time_right = np.full((len(depth_right_data),1),float(depth_right_pc.header.stamp.secs + depth_right_pc.header.stamp.nsecs/1000000000))
    #depth_right_data_full = np.c_[depth_right_data_trans[:,0:3],depth_right_data[:,3],allone,allone,time_right]
    depth_right_data_full = np.rec.fromarrays((depth_right_data_trans[:,0].astype(np.float32), depth_right_data_trans[:,1].astype(np.float32), depth_right_data_trans[:,2].astype(np.float32), depth_right_data[:,3].astype(np.float32),allone[:,0].astype(np.uint8),ring_right.astype(np.uint16),time_right[:,0].astype(np.float64)), names=('x', 'y', 'z', 'rgb', 'intensity', 'ring', 'time'))

    print("lidar_depth data:")
    # print((lidar_data_full[0]))
    print(depth_left_data_full[2735])
    # print(depth_right_data_full.shape[0])
    #数据拼接
    # fusion_data = np.r_[lidar_data_full, depth_left_data_full, depth_right_data_full]
    #fusion_data = np.append(lidar_data_full, depth_left_data_full, depth_right_data_full,axis=0)
    lenfusion = len(lidar_data_full) + len(depth_left_data_full) + len(depth_right_data_full)
    # print(lenfusion)

    
    
    #print(float(lidar_pc.header.stamp.secs+lidar_pc.header.stamp.nsecs/1000000000),float(depth_left_pc.header.stamp.secs + depth_left_pc.header.stamp.nsecs/1000000000),float(depth_right_pc.header.stamp.secs + depth_right_pc.header.stamp.nsecs/1000000000))
    #发布数据话题
    while not rospy.is_shutdown():
        msg = PointCloud2()
        msg.header.stamp = rospy.Time().now()
        msg.header.frame_id = "rslidar"                                                     #修改发布点云的相对坐标系（应是激光雷达坐标系）


        
        msg.height = 1
        msg.width = lenfusion

        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1),
            PointField('intensity', 16, PointField.UINT8, 1),
            PointField('ring', 17, PointField.UINT16, 1),
            PointField('timestamp', 19, PointField.FLOAT64, 1)]
        msg.is_bigendian = False
        msg.point_step = 27
        msg.row_step = msg.point_step * lenfusion
        msg.is_dense = False
        msg.data = lidar_data_full.tostring() + depth_left_data_full.tostring() + depth_right_data_full.tostring()
        # print("len")
        # print(lidar_data_full.shape[0])
        # print(len(lidar_data_full.tostring()))


        test_gen = point_cloud2.read_points(msg, field_names=("x", "y", "z","rgb","intensity","ring","timestamp"), skip_nans=True)
        test_data = np.array(list(test_gen))
        print("data:")
        # print(lidar_data_full[0])
        print(test_data[0])

        pub.publish(msg)
        print("publishing...")
        #end1 = time.time()
        rate.sleep()
        #end2 = time.time()
        #print("time1:",end1 - start,"   time2:",end2 - start)
        break


def listener():
    rospy.init_node('fusion_depth_pc_lidar_pc', anonymous=True)
    lidar_pc = message_filters.Subscriber('/rslidar_points', PointCloud2)                                      #修改这里的接受话题名
    depth_left_pc = message_filters.Subscriber('/camera_front_left/depth/color/points', PointCloud2)
    depth_right_pc = message_filters.Subscriber('/camera_front_right/depth/color/points', PointCloud2)
    #print("received...")
    ts = message_filters.ApproximateTimeSynchronizer([lidar_pc, depth_left_pc, depth_right_pc], 10, 0.04, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()




if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
