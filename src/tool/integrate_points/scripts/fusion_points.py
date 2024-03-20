#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from concurrent.futures import process
from pickletools import uint8
import rospy
import message_filters
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
import numpy as np
import time


lidar_points_type = 0   #0: XYZI(rslidar)   2: XYZIRT(rslidar)
filter_depth = 2.5


#ring map -25 ~ 15
RING_MAP_RUBY = [62 ,83 ,84 ,1 ,86 ,88 ,2 ,4 ,5 ,7 ,8 ,9 ,11 ,12 ,13 ,14 ,15 ,16 ,18 ,19 ,20 ,21 ,22 ,24 ,25 ,26 ,27 ,29 ,30 ,31 ,33 ,34 ,35 ,37 ,38 ,39 ,41 ,42 ,44 ,46 ,48 ,50 ,52 ,54 ,56 ,58 ,60 ,63 ,65 ,67 ,69 ,71 ,73 ,75 ,77 ,79 ,81 ,89 ,91 ,93 ,95 ,97 ,99 ,101 ,103 ,105 ,107 ,109 ,111 ,113 ,115 ,117 ,119 ,121 ,123 ,125 ,127 ,128 ,126 ,124 ,122 ,120 ,118 ,116 ,114 ,112 ,110 ,108 ,106 ,104 ,102 ,100 ,98 ,96 ,94 ,92 ,90 ,82 ,80 ,78 ,76 ,74 ,72 ,70 ,68 ,66 ,64 ,61 ,59 ,57 ,55 ,53 ,51 ,49 ,47 ,45 ,43 ,40 ,36 ,32 ,28 ,23 ,17 ,10 ,6 ,3 ,87 ,85 ]
RING_MAP_16 = [0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8]        #16+(7-x)
#深度点云坐标到激光雷达坐标的转换矩阵的转置
T_left = np.array([0.4226184,0,0.9063077999999999953,0,-0.30997549999999998226,-0.9396926,0.144544,0,0.85165069999999995166,-0.34202019999999996824,-0.3971314,0,0.0988199999999999812,0.045970000000000029184,-0.10292999999999998608,1]).reshape(4,4)                                #需修改（转换矩阵的转置）
T_right = np.array([0.4226184,0,0.9063077999999999953,0,0.30997549999999998226,-0.9396926,-0.144544,0,0.85165069999999995166,0.34202019999999996824,-0.3971314,0,0.0988199999999999812,-0.045970000000000029184,-0.10292999999999998608,1]).reshape(4,4)                                #需修改（转换矩阵的转置）



def ConvertDepthPoints(depth_pc,T):
    import time
    readdepthtimestart = time.time()
    #获取深度相机数据并处理
    assert isinstance(depth_pc, PointCloud2)
    depth_gen = point_cloud2.read_points(depth_pc, field_names=("x", "y", "z"), skip_nans=True)
    depth_data = np.array(list(depth_gen))

    readdepthtimeend = time.time()

    #滤掉距离较远的点
    depth_data = np.delete(depth_data, np.where(depth_data[:,2] > filter_depth), axis = 0)

    filterdepthtimeend = time.time()

    len_points = len(depth_data)

    #深度相机数据坐标转换到激光雷达坐标系下
    allone = np.ones((len_points,1))       #创建全一矩阵
    depth_data_one = np.c_[depth_data,allone]
    depth_data_trans = np.matmul(depth_data_one, T)     #计算转换后的点的坐标

    RTdepthtimeend = time.time()

    #计算线数
    dist = np.linalg.norm(depth_data_trans[:,0:3], 2, axis=1)
    pitch = np.arcsin(depth_data_trans[:,2] / dist) # 俯仰角 arcsin(z, depth)  弧度制
    ring = (np.clip((np.around(pitch / np.pi * 180 / 2) + 40),0 , 51) % 52).astype(np.uint16)
    # ring = pitch_angle % 52
    # print(ring.shape)

    # for i in range(len_points):
    #     if pitch_angle[i] >= -15.999999 and pitch_angle[i] <= 15.99999:
    #         ring[i] = RING_MAP_16[int(round((pitch_angle[i] + 15.0) / 2.0))]
    #     elif pitch_angle[i] < -15.999999 and pitch_angle[i] > -79.95:
    #         ring[i] = round(-16.49 - pitch_angle[i] + 16)
    #     elif pitch_angle[i] <= -79.98:
    #         ring[i] = 79
    #     elif pitch_angle[i] > 15.999999 and pitch_angle[i] < 24.95:
    #         ring[i] = round(pitch_angle[i] - 16.49 + 80)
    #     else:
    #         ring[i] = 87
    #查看相机点云俯仰角范围和ring范围
    # print("max_left:",pitch.max()/np.pi*180.0,ring.max())
    # print("min_left:",pitch.min()/np.pi*180.0, ring.min())
    #转换为uint16数据格式

    ringdepthtimeend = time.time()


    #深度相机添加intensity、ring、time数据
    timestamp = np.full((len_points,1),float(depth_pc.header.stamp.secs + depth_pc.header.stamp.nsecs/1000000000))
    depth_data_full = np.rec.fromarrays((depth_data_trans[:,0].astype(np.float32), depth_data_trans[:,1].astype(np.float32), depth_data_trans[:,2].astype(np.float32), allone[:,0].astype(np.uint8),ring.astype(np.uint16), timestamp[:,0].astype(np.float64)), names=('x', 'y', 'z', 'intensity', 'ring', 'time'))

    fulldepthtimeend = time.time()
    #输出测试
    # print("depth data:")
    # print(depth_data_full[2735])


    print("depth read time:",readdepthtimeend - readdepthtimestart)
    print("depth filter time:",filterdepthtimeend - readdepthtimeend)
    print("depth RT time:",RTdepthtimeend - filterdepthtimeend)
    print("depth ring time:",ringdepthtimeend - RTdepthtimeend)
    print("depth full time:",fulldepthtimeend - ringdepthtimeend)


    return depth_data_full






def callback(lidar_pc, depth_left_pc, depth_right_pc):
    #开始运行时间
    start = time.time()

    #定义话题发布者
    pub = rospy.Publisher('depth_lidar_fusion_pc2', PointCloud2, queue_size=5)                  #融合点云的话题名
    rate = rospy.Rate(10)                                                                        #发布频率（hz）

    startlidar = time.time()

    #获取激光雷达数据并处理
    assert isinstance(lidar_pc, PointCloud2)
    #读取数据(按照不同的数据类型)
    if lidar_points_type == 0:
        lidar_gen = point_cloud2.read_points(lidar_pc, field_names=("x", "y", "z","intensity"), skip_nans=True)
    elif lidar_points_type == 2:
        lidar_gen = point_cloud2.read_points(lidar_pc, field_names=("x", "y", "z","intensity","ring","time"), skip_nans=True)

    #转换为numpy数组
    lidar_data = np.array(list(lidar_gen))

    readlidar = time.time()
    
    if lidar_points_type == 0:
        #XYZI设置ring为1
        allone = np.ones((len(lidar_data),1))
        #设置时间戳
        time_rslidar = np.full((len(lidar_data),1),float(lidar_pc.header.stamp.secs+lidar_pc.header.stamp.nsecs/1000000000))
        #合成为一个record数组(允许每一列列为不同类型的矩阵)
        lidar_data_full = np.rec.fromarrays((lidar_data[:,0].astype(np.float32), lidar_data[:,1].astype(np.float32), lidar_data[:,2].astype(np.float32), lidar_data[:,3].astype(np.uint8), allone[:,0].astype(np.uint16), time_rslidar[:,0].astype(np.float64)), names=('x', 'y', 'z', 'intensity', 'ring', 'time'))
    elif lidar_points_type == 2:
        #XYZIRT
        # ringUINT16 = lidar_data[:,4].astype(np.uint16)
        #重新生成为一个record数组
        lidar_data_full = np.rec.fromarrays((lidar_data[:,0].astype(np.float32), lidar_data[:,1].astype(np.float32), lidar_data[:,2].astype(np.float32), lidar_data[:,3].astype(np.uint8), lidar_data[:,4].astype(np.uint16), lidar_data[:,5].astype(np.float64)), names=('x', 'y', 'z', 'intensity', 'ring', 'time'))
    

    processlidar = time.time()
    print("lidar read time:",readlidar - startlidar)
    print("lidar process time:",processlidar - readlidar)
    print("lidar time:",processlidar - startlidar)


    #读取并处理左右相机点云
    depthlefttime = time.time()
    depth_left_data_full = ConvertDepthPoints(depth_left_pc,T_left)
    depthleftendtime = time.time()
    print("depth left time:",depthleftendtime - depthlefttime)


    depthrighttime = time.time()
    depth_right_data_full = ConvertDepthPoints(depth_right_pc,T_right)
    depthrightendtime = time.time()
    print("depth right time:",depthrightendtime - depthrighttime)
    
    #总数据点个数
    lenfusion = len(lidar_data_full) + len(depth_left_data_full) + len(depth_right_data_full)
    
    pubtimestart = time.time()

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
            PointField('intensity', 12, PointField.UINT8, 1),
            PointField('ring', 13, PointField.UINT16, 1),
            PointField('timestamp', 15, PointField.FLOAT64, 1)]
        msg.is_bigendian = False
        msg.point_step = 23
        msg.row_step = msg.point_step * lenfusion
        msg.is_dense = False
        msg.data = lidar_data_full.tostring() + depth_left_data_full.tostring() + depth_right_data_full.tostring()


        # test_gen = point_cloud2.read_points(msg, field_names=("x", "y", "z","intensity","ring","timestamp"), skip_nans=True)
        # test_data = np.array(list(test_gen))
        # print("data:")
        # print(lidar_data_full[0])
        # print(test_data[0])

        pub.publish(msg)
        # print("publishing...")
        end1 = time.time()
        print("pub time:",end1 - pubtimestart)
        rate.sleep()
        #end2 = time.time()
        print("total time:",end1 - start)
        break
    



def listener():
    import time
    print("start!!!")
    #设置节点名
    rospy.init_node('fusion_points', anonymous=True)
    subtimestart = time.time()
    #订阅三个话题
    lidar_pc = message_filters.Subscriber('/rslidar_points', PointCloud2)                                      #修改这里的接受话题名
    depth_left_pc = message_filters.Subscriber('/camera_front_left/depth/color/points', PointCloud2)
    depth_right_pc = message_filters.Subscriber('/camera_front_right/depth/color/points', PointCloud2)
    #print("received...")
    #时间近似同步
    ts = message_filters.ApproximateTimeSynchronizer([lidar_pc, depth_left_pc, depth_right_pc], 10, 0.04, allow_headerless=True)
    subtimeend = time.time()
    print("sub time:",subtimeend - subtimestart)
    #调用回调函数
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
