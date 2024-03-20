#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>



int input_type = 0;     //0:XYZI    2:XYZIRT

double depth_filter = 2.5;

ros::Publisher pubRobosensePC;

// 深度点云坐标到激光雷达坐标的转换矩阵的转置
// double T[2][16] = {{0.4226184,0,0.9063077999999999953,0,-0.30997549999999998226,-0.9396926,0.144544,0,0.85165069999999995166,-0.34202019999999996824,-0.3971314,0,0.0988199999999999812,0.045970000000000029184,-0.10292999999999998608,1},{0.4226184,0,0.9063077999999999953,0,0.30997549999999998226,-0.9396926,-0.144544,0,0.85165069999999995166,0.34202019999999996824,-0.3971314,0,0.0988199999999999812,-0.045970000000000029184,-0.10292999999999998608,1}};
double T[2][16] = {{0.4226184,0,0.9063077999999999953,0,-0.30997549999999998226,-0.9396926,0.144544,0,0.85165069999999995166,-0.34202019999999996824,-0.3971314,0,0.0988199999999999812,0.045970000000000029184,-0.10292999999999998608,1},{0.4226184,0,0.9063077999999999953,0,0.30997549999999998226,-0.9396926,-0.144544,0,0.85165069999999995166,0.34202019999999996824,-0.3971314,0,0.0988199999999999812,-0.045970000000000029184,-0.10292999999999998608,1}};


static int RING_ID_MAP_RUBY[] = {
        3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
        35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
        67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
        99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
        7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
        39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
        71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
        103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60
};

static int RING_MAP_16[] = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};

//depth camera point struct
struct DepthPointXYZC {
    PCL_ADD_POINT4D;
    float rgb = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(DepthPointXYZC,
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb))



struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
                                          (uint16_t, ring, ring)(double, timestamp, timestamp))



// velodyne的点云格式
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)(float, time, time)
)

struct VelodynePointXYZIR {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIR,
                                   (float, x, x)(float, y, y)
                                           (float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)
)




template<typename T>
bool has_nan(T point) {

    // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
    // pcl remove nan not work normally
    // ROS_ERROR("Containing nan point!");
    if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z)) {
        return true;
    } else {
        return false;
    }
}


//add x, y, z, intensity to pc_out
template<typename T_in_p, typename T_out_p>
void handle_pc_msg(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                   const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {

    // to new pointcloud
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        T_out_p new_point;
//        std::copy(pc->points[point_id].data, pc->points[point_id].data + 4, new_point.data);
        new_point.x = pc_in->points[point_id].x;
        new_point.y = pc_in->points[point_id].y;
        new_point.z = pc_in->points[point_id].z;
        new_point.intensity = pc_in->points[point_id].intensity;
//        new_point.ring = pc->points[point_id].ring;
//        // 计算相对于第一个点的相对时间
//        new_point.time = float(pc->points[point_id].timestamp - pc->points[0].timestamp);
        pc_out->points.push_back(new_point);
    }
}

//add x, y, z, i, r, t to pc_out
template<typename T_in_p, typename T_out_p>
void fusion_data(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                   const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {

    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        T_out_p new_point;
//        std::copy(pc->points[point_id].data, pc->points[point_id].data + 4, new_point.data);
        new_point.x = pc_in->points[point_id].x;
        new_point.y = pc_in->points[point_id].y;
        new_point.z = pc_in->points[point_id].z;
        new_point.intensity = pc_in->points[point_id].intensity;
        new_point.ring = pc_in->points[point_id].ring;
        new_point.time = pc_in->points[point_id].time;
        pc_out->points.push_back(new_point);
    }
}


template<typename T_in_p, typename T_out_p>
void add_ring(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    // to new pointcloud
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 跳过nan点
        pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
    }
}

template<typename T_in_p, typename T_out_p>
void add_time(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    // to new pointcloud
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 跳过nan点
        pc_out->points[valid_point_id++].time = float(pc_in->points[point_id].timestamp - pc_in->points[0].timestamp);
    }
}

//convert depth points to velodyneXYZIRT
template<typename T_in_p, typename T_out_p>
void convert_depth(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out, int camera, int time_depth_sec, int time_depth_nsec){
    // //ConditionalRemoval filter
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cr_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z",pcl::ComparisonOps::LT,depth_filter)));
    // pcl::ConditionalRemoval<pcl::PointXYZRGB> cr;
    // cr.setCondition(range_cond);
    // cr.setInputCloud(pc_in);
    // cr.filter(*cloud_cr_filtered);


    // //Radius Outlier Removal filter
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ror_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    // ror.setInputCloud(cloud_cr_filtered);
    // ror.setRadiusSearch(0.2);
    // ror.setMinNeighborsInRadius(10);
    // ror.filter(*cloud_ror_filtered);


    // //voxel Grid filter
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vg_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    // vg.setInputCloud(cloud_ror_filtered);
    // vg.setLeafSize(0.01f,0.01f,0.01f);
    // vg.filter(*cloud_vg_filtered);



    
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]) || pc_in->points[point_id].z > 2)
            continue;
        T_out_p new_point;
        double x = pc_in->points[point_id].x, y = pc_in->points[point_id].y, z = pc_in->points[point_id].z;
        // double rgb = cloud_cr_filtered->points[point_id].rgb;

        double dist = sqrt(x*x+y*y+z*z);
        double pitch = asin(z / dist)/3.1415926*180;
        uint16_t ring = 0;
        if(pitch >= -16 && pitch <= 16)
            ring = uint16_t(RING_MAP_16[int(round((pitch + 15.0) / 2.0))]);
        else if(pitch < -16 && pitch > -80)
            ring = uint16_t(round(-16.5 - pitch + 16));
        else if(pitch <= -80)
            ring = 79;
        else if(pitch > 16 and pitch < 25)
            ring = uint16_t(round(pitch - 16.5 + 80));
        else
            ring = 87;

        new_point.x = x * T[camera][0] + y * T[camera][4] + z * T[camera][8] + T[camera][12];
        new_point.y = x * T[camera][1] + y * T[camera][5] + z * T[camera][9] + T[camera][13];
        new_point.z = x * T[camera][2] + y * T[camera][6] + z * T[camera][10] + T[camera][14];
        new_point.intensity = 100;
        new_point.ring = ring;

        // float time_depth = time_depth_sec*1.0 + time_depth_nsec/1000000000.0;
        new_point.time = time_depth_sec*1.0 + time_depth_nsec/1000000000.0;
        pc_out->points.push_back(new_point);
    }
    
    // else if(camera == 1){
    //     for (int point_id = 0; point_id < cloud_vg_filtered->points.size(); ++point_id) {
    //         if (has_nan(cloud_vg_filtered->points[point_id]))
    //             continue;
    //         T_out_p new_point;
    //         double x = cloud_vg_filtered->points[point_id].x, y = cloud_vg_filtered->points[point_id].y, z = cloud_vg_filtered->points[point_id].z;
    //         // double rgb = cloud_ror_filteredd->points[point_id].rgb;

    //         double dist = sqrt(x*x+y*y+z*z);
    //         double pitch = asin(z / dist)/3.1415926*180;
    //         uint16_t ring = 0;
    //         if(pitch >= -16 && pitch <= 16)
    //             ring = uint16_t(RING_MAP_16[int(round((pitch + 15.0) / 2.0))]);
    //         else if(pitch < -16 && pitch > -80)
    //             ring = uint16_t(round(-16.5 - pitch + 16));
    //         else if(pitch <= -80)
    //             ring = 79;
    //         else if(pitch > 16 and pitch < 25)
    //             ring = uint16_t(round(pitch - 16.5 + 80));
    //         else
    //             ring = 87;

    //         new_point.x = x * T_right[0] + y * T_right[4] + z * T_right[8] + T_right[12];
    //         new_point.y = x * T_right[1] + y * T_right[5] + z * T_right[9] + T_right[13];
    //         new_point.z = x * T_right[2] + y * T_right[6] + z * T_right[10] + T_right[14];
    //         new_point.intensity = 100;
    //         new_point.ring = ring;
    //         // time_depth = time_depth_sec*1.0 + time_depth_nsec/1000000000.0;
    //         new_point.time = time_depth_sec*1.0 + time_depth_nsec/1000000000.0;
    //         pc_out->points.push_back(new_point);
    //     }

    // }

}


template<typename T>
void publish_points(T &new_pc, const sensor_msgs::PointCloud2ConstPtr& old_msg) {
    // pc properties
    new_pc->is_dense = true;

    // publish
    sensor_msgs::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*new_pc, pc_new_msg);
    pc_new_msg.header = old_msg->header;
    pc_new_msg.header.frame_id = "velodyne";
    pubRobosensePC.publish(pc_new_msg);
}







void callback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg, const sensor_msgs::PointCloud2ConstPtr& depth_left_msg, const sensor_msgs::PointCloud2ConstPtr& depth_right_msg){
    std::cout << "publishing!!" << std::endl;
    //create fusion data(VelodyneXYZIRT)
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_fusion(new pcl::PointCloud<VelodynePointXYZIRT>());
    //read lidar data
    if(input_type == 2){
        pcl::PointCloud<RsPointXYZIRT>::Ptr pc_lidar(new pcl::PointCloud<RsPointXYZIRT>());
        pcl::fromROSMsg(*lidar_msg, *pc_lidar);
        
        
        handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIRT>(pc_lidar, pc_fusion);
        add_ring<RsPointXYZIRT, VelodynePointXYZIRT>(pc_lidar, pc_fusion);
        add_time<RsPointXYZIRT, VelodynePointXYZIRT>(pc_lidar, pc_fusion);



    }
    else if(input_type == 0){       //XYZI
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*lidar_msg, *pc);

        // to new pointcloud
        for (int point_id = 0; point_id < pc->points.size(); ++point_id) {
            if (has_nan(pc->points[point_id]))
                continue;

            VelodynePointXYZIRT new_point;
            new_point.x = pc->points[point_id].x;
            new_point.y = pc->points[point_id].y;
            new_point.z = pc->points[point_id].z;
            new_point.intensity = pc->points[point_id].intensity;
            // remap ring id
            if (pc->height == 16) {
                new_point.ring = RING_MAP_16[point_id / pc->width];
            } else if (pc->height == 128) {
                new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
            }
            new_point.time = 0;
            pc_fusion->points.push_back(new_point);
        }
    }
    else{
        ROS_ERROR("Unsupported input pointcloud type. Currently only support XYZI and XYZIRT.");
        exit(1);
    }

    


    //read depth data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_left(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*depth_left_msg, *pc_depth_left);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_right(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*depth_right_msg, *pc_depth_right);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_back(new pcl::PointCloud<pcl::PointXYZRGB>());
    // pcl::fromROSMsg(depth_right_msg, *pc_depth_right);


    //convert depth points and add to pcfsion
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_depth_left_conversion(new pcl::PointCloud<VelodynePointXYZIRT>());
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_depth_right_conversion(new pcl::PointCloud<VelodynePointXYZIRT>());
    int time_left_sec = depth_left_msg->header.stamp.sec - lidar_msg->header.stamp.sec;
    int time_left_nsec = depth_left_msg->header.stamp.nsec - lidar_msg->header.stamp.nsec;
    int time_right_sec = depth_right_msg->header.stamp.sec - lidar_msg->header.stamp.sec;
    int time_right_nsec = depth_right_msg->header.stamp.nsec - lidar_msg->header.stamp.nsec;

    convert_depth<pcl::PointXYZRGB, VelodynePointXYZIRT>(pc_depth_left, pc_depth_left_conversion, 0, time_left_sec, time_left_nsec);
    convert_depth<pcl::PointXYZRGB, VelodynePointXYZIRT>(pc_depth_right, pc_depth_right_conversion, 1, time_right_sec, time_right_nsec);

    //add converted depth data to fusion data
    fusion_data<VelodynePointXYZIRT, VelodynePointXYZIRT>(pc_depth_left_conversion, pc_fusion);
    fusion_data<VelodynePointXYZIRT, VelodynePointXYZIRT>(pc_depth_right_conversion, pc_fusion);


    publish_points(pc_fusion, lidar_msg);


}


// //when input_type is XYZI:
// void callbackxyzi(const sensor_msgs::PointCloud2 &lidar_msg, const sensor_msgs::PointCloud2 &depth_left_msg, const sensor_msgs::PointCloud2 &depth_right_msg){
//     //read lidar data
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_fusion(new pcl::PointCloud<VelodynePointXYZIR>());
//     pcl::fromROSMsg(lidar_msg, *pc);

//     // to new pointcloud
//     for (int point_id = 0; point_id < pc->points.size(); ++point_id) {
//         if (has_nan(pc->points[point_id]))
//             continue;

//         VelodynePointXYZIR new_point;
//         new_point.x = pc->points[point_id].x;
//         new_point.y = pc->points[point_id].y;
//         new_point.z = pc->points[point_id].z;
//         new_point.intensity = pc->points[point_id].intensity;
//         // remap ring id
//         if (pc->height == 16) {
//             new_point.ring = RING_MAP_16[point_id / pc->width];
//         } else if (pc->height == 128) {
//             new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
//         }
//         pc_fusion->points.push_back(new_point);
//     }




//     //read depth data
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_left(new pcl::PointCloud<pcl::PointXYZRGB>());
//     pcl::fromROSMsg(depth_left_msg, *pc_depth_left);

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_right(new pcl::PointCloud<pcl::PointXYZRGB>());
//     pcl::fromROSMsg(depth_right_msg, *pc_depth_right);



//     //convert depth points and add to pcfsion
//     pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_depth_left_conversion(new pcl::PointCloud<VelodynePointXYZIRT>());
//     pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_depth_right_conversion(new pcl::PointCloud<VelodynePointXYZIRT>());
//     int time_left_sec = depth_left_msg.header.stamp.sec - lidar_msg.header.stamp.sec;
//     int time_left_nsec = depth_left_msg.header.stamp.nsec - lidar_msg.header.stamp.nsec;

//     int time_right_sec = depth_right_msg.header.stamp.sec - lidar_msg.header.stamp.sec;
//     int time_right_nsec = depth_right_msg.header.stamp.nsec - lidar_msg.header.stamp.nsec;

//     convert_depth<pcl::PointXYZRGB, VelodynePointXYZIRT>(pc_depth_left, pc_depth_left_conversion, 0, time_left_sec, time_left_nsec);
//     convert_depth<pcl::PointXYZRGB, VelodynePointXYZIRT>(pc_depth_right, pc_depth_right_conversion, 1, time_right_sec, time_right_nsec);

//     //add converted depth data to fusion data
//     fusion_data<VelodynePointXYZIRT, VelodynePointXYZIRT>(pc_depth_left_conversion, pc_fusion);
//     fusion_data<VelodynePointXYZIRT, VelodynePointXYZIRT>(pc_depth_right_conversion, pc_fusion);


//     publish_points(pc_fusion, lidar_msg);

// }
	





int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_points_c");
    ros::NodeHandle n;


    //, ros::TransportHints().tcpNoDelay()
    message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_lidar(n, "/rslidar_points", 4);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_depth_left(n, "/camera_front_left/depth/color/points", 4);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_depth_right(n, "/camera_front_right/depth/color/points", 4);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;
    // message_filters::TimeSynchronizer<sensor_msgs::LaserScan,geometry_msgs::PoseWithCovarianceStamped> sync(subscriber_laser, subscriber_pose, 10);
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(4), subscriber_lidar, subscriber_depth_left, subscriber_depth_right);
    // if(input_type == 2){
    //     sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    // }
    // else if(input_type == 0){
    //     sync.registerCallback(boost::bind(&callbackxyzi, _1, _2, _3));
    // }

    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    std::cout << "hahah" << std::endl;
    pubRobosensePC = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);


    ros::spin();
    return 0;
}
