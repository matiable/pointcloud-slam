#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>


int output_type; // 0:XYZRGB    2:XYZIRT
int camera_fusion_quantity;
std::vector<std::string> camera_topic_list;
std::vector<int> camera_fusion_index;
std::vector<std::vector<double> > camera_T;
double depth_filter;
std::string pub_topic;
std::string pub_frame_id;





// 定义发布者
ros::Publisher pubCameraFusion;


static int RING_ID_MAP_RUBY[] = {
    3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
    35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
    67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
    99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
    7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
    39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
    71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
    103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60};

// 线数对应
static int RING_MAP_16[] = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};

struct RsPointXYZIRT
{
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

// velodyne的点云格式
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct VelodynePointXYZIR
{
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

// 取出nan点
template <typename T>
bool has_nan(T point)
{

    // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
    // pcl remove nan not work normally
    // ROS_ERROR("Containing nan point!");
    if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z))
    {
        return true;
    }
    else
    {
        return false;
    }
}


// add x, y, z, i, r, t to pc_out
template <typename T_in_p, typename T_out_p>
void fusion_data(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                 const typename pcl::PointCloud<T_out_p>::Ptr &pc_out)
{

    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id)
    {
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


// add x, y, z, i, r, t to pc_out
template <typename T_in_p, typename T_out_p>
void fusion_data_rgb(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                 const typename pcl::PointCloud<T_out_p>::Ptr &pc_out)
{

    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id)
    {
        if (has_nan(pc_in->points[point_id]))
            continue;
        T_out_p new_point;
        //        std::copy(pc->points[point_id].data, pc->points[point_id].data + 4, new_point.data);
        new_point.x = pc_in->points[point_id].x;
        new_point.y = pc_in->points[point_id].y;
        new_point.z = pc_in->points[point_id].z;
        new_point.rgb = pc_in->points[point_id].rgb;
        pc_out->points.push_back(new_point);
    }
}



template <typename T_in_p, typename T_out_p>
void add_ring(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out)
{
    // to new pointcloud
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id)
    {
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 跳过nan点
        pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
    }
}

template <typename T_in_p, typename T_out_p>
void add_time(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out)
{
    // to new pointcloud
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id)
    {
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 跳过nan点
        pc_out->points[valid_point_id++].time = float(pc_in->points[point_id].timestamp - pc_in->points[0].timestamp);
    }
}

// convert depth points to velodyneXYZIRT
template <typename T_in_p, typename T_out_p>
void convert_depth(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                   const typename pcl::PointCloud<T_out_p>::Ptr &pc_out, int camera, int time_depth_sec, int time_depth_nsec)
{
    // // ConditionalRemoval filter
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cr_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, depth_filter)));
    // pcl::ConditionalRemoval<pcl::PointXYZRGB> cr;
    // cr.setCondition(range_cond);
    // cr.setInputCloud(pc_in);
    // cr.filter(*cloud_cr_filtered);

    // // Radius Outlier Removal filter
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ror_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    // ror.setInputCloud(cloud_cr_filtered);
    // ror.setRadiusSearch(0.2);
    // ror.setMinNeighborsInRadius(10);
    // ror.filter(*cloud_ror_filtered);

    // // voxel Grid filter
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vg_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    // vg.setInputCloud(cloud_ror_filtered);
    // vg.setLeafSize(0.01f, 0.01f, 0.01f);
    // vg.filter(*cloud_vg_filtered);

    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id)
    {
        if (has_nan(pc_in->points[point_id]) || (pc_in->points[point_id].z > depth_filter && depth_filter >= 0))
            continue;
        T_out_p new_point;
        double x = pc_in->points[point_id].x, y = pc_in->points[point_id].y, z = pc_in->points[point_id].z;
        // double rgb = cloud_cr_filtered->points[point_id].rgb;

        

        new_point.x = x * camera_T[camera][0] + y * camera_T[camera][4] + z * camera_T[camera][8] + camera_T[camera][12];
        new_point.y = x * camera_T[camera][1] + y * camera_T[camera][5] + z * camera_T[camera][9] + camera_T[camera][13];
        new_point.z = x * camera_T[camera][2] + y * camera_T[camera][6] + z * camera_T[camera][10] + camera_T[camera][14];
        new_point.intensity = 100;
        

        double dist = sqrt(new_point.x * new_point.x + new_point.y * new_point.y + new_point.z * new_point.z);
        double pitch = asin(new_point.z / dist) / 3.1415926 * 180;
        // std::cout << pitch << std::endl;
        uint16_t ring = 0;
        if (pitch >= -16 && pitch <= 16){
            ring = uint16_t(RING_MAP_16[int(round((pitch + 15.0) / 2.0))]);
            // std::cout << "part1" << std::endl;
        }
            
        else if (pitch < -16 && pitch > -80){
            ring = uint16_t(round(-16.5 - pitch + 16));
            // std::cout << "part2" << std::endl;
        }
        else if (pitch <= -80){
            ring = 79;
            // std::cout << "part3" << std::endl;
        }
        else if (pitch > 16 and pitch < 25){
            ring = uint16_t(round(pitch - 16.5 + 80));
            // std::cout << "part4" << std::endl;
        }
        else
            ring = 87;
        
        new_point.ring = ring;

        // float time_depth = time_depth_sec*1.0 + time_depth_nsec/1000000000.0;
        new_point.time = time_depth_sec * 1.0 + time_depth_nsec / 1000000000.0;
        pc_out->points.push_back(new_point);
    }
}


// convert depth points to velodyneXYZIRT
template <typename T_in_p, typename T_out_p>
void convert_depth_rgb(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                   const typename pcl::PointCloud<T_out_p>::Ptr &pc_out, int camera)
{
    // // ConditionalRemoval filter
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cr_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, depth_filter)));
    // pcl::ConditionalRemoval<pcl::PointXYZRGB> cr;
    // cr.setCondition(range_cond);
    // cr.setInputCloud(pc_in);
    // cr.filter(*cloud_cr_filtered);

    // // Radius Outlier Removal filter
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ror_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    // ror.setInputCloud(cloud_cr_filtered);
    // ror.setRadiusSearch(0.2);
    // ror.setMinNeighborsInRadius(10);
    // ror.filter(*cloud_ror_filtered);

    // // voxel Grid filter
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vg_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    // vg.setInputCloud(cloud_ror_filtered);
    // vg.setLeafSize(0.01f, 0.01f, 0.01f);
    // vg.filter(*cloud_vg_filtered);

    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id)
    {
        if (has_nan(pc_in->points[point_id]) || (pc_in->points[point_id].z > depth_filter && depth_filter >= 0))
            continue;
        T_out_p new_point;
        double x = pc_in->points[point_id].x, y = pc_in->points[point_id].y, z = pc_in->points[point_id].z;
        // double rgb = cloud_cr_filtered->points[point_id].rgb;

        

        new_point.x = x * camera_T[camera][0] + y * camera_T[camera][4] + z * camera_T[camera][8] + camera_T[camera][12];
        new_point.y = x * camera_T[camera][1] + y * camera_T[camera][5] + z * camera_T[camera][9] + camera_T[camera][13];
        new_point.z = x * camera_T[camera][2] + y * camera_T[camera][6] + z * camera_T[camera][10] + camera_T[camera][14];
        new_point.rgb = pc_in->points[point_id].rgb;
        
        pc_out->points.push_back(new_point);
    }
}


template <typename T>
void publish_points(T &new_pc, const sensor_msgs::PointCloud2ConstPtr& old_msg) {
    // pc properties
    new_pc->is_dense = true;

    // publish
    sensor_msgs::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*new_pc, pc_new_msg);
    pc_new_msg.header = old_msg->header;
    pc_new_msg.header.frame_id = pub_frame_id;
    pubCameraFusion.publish(pc_new_msg);
}


//重载两个相机
void callback(const sensor_msgs::PointCloud2ConstPtr& depth_0_msg, const sensor_msgs::PointCloud2ConstPtr& depth_1_msg){
    // std::cout << "publishing..." << std::endl;

    //read depth data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_0(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*depth_0_msg, *pc_depth_0);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_1(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*depth_1_msg, *pc_depth_1);



    
    
    if(output_type == 2){
        //create fusion data(VelodyneXYZIRT)
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_fusion(new pcl::PointCloud<VelodynePointXYZIRT>());
        //convert depth points and add to pcfsion
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_depth_0_conversion(new pcl::PointCloud<VelodynePointXYZIRT>());
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_depth_1_conversion(new pcl::PointCloud<VelodynePointXYZIRT>());
        int time_0_sec = depth_0_msg->header.stamp.sec - depth_0_msg->header.stamp.sec;
        int time_0_nsec = depth_0_msg->header.stamp.nsec - depth_0_msg->header.stamp.nsec;
        int time_1_sec = depth_1_msg->header.stamp.sec - depth_0_msg->header.stamp.sec;
        int time_1_nsec = depth_1_msg->header.stamp.nsec - depth_0_msg->header.stamp.nsec;

        convert_depth<pcl::PointXYZRGB, VelodynePointXYZIRT>(pc_depth_0, pc_depth_0_conversion, camera_fusion_index[0], time_0_sec, time_0_nsec);
        convert_depth<pcl::PointXYZRGB, VelodynePointXYZIRT>(pc_depth_1, pc_depth_1_conversion, camera_fusion_index[1], time_1_sec, time_1_nsec);

        //add converted depth data to fusion data
        fusion_data<VelodynePointXYZIRT, VelodynePointXYZIRT>(pc_depth_0_conversion, pc_fusion);
        fusion_data<VelodynePointXYZIRT, VelodynePointXYZIRT>(pc_depth_1_conversion, pc_fusion);
        publish_points(pc_fusion, depth_0_msg);
    
    }
    else if(output_type == 0){
        //create fusion data(pcl::PointXYZRGB)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_fusion(new pcl::PointCloud<pcl::PointXYZRGB>());

        //convert depth points and add to pcfsion
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_0_conversion(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_1_conversion(new pcl::PointCloud<pcl::PointXYZRGB>());

        convert_depth_rgb<pcl::PointXYZRGB, pcl::PointXYZRGB>(pc_depth_0, pc_depth_0_conversion, camera_fusion_index[0]);
        convert_depth_rgb<pcl::PointXYZRGB, pcl::PointXYZRGB>(pc_depth_1, pc_depth_1_conversion, camera_fusion_index[1]);

        //add converted depth data to fusion data
        fusion_data_rgb<pcl::PointXYZRGB, pcl::PointXYZRGB>(pc_depth_0_conversion, pc_fusion);
        fusion_data_rgb<pcl::PointXYZRGB, pcl::PointXYZRGB>(pc_depth_1_conversion, pc_fusion);
        publish_points(pc_fusion, depth_0_msg);
    }
    else{
        ROS_ERROR("Unsupported output pointcloud type. Currently only support XYZRGB and XYZIRT.");
        exit(1);
    }

}
    


//重载三个相机
void callback(const sensor_msgs::PointCloud2ConstPtr& depth_0_msg, const sensor_msgs::PointCloud2ConstPtr& depth_1_msg, const sensor_msgs::PointCloud2ConstPtr& depth_2_msg){
    // std::cout << "publishing..." << std::endl;


    //read depth data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_0(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*depth_0_msg, *pc_depth_0);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_1(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*depth_1_msg, *pc_depth_1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_2(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*depth_2_msg, *pc_depth_2);

    if(output_type == 2){
        //create fusion data(VelodyneXYZIRT)
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_fusion(new pcl::PointCloud<VelodynePointXYZIRT>());
        //convert depth points and add to pcfsion
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_depth_0_conversion(new pcl::PointCloud<VelodynePointXYZIRT>());
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_depth_1_conversion(new pcl::PointCloud<VelodynePointXYZIRT>());
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_depth_2_conversion(new pcl::PointCloud<VelodynePointXYZIRT>());
        int time_0_sec = depth_0_msg->header.stamp.sec - depth_0_msg->header.stamp.sec;
        int time_0_nsec = depth_0_msg->header.stamp.nsec - depth_0_msg->header.stamp.nsec;
        int time_1_sec = depth_1_msg->header.stamp.sec - depth_0_msg->header.stamp.sec;
        int time_1_nsec = depth_1_msg->header.stamp.nsec - depth_0_msg->header.stamp.nsec;
        int time_2_sec = depth_2_msg->header.stamp.sec - depth_0_msg->header.stamp.sec;
        int time_2_nsec = depth_2_msg->header.stamp.nsec - depth_0_msg->header.stamp.nsec;

        convert_depth<pcl::PointXYZRGB, VelodynePointXYZIRT>(pc_depth_0, pc_depth_0_conversion, camera_fusion_index[0], time_0_sec, time_0_nsec);
        convert_depth<pcl::PointXYZRGB, VelodynePointXYZIRT>(pc_depth_1, pc_depth_1_conversion, camera_fusion_index[1], time_1_sec, time_1_nsec);
        convert_depth<pcl::PointXYZRGB, VelodynePointXYZIRT>(pc_depth_2, pc_depth_2_conversion, camera_fusion_index[2], time_2_sec, time_2_nsec);


        //add converted depth data to fusion data
        fusion_data<VelodynePointXYZIRT, VelodynePointXYZIRT>(pc_depth_0_conversion, pc_fusion);
        fusion_data<VelodynePointXYZIRT, VelodynePointXYZIRT>(pc_depth_1_conversion, pc_fusion);
        fusion_data<VelodynePointXYZIRT, VelodynePointXYZIRT>(pc_depth_2_conversion, pc_fusion);
        publish_points(pc_fusion, depth_0_msg);
    }
    else if(output_type == 0){
        //create fusion data(pcl::PointXYZRGB)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_fusion(new pcl::PointCloud<pcl::PointXYZRGB>());

        //convert depth points and add to pcfsion
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_0_conversion(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_1_conversion(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_depth_2_conversion(new pcl::PointCloud<pcl::PointXYZRGB>());

        convert_depth_rgb<pcl::PointXYZRGB, pcl::PointXYZRGB>(pc_depth_0, pc_depth_0_conversion, camera_fusion_index[0]);
        convert_depth_rgb<pcl::PointXYZRGB, pcl::PointXYZRGB>(pc_depth_1, pc_depth_1_conversion, camera_fusion_index[1]);
        convert_depth_rgb<pcl::PointXYZRGB, pcl::PointXYZRGB>(pc_depth_2, pc_depth_2_conversion, camera_fusion_index[2]);


        //add converted depth data to fusion data
        fusion_data_rgb<pcl::PointXYZRGB, pcl::PointXYZRGB>(pc_depth_0_conversion, pc_fusion);
        fusion_data_rgb<pcl::PointXYZRGB, pcl::PointXYZRGB>(pc_depth_1_conversion, pc_fusion);
        fusion_data_rgb<pcl::PointXYZRGB, pcl::PointXYZRGB>(pc_depth_2_conversion, pc_fusion);
        publish_points(pc_fusion, depth_0_msg);
    }
    else{
        ROS_ERROR("Unsupported output pointcloud type. Currently only support XYZRGB and XYZIRT.");
        exit(1);
    }
    



    

}



int main(int argc, char **argv)
{
    int i,j;
    ros::init(argc, argv, "fusion_camera");
    ros::NodeHandle n;

    // 读取参数
    ROS_INFO("Reading Parameter...");

    // 输出格式 
    if (n.getParam("output_type", output_type)){
        if(output_type == 0)
            ROS_INFO("output_type: XYZRGB");
        else if(output_type == 2)
            ROS_INFO("output_type: XYZIRT");
    }
    else
        ROS_WARN("Didn't find parameter output_type.");


    //  相机数量
    if (n.getParam("camera_fusion_quantity", camera_fusion_quantity))
        ROS_INFO("camera_fusion_quantity: %d", camera_fusion_quantity);
    else
        ROS_WARN("Didn't find parameter camera_fusion_quantity.");

    // 相机话题
    if (n.getParam("camera_topic_list", camera_topic_list))
        ROS_INFO("get camera_topic_list.");
    else
        ROS_WARN("Didn't find parameter camera_topic_list.");



    // 要融合的相机编号
    if (n.getParam("camera_fusion_index",camera_fusion_index)){
        ROS_INFO("fusion camera topics: ");
        for(i=0;i<camera_fusion_quantity;i++){
            ROS_INFO("%s",camera_topic_list[camera_fusion_index[i]].c_str());
        }
    }
    else
        ROS_WARN("Didn't find parameter camera_fusion_index.");

    // 转换矩阵
    std::vector<double> camera_T0;
    std::vector<double> camera_T1;
    std::vector<double> camera_T2;
    if (n.getParam("camera_T0", camera_T0)){
        camera_T.push_back(camera_T0);
        ROS_INFO("camera T0: ");
            for(j=0;j<12;j++){
                ROS_INFO("%f ",camera_T0[j]);
            }
    }
    else
        ROS_WARN("Didn't find parameter camera_T0.");
    if (n.getParam("camera_T1", camera_T1)){
        camera_T.push_back(camera_T1);
        ROS_INFO("camera T1: ");
            for(j=0;j<12;j++){
                ROS_INFO("%f ",camera_T1[j]);
            }
    }
    else
        ROS_WARN("Didn't find parameter camera_T1.");
    if (n.getParam("camera_T2", camera_T2)){
        camera_T.push_back(camera_T2);
        ROS_INFO("camera T2: ");
            for(j=0;j<12;j++){
                ROS_INFO("%f ",camera_T2[j]);
            }
    }
    else
        ROS_WARN("Didn't find parameter camera_T2.");
    
    

    // z轴滤波范围
    if(n.getParam("depth_filter", depth_filter))
        ROS_INFO("depth_filter %f", depth_filter);
    else
        ROS_WARN("Didn't find parameter depth_filter.");


    // 发布话题
    if (n.getParam("pub_topic", pub_topic))
        ROS_INFO("pub_topic: %s", pub_topic.c_str());
    else
        ROS_WARN("Didn't find parameter pub_topic.");

    
    // 发布坐标系
    if (n.getParam("pub_frame_id", pub_frame_id))
        ROS_INFO("pub_frame_id: %s", pub_frame_id.c_str());
    else
        ROS_WARN("Didn't find parameter pub_frame_id.");



    std::cout << "Start Fusion..." << std::endl;
    
    if(camera_fusion_quantity == 2){
        message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_camera_0(n, camera_topic_list[camera_fusion_index[0]].c_str(), 2);
        message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_camera_1(n, camera_topic_list[camera_fusion_index[1]].c_str(), 2);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;
        message_filters::Synchronizer<syncPolicy> sync(syncPolicy(4), subscriber_camera_0, subscriber_camera_1);
        sync.registerCallback(boost::bind(&callback, _1, _2));
        pubCameraFusion = n.advertise<sensor_msgs::PointCloud2>(pub_topic.c_str(), 3);
        ros::spin();
    }
    else if(camera_fusion_quantity == 3){
        message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_camera_0(n, camera_topic_list[camera_fusion_index[0]].c_str(), 2);
        message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_camera_1(n, camera_topic_list[camera_fusion_index[1]].c_str(), 2);
        message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_camera_2(n, camera_topic_list[camera_fusion_index[2]].c_str(), 2);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;
        message_filters::Synchronizer<syncPolicy> sync(syncPolicy(4), subscriber_camera_0, subscriber_camera_1, subscriber_camera_2);
        sync.registerCallback(boost::bind(&callback, _1, _2, _3));
        pubCameraFusion = n.advertise<sensor_msgs::PointCloud2>(pub_topic.c_str(), 3);
        ros::spin();
    }
    else{
        ROS_WARN("Cannot fusion %d cameras.",camera_fusion_quantity);
    }


    
    
    return 0;
}