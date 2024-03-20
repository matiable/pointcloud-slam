#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp> 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include "dynamic_map.h"
using namespace std;

typedef pcl::PointXYZI PointType;

typedef std::numeric_limits< double > dbl;

class ParamServer
{
public:

    ros::NodeHandle nh;

    string globalSurfMap_dirctory;
    string globalCornerMap_dirctory;
    string globalCornerMap_pcd;
    string globalSurfMap_pcd;
    int area_size;
    int margin;
    float updateMapFrequency;
    string ndt_neighbor_search_method;
    float ndt_resolution;
    string Matching_method;
    string intialMethod;
    enum eintialMethod{
        human=0,
        gps=1,
    }mintialMethod;
    bool optimization_with_GPS;
    Eigen::Vector3d Pil;
    vector<double> initialPose;
    int initial_count_num;

    std::string robot_id; 

    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    bool useImuHeadingInitialization; 
    bool useGpsElevation; 
    float gpsCovThreshold;  
    float poseCovThreshold; 

    bool savePCD;
    string savePCDDirectory;

    

    float min_range;
    float max_range;
    float Vertical_angle;
    float ang_bottom;
    string lidar_type; 

    enum elidar_type{
        Velodyne=0,
        rslidar_ruby=1,
        rslidar_16=2,
        rslidar_32=3,
    }mlidar_type;
    int N_SCAN; 
    int Horizon_SCAN;
    string timeField; 
    int downsampleRate; 

    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity; 
    int imuFrequency;
    int area_num;
    int iter_num;
    float distance_limit;

    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;

    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    float edgeThreshold; 
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    int numberOfCores;
    double mappingProcessInterval; 

    float surroundingkeyframeAddingDistThreshold;  
    float surroundingkeyframeAddingAngleThreshold;  
    float surroundingKeyframeDensity; 
    float surroundingKeyframeSearchRadius;

    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize; 
    float historyKeyframeSearchRadius; 
    float historyKeyframeSearchTimeDiff; 
    int   historyKeyframeSearchNum;  
    float historyKeyframeFitnessScore;  

    float globalMapVisualizationSearchRadius; 
    float globalMapVisualizationPoseDensity; 
    float globalMapVisualizationLeafSize; 

    ParamServer()
    {
        nh.param<float>("globalmap_server/updateMapFrequency", updateMapFrequency, 0.1);
        nh.param<std::string>("globalmap_server/globalSurfMap_dirctory", globalSurfMap_dirctory, "../data/map.pcd");
        nh.param<std::string>("globalmap_server/globalCornerMap_dirctory", globalCornerMap_dirctory, "../data/map.pcd");
        nh.param<std::string>("globalmap_server/globalCornerMap_pcd", globalCornerMap_pcd, "../data/map.pcd");
        nh.param<std::string>("globalmap_server/globalSurfMap_pcd", globalSurfMap_pcd, "../data/map.pcd");
        nh.param<int>("globalmap_server/area_size", area_size, -1);
        nh.param<int>("globalmap_server/margin", margin, -1);  
        nh.param<std::string>("globalmap_server/ndt_neighbor_search_method", ndt_neighbor_search_method, "DIRECT7");
        nh.param<float>("globalmap_server/ndt_resolution", ndt_resolution, 1.0);
        nh.param<std::string>("globalmap_server/Matching_method", Matching_method, "loam");
        nh.param<vector<double>>("globalmap_server/initialPose", initialPose, vector<double>());
        nh.param<std::string>("globalmap_server/intialMethod", intialMethod, "gps");
        if(intialMethod=="human")
            mintialMethod=human;
        else if(intialMethod=="gps")
            mintialMethod=gps;
        else {
            std::cout << "Undefined intialMethod type " << std::endl;
            exit(-1);
        }
        nh.param<bool>("globalmap_server/optimization_with_GPS", optimization_with_GPS, "false");
        nh.param<int>("globalmap_server/initial_count_num", initial_count_num, -1);

        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("jueying_slam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("jueying_slam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("jueying_slam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("jueying_slam/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("jueying_slam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("jueying_slam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("jueying_slam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("jueying_slam/mapFrame", mapFrame, "map");

        nh.param<bool>("jueying_slam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("jueying_slam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("jueying_slam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("jueying_slam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("jueying_slam/savePCD", savePCD, false);
        nh.param<std::string>("jueying_slam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        nh.param<float>("jueying_slam/min_range", min_range, 1.0);
        nh.param<float>("jueying_slam/max_range", max_range, 150.0);
        nh.param<float>("jueying_slam/Vertical_angle", Vertical_angle, 30.0);
        nh.param<float>("jueying_slam/ang_bottom", ang_bottom, 15.0);
        nh.param<string>("jueying_slam/lidar_type", lidar_type, "rslidar_ruby");
        if(lidar_type=="Velodyne")
            mlidar_type=Velodyne;
        else if(lidar_type=="rslidar_ruby")
            mlidar_type=rslidar_ruby;
        else if(lidar_type=="rslidar_16")
             mlidar_type=rslidar_16;
        else if(lidar_type=="rslidar_32")
             mlidar_type=rslidar_32;
        else {
            std::cout << "Undefined lidar type " << std::endl;
            exit(-1);
        }

        nh.param<int>("jueying_slam/N_SCAN", N_SCAN, 16);
        nh.param<int>("jueying_slam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<std::string>("jueying_slam/timeField", timeField, "time");
        nh.param<int>("jueying_slam/downsampleRate", downsampleRate, 1);

        nh.param<float>("jueying_slam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("jueying_slam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("jueying_slam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("jueying_slam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("jueying_slam/imuGravity", imuGravity, 9.80511);
        nh.param<int>("jueying_slam/imuFrequency",imuFrequency,200);
        nh.param<int>("jueying_slam/area_num",area_num,6);
        nh.param<int>("jueying_slam/iter_num",iter_num,30);
        nh.param<float>("jueying_slam/distance_limit",distance_limit,2500.0);

        nh.param<vector<double>>("jueying_slam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("jueying_slam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("jueying_slam/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY.transpose());

        Pil<<-extTrans.x(), -extTrans.y(), -extTrans.z();
        nh.param<float>("jueying_slam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("jueying_slam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("jueying_slam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("jueying_slam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("jueying_slam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("jueying_slam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("jueying_slam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("jueying_slam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("jueying_slam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("jueying_slam/numberOfCores", numberOfCores, 2);
        nh.param<double>("jueying_slam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("jueying_slam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("jueying_slam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("jueying_slam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("jueying_slam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("jueying_slam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("jueying_slam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("jueying_slam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("jueying_slam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("jueying_slam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("jueying_slam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("jueying_slam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("jueying_slam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("jueying_slam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("jueying_slam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;

        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();

        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final=(q_from * extQRPY).normalized();

        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ")
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}


std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

#endif