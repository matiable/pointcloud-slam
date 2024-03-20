#ifndef OCCUPANCY_MAPPING_SERVER_H_
#define OCCUPANCY_MAPPING_SERVER_H_

#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

#include <queue>

#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/GetMap.h"
#include "occupancy_map.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "transformer.h"

namespace occupancy_mapping {

class OccupancyServer {
 public:
  struct MapServerConfig {
    double min_z;
    double max_z;
    double angle_increment;
    double min_range;
    double max_range;
    double max_radius;
    bool fill_with_white;
    bool use_nan;
    std::string map_frame_ = "map";
    std::string body_frame_ = "base_link";
    std::string sensor_frame_ = "velodyne";
    std::string pub_map_topic_ = "occupancy_map";
    std::string map_save_path_ = "/home/ysc/jy_cog/system/map";
    std::string map_save_name_ = "jueying";
  };
  OccupancyMap2D::MapParams getMapParamsFromRosParam(const ros::NodeHandle &nh_private);
  void getServerConfigFromRosParam(const ros::NodeHandle &nh_private);
  std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1);
  void pubLaserScan(sensor_msgs::LaserScan &laser_scan_output);
  void publishMap();
  void saveMap(std::string label_name="");
  void processScan(GeneralLaserScan &scan, std::vector<double> robot_pose);
  void getScan(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, GeneralLaserScan *final_scan, sensor_msgs::LaserScan &laser_scan_output);
  void getGridMap(OccupancyMap2D::MapTreeNode *node, std::vector<int> *map_data, int &minx, int &maxx, int &miny, int &maxy, bool &first_flag);

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer publish_occupancy_map_srv_;

  ros::Publisher map_pub_;  // 地图发布者
  ros::Publisher map_pub_2_;
  ros::Publisher laser_pub_;  // laser发布者

  std::shared_ptr<OccupancyMap2D> occupancy_map_;
  MapServerConfig map_server_config_;
};

class OccupancyServerFromFile : public OccupancyServer {
 public:
  OccupancyServerFromFile(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  void readLabelFile();
  void readRobotPoses();
  void getInitialCloud(int num_index, pcl::PointCloud<pcl::PointXYZI>::Ptr initial_cloud);
  void getFileList();
  void startMapping();

  bool publishOccupancyMapCallback(std_srvs::Empty::Request &request,  // NOLINT
                                   std_srvs::Empty::Response &response);

 protected:
  int use_file_num_ = 2;
  std::string data_file_1_ = "/home/hexiaowei/perception_map/other/Scans/";
  std::string data_file_2_ = "/home/hexiaowei/perception_map/other/surface/";
  std::string pose_file_ = "/home/hexiaowei/perception_map/other/Poses/2022.txt";
  std::string label_file_ = "";

  std::map<int, std::vector<int>> label_indices_;
  std::vector<std::vector<double>> robot_poses_;
  std::vector<int> file_list_num_;
};

class OccupancyServerRealTime : public OccupancyServer {
 public:
  OccupancyServerRealTime(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  bool publishOccupancyMapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
  void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg_in);
  void livoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr &pointcloud_msg_in);
  bool getNextPointcloudFromQueue(std::queue<sensor_msgs::PointCloud2::Ptr> *queue, sensor_msgs::PointCloud2::Ptr *pointcloud_msg, geometry_msgs::PoseStamped *robot_pose);
  void processPointCloudAndPose(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg, const geometry_msgs::PoseStamped &robot_pose);
  std::vector<double> getPoseFromPoseStamped(const geometry_msgs::PoseStamped &robot_pose);

 protected:
  ros::Subscriber pointcloud_sub_;
  std::string pointcloud_topic_ = "/scan";
  std::string odom_topic_ = "/odom_gps_in_station";
  int lidar_type_ = 0;

  Transformer transformer_;
  std::queue<sensor_msgs::PointCloud2::Ptr> pointcloud_queue_;

  bool is_initialized_ = false;

  int count = 0;
  double average_time = 0;

  // Eigen::Matrix4f eigen_transform_;
  tf::StampedTransform transform_st2s_;
  bool transform_flag_ = false;
};
}  // namespace occupancy_mapping

#endif  // OCCUPANCY_MAPPING_SERVER_H_