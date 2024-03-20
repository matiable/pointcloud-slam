#include "occupancy_mapping/mapping_server.h"

#include <sys/time.h>
#include <unistd.h>

namespace occupancy_mapping {

OccupancyMap2D::MapParams OccupancyServer::getMapParamsFromRosParam(const ros::NodeHandle &nh_private) {
  OccupancyMap2D::MapParams map_params;
  nh_private.getParam("/occupancy_mapping_2D/log_free", map_params.log_free);

  nh_private.getParam("/occupancy_mapping_2D/log_occ", map_params.log_occ);

  nh_private.getParam("/occupancy_mapping_2D/resolution", map_params.resolution);
  return map_params;
}

void OccupancyServer::getServerConfigFromRosParam(const ros::NodeHandle &nh_private) {
  nh_private.getParam("/occupancy_mapping_2D/min_z", map_server_config_.min_z);

  nh_private.getParam("/occupancy_mapping_2D/max_z", map_server_config_.max_z);
  nh_private.getParam("/occupancy_mapping_2D/angle_increment", map_server_config_.angle_increment);

  nh_private.getParam("/occupancy_mapping_2D/min_range", map_server_config_.min_range);

  nh_private.getParam("/occupancy_mapping_2D/max_range", map_server_config_.max_range);

  nh_private.getParam("/occupancy_mapping_2D/max_radius", map_server_config_.max_radius);

  nh_private.getParam("/occupancy_mapping_2D/fill_with_white", map_server_config_.fill_with_white);

  nh_private.getParam("/occupancy_mapping_2D/use_nan", map_server_config_.use_nan);

  nh_private.getParam("/occupancy_mapping_2D/map_frame", map_server_config_.map_frame_);
  nh_private.getParam("/occupancy_mapping_2D/body_frame", map_server_config_.body_frame_);
  nh_private.getParam("/occupancy_mapping_2D/sensor_frame", map_server_config_.sensor_frame_);
  nh_private.getParam("/occupancy_mapping_2D/pub_map_topic", map_server_config_.pub_map_topic_);
  nh_private.getParam("/occupancy_mapping_2D/map_save_path", map_server_config_.map_save_path_);
  nh_private.getParam("/occupancy_mapping_2D/map_save_name", map_server_config_.map_save_name_);
}

std::vector<GridIndex> OccupancyServer::TraceLine(int x0, int y0, int x1, int y1) {
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;
  int x_end = x1;
  int y_end = y1;

  bool steep = fabs((y1 * 1.0 - y0 * 1.0) / (x1 * 1.0 - x0 * 1.0)) >= 1;
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  int pointX;
  int pointY;

  for (int x = x0; x <= x1; x++) {
    if (steep) {
      pointX = y;
      pointY = x;
    } else {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX) {
      y += ystep;
      error -= deltaX;
    }

    // 不包含最后一个点．
    if (pointX == x_end && pointY == y_end) continue;

    tmpIndex.SetIndex(pointX, pointY);
    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}

void OccupancyServer::getScan(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, GeneralLaserScan *final_scan, sensor_msgs::LaserScan &laser_scan_output) {
  unsigned int beam_size = ceil((3.1415927 - (-3.1415927)) / map_server_config_.angle_increment);
  laser_scan_output.ranges.assign(beam_size, map_server_config_.max_range + 1);
  // laser_scan_output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

  for (int i = 0; i < cloud->points.size(); i++) {
    if (cloud->points[i].z >= map_server_config_.min_z && cloud->points[i].z <= map_server_config_.max_z) {
      float range = hypot(cloud->points[i].x, cloud->points[i].y);
      float angle = atan2(cloud->points[i].y, cloud->points[i].x);
      int index = (int)((angle - (-3.1415927)) / map_server_config_.angle_increment);
      if (index >= 0 && index < beam_size) {
        // if (range < map_server_config_.min_range)
        // {
        //     laser_scan_output.ranges[index] = map_server_config_.min_range - 0.05;
        // }
        if (range >= map_server_config_.min_range && range <= map_server_config_.max_range) {
          if (range < laser_scan_output.ranges[index]) {
            laser_scan_output.ranges[index] = range;
          }
        }

        // laser_scan_output.intensities[index] = cloud->points[i].intensity;
      }
    }
  }
  for (int i = 0; i < beam_size; i++) {
    if (laser_scan_output.ranges[i] > map_server_config_.max_range || laser_scan_output.ranges[i] < map_server_config_.min_range) {
      laser_scan_output.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }

    final_scan->ranges.push_back(laser_scan_output.ranges[i]);
    final_scan->angles.push_back(map_server_config_.angle_increment * i + map_server_config_.angle_increment / 2 - M_PI);
  }

  ///

  return;
}

void OccupancyServer::pubLaserScan(sensor_msgs::LaserScan &laser_scan_output) {
  laser_scan_output.header.stamp = ros::Time::now();
  laser_scan_output.header.frame_id = map_server_config_.sensor_frame_;
  // laser_scan_output.header.frame_id = "livox_frame_tr";
  laser_scan_output.angle_min = -3.1415927;
  laser_scan_output.angle_max = 3.1415927;
  laser_scan_output.range_min = map_server_config_.min_range;
  laser_scan_output.range_max = map_server_config_.max_range;
  laser_scan_output.angle_increment = map_server_config_.angle_increment;
  laser_scan_output.time_increment = 0.0;
  laser_scan_output.scan_time = 0.0;

  laser_pub_.publish(laser_scan_output);
}

void OccupancyServer::getGridMap(OccupancyMap2D::MapTreeNode *node, std::vector<int> *map_data, int &minx, int &maxx, int &miny, int &maxy, bool &first_flag) {
  // bool first_flag = false;
  if (node->level != 0) {
    for (int i = 0; i < 4; i++) {
      if (node->child[i] != nullptr) {
        getGridMap(node->child[i], map_data, minx, maxx, miny, maxy, first_flag);
      } else {
        int p_x, p_y;
        if (i == 0) {
          p_x = node->position_x + pow(2, node->level - 1);
          p_y = node->position_y + pow(2, node->level - 1);
        }

        else if (i == 1) {
          p_x = node->position_x;
          p_y = node->position_y + pow(2, node->level - 1);
        } else if (i == 2) {
          p_x = node->position_x;
          p_y = node->position_y;
        } else if (i == 3) {
          p_x = node->position_x + pow(2, node->level - 1);
          p_y = node->position_y;
        }

        for (int ix = p_x; ix < p_x + pow(2, node->level - 1); ix++) {
          for (int iy = p_y; iy < p_y + pow(2, node->level - 1); iy++) {
            GridIndex tmpIndex(ix, iy);
            map_data->at(tmpIndex.x - occupancy_map_->getMapRoot()->position_x + (tmpIndex.y - occupancy_map_->getMapRoot()->position_y) * pow(2, occupancy_map_->getMapRoot()->level)) = -1;
          }
        }
      }
    }
  } else {
    GridIndex tmpIndex(node->position_x, node->position_y);
    double prob = 1.0 / (1.0 + exp(-1.0 * node->logit));
    prob = prob * 100.0;
    if (prob >= 50) {
      prob = 100.1;
    } else {
      prob = 0.1;
    }
    map_data->at(tmpIndex.x - occupancy_map_->getMapRoot()->position_x + (tmpIndex.y - occupancy_map_->getMapRoot()->position_y) * pow(2, occupancy_map_->getMapRoot()->level)) = int(prob);
    if (first_flag) {
      if (tmpIndex.x - occupancy_map_->getMapRoot()->position_x > maxx) {
        maxx = tmpIndex.x - occupancy_map_->getMapRoot()->position_x;
      }
      if (tmpIndex.x - occupancy_map_->getMapRoot()->position_x < minx) {
        minx = tmpIndex.x - occupancy_map_->getMapRoot()->position_x;
      }
      if (tmpIndex.y - occupancy_map_->getMapRoot()->position_y > maxy) {
        maxy = tmpIndex.y - occupancy_map_->getMapRoot()->position_y;
      }
      if (tmpIndex.y - occupancy_map_->getMapRoot()->position_y < miny) {
        miny = tmpIndex.y - occupancy_map_->getMapRoot()->position_y;
      }
    } else {
      first_flag = true;
      minx = tmpIndex.x - occupancy_map_->getMapRoot()->position_x;
      maxx = tmpIndex.x - occupancy_map_->getMapRoot()->position_x;
      miny = tmpIndex.y - occupancy_map_->getMapRoot()->position_y;
      maxy = tmpIndex.y - occupancy_map_->getMapRoot()->position_y;
    }
  }
}

void OccupancyServer::publishMap() {
  if (occupancy_map_->getMapRoot() == nullptr) {
    return;
  }
  nav_msgs::OccupancyGrid *ros_map = new (nav_msgs::OccupancyGrid);
  std::vector<int> *map_data = new std::vector<int>;
  map_data->resize(pow(pow(2, occupancy_map_->getMapRoot()->level), 2));
  ros_map->info.resolution = occupancy_map_->getMapResolution();
  ros_map->info.origin.position.x = occupancy_map_->getMapRoot()->position_x * occupancy_map_->getMapResolution();
  ros_map->info.origin.position.y = occupancy_map_->getMapRoot()->position_y * occupancy_map_->getMapResolution();
  ros_map->info.origin.position.z = 0.0;
  ros_map->info.origin.orientation.x = 0.0;
  ros_map->info.origin.orientation.y = 0.0;
  ros_map->info.origin.orientation.z = 0.0;
  ros_map->info.origin.orientation.w = 1.0;
  int minx = 0, maxx = 0, miny = 0, maxy = 0;
  bool first_flag = false;
  getGridMap(occupancy_map_->getMapRoot(), map_data, minx, maxx, miny, maxy, first_flag);
  ros_map->info.origin.position.x = ros_map->info.origin.position.x + minx * occupancy_map_->getMapResolution();
  ros_map->info.origin.position.y = ros_map->info.origin.position.y + miny * occupancy_map_->getMapResolution();

  ros_map->info.width = maxx - minx + 1;
  ros_map->info.height = maxy - miny + 1;
  ros_map->data.resize(ros_map->info.width * ros_map->info.height);
  for (int i = 0; i < ros_map->info.width; i++) {
    for (int j = 0; j < ros_map->info.height; j++) {
      ros_map->data[i + j * ros_map->info.width] = map_data->at(i + minx + (j + miny) * pow(2, occupancy_map_->getMapRoot()->level));  /////
    }
  }
  ros_map->header.stamp = ros::Time::now();
  ros_map->header.frame_id = map_server_config_.map_frame_;
  map_pub_.publish(*ros_map);
}

void OccupancyServer::saveMap(std::string label_name) {
  std::cout << "-----Finish mapping, start saving occupancy grid map-----" << std::endl;
  if (occupancy_map_->getMapRoot() == nullptr) {
    std::cout << "No occupancy map to save" << std::endl;
    return;
  }
  nav_msgs::OccupancyGrid *ros_map = new (nav_msgs::OccupancyGrid);
  std::vector<int> *map_data = new std::vector<int>;
  map_data->resize(pow(pow(2, occupancy_map_->getMapRoot()->level), 2));
  ros_map->info.resolution = occupancy_map_->getMapResolution();
  ros_map->info.origin.position.x = occupancy_map_->getMapRoot()->position_x * occupancy_map_->getMapResolution();
  ros_map->info.origin.position.y = occupancy_map_->getMapRoot()->position_y * occupancy_map_->getMapResolution();
  ros_map->info.origin.position.z = 0.0;
  ros_map->info.origin.orientation.x = 0.0;
  ros_map->info.origin.orientation.y = 0.0;
  ros_map->info.origin.orientation.z = 0.0;
  ros_map->info.origin.orientation.w = 1.0;
  int minx = 0, maxx = 0, miny = 0, maxy = 0;
  bool first_flag = false;
  getGridMap(occupancy_map_->getMapRoot(), map_data, minx, maxx, miny, maxy, first_flag);
  ros_map->info.origin.position.x = ros_map->info.origin.position.x + minx * occupancy_map_->getMapResolution();
  ros_map->info.origin.position.y = ros_map->info.origin.position.y + miny * occupancy_map_->getMapResolution();

  ros_map->info.width = maxx - minx + 1;
  ros_map->info.height = maxy - miny + 1;
  ros_map->data.resize(ros_map->info.width * ros_map->info.height);
  for (int i = 0; i < ros_map->info.width; i++) {
    for (int j = 0; j < ros_map->info.height; j++) {
      ros_map->data[i + j * ros_map->info.width] = map_data->at(i + minx + (j + miny) * pow(2, occupancy_map_->getMapRoot()->level));  /////
    }
  }
  //   ros_map->header.stamp = ros::Time::now();
  ros_map->header.frame_id = map_server_config_.map_frame_;
  //   map_pub_.publish(*ros_map);
  //   ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width, map->info.height, map->info.resolution);

  std::cout << "    map size: " << ros_map->info.width << "*" << ros_map->info.height << std::endl;
  std::cout << "    map resolution: " << ros_map->info.resolution << std::endl;

  std::string mapdatafile = map_server_config_.map_save_path_ + "/" + map_server_config_.map_save_name_ + label_name + ".pgm";  ////////
                                                                                                                                //   ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
  std::cout << "Writing map occupancy data to " << mapdatafile.c_str() << std::endl;
  FILE *out = fopen(mapdatafile.c_str(), "w");
  if (!out) {
    // ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
    std::cout << "Couldn't save map file to " << mapdatafile.c_str() << std::endl;
    return;
  }

  fprintf(out, "P5\n# CREATOR: occupancy_mapping %.3f m/pix\n%d %d\n255\n", ros_map->info.resolution, ros_map->info.width, ros_map->info.height);
  for (unsigned int y = 0; y < ros_map->info.height; y++) {
    for (unsigned int x = 0; x < ros_map->info.width; x++) {
      unsigned int i = x + (ros_map->info.height - y - 1) * ros_map->info.width;
      if (ros_map->data[i] >= 0 && ros_map->data[i] <= 25) {  // [0,free)
        fputc(254, out);
      } else if (ros_map->data[i] >= 65) {  // (occ,255]
        fputc(000, out);
      } else {  // occ [0.25,0.65]
        fputc(205, out);
      }
    }
  }

  fclose(out);

  std::string mapmetadatafile = map_server_config_.map_save_path_ + "/" + map_server_config_.map_save_name_ + label_name + ".yaml";
  //   ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
  std::cout << "Writing map occupancy data to " << mapmetadatafile.c_str() << std::endl;
  FILE *yaml = fopen(mapmetadatafile.c_str(), "w");

  /*
    resolution: 0.100000
    origin: [0.000000, 0.000000, 0.000000]
    #
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196

   */

  geometry_msgs::Quaternion orientation = ros_map->info.origin.orientation;
  tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, 0.00]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n", mapdatafile.c_str(), ros_map->info.resolution,
          ros_map->info.origin.position.x, ros_map->info.origin.position.y);

  fclose(yaml);

  //   ROS_INFO("Done\n");
  std::cout << "Save occupancy map Done" << std::endl;
}

void OccupancyServer::processScan(GeneralLaserScan &scan, std::vector<double> robot_pose) {
  GridIndex robotIndex = occupancy_map_->ConvertWorld2GridIndex(robot_pose[3], robot_pose[4]);
  for (int id = 0; id < scan.ranges.size(); id++) {
    // 取出该激光雷达扫描点的距离和角度
    double dist = scan.ranges[id];
    double angle = scan.angles[id];
    if (std::isinf(dist) || std::isnan(dist)) {
      if (std::isnan(dist) && map_server_config_.use_nan) {
        dist = map_server_config_.max_radius + 0.1;
      } else {
        continue;
      }
    }

    if (dist > map_server_config_.max_radius) {
      dist = map_server_config_.max_radius + 0.1;
    }
    double theta = robot_pose[2];
    double laser_x = dist * cos(theta + angle);
    double laser_y = dist * sin(theta + angle);
    double world_x = laser_x + robot_pose[3];
    double world_y = laser_y + robot_pose[4];
    GridIndex occIndex = occupancy_map_->ConvertWorld2GridIndex(world_x, world_y);
    if (dist <= map_server_config_.max_radius) {
      occupancy_map_->updateGrid(occupancy_map_->getMapRoot(), occIndex, occ_status);
    }
    if (dist <= map_server_config_.max_radius || map_server_config_.fill_with_white) {
      std::vector<GridIndex> freeIndex = TraceLine(robotIndex.x, robotIndex.y, occIndex.x, occIndex.y);
      for (int i = 0; i < freeIndex.size(); i++) {
        if (freeIndex[i].x == occIndex.x && freeIndex[i].y == occIndex.y) {
          continue;
        }
        occupancy_map_->updateGrid(occupancy_map_->getMapRoot(), freeIndex[i], free_status);
      }
    }
  }
}

OccupancyServerFromFile::OccupancyServerFromFile(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) {
  nh_ = nh;
  nh_private_ = nh_private;

  OccupancyMap2D::MapParams map_params = getMapParamsFromRosParam(nh_private);
  getServerConfigFromRosParam(nh_private);
  nh_private_.getParam("/occupancy_mapping_2D/use_file_num", use_file_num_);
  nh_private_.getParam("/occupancy_mapping_2D/data_file_1", data_file_1_);
  nh_private_.getParam("/occupancy_mapping_2D/data_file_2", data_file_2_);
  nh_private_.getParam("/occupancy_mapping_2D/pose_file", pose_file_);
  nh_private_.getParam("/occupancy_mapping_2D/label_file", label_file_);

  occupancy_map_.reset(new OccupancyMap2D(map_params));
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_server_config_.pub_map_topic_, 1, true);
  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/laser_scan", 1, true);

  publish_occupancy_map_srv_ = nh_private_.advertiseService("publish_occupancy_map", &OccupancyServerFromFile::publishOccupancyMapCallback, this);
}

bool OccupancyServerFromFile::publishOccupancyMapCallback(std_srvs::Empty::Request & /*request*/, std_srvs::Empty::Response &) {
  publishMap();
  return true;
}

// void OccupancyServerFromFile::readRobotPoses() {
//   std::vector<std::vector<double>> poses_data;
//   std::ifstream file_data(pose_file_);  // 打开pose数据文件
//   int count = 0;
//   double temp_data;
//   std::vector<double> pose_data;
//   while (file_data >> temp_data) {
//     count++;
//     pose_data.push_back(temp_data);
//     if (count == 12) {
//       count = 0;
//       poses_data.push_back(pose_data);
//       pose_data.clear();
//     }
//   }
//   file_data.close();

//   float transformTemp[6];
//   std::vector<double> trans_temp_data;

//   for (int k = 0; k < poses_data.size(); k++) {
//     Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
//     for (int i = 0; i < 3; i++) {
//       for (int j = 0; j < 4; j++) {
//         transform_(i, j) = poses_data[k][i * 4 + j];
//       }
//     }
//     Eigen::Transform<float, 3, Eigen::Affine> transform(transform_);
//     pcl::getTranslationAndEulerAngles(transform, transformTemp[3], transformTemp[4], transformTemp[5], transformTemp[0], transformTemp[1], transformTemp[2]);
//     for (int i = 0; i < 6; i++) {
//       trans_temp_data.push_back(transformTemp[i]);  // roll pitch yaw x y z
//     }
//     robot_poses_.push_back(trans_temp_data);
//     trans_temp_data.clear();
//   }
// }

void OccupancyServerFromFile::readLabelFile() {
  std::ifstream file_data(label_file_);  // 打开pose数据文件
  if (!file_data.is_open()) {
    return;
  }
  int count = 0;
  int label_data;
  while (file_data >> label_data) {
    label_indices_[label_data].push_back(count);
    count++;
  }
  file_data.close();
}

void OccupancyServerFromFile::readRobotPoses() {
  robot_poses_.clear();
  std::vector<std::vector<double>> poses_data;
  std::ifstream file_data(pose_file_);  // 打开pose数据文件
  int count = 0;
  double temp_data;
  std::vector<double> pose_data;
  while (file_data >> temp_data) {
    count++;
    pose_data.push_back(temp_data);
    if (count == 7) {
      count = 0;
      poses_data.push_back(pose_data);
      pose_data.clear();
    }
  }
  file_data.close();

  float transformTemp[6];
  std::vector<double> trans_temp_data;

  for (int k = 0; k < poses_data.size(); k++) {
    Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q(poses_data[k][3], poses_data[k][4], poses_data[k][5], poses_data[k][6]);
    Eigen::Matrix3f rx = q.toRotationMatrix();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        transform_(i, j) = rx(i, j);
      }
    }
    transform_(0, 3) = poses_data[k][0];
    transform_(1, 3) = poses_data[k][1];
    transform_(2, 3) = poses_data[k][2];
    Eigen::Transform<float, 3, Eigen::Affine> transform(transform_);
    pcl::getTranslationAndEulerAngles(transform, transformTemp[3], transformTemp[4], transformTemp[5], transformTemp[0], transformTemp[1], transformTemp[2]);
    for (int i = 0; i < 6; i++) {
      trans_temp_data.push_back(transformTemp[i]);  // roll pitch yaw x y z
    }
    robot_poses_.push_back(trans_temp_data);
    trans_temp_data.clear();
  }
}

void OccupancyServerFromFile::getFileList() {
  if (data_file_1_.empty()) std::cout << "file open error" << std::endl;
  // 文件读取接口初始化
  DIR *dir;
  struct dirent *ptr;
  std::vector<std::string> file_list;
  const char *p1 = data_file_1_.c_str();
  if (!(dir = opendir(p1))) return;

  while ((ptr = readdir(dir)) != NULL) {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) {
      continue;
    }
    if (strcmp(ptr->d_name, "pcd") == 0) {
      continue;
    }
    file_list.push_back(ptr->d_name);
  }
  closedir(dir);
  file_list_num_.clear();
  for (int i = 0; i < file_list.size(); i++) {
    file_list[i] = file_list[i].substr(0, file_list[i].rfind("."));
    int temp1 = atoi(file_list[i].c_str());
    file_list_num_.push_back(temp1);
  }
  std::sort(file_list_num_.begin(), file_list_num_.end());
}

void OccupancyServerFromFile::getInitialCloud(int num_index, pcl::PointCloud<pcl::PointXYZI>::Ptr initial_cloud) {
  initial_cloud->clear();
  if (use_file_num_ >= 1) {
    std::string string_input1(data_file_1_);
    // std::string pcd_name_1 = string_input1 + padZeros(file_list_num_[num_index]) + ".pcd";
    std::string pcd_name_1 = string_input1 + std::to_string(file_list_num_[num_index]) + ".pcd";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name_1, *cloud_1) == -1) {
      PCL_ERROR("[Occypancy mapping] Couldn't read the pcd file!\n");
    }
    *initial_cloud += *cloud_1;
  }
  if (use_file_num_ >= 2) {
    std::string string_input2(data_file_2_);
    // std::string pcd_name_2 = string_input2 + padZeros(file_list_num_[num_index]) + ".pcd";
    std::string pcd_name_2 = string_input2 + std::to_string(file_list_num_[num_index]) + ".pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name_2, *cloud_2) == -1) {
      PCL_ERROR("[Occypancy mapping] Couldn't read the pcd file!\n");
    }
    *initial_cloud += *cloud_2;
  }
  if (use_file_num_ >= 3) {
    ROS_WARN("[Occypancy mapping] Can't use 3 or more pcd files.");
  }
}

void OccupancyServerFromFile::startMapping() {
  readLabelFile();
  readRobotPoses();
  getFileList();
  int count = 0;
  int total_size = 0;
  // 读取label值
  if (label_indices_.size() != 0) {
    for (auto &label : label_indices_) {
      if (label.first >= 0) {
        occupancy_map_->initializeMap(robot_poses_[label.second[0]]);
        pcl::PointCloud<pcl::PointXYZI>::Ptr initial_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        double average_time = 0.0;
        for (int label_index = 0; label_index < label.second.size(); label_index++) {
          int num_index = label.second[label_index];
          struct timeval tv_begin, tv_end;
          // gettimeofday(&tv_begin, NULL);
          // printf("Processing index = %d / %d\n", num_index, robot_poses_.size() - 1);
          getInitialCloud(num_index, initial_cloud);

          sensor_msgs::LaserScan laser_scan_output;

          GeneralLaserScan scan;
          getScan(initial_cloud, &scan, laser_scan_output);
          pubLaserScan(laser_scan_output);
          processScan(scan, robot_poses_[num_index]);
          count++;
          std::cout << "Finish " << count << " / " << robot_poses_.size() << std::endl;

          // gettimeofday(&tv_end, NULL);
          // double time_used = 1000000 * (tv_end.tv_sec - tv_begin.tv_sec) + tv_end.tv_usec - tv_begin.tv_usec;
          // average_time = average_time * (num_index * 1.0 / (num_index + 1)) + time_used / (num_index + 1);
          // std::cout << "Processed " << num_index + 1 << " files. Use " << time_used / 1000 << " ms. Average time: " << average_time / 1000<<" ms."<<std::endl;
        }
        std::cout << "Saving Map " << std::endl;
        saveMap(((label.first == 0) ? "" : std::to_string(label.first)));
      } else {
        count += label.second.size();
      }
    }
  } else {
    occupancy_map_->initializeMap(robot_poses_[0]);
    pcl::PointCloud<pcl::PointXYZI>::Ptr initial_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    double average_time = 0.0;
    for (int num_index = 0; num_index < robot_poses_.size(); num_index++) {
      struct timeval tv_begin, tv_end;
      // gettimeofday(&tv_begin, NULL);
      // printf("Processing index = %d / %d\n", num_index, robot_poses_.size() - 1);
      getInitialCloud(num_index, initial_cloud);

      sensor_msgs::LaserScan laser_scan_output;

      GeneralLaserScan scan;
      getScan(initial_cloud, &scan, laser_scan_output);
      pubLaserScan(laser_scan_output);
      processScan(scan, robot_poses_[num_index]);

      // gettimeofday(&tv_end, NULL);
      // double time_used = 1000000 * (tv_end.tv_sec - tv_begin.tv_sec) + tv_end.tv_usec - tv_begin.tv_usec;
      // average_time = average_time * (num_index * 1.0 / (num_index + 1)) + time_used / (num_index + 1);
      // std::cout << "Processed " << num_index + 1 << " files. Use " << time_used / 1000 << " ms. Average time: " << average_time / 1000<<" ms."<<std::endl;
    }
    saveMap();
  }
}

OccupancyServerRealTime::OccupancyServerRealTime(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : transformer_(nh, nh_private) {
  nh_ = nh;
  nh_private_ = nh_private;
  OccupancyMap2D::MapParams map_params = getMapParamsFromRosParam(nh_private);
  getServerConfigFromRosParam(nh_private);
  nh_private_.getParam("/occupancy_mapping_2D/lidar_type", lidar_type_);
  nh_private_.getParam("/occupancy_mapping_2D/pointcloud_topic", pointcloud_topic_);
  nh_private_.getParam("/occupancy_mapping_2D/odom_topic", odom_topic_);

  occupancy_map_.reset(new OccupancyMap2D(map_params));
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_server_config_.pub_map_topic_, 1, true);
  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/laser_scan", 1, true);
  if (lidar_type_ == 1) {
    std::cout << "use livox" << std::endl;
    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 5, &OccupancyServerRealTime::livoxCallback, this);
  } else if (lidar_type_ == 0) {
    std::cout << "use common" << std::endl;
    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 5, &OccupancyServerRealTime::pointcloudCallback, this);
  } else {
    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 5, &OccupancyServerRealTime::pointcloudCallback, this);
  }

  publish_occupancy_map_srv_ = nh_private_.advertiseService("publish_occupancy_map", &OccupancyServerRealTime::publishOccupancyMapCallback, this);

  tf::TransformListener tf_listener;
  transform_st2s_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  transform_st2s_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  double roll, pitch, yaw;
  try {
    tf_listener.waitForTransform(map_server_config_.body_frame_, map_server_config_.sensor_frame_, ros::Time(0), ros::Duration(1.0));
    tf_listener.lookupTransform(map_server_config_.body_frame_, map_server_config_.sensor_frame_, ros::Time(0), transform_st2s_);
    tf::Matrix3x3(transform_st2s_.getRotation()).getEulerYPR(yaw, pitch, roll);
    if (fabs(roll - 0) > 0.000001 || fabs(pitch - 0) > 0.000001) {
      transform_flag_ = true;
      transform_st2s_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    }
  } catch (tf::LookupException &e) {
    ROS_ERROR("%s", e.what());
  }
}

bool OccupancyServerRealTime::publishOccupancyMapCallback(std_srvs::Empty::Request & /*request*/, std_srvs::Empty::Response &) {
  publishMap();
  return true;
}

bool OccupancyServerRealTime::getNextPointcloudFromQueue(std::queue<sensor_msgs::PointCloud2::Ptr> *queue, sensor_msgs::PointCloud2::Ptr *pointcloud_msg, geometry_msgs::PoseStamped *robot_pose) {
  const size_t kMaxQueueSize = 50;
  // std::cout<<"queue size: "<<queue->size()<<std::endl;
  if (queue->empty()) {
    return false;
  }
  *pointcloud_msg = queue->front();
  int transformer_flag = transformer_.lookupTransform(map_server_config_.map_frame_, (*pointcloud_msg)->header.frame_id, (*pointcloud_msg)->header.stamp, robot_pose);
  if (transformer_flag == 1) {
    queue->pop();
    return true;
  } else if (transformer_flag == 0) {
    if (queue->size() >= kMaxQueueSize) {
      ROS_WARN_THROTTLE(60,
                        "[Occypancy mapping] Input pointcloud queue getting too long! Dropping "
                        "some pointclouds. Either unable to look up transform "
                        "timestamps or the processing is taking too long.");
      while (queue->size() >= kMaxQueueSize) {
        queue->pop();
      }
    }
  } else if (transformer_flag == 2) {
    queue->pop();
    return false;
  } else {
    std::cout << "Unknown transformer_flag" << std::endl;
  }
  return false;
}
std::vector<double> OccupancyServerRealTime::getPoseFromPoseStamped(const geometry_msgs::PoseStamped &robot_pose) {
  std::vector<double> pose;  // roll pitch yaw x y z
  tf::Quaternion quat;
  tf::quaternionMsgToTF(robot_pose.pose.orientation, quat);
  double roll, pitch, yaw;
  if (transform_flag_) {
    tf::Transform transform_m2s;
    transform_m2s.setRotation(quat);
    transform_m2s.setOrigin(tf::Vector3(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z));
    tf::Transform transform_m2st = transform_m2s * (transform_st2s_.inverse());
    tf::Matrix3x3(transform_m2st.getRotation()).getRPY(roll, pitch, yaw);
  } else {
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  }

  pose.push_back(roll);
  pose.push_back(pitch);
  pose.push_back(yaw);
  pose.push_back(robot_pose.pose.position.x);
  pose.push_back(robot_pose.pose.position.y);
  pose.push_back(robot_pose.pose.position.z);
  return pose;
}

void OccupancyServerRealTime::processPointCloudAndPose(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg, const geometry_msgs::PoseStamped &robot_pose) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr initial_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*pointcloud_msg, *initial_cloud);

  double yaw = tf::getYaw(robot_pose.pose.orientation);
  std::vector<double> pose;
  pose = getPoseFromPoseStamped(robot_pose);

  if (!is_initialized_) {
    occupancy_map_->initializeMap(pose);
    is_initialized_ = true;
  }

  sensor_msgs::LaserScan laser_scan_output;
  GeneralLaserScan scan;
  getScan(initial_cloud, &scan, laser_scan_output);
  pubLaserScan(laser_scan_output);
  processScan(scan, pose);
}

void OccupancyServerRealTime::pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg_in) {
  sensor_msgs::PointCloud2::Ptr pointcloud_msg_out(new sensor_msgs::PointCloud2);
  if (transform_flag_) {
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transform_st2s_, eigen_transform);
    pcl_ros::transformPointCloud(eigen_transform, *pointcloud_msg_in, *pointcloud_msg_out);
    Eigen::Vector4f testpoint;
    testpoint << 1, 0, 0, 1;
  } else {
    pointcloud_msg_out = pointcloud_msg_in;
  }
  pointcloud_queue_.push(pointcloud_msg_out);
  geometry_msgs::PoseStamped robot_pose;
  sensor_msgs::PointCloud2::Ptr pointcloud_msg;
  // bool processed_any = false;
  while (getNextPointcloudFromQueue(&pointcloud_queue_, &pointcloud_msg, &robot_pose)) {
    // struct timeval tv_begin, tv_end;
    // gettimeofday(&tv_begin, NULL);

    processPointCloudAndPose(pointcloud_msg, robot_pose);
    // processed_any = true;
    count++;
    // gettimeofday(&tv_end, NULL);
    // double time_used = 1000000 * (tv_end.tv_sec - tv_begin.tv_sec) + tv_end.tv_usec - tv_begin.tv_usec;
    // average_time = average_time * ((count - 1) * 1.0 / count) + time_used / count;
    // std::cout << "processed " << count << " msgs. Use " << time_used/1000 << " ms. Average time: " << average_time / 1000 <<" ms. "<<std::endl;
  }

  // if (!processed_any)
  // {
  //     return;
  // }
}

void OccupancyServerRealTime::livoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr &pointcloud_msg_in) {
  sensor_msgs::PointCloud2::Ptr pointcloud_msg_out(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr pointcloud_msg_origin(new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (unsigned int i = 0; i < pointcloud_msg_in->point_num; ++i) {
    pcl::PointXYZI pt;
    pt.x = pointcloud_msg_in->points[i].x;
    pt.y = pointcloud_msg_in->points[i].y;
    pt.z = pointcloud_msg_in->points[i].z;

    pt.intensity = pointcloud_msg_in->points[i].reflectivity;  // The integer part is line number and the decimal part is timestamp
    origin_cloud->push_back(pt);
  }
  pcl::toROSMsg(*origin_cloud, *pointcloud_msg_origin);

  if (transform_flag_) {
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transform_st2s_, eigen_transform);
    pcl_ros::transformPointCloud(eigen_transform, *pointcloud_msg_origin, *pointcloud_msg_out);
    Eigen::Vector4f testpoint;
    testpoint << 1, 0, 0, 1;
  } else {
    pointcloud_msg_out = pointcloud_msg_origin;
  }
  pointcloud_queue_.push(pointcloud_msg_out);
  geometry_msgs::PoseStamped robot_pose;
  sensor_msgs::PointCloud2::Ptr pointcloud_msg;
  // bool processed_any = false;
  while (getNextPointcloudFromQueue(&pointcloud_queue_, &pointcloud_msg, &robot_pose)) {
    // struct timeval tv_begin, tv_end;
    // gettimeofday(&tv_begin, NULL);

    processPointCloudAndPose(pointcloud_msg, robot_pose);
    // processed_any = true;
    count++;
    // gettimeofday(&tv_end, NULL);
    // double time_used = 1000000 * (tv_end.tv_sec - tv_begin.tv_sec) + tv_end.tv_usec - tv_begin.tv_usec;
    // average_time = average_time * ((count - 1) * 1.0 / count) + time_used / count;
    // std::cout << "processed " << count << " msgs. Use " << time_used/1000 << " ms. Average time: " << average_time / 1000 <<" ms. "<<std::endl;
  }

  // if (!processed_any)
  // {
  //     return;
  // }
}

}  // namespace occupancy_mapping
