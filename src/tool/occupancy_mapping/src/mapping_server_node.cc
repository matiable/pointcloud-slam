#include "occupancy_mapping/mapping_server.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_server_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  bool use_topic = true;
  nh_private.getParam("/occupancy_mapping_2D/use_topic", use_topic);

  if (use_topic) {
    occupancy_mapping::OccupancyServerRealTime node(nh, nh_private);
    ros::spin();
    node.saveMap();
  } else {
    occupancy_mapping::OccupancyServerFromFile node(nh, nh_private);
    node.startMapping();
    // ros::spin();
    
  }

  return 0;
}