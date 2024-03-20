#include "occupancy_mapping/mapping_server.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_server_rt_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    occupancy_mapping::OccupancyServerRealTime node(nh, nh_private);

    ros::spin();
    return 0;
}