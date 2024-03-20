#!/bin/bash
home_path="/home/ysc/jy_cog"
workspace_name="slam"
execute_name="pub_occ_map"

#bash ${home_path}/${workspace_name}/scripts/${execute_name}_stop.sh
#sleep 1s

source ${home_path}/${workspace_name}/setup.bash
rosservice call /occupancy_mapping_node/publish_occupancy_map
