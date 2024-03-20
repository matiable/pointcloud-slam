#!/bin/bash
map_file_name=${1:-jueying}
home_path="/home/ysc/jy_cog"
workspace_name="slam"
execute_name="save_map"
save_map_path="/home/ysc/jy_cog/system/map/"
map_topic_name="/projected_map"

source ${home_path}/${workspace_name}/setup.bash
rosrun map_server map_saver  map:=${map_topic_name} -f ${save_map_path}${map_file_name}
