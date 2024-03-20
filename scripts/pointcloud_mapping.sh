#!/bin/bash
home_path="/home/ysc/jy_cog"
workspace_name="slam"
package_name="jueying_lio"
# execute_name="mapping"
map_path=${1:-$home_path/system/maps/default}
scene=${2:-fast}
enable_rviz=${3:-true}
# lidar_type=${3:-rslidar}


bash ${home_path}/${workspace_name}/scripts/pointcloud_mapping_stop.sh
sleep 1s

if [ $scene = "fast" ] || [ $scene = "odom" ]
then
    source ${home_path}/${workspace_name}/setup.bash
    roslaunch jueying_lio mapping.launch enable_rviz:=$enable_rviz map_path:=$map_path scene:=$scene
elif [ $scene = "indoor" ] || [ $scene = "outdoor" ]
then
    source ${home_path}/${workspace_name}/setup.bash
    roslaunch jueying_lio mapping.launch enable_rviz:=false map_path:=$map_path scene:=$scene &
    roslaunch jueying_pgo mapping_pgo.launch enable_rviz:=$enable_rviz map_path:=$map_path/details scene:=$scene
else
	echo "Please input scene(fast / indoor / outdoor) correctly."
fi
