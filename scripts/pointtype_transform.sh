#!/bin/bash
pc_in=${1:-rslidar}
pc_out=${2:-velodyne}
home_path="/home/ysc/jy_cog"
workspace_name="slam"
package_name="rs_to_velodyne"
execute_name="pointtype_transform"

bash ${home_path}/${workspace_name}/scripts/${execute_name}_stop.sh
sleep 1s

source ${home_path}/${workspace_name}/setup.bash

if [ $pc_in = "rslidar" ]
then
	roslaunch ${package_name} rs_to_velodyne.launch
	#echo "Running pointtype_transform.sh pc_in:$pc_in pc_out:$pc_out"
elif [ $pc_in = "hesai" ]
	then
	roslaunch hesai_to_velodyne hesai_to_velodyne.launch
	echo "Running pointtype_transform.sh pc_in:$pc_in pc_out:$pc_out need to edit"
else
	echo "Please input correct lidar point type."
fi