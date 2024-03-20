#!/bin/bash
home_path="/home/ysc/jy_cog"
workspace_name="slam"
package_name="pcd2map"
execute_name="pcd2map"

bash ${home_path}/${workspace_name}/scripts/${execute_name}_stop.sh
sleep 1s

source ${home_path}/${workspace_name}/setup.bash
roslaunch ${package_name} ${execute_name}.launch

