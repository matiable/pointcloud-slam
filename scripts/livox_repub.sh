#!/bin/bash
home_path="/home/ysc/jy_cog"
workspace_name="slam"
package_name="livox_repub"
execute_name="livox_repub"

bash ${home_path}/${workspace_name}/scripts/${execute_name}_stop.sh
sleep 1s

source ${home_path}/${workspace_name}/setup.bash
roslaunch ${package_name} ${execute_name}.launch