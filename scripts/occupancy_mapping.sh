#!/bin/bash
home_path="/home/ysc/jy_cog"
workspace_name="slam"
package_name="occupancy_mapping"
execute_name="occupancy_mapping"

echo "Saving current occ-map. Waiting for 5 seconds"
bash ${home_path}/${workspace_name}/scripts/${execute_name}_stop.sh
sleep 5s




# 设置文件夹路径和文件前缀
folder_path="$home_path/system/maps/default"
map_save_path=${3:-$folder_path}

#判断map_save_path是否是软链接
if [ -L "$map_save_path" ]; then    # default是个软链接
    map_save_path=$(readlink $map_save_path)
    echo "default是软链接，实际路径为$map_save_path"
fi
file_prefix="jueying"

# 初始化最大序号为0
max_num=0
exist_flag="0"

# 遍历文件夹中所有匹配前缀的文件
for file in "$folder_path"/"$file_prefix"*".yaml"; do
    if [[ -f "$file" ]]; then
        exist_flag="1"
        # 提取文件名中的序号，假设序号后面有一个点和其他后缀
        num=$(echo "$file" | sed -n "s/^.*$file_prefix\([0-9]*\)\..*$/\1/p")
        echo "num: $num"
        # 更新最大序号
        if [[ "$num" -gt "$max_num" ]]; then
            max_num=$num
        fi
    fi
done
if [ $exist_flag = "0" ]
then
    new_file_name="${file_prefix}"
else
    # 计算新序号（最大序号 + 1）
    new_num=$((max_num + 1))
    # 创建新文件名
    new_file_name="${file_prefix}${new_num}"
fi

map_save_name=${2:-$new_file_name}
mapping_function=${1:-online}


if [ $mapping_function = "online" ]
then
	echo "新栅格地图文件名：$map_save_name"
    source ${home_path}/${workspace_name}/setup.bash
    roslaunch ${package_name} mapping_occupancy.launch map_save_path:=${map_save_path} map_save_name:=${map_save_name}
elif [ $mapping_function = "offline" ]
	then
    source ${home_path}/${workspace_name}/setup.bash
    roslaunch ${package_name} mapping_occupancy.launch map_save_path:=${map_save_path} map_save_name:=${map_save_name} use_topic:=false
else
	echo "Please input correct mapping_function."
fi
