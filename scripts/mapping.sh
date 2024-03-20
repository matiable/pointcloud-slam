#!/bin/bash
map_name=${1:-ProjectName-LocationName}
validate=${2:-y}
slam_function=${3:-fast}
enable_rviz=${4:-true}

jy_cog_home=/home/ysc/jy_cog
maps_path=$jy_cog_home/system/maps

echo "----------- Mapping -----------"

# get map path
date_time=$(date +"%Y%m%d-%H%M%S")

# 创建文件夹名（包含字符串、日期和时间）
folder_name="${map_name}-${date_time}"
directory="${maps_path}/${folder_name}"

# 创建目录
mkdir -p "${directory}/details/frames"

# 检查目录是否创建成功
if [ ! -d "${directory}" ]; then
    echo "创建目录失败：${directory}"
    exit 1
fi

# 检查是否存在maps/default文件夹
if [ -d "$maps_path/default" ]; then        # 存在default路径
    echo "存在default"
    if [ -L "$maps_path/default" ]; then    # default是个软链接
        echo "default是软链接，直接删除"
        rm -rf $maps_path/default
    else
        echo "default不是软链接，备份"
        mv $maps_path/default $maps_path/default-${date_time}   # 不是软链接，则备份一下
    fi
else
    echo "default不存在"
    rm -rf $maps_path/default
fi

ln -s ${directory} $jy_cog_home/system/maps/default     # default做为软链接，成为当前建图的工作目录

echo "创建并使用地图目录：${directory}"


if [ $validate = "y" ]
then
	echo "启用新地图"
    rm -rf $jy_cog_home/system/map
    ln -s ${directory} $jy_cog_home/system/map
elif [ $validate = "n" ]
then
	echo "暂时不启用新地图"
else
	echo "[Warn] Please enter y/n for the second parameter."
fi


if [ $slam_function = "fast" ]
then
    cd $jy_cog_home/slam/scripts/
    bash occupancy_mapping.sh online jueying $directory &
    sleep 1
fi

cd $jy_cog_home/slam/scripts/
bash pointcloud_mapping.sh $directory $slam_function $enable_rviz