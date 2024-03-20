#!/bin/bash

jy_cog_home=/home/ysc/jy_cog

cd $jy_cog_home/slam/scripts/
bash occupancy_mapping_stop.sh

cd $jy_cog_home/slam/scripts/
bash pointcloud_mapping_stop.sh