pose_source=${1:-opt}
cd /home/ysc/jy_cog/slam/lib/dynamic_map/
source /home/ysc/jy_cog/slam/setup.bash
if [ $pose_source = "opt" ]
then
    ./construct_full_map /home/ysc/jy_cog/system/maps/default/details/poses_opt.txt /home/ysc/jy_cog/system/maps/default/details/frames /home/ysc/jy_cog/system/maps/default/jueying.pcd 0.1
elif [ $pose_source = "ori" ]
then
    ./construct_full_map /home/ysc/jy_cog/system/maps/default/details/poses_ori.txt /home/ysc/jy_cog/system/maps/default/details/frames /home/ysc/jy_cog/system/maps/default/jueying.pcd 0.1
fi