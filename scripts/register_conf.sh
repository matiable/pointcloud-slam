home_path="/home/ysc/jy_cog"
workspace_name="slam"
package_list_type=("jueying_lio" "jueying_slam" "occupancy_mapping")
package_list_common=("pcd2map" "jueying_pgo")

rm -rf $home_path/system/conf/slam

lidar_type="livox"

while read -r line; do
    lidar_type="$line"
done < "$home_path/system/conf/lidar_type"

echo "----------- Register Config -----------"
echo "lidar_type: $lidar_type"


for package_name in ${package_list_type[@]}
do
    echo "---------------------------------"
    echo "Registering $package_name config"
    
    if [ ! -e $home_path/system/conf/$workspace_name/$package_name/params.yaml ]; then
        echo "params.yaml does not exist in $home_path/system/conf/$workspace_name/$package_name. Register it"
        mkdir -p $home_path/system/conf/$workspace_name/$package_name && cp $home_path/$workspace_name/share/$package_name/config/$lidar_type.yaml $home_path/system/conf/$workspace_name/$package_name/params.yaml
    else
        echo "params.yaml already exists in $home_path/system/conf/$workspace_name/$package_name."
    fi
    
done

for package_name in ${package_list_common[@]}
do
    echo "---------------------------------"
    echo "Registering $package_name config"
    
    for file_path in $home_path/$workspace_name/share/$package_name/config/*
    do
        file_name=$(basename "$file_path")
        echo "Find config file: $file_path"
        if [ ! -e $home_path/system/conf/$workspace_name/$package_name/$file_name ]; then
            echo "$file_name does not exist in $home_path/system/conf/$workspace_name/$package_name. Register it"
            mkdir -p $home_path/system/conf/$workspace_name/$package_name && cp $file_path "$_"
            else
            echo "$file_name already exists in $home_path/system/conf/$workspace_name/$package_name."
        fi
    done
done



chmod -R 777 $home_path/system/conf/$workspace_name

