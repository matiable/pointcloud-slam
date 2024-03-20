label=${1:--2}
rostopic pub -1 /mapping/floor_label std_msgs/Int8 "data: ${label}"