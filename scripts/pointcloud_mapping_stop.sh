#!/bin/bash
execute_names=("mapping.launch" "mapping_pgo.launch")

for execute_name in ${execute_names[*]}
do
    ps aux | grep -e ${execute_name} | grep -v grep | awk '{print $2}' | xargs -i kill -SIGINT {}
done