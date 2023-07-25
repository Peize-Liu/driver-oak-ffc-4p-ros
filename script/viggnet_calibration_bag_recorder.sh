#!/bin/bash
# this script only for start 

WORK_DIR=/root/oak_ffc_ws/
FPS=30
SEC_TO_US=1000000
ESPOUSE_TIME_US_MAX=${SEC_TO_US}/${FPS}
source /root/oak_ffc_ws/devel/setup.bash
for ((i = 1000; i< ${ESPOUSE_TIME_US_MAX}  ; i+=1000))
do
  roslaunch oakcam_ffc_4p_ros oakcam_ffc_4p.launch expose_time_us:="$i" &
  sleep 5
  rosnode kill /oakcam_ffc_4p_ros
  echo "kill expose_time_us: $i node"
  sleep 2
done
echo "finish"