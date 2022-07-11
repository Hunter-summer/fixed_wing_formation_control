#!/bin/bash
roslaunch ./launch/uav1.launch &
sleep 5
echo "uav1.launchstarting success!"

rosrun fixed_wing_formation_control pack_fw1_states &
sleep 1

rosrun fixed_wing_formation_control follower1_main &
sleep 1

wait
exit 0