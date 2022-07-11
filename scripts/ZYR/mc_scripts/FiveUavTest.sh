#bash  \必须跟一个空格
gnome-terminal --window -e 'bash -c "cd ~/catkin_ws; catkin_make; exec bash"' \
--tab -e 'bash -c "sleep 2; source ~/.bashrc;roslaunch px4 multi_uav_mavros_sitl_five.launch;  exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun fixed_wing_formation_control vir_sim_leader;  exec bash"' \
--tab -e 'bash -c "sleep 4; rosrun fixed_wing_formation_control pack_leader_states; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control pack_fw1_states;exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control pack_fw2_states;exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control pack_fw3_states;exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control pack_fw4_states;exec bash"' \
--window -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control follower1_main;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control follower2_main;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control follower3_main;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control follower4_main;exec bash"' \
--window -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control switch_fw4_mode;exec bash"' \
--tab -e 'bash -c "cd ~/zyr_sim/Multi_UAV; rosbag record /uav0/fixed_wing_formation_control/fw_states /uav0/fixed_wing_formation_control/leader_states /uav1/fixed_wing_formation_control/formation_control_states /uav1/fixed_wing_formation_control/fw_states /uav2/fixed_wing_formation_control/formation_control_states /uav2/fixed_wing_formation_control/fw_states /uav3/fixed_wing_formation_control/formation_control_states /uav3/fixed_wing_formation_control/fw_states /uav4/fixed_wing_formation_control/formation_control_states /uav4/fixed_wing_formation_control/fw_states;  exec bash"' \
#--tab -e 'bash -c "sleep 8; rosrun plotjuggler plotjuggler;exec bash"' \
#--tab -e 'bash -c "sleep 12; rosrun plotjuggler plotjuggler;exec bash"' \
#--tab -e 'bash -c "sleep 4; rosrun fixed_wing_formation_control pack_leader_states; exec bash"' \
#--tab -e 'bash -c "sleep 6; rosrun fixed_wing_formation_control vir_dir_leader;  exec bash"' \
#--tab -e 'bash -c "sleep 8; rosrun rviz rviz;exec bash"' \
#--tab -e 'bash -c "sleep 8; rqt ;exec bash"' \
