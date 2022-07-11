#bash  \必须跟一个空格
gnome-terminal --window -e 'bash -c "cd ~/catkin_ws; catkin_make; exec bash"' \
--tab -e 'bash -c "sleep 2; source ~/.bashrc;roslaunch px4 single_uav_mavros_sitl_sdf_plane.launch;  exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control switch_fw4_mode;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control leader_main;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control vir_sim_leader;exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control pack_leader_states;exec bash"' \
#--tab -e 'bash -c "cd ~/zyr_sim/Single_UAV; rosbag record -a; exec bash"' \

#--tab -e 'bash -c "sleep 12; rosrun plotjuggler plotjuggler;exec bash"' \
