#bash  \必须跟一个空格
gnome-terminal --window -e 'bash -c "cd ~/catkin_ws; catkin_make; exec bash"' \
--tab -e 'bash -c "sleep 2; source ~/catkin_ws/devel/setup.bash;roslaunch uav1.launch;  exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control pack_fw1_states;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control follower1_main;exec bash"' \

