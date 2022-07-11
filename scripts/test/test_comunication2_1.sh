#bash  \必须跟一个空格
gnome-terminal --window -e 'bash -c "cd ~/catkin_ws; catkin_make; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun fixed_wing_formation_control vir_dir_leader;  exec bash"' \
--tab -e 'bash -c "sleep 4; rosrun fixed_wing_formation_control pack_leader_states; exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun plotjuggler plotjuggler;exec bash"' \
#--tab -e 'bash -c "sleep 12; rosrun plotjuggler plotjuggler;exec bash"' \
#--tab -e 'bash -c "sleep 4; rosrun fixed_wing_formation_control pack_leader_states; exec bash"' \
#--tab -e 'bash -c "sleep 6; rosrun fixed_wing_formation_control vir_dir_leader;  exec bash"' \
#--tab -e 'bash -c "sleep 8; rosrun rviz rviz;exec bash"' \
#--tab -e 'bash -c "sleep 8; rqt ;exec bash"' \
