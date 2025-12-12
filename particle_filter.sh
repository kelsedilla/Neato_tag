#!/bin/bash

#ask and create for first neato ip
echo "Please enter the last 2 or 3 digits of Neato IP 1:"
read neato_ip_1_ending
neato_ip_1="192.168.16.${neato_ip_1_ending}"

#ask then make second Neato ip
echo "Please enter the last 2 or 3 digits of Neato IP 2:"
read neato_ip_2_ending
neato_ip_2="192.168.16.${neato_ip_2_ending}"

#Run terminal with neato ip 1
gnome-terminal --tab --title="First Neato" -- bash -c " 
source ~/ros2_ws/install/setup.bash
ros2 launch neato_tag bringup_multi.py host:=$neato_ip_1 robot_name:=neato1 udp_video_port:=5002 udp_sensor_port:=7777
; exec bash"

#run separate terminal neato ip 2
gnome-terminal --tab --title="Second Neato" -- bash -c " 
source ~/ros2_ws/install/setup.bash
echo $neato_ip_2
ros2 launch neato_tag bringup_multi.py host:=$neato_ip_2 robot_name:=neato2 udp_video_port:=5003 udp_sensor_port:=7778
; exec bash"

sleep 10

#run neato tag teleop file
gnome-terminal --tab --title="Basic Teleop" -- bash -c "
source ~/ros2_ws/install/setup.bash
ros2 run neato_tag teleop_multi_simple; exec bash"

#Run special config
gnome-terminal --tab --title="Special Config for Ros" -- bash -c "
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/src/robot_localization/rviz/amcl.rviz; exec bash"

#Run particle filter
gnome-terminal --tab --title="Ros2 Particle Filter - Neato 1" -- bash -c "
source ~/ros2_ws/install/setup.bash
ros2 launch neato_tag test_amcl.py robot_name:=neato1 map_yaml:=/home/zara/classroom2.yaml use_sim_time:=false; exec bash"

#Run particle filter Neato 2
gnome-terminal --tab --title="Ros2 Particle Filter - Neato 2" -- bash -c "
source ~/ros2_ws/install/setup.bash
ros2 launch neato_tag test_amcl.py robot_name:=neato2 map_yaml:=/home/zara/classroom2.yaml use_sim_time:=false; exec bash"