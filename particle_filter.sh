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
gnome-terminal --title="First Neato" -- bash -c " 
ros2 launch neato_node2 bringup_multi.py host:=$neato_ip_1 robot_name:=neato1 udp_video_port:=5002 udp_sensor_port:=7777
; exec bash"

#run separate terminal neato ip 2
gnome-terminal --title="Second Neato" -- bash -c " 
echo $neato_ip_2
ros2 launch neato_node2 bringup_multi.py host:=$neato_ip_2 robot_name:=neato2 udp_video_port:=5003 udp_sensor_port:=7778
; exec bash"

#run neato tag teleop file
gnome-terminal --title = "Basic Teleop" -- bash -c "
ros2 run neato_tag teleop; exec bash"

#Run special config
gnome-terminal --title = "Special Config for Ros" -- bash -c"
rviz2 -d ~/ros2_ws/src/robot_localization/rviz/amcl.rviz; exec bash"

#Run particle filter
gnome-terminal --title "Ros2 Particle Filter" -- bash -c"
ros2 launch robot_localization test_amcl.py map_yaml:=/ros2_ws/src/Neato_tag/maps/classroom2.yaml; exec bash"
