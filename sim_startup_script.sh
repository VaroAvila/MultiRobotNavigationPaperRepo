#!/bin/bash


echo Launching navigation...
gnome-terminal -- bash -c "ros2 launch neo_simulation2 multi_robot_navigation_navfn.launch.py; exec bash"

echo Launching simulation...
sleep 2
gnome-terminal -- bash -c "ros2 launch neo_simulation2 multi_robot_simulation.launch.py; exec bash"

echo Launching RVIZ...
sleep 3
gnome-terminal -- bash -c "ros2 launch neo_nav2_bringup rviz_launch.py rviz_config:=install/neo_nav2_bringup/share/neo_nav2_bringup/rviz/multi_robot.rviz; exec bash"



