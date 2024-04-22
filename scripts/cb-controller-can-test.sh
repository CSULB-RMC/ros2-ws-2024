#!/bin/bash
source ./install/setup.bash

#echo "Starting Joy Listener"
#ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0 &
#sleep 3
echo "Starting Controller Interface"
colcon build --build-base build --install-base install --packages-select joy_control && ros2 run joy_control control_cb &
sleep 7
echo "Starting drivetrain_cb"
colcon build --build-base build --install-base install --packages-select drive_cb && ros2 run drive_cb drivetrain_cb &
sleep 10
echo "Starting Can View"
cansniffer can0
