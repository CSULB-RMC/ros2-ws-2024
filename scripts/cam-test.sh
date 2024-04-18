#!/bin/bash
source ./install/setup.bash


ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0 &
colcon build --build-base build --install-base install --packages-select camera_test
ros2 run camera_test talker &
ros2 run camera_test listener

