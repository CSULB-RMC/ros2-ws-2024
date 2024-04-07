source ./install/setup.bash

echo "Starting Joy Listener"
ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0 &
sleep 3
echo "Starting Controller Interface"
colcon build --build-base build --install-base install --packages-select joy_control && ros2 run joy_control talker &
sleep 7
echo "Starting drivetrain_ex"
colcon build --build-base build --install-base install --packages-select drive_ex && ros2 run drive_ex drivetrain_ex &
sleep 7
echo "Starting Can View"
candump vcan0
