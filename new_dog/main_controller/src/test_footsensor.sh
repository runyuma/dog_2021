#!/bin/bash
roslaunch stm32_Communication stm32_Communication.launch &
sleep 5
echo "stm32_Communication launched successfully"
rosrun leg_controller leg_controller &
roslaunch main_controller test_footsensor.launch &
echo "test_footsensor launched sucessfully"

sleep 0.1
wait
exit 0
