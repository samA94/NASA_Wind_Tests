#!//bin/bash
#
#sshpass -p ubuntu ssh ubuntu@192.168.240.145
#
pkill ros
sleep 1
roscore &
sleep 3
roslaunch phidgets_imu imu.launch &
python Capture_IMU_Data.py
pkill ros
killall python
