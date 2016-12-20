import rospy
from sensor_msgs.msg import Imu
import time
import os

global pos_File

rospy.init_node('Onboard_IMU_Data_Capture')


#filename = raw_input("Please enter the desired file name, with the extension.")

filename = "data/Onboard_IMU_Data1.txt"
i=1
while os.path.isfile(filename):
    i = i + 1
    filename = filename[:21] + str(i) + '.txt'


pos_File = open(filename, 'w')
pos_File.write('Onboard IMU Data Storage' + '\n')

def callback(data):
    global pos_File

    timestamp = float(imu_data.header.stamp.secs) + float(imu_data.header.stamp.nsecs)/1000000000
    timestamp = repr(timestamp)

    pos_File.write(timestamp + ',')
    pos_File.write(str(data.linear_acceleration.x) + ',')
    pos_File.write(str(data.linear_acceleration.y) + ',')
    pos_File.write(str(data.linear_acceleration.z) + ',')
    pos_File.write(str(data.angular_acceleration.x) + ',')
    pos_File.write(str(data.angular_acceleration.y) + ',')
    pos_File.write(str(data.angular_acceleration.z) + '\n')
    pos_File.flush()

rospy.Subscriber("/mavros/imu/data_raw", Imu, callback)
rospy.spin()
