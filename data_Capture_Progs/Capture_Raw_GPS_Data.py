import rospy
from sensor_msgs.msg import NavSatFix
import time
import os

global pos_File

rospy.init_node('Raw_GPS_Position_Data_Capture')


#filename = raw_input("Please enter the desired file name, with the extension.")

filename = "data/Raw_GPS_Position_Data1.txt"
i=1
while os.path.isfile(filename):
    i = i + 1
    filename = filename[:26] + str(i) + '.txt'


pos_File = open(filename, 'w')
pos_File.write('Raw GPS Onboard Data Storage' + '\n')

def callback(data):
    global pos_File

    timestamp = float(imu_data.header.stamp.secs) + float(imu_data.header.stamp.nsecs)/1000000000
    timestamp = repr(timestamp)

    pos_File.write(timestamp + ',')
    pos_File.write(str(data.latitude) + ',')
    pos_File.write(str(data.longitude) + ',')
    pos_File.write(str(data.altitude) + '\n')
    pos_File.flush()

rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, callback)
rospy.spin()
