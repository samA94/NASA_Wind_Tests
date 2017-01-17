import rospy
from geometry_msgs.msg import TwistStamped
import time
import os

global pos_File

rospy.init_node('Raw_GPS_Velocity_Data_Capture')


#filename = raw_input("Please enter the desired file name, with the extension.")

filename = "data/Raw_GPS_Velocity_Data1.txt"
i=1
while os.path.isfile(filename):
    i = i + 1
    filename = filename[:26] + str(i) + '.txt'


pos_File = open(filename, 'w')
pos_File.write('Raw GPS Position Onboard Data Storage' + '\n')

def callback(data):
    global pos_File

    timestamp = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)/1000000000
    timestamp = repr(timestamp)

    pos_File.write(timestamp + ',')
    pos_File.write(str(data.linear.x) + ',')
    pos_File.write(str(data.linear.y) + ',')
    pos_File.write(str(data.linear.z) + ',')
    pos_File.write(str(data.angular.z) + '\n')
    pos_File.flush()

rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, callback)
rospy.spin()
