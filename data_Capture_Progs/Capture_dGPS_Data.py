import rospy
import message_filters
from sensor_msgs.msg import NavSatFix, TimeReference
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import time
import os

global pos_File, imu_data

rospy.init_node('dGPS_Data_Capture')

#filename = raw_input("Please enter the desired file name, with the extension.")

filename = "data/dGPS_Data1.txt"
i=1
while os.path.isfile(filename):
    i = i + 1
    filename = filename[:14] + str(i) + '.txt'


pos_File = open(filename, 'w')
pos_File.write('dGPS Data Storage' + '\n')

def callback(glob_Pos, diff_Pos, gps_Time):
    global pos_File

    write_String = (str(gps_Time.time_ref) + ',' + str(glob_Pos.latitude) + ','
      + str(glob_Pos.longitude) + ',' + str(glob_Pos.altitude)
      + ',' + str(diff_Pos.pose.position.x) + ',' + str(diff_Pos.pose.position.y)
      + str(diff_Pos.pose.position.z) + ',' + str(diff_Pos.twist.linear.x) + ','
      + str(diff_Pos.twist.linear.y) + ',' + str(diff_Pos.twist.linear.z) + ','
      + str(diff_Pos.twist.angular.z))


    timestamp = float(imu_data.header.stamp.secs) + float(imu_data.header.stamp.nsecs)/1000000000
    timestamp = repr(timestamp)

    write_String = timestamp + write_String

    pos_File.write(write_String)
    pos_File.flush()


glob_sub = message_filters.Subscriber("/gps/fix", NavSatFix)
diff_sub = message_filters.Subscriber("/gps/rtkfix", Odometry)
time_sub = message_filters.Subscriber("/gps/time", TimeReference)

ts = message_filters.TimeSynchronizer([glob_sub, diff_sub, time_sub], 1)
ts.registerCallback(callback)

rospy.spin()
