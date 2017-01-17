import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget
from sensor_msgs.msg import NavSatFix
import time
import sys

from basicCommands import set_Waypoint, has_Reached_Position

rospy.init_node("send_Waypoints")

rospy.wait_for_service("mavros/set_stream_rate")
setRate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

setRate(0, 50, 1)
rospy.Rate(50.0)

global read_Position

filename = raw_input("Please enter the desired file name, with the extension.")

pos_File = open(filename, 'w')

def position_callback(GPS_Position_From_Quad):
    #function to get the position of the quad
    global read_Position
    read_Position = GPS_Position_From_Quad

    with open(filename, 'w') as pos_File:
        pos_File.write(str(read_Position.latitude) + ',')
        pos_File.write(str(read_Position.longitude) + ',')
        pos_File.write(str(read_Position.altitude) + '\n')

def main():
    global read_Position

    rospy.Subscriber("/mavros/global_position/global", NavSatFix,
        position_callback)

    time.sleep(.1)
    #Set home position
    home_Position = read_Position

    #Assign tuple with maximum allowable altitude
    max_Height = (read_Position.altitude + 50,)
    ground_Level = (read_Position.altitude,)
    print "The home position is: ", home_Position


    #Take target location inputs
    target_Direction = raw_input("Please enter whether you would rather initially travel north=0, south=1, east=2, or west=3, as an integer")

    travel_Height = raw_input("Enter the height you would like to travel at, in meters")


    pub_Position = rospy.Publisher("/mavros/setpoint_raw/global", GlobalPositionTarget, queue_size = 10)
    
    #initialize waypoints
    waypoint1 = PositionTarget()
    waypoint2 = PositionTarget()

    dummyVar = input("Press enter after the quad has been armed, set to OFFBOARD mode, and has taken off.")

    takeoff_Waypoint = set_Waypoint(home_Position, int(travel_Height)+ground_Level[0], target_Direction,
                -0.001, [0,0,2,2])

    #take off to requested height
    while read_Position.altitude - ground_Level[0] < 0.95 * int(travel_Height):
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.2)
        height = read_Position.altitude - ground_Level[0]
        print "Taking off.  The height is: ", height       

    print "The desired height has been reached: ", read_Position.altitude - ground_Level[0]

    rospy.spin()

if __name__ == "__main__":
    main()
