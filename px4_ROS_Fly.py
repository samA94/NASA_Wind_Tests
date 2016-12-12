import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from mavros_msgs.msg import GlobalPositionTarget
import time
import sys

from basicCommands import set_Waypoint, has_Reached_Position, quad_Command, takeoff

rospy.init_node("send_Waypoints")

rospy.wait_for_service("mavros/set_stream_rate")
setRate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

setRate(0, 50, 1)
rospy.Rate(50.0)

global read_Position


def position_callback(GPS_Position_From_Quad):
    #function to get the position of the quad
    global read_Position
    read_Position = GPS_Position_From_Quad


def main():
    global read_Position


    #Take target location inputs
    target_Direction = input("Please enter whether you would rather initially travel
            north=0, south=1, east=2, or west=3, as an integer")

    target_Distance = input("Enter the distance you with to travel in this direction, in degrees")
    target_Height = input("Enter the height you would like to travel at, in meters")

    turn_Direction = input("Enter the direction you would like to turn in, using the above format")
    turn_Distance = input("Enter the distance you would like to travel in this direction, in degrees")

    travel_Velocity = input("Enter the desired travel velocity in m/s, in [forward, left, up] coordinates")


    rospy.Subscriber("/mavros/global_position/global", PoseWithCovariance,
        position_callback)

    mode_List = [0, "OFFBOARD"]
    quad_Command(mode_List, True)

    #Set home position
    home_Position = read_Position

    #Assign tuple with maximum allowable altitude
    max_Height = (read_Position.altitude + 50,)
    ground_Level = (read_Position.altitude,)
    
    pub_Position = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size = 10)
    
    #initialize waypoints
    waypoint1 = PositionTarget()
    waypoint2 = PositionTarget()

    #take off to requested height
    if target_Height+ground_Level < max_Height[0]:
        check = False
        while check==False:
            check = takeoff(target_Height+ground_Level, pub_Position, read_Position)
            time.sleep(0.2)

    #set the first waypoint location and velocity
    waypoint1 = set_Waypoint(read_Position, travel_Height+ground_Level, travel_Direction,
            travel_Distance, travel_Velocity)

    #publish waypoint
    pub_Position.publish(waypoint1)

    while True:
        #resend the waypoint and check to see if the quad has
        #reached the target position.  Break if it has reach target position

        if has_Reached_Position(final_Target_Position, read_Position) == True:
            #Check to see if final position has been reached
            break

        else:
            pub_Position.publish(waypoint1)
            time.sleep(0.1)

    #Set turn waypoint
    waypoint2 = set_Waypoint(read_Position, travel_Height+ground_Level, turn_Direction,
            turn_Distance, travel_Velocity)

    start_Time = time.time()

    while start_Time - time.time() < 5:
        pub.publish(waypoint2)
        time.sleep(0.1)

    #publish home waypoint
    home_Waypoint.latitude = home_Position.latitude
    home_Waypoint.longitude = home_Position.longitude
    home_Waypoint.altitude = travel_Height+ground_Level
    


    #land



    #disarm






