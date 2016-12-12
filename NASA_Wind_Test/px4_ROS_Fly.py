import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from mavros_msgs.msg import GlobalPositionTarget
import time
import sys

rospy.init_node("send_Waypoints")

rospy.wait_for_service("mavros/set_stream_rate")
setRate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

setRate(0, 50, 1)
rospy.Rate(50.0)

global read_Position

global max_Height

def quad_Command(mode, armVar = False):
    #initialize topics for arming quad
    rospy.wait_for_service("mavros/cmd/arming")
    armQuad = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    modeSet = rospy.ServiceProxy("mavros/set_mode", SetMode) 

    #arm quadrotor and initialize proper mode

    armQuad(armVar)
    print "System Arm Status: ", armVar
    time.sleep(3)

    modeSet(mode[0], mode[1])
    print "Mode set to: ", mode


def position_callback(GPS_Position_From_Quad):
    #function to get the position of the quad
    global read_Position
    read_Position = GPS_Position_From_Quad

def takeoff(height, pub_Position):
    #quadrotor takeoff to desired height
    global read_Position

    initial_height_Target = GlobalPositionTarget()

    current_Lat = read_Position.latitude
    current_Lon = read_Position.longitude

    while read_Position.altitude < 0.95 * height:
        initial_height_Target.altitude = height
        initial_height_Target.latitude = current_Lat
        initial_height_Target.longitude = current_Lon
        initial_height_Target.velocity.x = 0
        initial_height_Target.velocity.y = 0
        initial_height_Target.velocity.z = 1

        print "The current height is: " + read_Position.altitude
        pub_Position.publish(initial_height_Target)
        time.sleep(0.5)



def has_Reached_Position(Target_Position):
    #function to check if the quad has reached the target position
    global read_Position

    lat = read_Position.latitude
    lon = read_Position.longitude
    alt = read_Position.altitude

    distance_to_Lat = lat - final_Target_Position[0]
    distance_to_Lon = lon - final_Target_Position[1]
    distance_to_alt = alt - final_Target_Position[2]

    if alt > 0.95 * final_Target_Position and distance_to_Lat < 0.00001 and distance_to_Lon < 0.00001:
        return True
    else:
        return False


def main():
    global read_Position
    global max_Height, ground_Level

    turn_Target_Location = GlobalPositionTarget

    #Take target location inputs
    target_Direction = input("Please enter whether you would rather initially travel
            north=0, south=1, east=2, or west=3, as an integer")

    target_Distance = input("Enter the distance you with to travel in this direction, in degrees")
    target_Height = input("Enter the height you would like to travel at, in meters")

    turn_Direction = input("Enter the direction you would like to turn in, using the above format")
    turn_Distance = input("Enter the distance you would like to travel in this direction, in degrees")

    travel_Velocity = input("Enter the desired travel velocity in m/s, in [forward, left, up] coordinates")


    mode_List = [0, "OFFBOARD"]
    quad_Command(mode_List, True)

    rospy.Subscriber("/mavros/global_position/global", PoseWithCovariance,
        read_Position)

    #Assign tuple with maximum allowable altitude
    max_Height = (read_Position.altitude + 50,)
    ground_Level = (read_Position.altitude,)
    
    pub_Position = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size = 10)
    
    
    waypoint1 = PositionTarget()
    waypoint1.


    if target_Height < max_Height[0]:
        takeoff(target_Height)

    if target_Distance < 0.0011:
        pub_Position.publish(

    #Set waypoint in direction specified



    #publish waypoint




    while True:
        #resend the waypoint and check to see if the quad has
        #reached the target position.  Break if it has reach target position

        if has_Reached_Position(turn_Target_Location) == True:
            #Check to see if final position has been reached
            break

        else:
            pub_Position(turn_Target_Location)









