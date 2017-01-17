import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import NavSatFix
import time
import sys

from fnxnsForRaw import set_Local_Waypoint

rospy.init_node("send_Waypoints")

rospy.wait_for_service("mavros/set_stream_rate")
setRate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

setRate(0, 50, 1)
rospy.Rate(50.0)

global read_Position, local_Pose

def quad_Command(mode, armVar = False):
    rospy.wait_for_service("mavros/cmd/arming")
    armQuad = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    modeSet = rospy.ServiceProxy("mavros/set_mode", SetMode) 

    #armQuad(armVar)
    #print "System Arm Status: ", armVar
    for i in range(10):
        modeSet(mode[0],mode[1])
        time.sleep(0.1)
    print "Mode set to: ", mode

    armQuad(armVar)
    print "System Arm Status: ", armVar


def position_callback(GPS_Position_From_Quad):
    #function to get the position of the quad
    global read_Position
    read_Position = GPS_Position_From_Quad

def local_callback(data):
    global local_Pose
    local_Pose = data

def main():
    global read_Position, local_Pose

    rospy.Subscriber("/mavros/global_position/global", NavSatFix, position_callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_callback)

    pub_Position = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size = 5)
    
    time.sleep(.1)
    #Set home position
    home_Position = read_Position
    #NED conventions
    travel_Height = 10

    #Assign tuple with maximum allowable altitude
    max_Height = (read_Position.altitude + 50,)
    ground_Level = (read_Position.altitude,)
    print "The home position is: ", home_Position

    takeoff_Waypoint = set_Local_Waypoint(0,0,10,0.01,0.01,2)

    print takeoff_Waypoint

    i = 0
    time.sleep(1)
    while i < 100:
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.01)
        i = i + 1
        print i

    mode_List = [0, "OFFBOARD"]

    quad_Command(mode_List, True)

    #take off to requested height
    while local_Pose.pose.position.z < .95 * int(travel_Height):
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.2)
        height = read_Position.altitude - ground_Level[0]
        print "Taking off.  The height is: ", height       
    
    print "The desired height has been reached: ", read_Position.altitude - ground_Level[0]
    time.sleep(0.4)

    first_Waypoint = set_Local_Waypoint(0,40,10, 0.01, 10, 0.01)

    while local_Pose.pose.position.y < 20:
        pub_Position.publish(first_Waypoint)
        time.sleep(0.2)
        print local_Pose.pose.position.y

    final_Waypoint = set_Local_Waypoint(40, 20, 10, 10, 0.01, 0.01)

    while i < 250:
        i = i + 1
        pub_Position.publish(final_Waypoint)
        time.sleep(0.1)
        print local_Pose.pose.position.y


    rospy.wait_for_service("mavros/set_mode")
    modeSet = rospy.ServiceProxy("mavros/set_mode", SetMode)

    modeSet(0, "AUTO.LOITER")
    time.sleep(5)

if __name__ == "__main__":
    main()
