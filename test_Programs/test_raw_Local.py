import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget
from sensor_msgs.msg import NavSatFix
import time
import sys


rospy.init_node("send_Waypoints")

rospy.wait_for_service("mavros/set_stream_rate")
setRate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

setRate(0, 50, 1)
rospy.Rate(50.0)

global read_Position

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

def main():
    global read_Position

    rospy.Subscriber("/mavros/global_position/global", NavSatFix, position_callback)
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

    takeoff_Waypoint = PositionTarget()

    takeoff_Waypoint.header.stamp = rospy.get_rostime()
    takeoff_Waypoint.header.frame_id = "1"

    takeoff_Waypoint.coordinate_frame = 1

    #takeoff_Waypoint.type_mask = 0b111111000000

    #Taken from ardupilot documentation
    takeoff_Waypoint.type_mask = 0b0000111111000000

    takeoff_Waypoint.position.x = 5
    takeoff_Waypoint.position.y = 5
    takeoff_Waypoint.position.z = travel_Height

    takeoff_Waypoint.velocity.x = 1
    takeoff_Waypoint.velocity.y = 1
    takeoff_Waypoint.velocity.z = 1

    #takeoff_Waypoint.acceleration_or_force.x = 1
    #takeoff_Waypoint.acceleration_or_force.y = 1
    #takeoff_Waypoint.acceleration_or_force.z = 5

    #takeoff_Waypoint.yaw = 4
    #takeoff_Waypoint.yaw_rate = 1

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
    while read_Position.altitude - ground_Level[0] < -.95 * int(travel_Height):
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.2)
        height = read_Position.altitude - ground_Level[0]
        print "Taking off.  The height is: ", height       

    print "The desired height has been reached: ", read_Position.altitude - ground_Level[0]

    rospy.wait_for_service("mavros/set_mode")
    modeSet = rospy.ServiceProxy("mavros/set_mode", SetMode)

    modeSet(0, "AUTO.LOITER")
    #time.sleep(10)

    #takeoff_Waypoint.acceleration_or_force.z = 0
    #takeoff_Waypoint.velocity.z = 0

    while i < 200:
        i = i + 1
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.2)
        print read_Position.altitude - ground_Level[0]

    sys.exit()
    #rospy.spin()

if __name__ == "__main__":
    main()
