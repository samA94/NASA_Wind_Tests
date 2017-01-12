import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
import time
import sys

rospy.init_node("send_Waypoints")

rospy.wait_for_service("mavros/set_stream_rate")
setRate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

setRate(0, 10, 1)

rospy.Rate(10.0)

def quad_Command(mode, armVar = False):
    rospy.wait_for_service("mavros/cmd/arming")
    armQuad = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    modeSet = rospy.ServiceProxy("mavros/set_mode", SetMode) 

    #armQuad(armVar)
    #print "System Arm Status: ", armVar

    modeSet(mode[0],mode[1])
    print "Mode set to: ", mode

    armQuad(armVar)
    print "System Arm Status: ", armVar

    time.sleep(3)
    sys.exit()

mode_List = [0, "GUIDED"]

quad_Command(mode_List, True)

def position_callback(data):




def main():

    mode_List = [0, "GUIDED"]
    quad_Command(mode_List,
    rospy.Subscriber("/mavros/global_position/local", PoseWithCovariance,
        position_callback)















