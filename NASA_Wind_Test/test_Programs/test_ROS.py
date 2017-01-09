import rospy
from geometry_msgs.msg import PoseWithCovariance
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from mavros_msgs.msg import OverrideRCIn
import time

rospy.init_node("send_Waypoints")

rospy.wait_for_service("mavros/set_stream_rate")
setRate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

setRate(0, 50, 1)
rospy.Rate(50.0)




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

mode_List = [0, "OFFBOARD"]
quad_Command(mode_List, True)

pub_Velocity = rospy.Publisher('/mavros/RC/override', OverrideRCIn)

i = 0
velocity = OverrideRCIn()

while i < 50:
    
    if i%2 == 0:
        velocity.channels[2] = 1200

    else:
        velocity.channels[2] = 1000

    pub_Velocity.publish(velocity)
    i = i + 1
    time.sleep(0.1)

velocity.channels[2] = 900
pub_Velocity.publish(velocity)



