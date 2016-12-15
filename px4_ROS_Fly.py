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

    rospy.Subscriber("/mavros/global_position/global", PoseWithCovariance,
        position_callback)

    #Set home position
    home_Position = read_Position

    #Assign tuple with maximum allowable altitude
    max_Height = (read_Position.altitude + 50,)
    ground_Level = (read_Position.altitude,)
    print "The home position is: ", home_Position


    #Take target location inputs
    target_Direction = input("Please enter whether you would rather initially travel
            north=0, south=1, east=2, or west=3, as an integer")

    target_Distance = input("Enter the distance you with to travel in this direction, in degrees")
    travel_Height = input("Enter the height you would like to travel at, in meters")

    turn_Direction = input("Enter the direction you would like to turn in, using the above format")
    turn_Travel_Time = input("Enter the time you would like to travel in this direction, in seconds")

    initial_Travel_Velocity = input("Enter the desired travel velocity in m/s, in
            [North_vel, East_vel, Down_vel, yaw_rate] coordinates")

    after_Turn_Travel_Velocity = input("Enter the desired velocity after the turn, using the same
            coordinate convention.")

    #Set the goal location for the turn
    final_Target_Position = waypoint(home_Position, travel_Height+ground_Level, 
            target_Direction, travel_Distance-.001, travel_Velocity)

    print "The position for the turn location has been set as: ", final_Target_Position

    pub_Position = rospy.Publisher("/mavros/setpoint_raw/global", PositionTarget, queue_size = 10)
    
    #initialize waypoints
    waypoint1 = PositionTarget()
    waypoint2 = PositionTarget()

    dummyVar = input("Press enter after the quad has been armed, set to OFFBOARD mode, and has taken off.")

    takeoff_Waypoint = waypoint(home_Position, travel_Height+ground_Level, target_Direction,
                -0.001, [0,0,0,travel_velocity[3]])

    #take off to requested height
    while read_Position.altitude - ground_Level < 0.95 * travel_Height:
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.4)
        print "Taking off.  The height is: " read_Position.altitude - ground_Level       

    print "The desired height has been reached: ", read_Position.altitude - ground_Level

    #set the first waypoint location and velocity
    waypoint1 = set_Waypoint(home_Position, travel_Height+ground_Level, travel_Direction,
            travel_Distance, travel_Velocity)

    #publish waypoint
    pub_Position.publish(waypoint1)
    print "The first waypoint is set at: ", waypoint1

    while True:
        #resend the waypoint and check to see if the quad has
        #reached the target position.  Break if it has reach target position

        if has_Reached_Position(final_Target_Position, read_Position) == True:
            #Check to see if final position has been reached
            break

        else:
            pub_Position.publish(waypoint1)
            print "The current location is: ", read_Position
            time.sleep(0.4)


    waypoint2 = set_Waypoint(read_Position, travel_Height+ground_Level, turn_Direction,
            .001, after_Turn_Travel_Velocity)

    start_Time = time.time()
    #Published the turn
    while time.time() - start_Time < turn_Travel_Time:
        pub_Position.publish(waypoint2)
        print "The current location is: ", read_Position
        time.sleep(0.4)
    

    print "The destination has been reached.  Please use the return to launch feature on 
           the controller or on your ground control station"

    end_Waypoint = waypoint(read_Position, travel_Height+ground_Level, turn_Direction,
                -0.001, [0,0,0,travel_velocity[3]])
    pub_Position.publish(end_Waypoint)

if __name__ == '__main__':
    main()



#Possible changes and improvements:

#All sections need to be tested

#Assuming NED conventions.  However, this is not explicitly stated in documentation

#Yaw is not explained in documentation.  Currently assuming radians, but might need to be changed
#Also, no explanation of yaw mechanics is given - does it stop and turn, or turn while flying?

#Velocities have similar issues.  Assumed to be global, but they could be local

#Could automate arming, mode set, and takeoff, but would not be as safe or controllable
#Might be useful to have ROS automatically send quad home, but don't currently see a RTL function in mavros

#Information was gathered from mavros msg documentation, ROS standards, and mavlink msg documentation


