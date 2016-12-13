import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from mavros_msgs.msg import GlobalPositionTarget
import time

def set_Waypoint(read_Position, travel_Height, travel_Direction, travel_Distance, velocity):

    if target_Distance < 0.002:

        waypoint.velocity.x = velocity[0]
        waypoint.velocity.y = velocity[1]
        waypoint.velocity.z = velocity[2]

        if target_Direction == 0:
            waypoint.latitude = read_Position.latitude + 0.001 + travel_Distance
            waypoint.longitude = read_Position.longitude
            waypoint.altitude = travel_Height
            waypoint.yaw = 90

        elif target_Direction == 1:
            waypoint.latitude = read_Position.latitude - 0.001 - travel_Distance
            waypoint.longitude = read_Position.longitude
            waypoint.altitude = travel_Height
            waypoint.yaw = 270

        elif target_Direction == 2:
            waypoint.longitude = read_Position.longitude + 0.001 + travel_Distance
            waypoint.latitude = read_Position.latitude
            waypoint.altitude = travel_Height
            waypoint.yaw = 0

        elif target_Direction == 3:
            waypoint.longitude = read_Position.longitude - 0.001 - travel_Distance
            waypoint.latitude = read_Position.latitude
            waypoint.altitude = travel_Height
            waypoint.yaw = 180

        else:
            print "Error"
            waypoint.velocity.x = 0
            waypoint.velocity.y = 0
            wayoint.velocity.z = 0

        return waypoint

    else:
        print "The distance you entered was too large"

        while True:
            1==1

def has_Reached_Position(final_Target_Position, read_Position):
    #function to check if the quad has reached the target position

    lat = read_Position.latitude
    lon = read_Position.longitude
    alt = read_Position.altitude

    distance_to_Lat = lat - final_Target_Position.latitude
    distance_to_Lon = lon - final_Target_Position.longitude

    if alt > 0.95 * final_Target_Position.altitude and distance_to_Lat < 0.0001 and distance_to_Lon < 0.0001:
        return True
    else:
        return False


