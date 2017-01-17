import rospy
from mavros_msgs.msg import PositionTarget


def set_Local_Waypoint(x_Pos, y_Pos, travel_Height, x_Vel, y_Vel, z_Vel):

    takeoff_Waypoint = PositionTarget()

    takeoff_Waypoint.header.stamp = rospy.get_rostime()
    takeoff_Waypoint.header.frame_id = "1"

    takeoff_Waypoint.coordinate_frame = 1


    #Taken from ardupilot documentation
    takeoff_Waypoint.type_mask = 0b0000111111000000

    takeoff_Waypoint.position.x = x_Pos
    takeoff_Waypoint.position.y = y_Pos
    takeoff_Waypoint.position.z = travel_Height

    takeoff_Waypoint.velocity.x = x_Vel
    takeoff_Waypoint.velocity.y = y_Vel
    takeoff_Waypoint.velocity.z = z_Vel

    return takeoff_Waypoint
