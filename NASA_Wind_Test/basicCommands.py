def set_Waypoint(read_Position, travel_Height, travel_Direction, travel_Distance, velocity):

    if target_Distance < 0.002:

        waypoint.velocity.x = velocity[0]
        waypoint.velocity.y = velocity[1]
        waypoint.velocity.z = velocity[2]

        if target_Direction == 0:
            waypoint.latitude = read_Position.latitude + 0.001 + travel_Distance
            waypoint.longitude = read_Position.longitude
            waypoint.altitude = travel_Height

        elif target_Direction == 1:
            waypoint.latitude = read_Position.latitude - 0.001 - travel_Distance
            waypoint.longitude = read_Position.longitude
            waypoint.altitude = travel_Height

        elif target_Direction == 2:
            waypoint.longitude = read_Position.longitude + 0.001 + travel_Distance
            waypoint.latitude = read_Position.latitude
            waypoint.altitude = travel_Height

        elif target_Direction == 3:
            waypoint.longitude = read_Position.longitude + 0.001 + travel_Distance
            waypoint.latitude = read_Position.latitude
            waypoint.altitude = travel_Height

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

def has_Reached_Position(Target_Position, read_Position):
    #function to check if the quad has reached the target position

    lat = read_Position.latitude
    lon = read_Position.longitude
    alt = read_Position.altitude

    distance_to_Lat = lat - final_Target_Position[0]
    distance_to_Lon = lon - final_Target_Position[1]
    distance_to_alt = alt - final_Target_Position[2]

    if alt > 0.95 * final_Target_Position and distance_to_Lat < 0.0001 and distance_to_Lon < 0.0001:
        return True
    else:
        return False


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



def takeoff(height, pub_Position, read_Position):
    #quadrotor takeoff to desired height

    initial_height_Target = GlobalPositionTarget()

    current_Lat = read_Position.latitude
    current_Lon = read_Position.longitude

    if read_Position.altitude < 0.95 * height:
        initial_height_Target.altitude = height
        initial_height_Target.latitude = current_Lat
        initial_height_Target.longitude = current_Lon
        initial_height_Target.velocity.x = 0
        initial_height_Target.velocity.y = 0
        initial_height_Target.velocity.z = 1

        print "The current height is: " + read_Position.altitude
        pub_Position.publish(initial_height_Target)

        return False

    elif read_Position.altitude >= 0.95 * height:
        return True


