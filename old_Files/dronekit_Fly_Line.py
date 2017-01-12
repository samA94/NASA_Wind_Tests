import dronekit
from pymavlink import mavutil
import time

print "Connecting to vehicle"
#vehicle = dronekit.connect('udpin:0.0.0.0:14550', wait_ready=True)
vehicle = dronekit.connect('udpin:127.0.0.1:14540', wait_ready=True)

print "Arming"
#vehicle.mode = dronekit.VehicleMode("OFFBOARD")
vehicle.armed = True

while not vehicle.armed:
    print "Waiting for arming"
    time.sleep(0.5)

location = vehicle.location.global_relative_frame

print "The current location is: ", location
print "The battery level is: ", vehicle.battery

def takeoff(des_altitude = 10):
    vehicle.simple_takeoff(des_altitude)
    while True:
        print "Altitude: ", vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt >= des_altitude*.95:
            print "Reached targed altitude"
            break
        time.sleep(0.5)


def goto_position_target_local_int(xPos, yPos, alt, x_Vel, y_Vel, z_Vel, yaw):
    """
    Send SET_POSITION_TARGET_local_INT command to request the vehicle fly to a specified location.
    """
    msg = vehicle.message_factory.set_position_target_local_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000101111000000, # type_mask (only speeds enabled)
        xPox, # lat_int - X Position in WGS84 frame in 1e7 * meters
        yPos, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        x_Vel, # X velocity in NED frame in m/s
        y_Vel, # Y velocity in NED frame in m/s
        z_Vel, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        yaw, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle
    vehicle.send_mavlink(msg)

takeoff(10)

time.sleep(5)

while location.altitude < 0.95*20:
    goto_position_target_local_int(0, 0, 20, 0, 0, 2, 0)
    time.sleep(0.1)

time.sleep(5)


