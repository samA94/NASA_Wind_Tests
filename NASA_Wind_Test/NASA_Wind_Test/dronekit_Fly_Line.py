import dronekit
from pymavlink import mavutil
import time

print "Connecting to vehicle"
vehicle = dronekit.connect('udpin:0.0.0.0:14550', wait_ready=True)

print "Arming"
vehicle.mode = dronekit.VehicleMode("GUIDED")
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


def goto_position_target_global_int(aLocation, aVelocity):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        aVelocity.x, # X velocity in NED frame in m/s
        aVelocity.y, # Y velocity in NED frame in m/s
        aVelocity.z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle
    vehicle.send_mavlink(msg)

takeoff(10)






