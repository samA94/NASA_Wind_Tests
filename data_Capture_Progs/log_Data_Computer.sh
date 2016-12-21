#!//bin/bash
#
#roslaunch swiftnav_piksi swiftnav_piksi.launch &
#
python Capture_dGPS_Data.py &
python Capture_Fused_Position_Data.py &
python Capture_Fused_Velocity_Data.py &
python Capture_Raw_GPS_Position_Data.py &
python Capture_Raw_GPS_Velocity_Data.py &
python Capture_Onboard_IMU_Data.py
