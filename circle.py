from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import math
import sys

connection_string = 'udp:127.0.0.1:14550'  # Replace with your connection (e.g., /dev/ttyACM0 for hardware)

master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()

roi_lat = -35.36178185967436
roi_lon = 149.16717138579642
roi_alt = 10


# circle params
loiter_rad = 20.0
loiter_dur = sys.float_info.max

master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,1000000,
        0,0,0,0,0
        )

master.set_mode("GUIDED")
print("set to GUIDED mode")

master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        0,
        0,0,0,0,
        roi_lat * 1e7,
        roi_lon * 1e7,
        roi_alt
)


print("Switching to LOITER mode to circle ROI...")
master.set_mode("LOITER")
