from pymavlink import mavutil
import time

# Configuration
connection_string = 'udp:127.0.0.1:14552'  # Replace with your connection (e.g., /dev/ttyACM0 for hardware)
mode_name = 'GUIDED'  # Replace with desired mode (e.g., STABILIZE, AUTO, RTL, GUIDED)

print(f"Connecting to {connection_string}...")
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()
print("Heartbeat received, vehicle connected.")

mode_id = master.mode_mapping().get(mode_name)
if mode_id is None:
    print(f"Error: Mode {mode_name} not supported.")
    exit(1)

print(f"Requesting mode {mode_name}...")
master.set_mode(mode_id)

time.sleep(2)
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
if msg and msg.custom_mode == mode_id:
    print(f"Mode successfully set to {mode_name}")
else:
    print(f"Failed to set mode to {mode_name}")

print("Arming throttle...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0, 0  # 1 = arm, others are unused params
)

timeout = time.time() + 10  # 10-second timeout
armed = False
while not armed and time.time() < timeout:
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        print("Vehicle armed successfully.")
if not armed:
    print("Failed to arm vehicle.")
    exit(1)


takeoff_altitude = 10

# send takeoff command
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, takeoff_altitude  # Altitude in meters
)


#start_time = time.time()

#while time.time() - start_time < timeout:
#msg = master.recv_match(blocking=True, timeout=1.0)

'''if msg.get_type() == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:

        print("Takeoff command accepted")
        takeoff_command_accepted = True
    else:
        print(f"Takeoff command failed with result: {msg.result}")
        break'''


'''print("Sending land command...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,  # Confirmation
    0, 0, 0, 0,  # Unused parameters
    0, 0, 0  # Latitude, longitude, altitude (0 for current location)
)'''


