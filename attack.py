from pymavlink import mavutil
import cv2
from ultralytics import YOLO

model = YOLO("yolov8.pt")

def vehicle_connect():
    vehicle = mavutil.mavlink_connection("udp:127.0.0.1:14552", wait_ready=True)
    vehicle.wait_heartbeat()
    return vehicle


cap = cv2.VideoCapture("video.mp4")
#cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


def arm_takeoff(takeoff_alt):
    # set mode to GUIDED
    master.set_mode('GUIDED')

    # send command to ARM
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()

    # send command to takeoff
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, takeoff_alt)

def velocity(dx, dy, dz, yaw_angle):
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111110000111,
        0, 0, 0,
        dx, dy, dz,
        0, 0, 0,
        0, yaw_angle)


# main func
def main():
    arm_takeoff(10)

    max_speed = 1.0  # m/s
    max_yaw_angle = 30.0  # degrees/s
    target_class = "red_box"  # Replace with your target class
    obstacle_classes = ["person", "car"]  # Classes to avoid
    frame_center_x = 320  # Camera frame center (640/2)
    frame_center_y = 240  # Camera frame center (480/2)

    while true:
        ret, frame = cap.read()

        if not ret:
            print("failed to capture frame")
            continue

        results = model(frame)
        detections = results[0].boxes

        dx, dy, dz = 0, 0, 0
        yaw_angle = 0

        for box in detections:
            cls = int(box.cls[0])
            class_name = model.names[cls]
            x1, y1, x2, y2 = box.xyxy[0]
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            

            # obstacle avoidance needs to happen here:...


            if class_name == target_class:
                target_detected = True
                print(f"Target ({target_class}) detected")
                # Calculate error from frame center
                error_x = center_x - frame_center_x
                error_y = center_y - frame_center_y

                dx = 0.005 * error_x * max_speed  # Adjust forward/back
                dy = -0.005 * error_y * max_speed  # Adjust left/right
                dz = 0  # Maintain altitude

                if (x2 - x1) < 0.2 * 640:  # Target is small in frame
                    dx += max_speed * 0.5  # Move forward

                yaw_angle = 0.05 * error_x  # Adjust yaw to center target
                yaw_angle = max(min(yaw_angle, max_yaw_angle), - max_yaw_angle)

            velocity(dx, dy, dz, yaw_angle)

cap.release()
cv2.destroyAllWindows()
master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)


