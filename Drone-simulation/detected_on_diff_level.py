from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Connect to vehicle
print("Connecting to vehicle...")
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)
print("Connected.")

def condition_yaw(heading, relative=True, yaw_speed=20):
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading, yaw_speed, 1, is_relative,
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def arm_and_takeoff(target_altitude):
    print("Arming...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.2f} m")
        if alt >= target_altitude * 0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)

# Step 1: Takeoff to 10 meters
arm_and_takeoff(10)

# Step 2: Simulate fruit bounding box detection
image_width = 800
image_height = 600
camera_fov_deg = 90  # horizontal FOV

# Simulated bounding box
bbox_x = 600
bbox_y = 300
bbox_w = 100
bbox_h = 100

# Calculate center of the bounding box
fruit_cx = bbox_x + bbox_w / 2
fruit_cy = bbox_y + bbox_h / 2

# Calculate yaw angle based on horizontal offset
dx = fruit_cx - image_width / 2
yaw_angle = (dx / image_width) * camera_fov_deg

# Calculate altitude adjustment based on vertical offset
dy = fruit_cy - image_height / 2
max_altitude_change = 5  # From 10m max, we expect to go down to ~5m if fruit at bottom
altitude_change = -(dy / image_height) * max_altitude_change

# Clamp altitude
current_alt = vehicle.location.global_relative_frame.alt
target_altitude = max(1.0, min(current_alt + altitude_change, 10.0))  # Safe bounds: 1m to 10m

print(f"Yaw angle to fruit: {yaw_angle:.2f}°")
print(f"Target altitude to reach fruit: {target_altitude:.2f} m")

# Step 3: Yaw to face the fruit
condition_yaw(yaw_angle, relative=True)
time.sleep(5)

# Step 4: Descend to fruit level
current_location = vehicle.location.global_relative_frame
target_location = LocationGlobalRelative(current_location.lat, current_location.lon, target_altitude)

print(f"Descending from {current_location.alt:.2f} m to {target_altitude:.2f} m...")
vehicle.simple_goto(target_location)

# Monitor descent until target is reached
while True:
    current_alt = vehicle.location.global_relative_frame.alt
    print(f" Current altitude: {current_alt:.2f} m")
    if abs(current_alt - target_altitude) <= 0.3:
        print("Reached fruit altitude.")
        break
    time.sleep(1)

# Step 5: Land
time.sleep(5)  # Optional: wait at fruit level before landing
# print("Landing...")
# vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 0.1:
    print(f" Landing... Altitude: {vehicle.location.global_relative_frame.alt:.2f} m")
    time.sleep(1)

vehicle.close()
print("Mission complete.")
