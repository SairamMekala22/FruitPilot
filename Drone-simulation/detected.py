from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# ----------------------- CONNECT -----------------------
print("Connecting to vehicle...")
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)
if vehicle:
    print("Connected to vehicle")


# ----------------------- FUNCTIONS -----------------------

def condition_yaw(heading, relative=False, yaw_speed=20):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    :param heading: Target heading in degrees (0-360)
    :param relative: True for relative angle, False for absolute heading
    :param yaw_speed: Speed of yaw in deg/sec
    """
    
    is_relative = 1 if relative else 0

    msg = vehicle.message_factory.command_long_encode(
        0, 0,                                   # target system, component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,                                      # confirmation
        heading,                                # param 1: target angle (deg)
        yaw_speed,                              # param 2: yaw speed (deg/sec)
        1,                                      # param 3: direction (-1: ccw, 1: cw)
        is_relative,                            # param 4: relative (1) or absolute (0)
        0, 0, 0                                 # param 5-7 (unused)
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()


def arm_and_takeoff(target_altitude):
    """
    Arms vehicle and fly to target_altitude.
    """
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {current_alt:.1f} m")
        if current_alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def adjust_yaw_and_altitude(cx, cy, image_width, image_height, fov_deg=90):
    """
    Adjusts yaw and altitude based on object position in camera frame.
    :param cx: X center of object in image
    :param cy: Y center of object in image
    :param image_width: Width of the camera frame
    :param image_height: Height of the camera frame
    :param fov_deg: Horizontal field of view of the camera in degrees
    """
    current_alt = vehicle.location.global_relative_frame.alt

    # Calculate pixel offset from center
    dx = cx - (image_width / 2)
    dy = cy - (image_height / 2)

    # Normalize horizontal offset and convert to yaw angle
    yaw_angle = (dx / image_width) * fov_deg
    print(f"Adjusting yaw by {yaw_angle:.2f} degrees...")
    condition_yaw(yaw_angle, relative=True)

    # Convert vertical offset to altitude change
    max_altitude_change = 2.0  # max ±2 meters
    altitude_change = -(dy / image_height) * max_altitude_change
    new_altitude = current_alt + altitude_change
    print(f"Adjusting altitude from {current_alt:.2f} → {new_altitude:.2f} meters")

    # Maintain same lat/lon, update only altitude
    current_location = vehicle.location.global_relative_frame
    target_location = LocationGlobalRelative(
        current_location.lat,
        current_location.lon,
        new_altitude
    )
    vehicle.simple_goto(target_location)


# ----------------------- EXECUTION -----------------------

# Step 1: Takeoff
arm_and_takeoff(5)

# Step 2: Simulated detection result (replace with actual model output)
cx = 500  # x-center of object in pixels
cy = 200  # y-center of object in pixels
image_width = 800
image_height = 600

# Step 3: Adjust drone orientation and altitude
adjust_yaw_and_altitude(cx, cy, image_width, image_height)

# Wait for motion to settle
time.sleep(10)

# Step 4: Land
print("Landing...")
vehicle.mode = VehicleMode("LAND")
time.sleep(10)

# Step 5: Close connection
vehicle.close()
print("Completed mission and disconnected.")
