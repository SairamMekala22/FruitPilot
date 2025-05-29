# from dronekit import connect, VehicleMode, LocationGlobalRelative
# import time
# from pymavlink import mavutil

# # Connect to the Vehicle
# print("Connecting to vehicle...")
# vehicle = connect('127.0.0.1:14550', wait_ready=True)

# # Define yaw command function
# def condition_yaw(vehicle, heading, relative=True):
#     is_relative = 1 if relative else 0
#     msg = vehicle.message_factory.command_long_encode(
#         0, 0,
#         mavutil.mavlink.MAV_CMD_CONDITION_YAW,
#         0,
#         heading,    # degrees to turn
#         10,         # yaw speed (deg/s)
#         1,          # direction: 1 = clockwise
#         is_relative,
#         0, 0, 0
#     )
#     vehicle.send_mavlink(msg)
#     vehicle.flush()
#     print(f"Yaw command sent: {heading:.2f} degrees ({'relative' if relative else 'absolute'})")
#     # Wait a bit for the drone to turn
#     time.sleep(2)
#     print(f"Current heading after yaw: {vehicle.heading}°")

# # Function to calculate heading based on bounding box center
# def fruit_detected(x_center_normalized, image_width=640):
#     # Assuming the bounding box center's x-coordinate normalized between -1 and 1
#     x_center_pixel = x_center_normalized * image_width
#     pixel_offset = x_center_pixel - (image_width / 2)

#     # Calculate heading based on the offset (angle in degrees)
#     angle = (pixel_offset / (image_width / 2)) * 90  # Mapping to -90 to 90 degrees
#     return angle

# # Arm and takeoff function
# def arm_and_takeoff(target_altitude):
#     print("Arming motors")
#     while not vehicle.is_armable:
#         print(" Waiting for vehicle to initialise...   ")
#         time.sleep(1)

#     vehicle.mode = VehicleMode("GUIDED")
#     vehicle.armed = True

#     while not vehicle.armed:
#         print(" Waiting for arming...")
#         time.sleep(1)

#     print(f"Taking off to {target_altitude} meters")
#     vehicle.simple_takeoff(target_altitude)

#     # Wait until the vehicle reaches a safe height
#     while True:
#         current_alt = vehicle.location.global_relative_frame.alt
#         print(f" Altitude: {current_alt:.2f}m")
#         if current_alt >= target_altitude * 0.95:
#             print("Reached target altitude")
#             break
#         time.sleep(1)

# # Start sequence
# arm_and_takeoff(10)

# # Get detected fruit's bounding box center (this is just an example; replace with your actual data)
# x_center_normalized = 0.1  # For example, 10% to the right of the center

# # Calculate the heading based on the bounding box location
# yaw_angle = fruit_detected(x_center_normalized)

# # Send the yaw command to the drone to face the fruit
# condition_yaw(vehicle, 180,relative=False)

# # Fly to each waypoint
# # for wp in waypoints:
# #     fly_to_waypoint(wp)

# # Return to Launch
# print("Returning to Launch")
# vehicle.mode = VehicleMode("RTL")

# time.sleep(20)
# vehicle.close()
# print("Mission complete.")

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time

# Connect to the Vehicle (adjust the connection string as needed)
print("Connecting to vehicle...")
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)
if vehicle:
    print("Connected to vehicle")
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

# Arm the drone and takeoff to 5 meters (optional, for testing yaw mid-air)
def arm_and_takeoff(aTargetAltitude):
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:


        print(" Waiting for arming...")
        time.sleep(1)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.1f} m")
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Example usage
arm_and_takeoff(5)

print(f"Current heading: {vehicle.heading}")
desired_yaw = 0  # Face East
print(f"Rotating to {desired_yaw} degrees...")
condition_yaw(desired_yaw)


# Wait for yaw to complete
time.sleep(20)

print(f"New heading: {vehicle.heading}")

# Land and close connection
vehicle.mode = VehicleMode("LAND")
time.sleep(10)
vehicle.close()
