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

