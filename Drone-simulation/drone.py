from dronekit import connect, VehicleMode
import time

# Connect to the vehicle on MAVProxy's output port
print("Connecting to vehicle on: udp:127.0.0.1:14550")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# Wait for vehicle to become armable
print("Checking if vehicle is armable...")
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

# Set mode to GUIDED
print("Setting mode to GUIDED...")
vehicle.mode = VehicleMode("GUIDED")
while not vehicle.mode.name == "GUIDED":
    print(" Waiting for mode change...")
    time.sleep(1)

# Arm the vehicle
print("Arming the drone...")
vehicle.armed = True
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

# Take off to 3 meters
takeoff_alt = 100
print("Taking off!")
vehicle.simple_takeoff(takeoff_alt)

# Wait until it reaches a safe height
while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= takeoff_alt * 0.95:
        print("Reached target altitude!")
        break
    time.sleep(1)

# Close the connection
print("Closing vehicle connection")
vehicle.close()