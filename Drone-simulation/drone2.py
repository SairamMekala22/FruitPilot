from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the Vehicle (adjust if using Mission Planner)
vehicle = connect('udp:127.0.0.1:14552', wait_ready=True)  # Use 14550 for SITL/Mission Planner
if (vehicle):
    print("Connected to vehicle")

def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# --- Mission Execution ---
arm_and_takeoff(10)

# Fly to a point (example coordinates)
point1 = LocationGlobalRelative(-35.363261, 149.165230, 10)  # Change to your own coordinates
vehicle.simple_goto(point1)

# Wait while flying
time.sleep(15)

# RTL
print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
time.sleep(10)
vehicle.close()
