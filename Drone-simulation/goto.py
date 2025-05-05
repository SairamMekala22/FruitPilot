from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the Vehicle
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14551', wait_ready=True)

def arm_and_takeoff(target_altitude):
    print("Arming motors")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...   ")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {target_altitude} meters")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {current_alt:.2f}m")
        if current_alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def fly_to_waypoint(location):
    print(f"Flying to: Lat {location.lat}, Lon {location.lon}")
    vehicle.simple_goto(location)
    time.sleep(20)  # Adjust this depending on distance

# Start sequence
arm_and_takeoff(10)

# Define waypoints
waypoints = [
    LocationGlobalRelative(17.3975000, 78.4899000, 10),
    LocationGlobalRelative(17.3977000, 78.4896000, 10),
    LocationGlobalRelative(17.3972861, 78.4897848, 10),  # Return to original position manually before RTL
    LocationGlobalRelative(17.3972900, 78.4897800, 10)
]

# Fly to each waypoint
for wp in waypoints:
    fly_to_waypoint(wp)

# Return to Launch
print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

time.sleep(20)
vehicle.close()
print("Mission complete.")
