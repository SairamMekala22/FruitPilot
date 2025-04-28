from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time

# Connect to vehicle
print("Connecting to vehicle...")
vehicle = connect('udp:127.0.0.1:14552', wait_ready=True)

def clear_mission():
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()

def add_mission():
    cmds = vehicle.commands
    cmds.clear()

    # List of waypoints (lat, lon, alt)
    waypoints = [
        (-35.363261, 149.165230, 10),  # WP1
        (-35.363000, 149.166000, 10),  # WP2
        (-35.362500, 149.165500, 10),  # WP3
    ]

    cmds.add(Command(  # TAKEOFF
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0,
        waypoints[0][0], waypoints[0][1], waypoints[0][2]
    ))

    for wp in waypoints:
        cmds.add(Command(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,
            wp[0], wp[1], wp[2]
        ))

    cmds.add(Command(  # RTL
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0,
        0, 0, 0
    ))

    cmds.upload()
    print("Mission uploaded.")

def arm_and_takeoff(aTargetAltitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    print("Vehicle armed succefully")
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# --- Run Mission ---
clear_mission()
add_mission()

arm_and_takeoff(10)

print("Starting mission")
vehicle.mode = VehicleMode("AUTO")

# Monitor until RTL or mission end
while True:
    print(" Mode:", vehicle.mode.name)
    if vehicle.mode.name == "RTL":
        print("Returning to Launch...")
        break
    time.sleep(2)

# Close vehicle
vehicle.close()
