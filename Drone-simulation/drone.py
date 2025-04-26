# from dronekit import connect, VehicleMode
# import time

# # Connect to the vehicle on MAVProxy's output port
# print("Connecting to vehicle on: udp:127.0.0.1:14550")
# vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# # Wait for vehicle to become armable
# print("Checking if vehicle is armable...")
# while not vehicle.is_armable:
#     print(" Waiting for vehicle to initialise...")
#     time.sleep(1)

# # Set mode to GUIDED
# print("Setting mode to GUIDED...")
# vehicle.mode = VehicleMode("GUIDED")
# while not vehicle.mode.name == "GUIDED":
#     print(" Waiting for mode change...")
#     time.sleep(1)

# # Arm the vehicle
# print("Arming the drone...")
# vehicle.armed = True
# while not vehicle.armed:
#     print(" Waiting for arming...")
#     time.sleep(1)

# # Take off to 3 meters
# takeoff_alt = 100
# print("Taking off!")
# vehicle.simple_takeoff(takeoff_alt)

# # Wait until it reaches a safe height
# while True:
#     print(" Altitude: ", vehicle.location.global_relative_frame.alt)
#     if vehicle.location.global_relative_frame.alt >= takeoff_alt * 0.95:
#         print("Reached target altitude!")
#         break
#     time.sleep(1)

# # Close the connection
# print("Closing vehicle connection")
# vehicle.close()



import time
import math
import threading
from pymavlink import mavutil # For MAVLink commands
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException

# --- Requires Installation: pip install keyboard ---
import keyboard

# --- Global Variables ---
vehicle = None
running = True # Flag to control background threads
takeoff_altitude = 15.0
default_speed = 2.0 # m/s for horizontal movement
altitude_change_speed = 0.5 # m/s for vertical movement
yaw_rate_deg_s = 30 # degrees per second for yaw

# --- DroneKit Connection ---
keyboard_thread = None
def connect_vehicle():
    """ Connects to the vehicle using DroneKit. """
    global vehicle 
    # connection_string = 'tcp:127.0.0.1:5762' # SITL
    # connection_string = '/dev/ttyACM0' # Example for Serial connection (Linux) - CHANGE AS NEEDED
    connection_string = 'udp:127.0.0.1:14552' # Example for Serial connection (Windows) - CHANGE TO YOUR PORT # Example for Serial connection (Linux) - CHANGE AS NEEDED
    # connection_string = 'com3' # Example for Serial connection (Windows) - CHANGE AS NEEDED
    print(f"Attempting to connect to vehicle on: {connection_string}")

    while running and vehicle is None:
        try:
            print(f"Connecting to vehicle on: {connection_string}")
            vehicle = connect(connection_string, wait_ready=True, timeout=60, heartbeat_timeout=30)
            print("Vehicle connected successfully.")
            # Display basic vehicle info
            print(f" Firmware: {vehicle.version}")
            print(f" Global Location: {vehicle.location.global_relative_frame}")
            print(f" Mode: {vehicle.mode.name}")
            print(f" Armed: {vehicle.armed}")

            # Add listener for mode changes (optional but informative)
            @vehicle.on_attribute('mode')
            def mode_callback(self, attr_name, value):
                print(f">> Mode changed to: {value.name}")

            @vehicle.on_attribute('armed')
            def armed_callback(self, attr_name, value):
                print(f">> Armed status changed to: {value}")

            break # Exit loop on successful connection
        except APIException as api_error:
            print(f"DroneKit API Error connecting to vehicle: {api_error}")
            vehicle = None
        except Exception as e:
            print(f"Error connecting to vehicle: {e}")
            vehicle = None

        if vehicle is None and running:
            print("Connection failed, retrying in 5 seconds...")
            time.sleep(5)

    if not running:
        print("Connection attempt aborted.")

# --- Drone Control Functions ---

def arm_and_takeoff(target_altitude):
    """ Arms vehicle and fly to target_altitude. """
    if not vehicle:
        print("Vehicle not connected.")
        return
    if vehicle.armed:
        print("Vehicle already armed.")
        # If already armed, check if flying, if not take off again? Or just proceed?
        # For now, just print and allow takeoff if already armed but on ground
        if vehicle.location.global_relative_frame.alt > 1.0:
             print("Vehicle appears to be flying already.")
             return

    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {target_altitude}m!")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height
    while running:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.2f}m")
        if alt >= target_altitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)

def land_vehicle():
    """ Sets vehicle mode to LAND. """
    if not vehicle:
        print("Vehicle not connected.")
        return
    print("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")
    # Wait for disarm? Monitor altitude?
    while vehicle.armed:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Landing... Altitude: {alt:.2f}m")
        if alt < 0.3: # Check if close to ground
             # Arducopter might take a moment to disarm after touching down
             print("Landed (or close to ground), waiting for disarm...")
        time.sleep(1)
    print("Vehicle landed and disarmed.")


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    Positive X is North, Positive Y is East, Positive Z is Down.
    """
    if not vehicle:
        print("Vehicle not connected.")
        return
    if vehicle.mode.name != "GUIDED":
        print("Vehicle must be in GUIDED mode to accept velocity commands.")
        # Optionally try to set GUIDED mode here
        # vehicle.mode = VehicleMode("GUIDED")
        # time.sleep(0.5)
        # if vehicle.mode.name != "GUIDED":
        #    print("Failed to set GUIDED mode.")
        #    return
        return

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # Send command to vehicle for duration
    for _ in range(0, int(duration)): # Send for a few seconds
        vehicle.send_mavlink(msg)
        time.sleep(1)
    # Send a zero velocity message to stop
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(stop_msg)
    print(f"Sent velocity command: N:{velocity_x:.1f}, E:{velocity_y:.1f}, D:{velocity_z:.1f} for {duration}s, then stopped.")


def condition_yaw(heading_deg, relative=False, direction=1, rate_deg_s=None):
    """
    Send MAV_CMD_CONDITION_YAW message to control yaw.
    heading_deg: target angle in degrees (0-360 absolute, or +/- relative)
    relative: True for relative angle change, False for absolute heading
    direction: -1 for CCW (Left), 1 for CW (Right) - only matters for relative turns
    rate_deg_s: speed of turn in degrees/second (None to use default/max)
    """
    if not vehicle:
        print("Vehicle not connected.")
        return
    if vehicle.mode.name != "GUIDED":
        print("Vehicle must be in GUIDED mode to accept yaw commands.")
        return

    is_relative = 1 if relative else 0
    yaw_speed = rate_deg_s if rate_deg_s is not None else 0 # 0 lets autopilot control speed

    print(f"Sending YAW command: Angle={heading_deg}, Relative={relative}, Dir={direction}, Rate={yaw_speed}")

    msg = vehicle.message_factory.command_long_encode(
        0, 0,       # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,          # confirmation
        heading_deg, # param 1: target angle (absolute or relative)
        yaw_speed,  # param 2: yaw speed deg/s (0 for AP default)
        direction,  # param 3: direction (-1 CCW, 1 CW)
        is_relative,# param 4: 1 for relative, 0 for absolute
        0, 0, 0)    # param 5 ~ 7 not used

    vehicle.send_mavlink(msg)


# --- Keyboard Control Loop ---
def keyboard_control_loop():

    """ Listens for keyboard input and controls the drone. """
    global running
    print("\n--- Keyboard Control Enabled ---")
    print(" t: Takeoff to {:.1f}m".format(takeoff_altitude))
    print(" l: Land")
    print(" w: Move Forward")
    print(" s: Move Backward")
    print(" a: Move Left")
    print(" d: Move Right")
    print(" UP_ARROW: Move Up")
    print(" DOWN_ARROW: Move Down")
    print(" LEFT_ARROW: Yaw Left")
    print(" RIGHT_ARROW: Yaw Right")
    print(" g: Set GUIDED mode (needed for movement)")
    print(" q: Quit")
    print("---------------------------------")

    move_duration = 1 # How long to apply velocity for each key press (seconds)

    while running:
        try:
            # Use keyboard.read_key() for blocking read or setup hooks for non-blocking
            # Using is_pressed for a polling approach here for simplicity
            # Note: This will consume CPU. Hooks are better for non-blocking.

            if keyboard.is_pressed('t'):
                print("[T] Key pressed - Takeoff Initiated")
                arm_and_takeoff(takeoff_altitude)
                time.sleep(0.5) # Debounce

            elif keyboard.is_pressed('l'):
                print("[L] Key pressed - Land Initiated")
                land_vehicle()
                time.sleep(0.5) # Debounce

            elif keyboard.is_pressed('g'):
                if vehicle and vehicle.mode.name != "GUIDED":
                     print("[G] Key pressed - Setting GUIDED mode")
                     vehicle.mode = VehicleMode("GUIDED")
                elif vehicle:
                     print("[G] Key pressed - Already in GUIDED mode")
                else:
                     print("[G] Key pressed - Vehicle not connected")
                time.sleep(0.5) # Debounce

            # --- Movement --- Needs GUIDED mode
            elif keyboard.is_pressed('w'): # Forward (North)
                print("[W] Key pressed - Move Forward")
                send_ned_velocity(default_speed, 0, 0, move_duration)
                time.sleep(0.1) # Shorter debounce during movement likely needed

            elif keyboard.is_pressed('s'): # Backward (South)
                print("[S] Key pressed - Move Backward")
                send_ned_velocity(-default_speed, 0, 0, move_duration)
                time.sleep(0.1)

            elif keyboard.is_pressed('a'): # Left (West)
                print("[A] Key pressed - Move Left")
                send_ned_velocity(0, -default_speed, 0, move_duration)
                time.sleep(0.1)

            elif keyboard.is_pressed('d'): # Right (East)
                print("[D] Key pressed - Move Right")
                send_ned_velocity(0, default_speed, 0, move_duration)
                time.sleep(0.1)

            elif keyboard.is_pressed('up'): # Up (Decrease Z velocity - remember Z is down)
                print("[Up Arrow] Key pressed - Move Up")
                send_ned_velocity(0, 0, -altitude_change_speed, move_duration)
                time.sleep(0.1)

            elif keyboard.is_pressed('down'): # Down (Increase Z velocity)
                print("[Down Arrow] Key pressed - Move Down")
                send_ned_velocity(0, 0, altitude_change_speed, move_duration)
                time.sleep(0.1)

            # --- Yaw --- Needs GUIDED mode
            elif keyboard.is_pressed('left'): # Yaw Left (Counter-Clockwise)
                print("[Left Arrow] Key pressed - Yaw Left")
                condition_yaw(yaw_rate_deg_s * move_duration, relative=True, direction=-1)
                # Yaw is often rate controlled, so sending once might be enough, or send continuously while pressed
                time.sleep(0.2) # Debounce for yaw

            elif keyboard.is_pressed('right'): # Yaw Right (Clockwise)
                print("[Right Arrow] Key pressed - Yaw Right")
                condition_yaw(yaw_rate_deg_s * move_duration, relative=True, direction=1)
                time.sleep(0.2) # Debounce for yaw


            elif keyboard.is_pressed('q'):
                print("[Q] Key pressed - Quitting")
                running = False # Signal threads and loop to stop
                break

            # Prevent busy-waiting - sleep briefly if no key relevant key is pressed
            time.sleep(0.05)

        except Exception as e:
            print(f"Error in keyboard loop: {e}")
            # Consider adding a flag to break if too many errors occur rapidly
            time.sleep(1) # Pause longer after an error


# --- Main Execution ---
if __name__ == '__main__':
    connect_thread = threading.Thread(target=connect_vehicle, name='DroneConnectThread', daemon=True)
    connect_thread.start()

    # Wait for connection attempt
    connect_thread.join(timeout=10.0) # Give it time to try connecting
    if vehicle:
        # Start keyboard listener only if connection was successful
        keyboard_thread = threading.Thread(target=keyboard_control_loop, name='KeyboardThread', daemon=True)
        keyboard_thread.start()
        # Keep main thread alive while keyboard thread runs
        try:
            while running:
                # Main thread can do other periodic checks if needed
                if not keyboard_thread.is_alive():
                     print("Keyboard control thread stopped.")
                     running = False # Stop if keyboard thread exits
                time.sleep(1)
        except KeyboardInterrupt:
            print("Ctrl+C detected. Shutting down...")
            running = False # Signal threads to stop
    else:
        print("Failed to connect to vehicle after initial attempt. Exiting.")
        running = False # Ensure connect_thread exits if it's still retrying


    # --- Cleanup ---
    print("Initiating shutdown sequence...")
    # running = False # Ensure flag is set

    if keyboard_thread and keyboard_thread.is_alive():
         print("Waiting for keyboard thread to stop...")
         keyboard_thread.join(timeout=2.0)

    # Join connect thread if it's still alive (e.g., stuck in retry loop)
    if connect_thread.is_alive():
         print("Waiting for connection thread to stop...")
         connect_thread.join(timeout=3.0) # Allow some time to exit cleanly

    if vehicle:
        print("Closing vehicle connection...")
        try:
            # Attempt to land if still flying? Risky on uncontrolled exit.
            # if vehicle.armed and vehicle.location.global_relative_frame.alt > 2:
            #     print("Attempting to land vehicle before closing...")
            #     land_vehicle() # This might block, consider timeout
            vehicle.close()
            print("Vehicle connection closed.")
        except Exception as e:
            print(f"Exception while closing vehicle: {e}")

    print("Shutdown complete.")