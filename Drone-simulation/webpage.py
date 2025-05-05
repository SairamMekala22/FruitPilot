# IMPORTANT: Eventlet monkey patching must happen FIRST
import eventlet
eventlet.monkey_patch()

# Now import other modules
import time
import math
import threading
# import eventlet # Already imported and patched above
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit

# --- Flask App Setup ---
app = Flask(_name_)
# IMPORTANT: Change this secret key for production environments!
app.config['SECRET_KEY'] = 'your_very_secret_key_change_me_too!'
# Use eventlet as the async mode
socketio = SocketIO(app, async_mode='eventlet')

# --- Global Variables ---
vehicle = None
telemetry_data = {}
update_interval = 1.0
running = True # Flag to control background threads

# --- DroneKit Functions ---

def connect_vehicle():
    """ Connects to the vehicle using DroneKit. Runs in a separate thread. """
    global vehicle
    connection_string = 'tcp:127.0.0.1:5762' # Use the correct port identified earlier
    print(f"Attempting to connect to vehicle on: {connection_string}")

    while running and vehicle is None: # Keep trying while running and not connected
        try:
            print(f"Connecting to vehicle on: {connection_string}")
            vehicle = connect(connection_string, wait_ready=True, timeout=60, heartbeat_timeout=30)
            print("Vehicle connected successfully.")

            if not any(t.name == 'TelemetryThread' for t in threading.enumerate()):
                telemetry_thread = threading.Thread(target=telemetry_update_loop, name='TelemetryThread', daemon=True)
                telemetry_thread.start()

            break # Exit loop on successful connection
        except APIException as api_error:
            print(f"DroneKit API Error connecting to vehicle: {api_error}")
            vehicle = None # Ensure vehicle is None on failure
        except Exception as e:
            print(f"Error connecting to vehicle: {e}")
            vehicle = None # Ensure vehicle is None on failure

        if vehicle is None and running:
            print("Connection failed, retrying in 5 seconds...")
            time.sleep(5) # Wait before retrying

    if not running:
        print("Connection attempt aborted.")

def get_telemetry():
    """ Fetches telemetry data from the connected vehicle. """
    # Check if vehicle object exists and seems valid before accessing attributes
    if not vehicle:
         return {}
    # Basic check for essential attributes existence
    if not vehicle.location or not vehicle.location.global_relative_frame or \
       not vehicle.attitude or not vehicle.battery or not vehicle.gps_0 or \
       not vehicle.mode or not vehicle.system_status:
        print("Warning: Vehicle object present but some essential attributes are None.")
        return {}

    telemetry = {}
    try:
        loc = vehicle.location.global_relative_frame
        att = vehicle.attitude
        bat = vehicle.battery
        gps = vehicle.gps_0

        telemetry = {
            "latitude": loc.lat, "longitude": loc.lon, "altitude": loc.alt,
            "groundspeed": vehicle.groundspeed, "airspeed": vehicle.airspeed,
            "heading": vehicle.heading, "roll": math.degrees(att.roll),
            "pitch": math.degrees(att.pitch), "yaw": math.degrees(att.yaw),
            "battery_voltage": bat.voltage, "battery_current": bat.current,
            "battery_level": bat.level, "mode": vehicle.mode.name,
            "armed": vehicle.armed, "is_armable": vehicle.is_armable,
            "system_status": vehicle.system_status.state,
            "gps_fix": gps.fix_type, "gps_satellites": gps.satellites_visible,
        }
    except Exception as e:
        # Catch potential errors if attributes become invalid mid-access (e.g., during disconnect)
        print(f"Error accessing vehicle attributes for telemetry: {e}")
        return {} # Return empty on error

    return telemetry

def telemetry_update_loop():
    """ Periodically fetches telemetry and emits it via WebSocket. """
    global telemetry_data
    print("Telemetry loop started.")
    while running:
        # Use simplified check - rely on vehicle object existing
        if vehicle:
            try:
                current_telemetry = get_telemetry()
                if current_telemetry: # Only update if we got valid data
                    telemetry_data = current_telemetry
                    # print("Emitting telemetry update") # Debugging
                    socketio.emit('telemetry_update', telemetry_data)
                # else:
                    # print("Got empty telemetry data.") # Debugging
            except Exception as e:
                print(f"Error in telemetry loop getting/emitting data: {e}")
        else:
             # Vehicle object is None, connection likely lost or never established
             pass # connect_vehicle handles reconnection attempts

        eventlet.sleep(update_interval)
    print("Telemetry loop stopped.")


# --- Flask Routes ---

@app.route('/')
def index():
    """ Serves the main HTML page. """
    return render_template('index.html') # Assumes index.html is in 'templates' folder

@app.route('/command/arm', methods=['POST'])
def command_arm():
    """ Handles the ARM command from the web interface. """
    if vehicle:
        if not vehicle.armed:
            # Re-check armability right before arming
            if vehicle.is_armable:
                try:
                    print("Received ARM command from web")
                    vehicle.mode = VehicleMode("GUIDED")
                    time.sleep(0.5) # Short pause
                    vehicle.armed = True
                    start_time = time.time()
                    while not vehicle.armed:
                        if time.time() - start_time > 5:
                             print("Arming timed out.")
                             return jsonify({"status": "error", "message": "Arming timed out"}), 500
                        time.sleep(0.2)
                    print("Vehicle armed successfully via web command.")
                    return jsonify({"status": "success", "message": "Vehicle armed"})
                except Exception as e:
                    print(f"Error during arming: {e}")
                    return jsonify({"status": "error", "message": f"Arming failed: {e}"}), 500
            else:
                print("Arm command received, but vehicle not armable (check pre-arm checks).")
                return jsonify({"status": "error", "message": "Vehicle not armable (check pre-arm checks)"}), 400
        else:
             print("Arm command received, but vehicle already armed.")
             return jsonify({"status": "success", "message": "Vehicle already armed"})
    else:
        print("Arm command received, but vehicle not connected.")
        return jsonify({"status": "error", "message": "Vehicle not connected"}), 500

# --- MODIFIED: Renamed route and function ---
@app.route('/command/start_mission', methods=['POST'])
def command_start_mission():
    """ Handles the START MISSION command with the specified flight plan """
    if not vehicle:
        print("Start Mission command received, but vehicle not connected.")
        return jsonify({"status": "error", "message": "Vehicle not connected"}), 500

    if not vehicle.armed:
        print("Start Mission command received, but vehicle not armed.")
        return jsonify({"status": "error", "message": "Vehicle not armed"}), 400

    print("Received START MISSION command from web")

    # --- Mission Parameters ---
    takeoff_alt = 10.0
    first_leg_dist = 30.0
    turn_angle_deg = 30.0 # Degrees left
    second_leg_dist = 20.0
    final_alt = 15.0
    groundspeed = 5 # m/s, adjust as needed

    try:
        # 1. Set Mode to GUIDED
        if vehicle.mode.name != "GUIDED":
            print("Setting mode to GUIDED...")
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1) # Wait for mode change
            if vehicle.mode.name != "GUIDED":
                raise Exception("Failed to set GUIDED mode.")
        print("Mode set to GUIDED.")

        # 2. Takeoff to 10 meters
        print(f"Taking off to {takeoff_alt}m...")
        vehicle.simple_takeoff(takeoff_alt)
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            print(f"  Altitude: {current_altitude:.2f}m")
            if current_altitude >= takeoff_alt * 0.95:
                print("Reached target takeoff altitude.")
                break
            time.sleep(1)

        # 3. Fly straight 30 meters
        print(f"Flying forward {first_leg_dist}m...")
        start_location = vehicle.location.global_relative_frame
        current_heading_rad = math.radians(vehicle.heading)
        dNorth = math.cos(current_heading_rad) * first_leg_dist
        dEast = math.sin(current_heading_rad) * first_leg_dist
        target_location1 = get_location_metres(start_location, dNorth, dEast)
        # Maintain takeoff altitude for this leg
        target_location1.alt = takeoff_alt
        print(f"  Target Location 1: Lat={target_location1.lat}, Lon={target_location1.lon}, Alt={target_location1.alt}")
        vehicle.simple_goto(target_location1, groundspeed=groundspeed)

        # Wait to reach the first target
        while True:
            current_location = vehicle.location.global_relative_frame
            remaining_distance = get_distance_metres(current_location, target_location1)
            print(f"  Distance to target 1: {remaining_distance:.2f}m")
            if remaining_distance <= 2.0: # Allow some tolerance
                print("Reached target location 1.")
                break
            time.sleep(1)

        # 4. Turn left 30 degrees
        # 4. Turn left 30 degrees
        print(f"Turning left by {turn_angle_deg} degrees...")
        # --- REMOVE THIS LINE ---
        # vehicle.condition_yaw(turn_angle_deg, relative=True, direction=-1)

        # --- ADD THESE LINES ---
        # Import mavutil if you haven't already at the top of the file
        from pymavlink import mavutil # Add this import at the top

        # Create the MAV_CMD_CONDITION_YAW command
        msg = vehicle.message_factory.command_long_encode(
            0, 0,       # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0,          #confirmation
            turn_angle_deg, # param 1: target angle in degrees
            0,          # param 2: yaw speed (ignored by ArduPilot, use 0)
            -1,         # param 3: direction (-1: ccw, 1: cw)
            1,          # param 4: relative offset (1 for relative, 0 for absolute)
            0, 0, 0)    # param 5 ~ 7 not used
        # Send command to vehicle
        vehicle.send_mavlink(msg)
        # --- END OF ADDED LINES ---

        time.sleep(5) # Increase sleep time slightly to ensure turn completes
        print(f"  New heading: {vehicle.heading:.2f} degrees")


        # 5. Fly straight 20 meters and change altitude to 15 meters
        print(f"Flying forward {second_leg_dist}m to {final_alt}m altitude...")
        start_location2 = vehicle.location.global_relative_frame
        current_heading_rad = math.radians(vehicle.heading) # Use the new heading
        dNorth = math.cos(current_heading_rad) * second_leg_dist
        dEast = math.sin(current_heading_rad) * second_leg_dist

        # Calculate the location part first (altitude will be set directly)
        target_location2_intermediate = get_location_metres(start_location2, dNorth, dEast)

        # Create the final target location with the desired altitude
        target_location2 = LocationGlobalRelative(
            target_location2_intermediate.lat,
            target_location2_intermediate.lon,
            final_alt # Set the target altitude here
        )

        print(f"  Target Location 2: Lat={target_location2.lat}, Lon={target_location2.lon}, Alt={target_location2.alt}")
        vehicle.simple_goto(target_location2, groundspeed=groundspeed)

        # Wait to reach the final target location and altitude
        while True:
            current_location = vehicle.location.global_relative_frame
            remaining_distance = get_distance_metres(current_location, target_location2)
            altitude_diff = abs(current_location.alt - final_alt)
            print(f"  Distance to target 2: {remaining_distance:.2f}m, Altitude difference: {altitude_diff:.2f}m")
            # Check both distance and altitude are close
            if remaining_distance <= 2.0 and altitude_diff <= 1.0: # Allow tolerance
                print("Reached final target location and altitude.")
                break
            time.sleep(1)

        print("Mission completed successfully.")
        return jsonify({"status": "success", "message": "Mission completed successfully"})

    except APIException as api_error:
        print(f"DroneKit API Error during mission: {api_error}")
        return jsonify({"status": "error", "message": f"DroneKit API Error: {api_error}"}), 500
    except Exception as e:
        print(f"Error during mission execution: {e}")
        return jsonify({"status": "error", "message": f"Mission failed: {e}"}), 500

@app.route('/command/rtl', methods=['POST'])
def command_rtl():
    """ Handles the RTL command """
    if vehicle:
        try:
             print("Received RTL command from web")
             vehicle.mode = VehicleMode("RTL")
             return jsonify({"status": "success", "message": "RTL mode initiated"})
        except Exception as e:
             print(f"Error setting RTL: {e}")
             return jsonify({"status": "error", "message": f"Failed to set RTL: {e}"}), 500
    else:
         print("RTL command received, but vehicle not connected.")
         return jsonify({"status": "error", "message": "Vehicle not connected"}), 500


# --- WebSocket Events ---
@socketio.on('connect')
def handle_connect():
    """ Handles new WebSocket connections from web clients. """
    print('Web client connected:', request.sid)
    if telemetry_data: emit('telemetry_update', telemetry_data)

@socketio.on('disconnect')
def handle_disconnect():
    """ Handles WebSocket disconnections. """
    print('Web client disconnected:', request.sid)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobalRelative object containing the latitude/longitude
    'dNorth' and 'dEast' metres from the specified 'original_location'.
    The returned LocationGlobalRelative has the same altitude as the original location.

    Args:
        original_location: LocationGlobalRelative object
        dNorth: Distance north in meters
        dEast: Distance east in meters

    Returns:
        LocationGlobalRelative object
    """
    earth_radius = 6378137.0 # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py

    Args:
        aLocation1: LocationGlobal or LocationGlobalRelative object
        aLocation2: LocationGlobal or LocationGlobalRelative object

    Returns:
        Ground distance in meters
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5 # Distance in meters



# --- Main Execution ---
if _name_ == '_main_':
    connect_thread = threading.Thread(target=connect_vehicle, name='DroneConnectThread', daemon=True)
    connect_thread.start()

    print("Starting web server on http://127.0.0.1:5000")
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("Ctrl+C detected. Shutting down server...")
    finally:
        print("Stopping background threads...")
        running = False
        if vehicle:
            print("Closing vehicle connection (if open)...")
            try: vehicle.close()
            except Exception as e: print(f"Exception while closing vehicle: {e}")
            print("Vehicle connection closed.")
        # Wait briefly for threads to notice 'running' flag
        time.sleep(0.5)
        connect_thread.join(timeout=2.0)
        for t in threading.enumerate():
            if t.name == 'TelemetryThread':
                 t.join(timeout=2.0)
                 break
        print("Server stopped.")