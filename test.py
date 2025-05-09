# IMPORTANT: Eventlet monkey patching must happen FIRST
import eventlet
eventlet.monkey_patch()

# Now import other modules
import time
import math
import threading
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO, emit
from pymavlink import mavutil

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
running = True  # Flag to control background threads

# --- Computer Vision Variables ---
cap = None  # Webcam capture object
frame = None  # Current frame
mango_detected = False
mango_position = (0, 0)  # (x, y) center of detected mango
frame_center = (0, 0)  # Will be calculated when webcam starts
mango_tracking_active = False
cv_lock = threading.Lock()  # Lock for thread-safe access to video frames

# --- YOLO Model Loading ---
try:
    # For YOLOv8, we need to use ultralytics package instead of torch.hub
    from ultralytics import YOLO
    
    # Load the YOLOv8 model
    model = YOLO('mango_yolov8.pt')  # Load a custom model
    
    print("YOLOv8 model loaded successfully.")

except ImportError:
    print("Error: PyTorch not found. Please install it (pip install torch torchvision torchaudio).")
    model = None
except Exception as e:
    print(f"Error loading YOLO model: {e}")
    model = None
# --- Computer Vision Functions ---

def initialize_webcam():
    """Initialize the webcam capture."""
    global cap, frame_center
    try:
        cap = cv2.VideoCapture(0)  # Use 0 for default webcam
        if not cap.isOpened():
            print("Error: Could not open webcam.")
            return False
        
        # Get webcam frame dimensions
        ret, test_frame = cap.read()
        if ret:
            height, width = test_frame.shape[:2]
            frame_center = (width // 2, height // 2)
            print(f"Webcam initialized. Frame size: {width}x{height}, Center: {frame_center}")
            return True
        else:
            print("Error: Failed to read frame from webcam.")
            return False
    except Exception as e:
        print(f"Error initializing webcam: {e}")
        return False

def detect_mango(image):
    """
    Detect mangoes in the image using the YOLOv8 model.
    Returns (success, center_x, center_y, width, height) of the first detected mango.
    """
    global model, frame_center
    if model is None:
        print("YOLO model not loaded.")
        return False, 0, 0, 0, 0

    try:
        # Perform inference with YOLOv8
        results = model(image, verbose=False)  # Suppress extra output
        
        # YOLOv8 returns a list of Results objects - get the first one
        result = results[0] if results else None
        
        if result is None:
            return False, 0, 0, 0, 0
        
        # Get all detections
        boxes = result.boxes
        
        # Check if any detections exist
        if len(boxes) == 0:
            return False, 0, 0, 0, 0
            
        # Find mango detections - YOLOv8 uses names in model.names
        mango_detections = []
        for i, box in enumerate(boxes):
            # Get class id
            cls_id = int(box.cls.item()) if hasattr(box, 'cls') else -1
            # Check if this class is a mango
            if cls_id >= 0 and cls_id < len(model.names) and 'mango' in model.names[cls_id].lower():
                mango_detections.append(box)
            
        # If no mangoes found
        if not mango_detections:
            return False, 0, 0, 0, 0
            
        # Get first mango detection
        mango_box = mango_detections[0]
        
        # Extract bounding box
        x1, y1, x2, y2 = map(int, mango_box.xyxy[0].tolist())
        
        # Calculate center and dimensions
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        width = x2 - x1
        height = y2 - y1
        
        return True, center_x, center_y, width, height

    except Exception as e:
        print(f"Error during YOLO detection: {e}")
        return False, 0, 0, 0, 0

def video_processing_loop():
    """Process video frames and detect mango position."""
    global frame, mango_detected, mango_position, mango_tracking_active
    
    print("Video processing loop started.")
    while running:
        if cap and cap.isOpened() and mango_tracking_active:
            # Capture frame
            ret, current_frame = cap.read()
            if ret:
                # Process frame for mango detection
                detected, x, y, w, h = detect_mango(current_frame)
                
                with cv_lock:
                    if detected:
                        mango_detected = True
                        mango_position = (x, y)
                        
                        # Draw rectangle around mango
                        cv2.rectangle(current_frame, (x - w//2, y - h//2), (x + w//2, y + h//2), (0, 255, 0), 2)
                        # Draw circle at center of mango
                        cv2.circle(current_frame, (x, y), 5, (0, 0, 255), -1)
                        # Draw crosshair at center of frame
                        cv2.line(current_frame, (frame_center[0] - 20, frame_center[1]), 
                                  (frame_center[0] + 20, frame_center[1]), (255, 0, 0), 2)
                        cv2.line(current_frame, (frame_center[0], frame_center[1] - 20), 
                                  (frame_center[0], frame_center[1] + 20), (255, 0, 0), 2)
                    else:
                        mango_detected = False
                    
                    # Store processed frame
                    frame = current_frame
                    
                    # Emit mango position via websocket
                    socketio.emit('mango_update', {
                        'detected': mango_detected,
                        'x': x if detected else 0,
                        'y': y if detected else 0,
                        'frame_width': frame.shape[1] if frame is not None else 0,
                        'frame_height': frame.shape[0] if frame is not None else 0
                    })
        
        eventlet.sleep(0.1)  # Process at approximately 10fps
    
    print("Video processing loop stopped.")

def generate_frames():
    """Generate frames for the video feed."""
    global frame
    while True:
        with cv_lock:
            if frame is not None:
                # Encode the frame
                success, encoded_frame = cv2.imencode('.jpg', frame)
                if success:
                    # Convert to bytes and yield
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + encoded_frame.tobytes() + b'\r\n')
        
        eventlet.sleep(0.1)

def mango_tracking_loop():
    """Control drone to keep mango centered in frame."""
    global mango_detected, mango_position, frame_center, mango_tracking_active
    
    print("Mango tracking loop started.")
    while running:
        if vehicle and vehicle.armed and mango_tracking_active and mango_detected:
            try:
                # Calculate offset from center
                x_offset = mango_position[0] - frame_center[0]
                y_offset = mango_position[1] - frame_center[1]
                
                # Define thresholds for movement (dead zone)
                x_threshold = frame_center[0] * 0.1  # 10% of width
                y_threshold = frame_center[1] * 0.1  # 10% of height
                
                # Only move if offset exceeds threshold
                if abs(x_offset) > x_threshold or abs(y_offset) > y_threshold:
                    # Convert pixel offset to velocity commands
                    # Adjust these scaling factors as needed for your drone
                    vx = -y_offset * 0.01  # Forward/backward (inverted because y increases downward)
                    vy = -x_offset * 0.01  # Left/right (negative = move right)
                    vz = 0  # Maintain altitude
                    
                    # Cap velocities
                    vx = max(min(vx, 1.0), -1.0)
                    vy = max(min(vy, 1.0), -1.0)
                    
                    print(f"Sending velocity command: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")
                    
                    # Send velocity command to vehicle
                    msg = vehicle.message_factory.set_position_target_local_ned_encode(
                        0,      # time_boot_ms (not used)
                        0, 0,    # target system, target component
                        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # relative to drone orientation
                        0b0000111111000111, # type_mask (only speeds enabled)
                        0, 0, 0,  # x, y, z positions (not used)a
                        vx, vy, vz,  # x, y, z velocity in m/s
                        0, 0, 0,  # x, y, z acceleration (not used)
                        0, 0)    # yaw, yaw_rate (not used)
                    
                    vehicle.send_mavlink(msg)
            except Exception as e:
                print(f"Error in mango tracking control: {e}")
        
        eventlet.sleep(0.2)  # Control at 5Hz
    
    print("Mango tracking loop stopped.")

# --- DroneKit Functions ---

def connect_vehicle():
    """ Connects to the vehicle using DroneKit. Runs in a separate thread. """
    global vehicle
    connection_string = 'tcp:127.0.0.1:5762'  # Use the correct port identified earlier
    print(f"Attempting to connect to vehicle on: {connection_string}")

    while running and vehicle is None:  # Keep trying while running and not connected
        try:
            print(f"Connecting to vehicle on: {connection_string}")
            vehicle = connect(connection_string, wait_ready=True, timeout=60, heartbeat_timeout=30)
            print("Vehicle connected successfully.")

            if not any(t.name == 'TelemetryThread' for t in threading.enumerate()):
                telemetry_thread = threading.Thread(target=telemetry_update_loop, name='TelemetryThread', daemon=True)
                telemetry_thread.start()

            break  # Exit loop on successful connection
        except APIException as api_error:
            print(f"DroneKit API Error connecting to vehicle: {api_error}")
            vehicle = None  # Ensure vehicle is None on failure
        except Exception as e:
            print(f"Error connecting to vehicle: {e}")
            vehicle = None  # Ensure vehicle is None on failure

        if vehicle is None and running:
            print("Connection failed, retrying in 5 seconds...")
            time.sleep(5)  # Wait before retrying

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
        return {}  # Return empty on error

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
                if current_telemetry:  # Only update if we got valid data
                    telemetry_data = current_telemetry
                    socketio.emit('telemetry_update', telemetry_data)
            except Exception as e:
                print(f"Error in telemetry loop getting/emitting data: {e}")
        else:
            # Vehicle object is None, connection likely lost or never established
            pass  # connect_vehicle handles reconnection attempts

        eventlet.sleep(update_interval)
    print("Telemetry loop stopped.")


# --- Flask Routes ---

@app.route('/')
def index():
    """ Serves the main HTML page. """
    return render_template('index1.html')  # Assumes index.html is in 'templates' folder

@app.route('/video_feed')
def video_feed():
    """ Route for streaming video feed """
    return Response(generate_frames(), 
                    mimetype='multipart/x-mixed-replace; boundary=frame')

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
                    time.sleep(0.5)  # Short pause
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

@app.route('/command/takeoff', methods=['POST'])
def command_takeoff():
    """ Handles the TAKEOFF command """
    if not vehicle:
        print("Takeoff command received, but vehicle not connected.")
        return jsonify({"status": "error", "message": "Vehicle not connected"}), 500

    if not vehicle.armed:
        print("Takeoff command received, but vehicle not armed.")
        return jsonify({"status": "error", "message": "Vehicle not armed"}), 400

    try:
        # Get altitude from request
        data = request.get_json()
        altitude = float(data.get('altitude', 5.0))  # Default to 5m if not specified
        
        print(f"Received TAKEOFF command from web to {altitude}m")
        
        # Set mode to GUIDED
        if vehicle.mode.name != "GUIDED":
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)  # Wait for mode change
        
        # Issue takeoff command
        vehicle.simple_takeoff(altitude)
        
        # Return success immediately (don't wait for altitude)
        return jsonify({"status": "success", "message": f"Takeoff command issued to {altitude}m"})
    
    except Exception as e:
        print(f"Error during takeoff: {e}")
        return jsonify({"status": "error", "message": f"Takeoff failed: {e}"}), 500

@app.route('/command/start_tracking', methods=['POST'])
def command_start_tracking():
    """ Starts mango tracking mission """
    global mango_tracking_active
    
    if not vehicle:
        print("Start tracking command received, but vehicle not connected.")
        return jsonify({"status": "error", "message": "Vehicle not connected"}), 500

    if not vehicle.armed:
        print("Start tracking command received, but vehicle not armed.")
        return jsonify({"status": "error", "message": "Vehicle not armed"}), 400
        
    if not cap or not cap.isOpened():
        print("Start tracking command received, but webcam not available.")
        return jsonify({"status": "error", "message": "Webcam not available"}), 500

    try:
        print("Received START TRACKING command from web")
        
        # Set mode to GUIDED
        if vehicle.mode.name != "GUIDED":
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)  # Wait for mode change
        
        # Activate tracking
        mango_tracking_active = True
        
        return jsonify({"status": "success", "message": "Mango tracking activated"})
    
    except Exception as e:
        print(f"Error starting tracking: {e}")
        return jsonify({"status": "error", "message": f"Failed to start tracking: {e}"}), 500

@app.route('/command/stop_tracking', methods=['POST'])
def command_stop_tracking():
    """ Stops mango tracking mission """
    global mango_tracking_active
    
    try:
        print("Received STOP TRACKING command from web")
        
        # Deactivate tracking
        mango_tracking_active = False
        
        # Send zero velocity command to stop movement
        if vehicle and vehicle.armed:
            msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0, 0, 0,    # time_boot_ms, target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # relative to drone orientation
                0b0000111111000111, # type_mask (only speeds enabled)
                0, 0, 0,    # x, y, z positions (not used)
                0, 0, 0,    # x, y, z velocity in m/s (stop movement)
                0, 0, 0,    # x, y, z acceleration (not used)
                0, 0)      # yaw, yaw_rate (not used)
            
            vehicle.send_mavlink(msg)
        
        return jsonify({"status": "success", "message": "Mango tracking deactivated"})
    
    except Exception as e:
        print(f"Error stopping tracking: {e}")
        return jsonify({"status": "error", "message": f"Failed to stop tracking: {e}"}), 500

@app.route('/command/rtl', methods=['POST'])
def command_rtl():
    """ Handles the RTL command """
    if vehicle:
        try:
            print("Received RTL command from web")
            # First stop tracking if active
            global mango_tracking_active
            mango_tracking_active = False
            # Then return to launch
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
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
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
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5  # Distance in meters


# --- Main Execution ---
if _name_ == '_main_':
    # Initialize webcam
    if initialize_webcam():
        # Start video processing thread
        video_thread = threading.Thread(target=video_processing_loop, name='VideoThread', daemon=True)
        video_thread.start()
        
        # Start mango tracking thread
        tracking_thread = threading.Thread(target=mango_tracking_loop, name='TrackingThread', daemon=True)
        tracking_thread.start()
    else:
        print("Warning: Webcam initialization failed. Mango tracking will not be available.")
    
    # Start drone connection thread
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
        
        # Close webcam if open
        if cap:
            cap.release()
            print("Webcam released.")
        
        if vehicle:
            print("Closing vehicle connection (if open)...")
            try:
                vehicle.close()
            except Exception as e:
                print(f"Exception while closing vehicle: {e}")
            print("Vehicle connection closed.")
        
        # Wait briefly for threads to notice 'running' flag
        time.sleep(0.5)
        connect_thread.join(timeout=2.0)
        for t in threading.enumerate():
            if t.name in ['TelemetryThread', 'VideoThread', 'TrackingThread']:
                t.join(timeout=2.0)
        
        print("Server stopped.")