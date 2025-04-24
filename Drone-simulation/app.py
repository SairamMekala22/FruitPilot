from flask import Flask, jsonify
from dronekit import connect, VehicleMode, APIException
import time

app = Flask(__name__)
vehicle = None  # Initialize globally

# Change this to the connection string you're using
# For UDP:
connection_string = 'udp:127.0.0.1:14551'
# For TCP (e.g., if you forwarded MAVProxy to tcp:127.0.0.1:5765):
# connection_string = 'tcp:127.0.0.1:5765'

def safe_connect():
    """Tries to connect to the vehicle, retrying up to 5 times."""
    for _ in range(5):
        try:
            print(f"Connecting to vehicle on {connection_string}")
            vehicle = connect(connection_string, wait_ready=True, timeout=60)
            print("Connection successful.")
            return vehicle
        except APIException as e:
            print(f"Connection failed: {e}. Retrying...")
            time.sleep(5)
    return None

@app.route('/status')
def get_status():
    global vehicle
    if not vehicle:
        vehicle = safe_connect()
        if not vehicle:
            return jsonify({'error': 'Failed to connect to vehicle'}), 500

    try:
        return jsonify({
            'altitude': vehicle.location.global_relative_frame.alt,
            'latitude': vehicle.location.global_frame.lat,
            'longitude': vehicle.location.global_frame.lon,
            'battery': vehicle.battery.level,
            'mode': str(vehicle.mode.name),
            'armed': vehicle.armed
        })
    except Exception as e:
        return jsonify({'error': f'Failed to get vehicle status: {e}'}), 500

@app.route('/')
def home():
    return "<h2>Drone Status API</h2><p>Use <code>/status</code> to get current telemetry.</p>"

if __name__ == '__main__':
    # Try connecting when the server starts
    vehicle = safe_connect()
    if not vehicle:
        print("WARNING: Could not connect at startup. Will retry on first request.")

    # Run the Flask app with threading enabled
    app.run(debug=False, threaded=True)
