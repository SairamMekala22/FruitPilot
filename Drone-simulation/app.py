from flask import Flask, jsonify
from dronekit import connect, VehicleMode, APIException
import time

app = Flask(__name__)

def safe_connect():
    for _ in range(5):  # retry 5 times
        try:
            vehicle = connect('127.0.0.1:14550', wait_ready=True, timeout=60)
            return vehicle
        except APIException as e:
            print(f"Connection failed: {e}. Retrying...")
            time.sleep(5)
    return None

vehicle = safe_connect()
if not vehicle:
    raise Exception("Could not connect to vehicle after several attempts.")

@app.route('/status')
def get_status():
    return jsonify({
        'altitude': vehicle.location.global_relative_frame.alt,
        'latitude': vehicle.location.global_frame.lat,
        'longitude': vehicle.location.global_frame.lon,
        'battery': vehicle.battery.level,
        'mode': str(vehicle.mode.name),
        'armed': vehicle.armed
    })

if __name__ == '__main__':
    app.run(debug=True)
