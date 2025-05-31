from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import cv2
import threading
import time
from pymavlink import mavutil
# from pynput import keyboard
import torch
import numpy as np
from ultralytics import YOLO

# ========================
# 1. Connect to the Vehicle
# ========================
print("Connecting to vehicle...")
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)  # Replace with your laptop's IP and port
print("Connected to vehicle.")

# ========================
# 2. Load Custom YOLO Model
# ========================
MODEL_PATH = 'old_150.pt'  # Path to your custom YOLO model
model = YOLO(MODEL_PATH)
model.fuse()

# ========================
# 3. Webcam Thread with Inference and Recording
# ========================
recording = False
recording_infer = False
out = None
out_infer = None

VIDEO_WIDTH = 1920  # Set for 4K: 3840x2160 or 1920x1080 for Full HD
VIDEO_HEIGHT = 1080

def show_webcam():
    global recording, recording_infer, out, out_infer
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)

    if not cap.isOpened():
        print("Failed to open webcam.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Run YOLO inference
        results = model(frame, verbose=False)
        annotated_frame = results[0].plot()

        cv2.imshow("Webcam Feed", frame)
        cv2.imshow("YOLO Inference", annotated_frame)

        if recording and out is not None:
            out.write(frame)

        if recording_infer and out_infer is not None:
            out_infer.write(annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    if out:
        out.release()
    if out_infer:
        out_infer.release()
    cv2.destroyAllWindows()

# Start webcam in a separate thread
threading.Thread(target=show_webcam, daemon=True).start()

# ========================
# 4. Drone Commands
# ========================
def arm_and_takeoff(aTargetAltitude):
    print("Arming motors...")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def move_relative(dx=0, dy=0, dz=0):
    current_location = vehicle.location.global_relative_frame
    new_location = LocationGlobalRelative(
        current_location.lat + dx * 0.000001,
        current_location.lon + dy * 0.000001,
        current_location.alt + dz
    )
    vehicle.simple_goto(new_location, groundspeed=0.25)
    print(f"Moving to: {new_location}")

def send_yaw_velocity(yaw_rate):
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        0,
        0,
        0b00000100,
        [0, 0, 0, 1],
        0, 0, 0,
        yaw_rate
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# ========================
# 5. Arrow Key Control
# ========================
def arrow_key_control():
    print("Use arrow keys to move. Press ESC to exit move control.")
    step = 1
    yaw_rate = 0.02

    def on_press(key):
        try:
            if key == keyboard.Key.esc:
                print("Exiting move control mode.")
                return False
            elif key == keyboard.Key.up:
                move_relative(dx=step)
            elif key == keyboard.Key.down:
                move_relative(dx=-step)
            elif key == keyboard.Key.left:
                print("Yaw left")
                send_yaw_velocity(-yaw_rate)
            elif key == keyboard.Key.right:
                print("Yaw right")
                send_yaw_velocity(yaw_rate)
            elif key.char == 'u':
                move_relative(dz=-step)
            elif key.char == 'j':
                move_relative(dz=step)
        except AttributeError:
            pass

    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

# ========================
# 6. CLI
# ========================
print("\nAvailable commands:")
print("  t - takeoff")
print("  m - move with arrow keys")
print("  r - start/stop normal recording")
print("  i - start/stop inference recording")
print("  l - land")
print("  e - exit\n")

while True:
    cmd = input(">>> ").strip().lower()
    if cmd == "t":
        try:
            alt = float(input("Enter altitude to take off (in meters): "))
            arm_and_takeoff(alt)
        except:
            print("Invalid altitude value.")
    elif cmd == "m":
        arrow_key_control()
    elif cmd == "r":
        if not recording:
            print("Starting normal video recording...")
            out = cv2.VideoWriter('drone_feed.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 20.0, (VIDEO_WIDTH, VIDEO_HEIGHT))
            recording = True
        else:
            print("Stopping normal video recording...")
            recording = False
            if out:
                out.release()
                out = None
    elif cmd == "i":
        if not recording_infer:
            print("Starting inference video recording...")
            out_infer = cv2.VideoWriter('drone_infer.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 20.0, (VIDEO_WIDTH, VIDEO_HEIGHT))
            recording_infer = True
        else:
            print("Stopping inference video recording...")
            recording_infer = False
            if out_infer:
                out_infer.release()
                out_infer = None
    elif cmd == "l":
        print("Landing...")
        vehicle.mode = VehicleMode("LAND")
    elif cmd == "e":
        print("Exiting...")
        vehicle.close()
        break
    else:
        print("Unknown command. Use 't', 'm', 'r', 'i', 'l', or 'e'.")
