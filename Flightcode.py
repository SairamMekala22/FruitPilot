# OBJECT DETECTION + DRONEKIT INTEGRATION WITH AUTO YAW SEARCH

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
import cv2
from ultralytics import YOLO
import math
from pymavlink import mavutil

# === CONFIGURATION ===
MODEL_PATH = 'kaggle100.pt'
REAL_FRUIT_WIDTH_CM = 8.0
FOCAL_LENGTH_MM = 3.6
SENSOR_WIDTH_MM = 4.8
IMAGE_WIDTH_PX = 640
IMAGE_HEIGHT_PX = 480
YAW_INCREMENT_DEG = 30

# === CONNECT TO DRONE ===
print("Connecting to drone...")
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

# === LOAD OBJECT DETECTION MODEL ===
print("Loading model...")
model = YOLO(MODEL_PATH)
model.fuse()

# === DISTANCE ESTIMATION ===
def estimate_distance(focal_length_mm, real_width_cm, bbox_width_px, image_width_px, sensor_width_mm):
    if bbox_width_px == 0:
        return float('inf')
    focal_px = (focal_length_mm / sensor_width_mm) * image_width_px
    return (real_width_cm * focal_px) / bbox_width_px

# === TAKEOFF ===
def arm_and_takeoff(altitude):
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {altitude}m")
    vehicle.simple_takeoff(altitude)
    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# === YAW ROTATION FUNCTION ===
def condition_yaw(heading, relative=False):
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        0,
        1,
        is_relative,
        0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

# === OBJECT DETECTION + DRONE INTERACTION ===
def detect_and_hover():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH_PX)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT_PX)
    detected = False
    yaw_angle = 0

    while not detected:
        ret, frame = cap.read()
        if not ret:
            print("Camera error.")
            break

        results = model.predict(source=frame, conf=0.5, verbose=False)[0]
        boxes = results.boxes

        if boxes is None or len(boxes.xyxy) == 0:
            print("No mango detected. Rotating...")
            cv2.imshow("Feed", frame)
            cv2.waitKey(1)
            yaw_angle += YAW_INCREMENT_DEG
            condition_yaw(YAW_INCREMENT_DEG, relative=True)
            time.sleep(3)
            continue

        bboxes = boxes.xyxy.cpu()
        confs = boxes.conf.cpu()
        clses = boxes.cls.cpu()

        for i, box in enumerate(bboxes):
            x1, y1, x2, y2 = box.int().tolist()
            bbox_width = x2 - x1
            conf = confs[i].item()
            dist_cm = estimate_distance(FOCAL_LENGTH_MM, REAL_FRUIT_WIDTH_CM, bbox_width, IMAGE_WIDTH_PX, SENSOR_WIDTH_MM)

            label = f"Conf: {conf:.2f} | Dist: {dist_cm:.1f}cm"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if conf > 0.6:
                print(f"Fruit detected at ~{dist_cm:.1f}cm")
                print("Switching to LOITER mode to hover.")
                vehicle.mode = VehicleMode("LOITER")
                time.sleep(5)
                detected = True
                break

        cv2.imshow("Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# === EXECUTE FULL FLOW ===
arm_and_takeoff(10)
detect_and_hover()
print("Returning to Launch (RTL)...")
vehicle.mode = VehicleMode("RTL")
vehicle.close()
print("Mission complete.")
