# OBJECT DETECTION + DRONEKIT INTEGRATION (UPDATED)
# Uses your custom YOLO model (kaggle100.pt) + distance estimation

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import cv2
from ultralytics import YOLO
import math

# === CONFIGURATION ===
MODEL_PATH = 'kaggle100.pt'
REAL_FRUIT_WIDTH_CM = 8.0
FOCAL_LENGTH_MM = 3.6
SENSOR_WIDTH_MM = 4.8
# IMAGE_WIDTH_PX = 640
# IMAGE_HEIGHT_PX = 480
IMAGE_WIDTH_PX = 3840
IMAGE_HEIGHT_PX = 2160

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

# === OBJECT DETECTION + YAW SWEEPING ===
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
            print("No objects detected. Rotating to next frame...")
            yaw_angle += 30  # rotate 30 degrees per scan
            condition_yaw(yaw_angle % 360)
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

# === ROTATE YAW FUNCTION ===
def condition_yaw(heading, relative=False):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        115,     # command: MAV_CMD_CONDITION_YAW
        0,       # confirmation
        heading, # param 1: target angle
        20,      # param 2: speed (deg/s)
        1 if relative else 0,  # param 3: direction (1 = clockwise)
        0, 0, 0, 0             # param 4-7: unused
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# === EXECUTE FULL FLOW ===
arm_and_takeoff(10)
detect_and_hover()
print("Returning to Launch (RTL)...")
vehicle.mode = VehicleMode("RTL")
vehicle.close()
print("Mission complete.")
