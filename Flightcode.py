# OBJECT DETECTION + DRONEKIT INTEGRATION WITH AUTO YAW SEARCH + AUTO FORWARD MOVE

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
import cv2
from ultralytics import YOLO
import math
from pymavlink import mavutil

# === CONFIGURATION ===
MODEL_PATH = 'kaggle100.pt'
REAL_FRUIT_WIDTH_CM = 8.0
REAL_FRUIT_HEIGHT_CM = 10.0  # Added approximate real height of fruit
FOCAL_LENGTH_MM = 3.6
SENSOR_WIDTH_MM = 4.8
SENSOR_HEIGHT_MM = 3.6
IMAGE_WIDTH_PX = 640
IMAGE_HEIGHT_PX = 480

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

def estimate_height_difference(focal_length_mm, real_height_cm, bbox_height_px, image_height_px, sensor_height_mm):
    if bbox_height_px == 0:
        return float('inf')
    focal_px = (focal_length_mm / sensor_height_mm) * image_height_px
    return (real_height_cm * focal_px) / bbox_height_px

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

# === FORWARD MOVEMENT ===
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0)

    for _ in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# === OBJECT DETECTION + DRONE INTERACTION ===
def detect_and_hover():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH_PX)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT_PX)

    # === Calculate camera FOV from parameters ===
    horizontal_fov_deg = 2 * math.degrees(math.atan((SENSOR_WIDTH_MM / 2) / FOCAL_LENGTH_MM))
    vertical_fov_deg = 2 * math.degrees(math.atan((SENSOR_HEIGHT_MM / 2) / FOCAL_LENGTH_MM))
    print(f"Calculated horizontal FOV: {horizontal_fov_deg:.2f} degrees")
    print(f"Calculated vertical FOV: {vertical_fov_deg:.2f} degrees")

    yaw_angle = 0
    detected = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera error.")
            break

        results = model.predict(source=frame, conf=0.5, verbose=False)[0]
        boxes = results.boxes

        if boxes is None or len(boxes.xyxy) == 0:
            print("No objects detected.")
            yaw_angle += int(horizontal_fov_deg / 2) - 5
            yaw_angle = yaw_angle % 360
            print(f"Rotating to yaw angle: {yaw_angle}")
            condition_yaw(yaw_angle)
            time.sleep(4)
        else:
            bboxes = boxes.xyxy.cpu()
            confs = boxes.conf.cpu()
            clses = boxes.cls.cpu()

            for i, box in enumerate(bboxes):
                x1, y1, x2, y2 = box.int().tolist()
                bbox_width = x2 - x1
                bbox_height = y2 - y1
                conf = confs[i].item()
                dist_cm = estimate_distance(FOCAL_LENGTH_MM, REAL_FRUIT_WIDTH_CM, bbox_width, IMAGE_WIDTH_PX, SENSOR_WIDTH_MM)
                height_cm = estimate_height_difference(FOCAL_LENGTH_MM, REAL_FRUIT_HEIGHT_CM, bbox_height, IMAGE_HEIGHT_PX, SENSOR_HEIGHT_MM)

                label = f"Conf: {conf:.2f} | Dist: {dist_cm:.1f}cm | Height Diff: {height_cm:.1f}cm"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if conf > 0.6:
                    print(f"Fruit detected at ~{dist_cm:.1f}cm | Height difference: ~{height_cm:.1f}cm")
                    bbox_center_x = (x1 + x2) // 2
                    frame_center_x = IMAGE_WIDTH_PX // 2
                    error = bbox_center_x - frame_center_x

                    if abs(error) > 20:
                        correction_angle = int((error / IMAGE_WIDTH_PX) * horizontal_fov_deg)
                        print(f"Aligning yaw by {correction_angle} degrees")
                        condition_yaw(correction_angle, relative=True)
                        time.sleep(3)
                    
                    print("Switching to GUIDED mode to move forward toward object")
                    vehicle.mode = VehicleMode("GUIDED")
                    dist_m = dist_cm / 100.0
                    forward_duration = int(dist_m / 0.25)  # Assuming 0.5 m/s forward speed
                    send_ned_velocity(0.25, 0, 0, forward_duration)

                    detected = True
                    break

        cv2.imshow("Live Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if detected:
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
