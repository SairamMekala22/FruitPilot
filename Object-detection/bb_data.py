# from ultralytics import YOLO
# import cv2
# import torch
# import math

# model = YOLO('kaggle80.pt')  
# model.fuse()

# real_mango_width_cm = 8.0  # Real-world mango width in cm (approximate)
# focal_length_mm = 3.6
# sensor_width_mm = 4.8
# sensor_height_mm = 3.6
# image_width_px = 640
# image_height_px = 480

# #VIDEO STREAM
# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width_px)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height_px)

# target_box = None
# target_lost = True

# def estimate_distance(focal_length_mm, real_width_cm, bbox_width_px, image_width_px, sensor_width_mm):
#     if bbox_width_px == 0:
#         return float('inf')
#     focal_px = (focal_length_mm / sensor_width_mm) * image_width_px
#     return (real_width_cm * focal_px) / bbox_width_px  # in cm

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("Failed to grab frame.")
#         break

#     results = model.predict(source=frame, conf=0.5, verbose=False)[0]
#     boxes = results.boxes

#     if boxes is None or len(boxes.xyxy) == 0:
#         print("No mangoes detected.")
#         target_box = None
#         target_lost = True
#         cv2.imshow("Target Mango", frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#         continue

#     bboxes = boxes.xyxy.cpu()
#     confs = boxes.conf.cpu()
#     clses = boxes.cls.cpu()

#     mango_mask = clses == 0
#     mango_boxes = bboxes[mango_mask]
#     mango_confs = confs[mango_mask]

#     if len(mango_boxes) == 0:
#         print("No mangoes detected.")
#         target_box = None
#         target_lost = True
#         cv2.imshow("Target Mango", frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#         continue

#     if target_lost:
#         # Pick best mango (area × confidence)
#         scores = []
#         for i, box in enumerate(mango_boxes):
#             w = box[2] - box[0]
#             h = box[3] - box[1]
#             area = w * h
#             scores.append((area * mango_confs[i], i))
#         best_score, best_idx = max(scores)
#         target_box = mango_boxes[best_idx]
#         target_conf = mango_confs[best_idx]
#         target_lost = False

#         # Bounding box and center
#         x1, y1, x2, y2 = target_box.int().tolist()
#         cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
#         bbox_width = x2 - x1

#         #Distance from camera
#         distance_cm = estimate_distance(focal_length_mm, real_mango_width_cm, bbox_width, image_width_px, sensor_width_mm)

#         #Horizontal angle (left/right from center)
#         horizontal_offset_px = cx - (image_width_px / 2)
#         angle_deg = math.degrees(math.atan((horizontal_offset_px * (sensor_width_mm / image_width_px)) / focal_length_mm))

#         #Altitude difference from camera center (real)
#         vertical_offset_px = cy - (image_height_px / 2)
#         vertical_offset_mm = vertical_offset_px * (sensor_height_mm / image_height_px)
#         vertical_angle_rad = math.atan(vertical_offset_mm / focal_length_mm)
#         altitude_diff_cm = distance_cm * math.tan(vertical_angle_rad)

#         print(f"[TARGET LOCKED]")
#         print(f"Bounding Box: ({x1}, {y1}), ({x2}, {y2})")
#         print(f"Center: ({cx}, {cy})")
#         print(f"Distance to mango: {distance_cm:.2f} cm")
#         print(f"Horizontal angle: {angle_deg:.2f}°")
#         print(f"Vertical angle: {math.degrees(vertical_angle_rad):.2f}°")
#         print(f"Altitude difference from camera center (real): {altitude_diff_cm:.2f} cm")

#     else:
#         # Check if target mango still exists
#         found = False
#         for box in mango_boxes:
#             iou = (
#                 torch.min(target_box[2], box[2]) - torch.max(target_box[0], box[0])
#             ) * (
#                 torch.min(target_box[3], box[3]) - torch.max(target_box[1], box[1])
#             )
#             if iou > 0:
#                 found = True
#                 break
#         if not found:
#             print("[TARGET LOST] — Scanning for next mango...")
#             target_box = None
#             target_lost = True
#             continue

#     #Draw Target on Frame
#     x1, y1, x2, y2 = target_box.int().tolist()
#     cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#     cv2.putText(frame, "Target Mango", (x1, y1 - 10),
#                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

#     # Show frame
#     cv2.imshow("Target Mango", frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()


from ultralytics import YOLO
import cv2
import torch
import math

model = YOLO('kaggle100.pt')
model.fuse()

real_mango_width_cm = 8.0  # Real-world mango width in cm (approximate)
focal_length_mm = 3.6
sensor_width_mm = 4.8
sensor_height_mm = 3.6
image_width_px = 640
image_height_px = 480

# Initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width_px)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height_px)

def estimate_distance(focal_length_mm, real_width_cm, bbox_width_px, image_width_px, sensor_width_mm):
    if bbox_width_px == 0:
        return float('inf')
    focal_px = (focal_length_mm / sensor_width_mm) * image_width_px
    return (real_width_cm * focal_px) / bbox_width_px  # in cm

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    results = model.predict(source=frame, conf=0.5, verbose=False)[0]
    boxes = results.boxes

    if boxes is None or len(boxes.xyxy) == 0:
        print("No objects detected.")
        cv2.imshow("Detections", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    bboxes = boxes.xyxy.cpu()
    confs = boxes.conf.cpu()
    clses = boxes.cls.cpu()

    for i, box in enumerate(bboxes):
        x1, y1, x2, y2 = box.int().tolist()
        bbox_width = x2 - x1
        conf = confs[i].item()
        distance_cm = estimate_distance(focal_length_mm, real_mango_width_cm, bbox_width, image_width_px, sensor_width_mm)

        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"Conf: {conf:.2f} | Dist: {distance_cm:.1f}cm"
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Detections", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()