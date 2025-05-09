import cv2
from ultralytics import YOLO

# Load model and image
model = YOLO('predict_mango.pt')
image = cv2.imread('mango-reverse.jpg')
results = model(image)[0]

# Image dimensions
frame_height, frame_width = image.shape[:2]
fov_h = 70  # horizontal FOV in degrees (adjust to your drone camera)

# Get bounding box and compute center x
for box in results.boxes:
    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
    bbox_center_x = (x1 + x2) / 2

    # Offset from image center
    dx = bbox_center_x - (frame_width / 2)

    # Angle computation
    angle = (dx / (frame_width / 2)) * (fov_h / 2)
    print(f"Angle between drone nose and mango: {angle:.2f} degrees")

def getAngle(frame):
    frame_height, frame_width = frame.shape[:2]
    fov_h = 90
    results = model(frame)[0]
    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        bbox_center_x = (x1 + x2) / 2
        dx = bbox_center_x - (frame_width / 2)
        angle = (dx / (frame_width / 2)) * (fov_h / 2)
        print("Amgle:- ${angle}")
        return angle