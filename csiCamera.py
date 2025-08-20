import cv2
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("mangodet_yolov8.pt")  # use your exact path

# Open CSI camera
def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, "
        f"format=NV12, framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={display_width}, height={display_height}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink"
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run detection
    results = model(frame)

    # Draw results on the frame
    annotated_frame = results[0].plot()

    # Display
    cv2.imshow("YOLOv8 Detection", annotated_frame)

    # Exit with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):