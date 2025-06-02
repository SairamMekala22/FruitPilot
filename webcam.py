import cv2
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("mangodet_yolov8.pt")  # use your exact path

# Open webcam
cap = cv2.VideoCapture(2)

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
        break

cap.release()
cv2.destroyAllWindows()
