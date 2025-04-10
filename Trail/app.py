import streamlit as st
import cv2
import requests
import numpy as np
from PIL import Image

st.title("🍎 FRUITPILOT: Drone Fruit Detection Dashboard 🍊")

# Capture video feed from drone (Replace 0 with your drone's camera feed URL if needed)
cap = cv2.VideoCapture(0)

stframe = st.empty()  # Placeholder for video feed
detect_button = st.button("Start Detection")

while detect_button:
    ret, frame = cap.read()
    if not ret:
        st.write("Failed to grab frame")
        break

    # Convert frame to JPEG
    _, img_encoded = cv2.imencode('.jpg', frame)
    
    # Send frame to FastAPI backend
    response = requests.post("http://127.0.0.1:8000/detect-fruit/", files={"file": img_encoded.tobytes()})
    detections = response.json().get("detections", [])

    # Draw detections on the frame
    for det in detections:
        label, conf = det["name"], det["confidence"]
        cv2.putText(frame, f"{label} ({conf:.2f})", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # Display frame with detections
    stframe.image(frame, channels="BGR")

cap.release()
