from fastapi import FastAPI, File, UploadFile
import uvicorn
import cv2
import numpy as np
import torch
from PIL import Image
from io import BytesIO

app = FastAPI()

# Load a pre-trained fruit detection model (replace with your own model)
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

@app.post("/detect-fruit/")
async def detect_fruit(file: UploadFile = File(...)):
    image = Image.open(BytesIO(await file.read()))  # Read uploaded image
    results = model(image)  # Run inference

    detections = results.pandas().xyxy[0][["name", "confidence"]].to_dict(orient="records")
    
    return {"detections": detections}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
