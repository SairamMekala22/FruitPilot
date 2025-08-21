# 🍊 FRUITPILOT: Vision-Guided Autonomous Fruit Plucking Drone  

![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)  
![YOLOv8](https://img.shields.io/badge/YOLOv8-Custom%20Model-green)  
![Flask](https://img.shields.io/badge/Flask-Backend-orange)  
![DroneKit](https://img.shields.io/badge/DroneKit-Autonomous%20Control-lightblue)  


---

## 🚀 Overview
**FRUITPILOT** is a computer vision–powered autonomous drone designed to detect and harvest fruits. The system integrates **object detection (YOLOv8)**, real-time inference on an **NVIDIA Jetson Nano**, and **autonomous drone control** using DroneKit.  
The goal is to assist farmers with efficient, precise, and scalable fruit harvesting.

---

## ✨ Features
- 🍋 **Fruit Detection** – Custom-trained YOLOv8 model for mango detection.  
- 📷 **4K Camera Integration** – Captures high-resolution images for robust detection.  
- 🤖 **Autonomous Drone Navigation** – Controlled via DroneKit and Mission Planner.  
- 🧠 **Jetson Nano Acceleration** – Real-time inference optimized for edge devices.  
- 🌐 **Flask Backend + Web Interface** – Control drone operations from a browser.  
- 🛰️ **Simulation Support** – MAVProxy & SITL integration for safe testing.  

---

## 🛠️ Hardware
- **Drone** (PX4 / Ardupilot compatible)  
- **NVIDIA Jetson Nano GPU**  
- **4K Camera Module**  
- **Telemetry Module**  
- **LiPo Battery & Propulsion System**  

---

## 📊 Object Detection
- Model: **YOLOv8**  
- Training: Custom dataset of mango images  
- Output: Bounding boxes + confidence scores in real time  

---

## 🎯 Final Output
- Web-based interface to **arm, take off, detect, and land the drone**  
- Drone automatically **detects fruits and navigates towards them**  
- Real-time detection stream available via browser  

---

## 🔮 Future Work
- 🍏 **Multi-fruit detection** – Extend the model to detect apples, oranges, and other fruits.  
- 🦾 **Robotic plucking arm integration** – Attach a mechanical arm for automated harvesting.  
- 🧭 **Enhanced navigation** – Implement obstacle avoidance and precision landing.  
- ☁️ **Cloud connectivity** – Enable data logging, analytics, and farm-scale monitoring.  
- 🔋 **Power optimization** – Improve flight time with efficient path planning and battery management.  

---
