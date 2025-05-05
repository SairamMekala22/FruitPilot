def mango_detection_and_tracking():
    """
    Captures video from the webcam, detects mangoes in the frame, updates mango positions,
    and adjusts the drone's yaw to face the mango.
    """
    global cap, frame, mango_detected, mango_position, frame_center, mango_tracking_active, vehicle

    print("Mango detection and tracking started.")

    # Initialize webcam
    try:
        cap = cv2.VideoCapture(0)  # Use 0 for default webcam
        if not cap.isOpened():
            print("Error: Could not open webcam.")
            return

        # Get webcam frame dimensions
        ret, test_frame = cap.read()
        if ret:
            height, width = test_frame.shape[:2]
            frame_center = (width // 2, height // 2)
            print(f"Webcam initialized. Frame size: {width}x{height}, Center: {frame_center}")
        else:
            print("Error: Failed to read frame from webcam.")
            return
    except Exception as e:
        print(f"Error initializing webcam: {e}")
        return

    while running:
        if cap and cap.isOpened() and mango_tracking_active:
            # Capture frame
            ret, current_frame = cap.read()
            if ret:
                try:
                    # Perform mango detection
                    results = model(current_frame, verbose=False)  # Suppress extra output
                    result = results[0] if results else None

                    if result and len(result.boxes) > 0:
                        # Find mango detections
                        mango_detections = [
                            box for box in result.boxes
                            if hasattr(box, 'cls') and 'mango' in model.names[int(box.cls.item())].lower()
                        ]

                        if mango_detections:
                            # Get first mango detection
                            mango_box = mango_detections[0]
                            x1, y1, x2, y2 = map(int, mango_box.xyxy[0].tolist())
                            center_x = (x1 + x2) // 2
                            center_y = (y1 + y2) // 2
                            width = x2 - x1
                            height = y2 - y1

                            # Update mango position
                            mango_detected = True
                            mango_position = (center_x, center_y)

                            # Draw rectangle and center marker
                            cv2.rectangle(current_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.circle(current_frame, (center_x, center_y), 5, (0, 0, 255), -1)

                            # Calculate yaw adjustment
                            x_offset = center_x - frame_center[0]
                            yaw_rate = -x_offset * 0.01  # Adjust scaling factor as needed

                            # Cap yaw rate
                            yaw_rate = max(min(yaw_rate, 1.0), -1.0)

                            print(f"Adjusting yaw: yaw_rate={yaw_rate:.2f}")

                            # Send yaw command to the drone
                            if vehicle and vehicle.armed:
                                msg = vehicle.message_factory.command_long_encode(
                                    0, 0,  # target system, target component
                                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
                                    0,  # confirmation
                                    0,  # target angle (not used here)
                                    yaw_rate,  # yaw rate
                                    1,  # direction (1 = clockwise, -1 = counterclockwise)
                                    0,  # relative (1 = relative to current heading)
                                    0, 0, 0  # unused parameters
                                )
                                vehicle.send_mavlink(msg)
                        else:
                            mango_detected = False
                    else:
                        mango_detected = False

                except Exception as e:
                    print(f"Error during mango detection and tracking: {e}")

            eventlet.sleep(0.1)  # Process at approximately 10fps

    # Release webcam on exit
    if cap:
        cap.release()
        print("Webcam released.")

    print("Mango detection and tracking stopped.")