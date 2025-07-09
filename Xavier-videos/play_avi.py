import cv2

# Replace with your video file path
video_path = 'output_20250604_172153.avi'

# Create video capture object
cap = cv2.VideoCapture(video_path)

# Check if video opened successfully
if not cap.isOpened():
    print("Error: Could not open video")
    exit()

# Read and display frames
while True:
    ret, frame = cap.read()
    
    # Break if frame not read successfully (end of video)
    if not ret:
        break
    
    # Display frame in a window
    cv2.imshow('Video Player', frame)
    
    # Exit on 'q' key press (ASCII code 113)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()