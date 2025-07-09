import subprocess
import cv2
import os

def convert_avi_to_mp4_ffmpeg(input_path, output_path):
    print(f"Converting {input_path} to {output_path} using ffmpeg...")
    command = [
        'ffmpeg',
        '-i', input_path,
        '-c:v', 'libx264',
        '-preset', 'fast',
        '-crf', '22',
        '-c:a', 'aac',
        '-b:a', '192k',
        output_path
    ]
    subprocess.run(command, check=True)
    print("Conversion complete.")

def play_video(file_path):
    print(f"Playing {file_path}...")
    cap = cv2.VideoCapture(file_path)
    
    if not cap.isOpened():
        print("Error: Cannot open video.")
        return
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow("MP4 Video", frame)
        if cv2.waitKey(25) & 0xFF == ord('q'):  # Press 'q' to quit
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Playback finished.")

if __name__ == "__main__":
    input_avi = "output_20250604_172349.avi"
    output_mp4 = "output.mp4"

    if not os.path.exists(output_mp4):
        convert_avi_to_mp4_ffmpeg(input_avi, output_mp4)

    play_video(output_mp4)
