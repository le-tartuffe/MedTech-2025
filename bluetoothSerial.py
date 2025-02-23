import time
import serial
import cv2
import numpy as np
from ultralytics import YOLO
 
ser = serial.Serial('/dev/tty.ESP32_MedTech', 115200)  # Adjust the baud rate if needed
 
def send_data(angle):
    i = 0
    while i < 10:
        ser.write(f"{angle}\n".encode())  # Send angle data
        print(f"Sent angle: {angle}")
        time.sleep(0.01)  # Delay 1 second before sending the next value
        i += 1
 
class FastYoloDistanceEstimator:
    def __init__(self):
            # Initialize camera
            self.cap = cv2.VideoCapture(0)
            self.width = 640
            self.height = 480
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            self.cap.set(cv2.CAP_PROP_FOCUS, 0.5)
    
            # Load YOLOv8 pose model
            self.model = YOLO("yolov8n-pose.pt")
            
    
    def get_keypoints(self, frame):
        results = self.model(frame, verbose=False)[0]

        # Check if keypoints exist before accessing them
        if results.keypoints is None:
            print("No keypoints detected.")
            return None, None, None, None  

        keypoints = results.keypoints.xy[0].cpu().numpy()
        confidences = results.keypoints.conf[0].cpu().numpy()

        head, shoulder, right_wrist = None, None, None

        if len(keypoints) >= 17 and len(confidences) >= 17:
            if confidences[0] > 0.5:
                head = (int(keypoints[0, 0]), int(keypoints[0, 1]))

            if confidences[5] > 0.5 and confidences[6] > 0.5:
                shoulder_x = (keypoints[5, 0] + keypoints[6, 0]) / 2
                shoulder_y = (keypoints[5, 1] + keypoints[6, 1]) / 2
                shoulder = (int(shoulder_x), int(shoulder_y))

            if confidences[10] > 0.55:
                right_wrist = (int(keypoints[10, 0]), int(keypoints[10, 1]))

            return head, shoulder, right_wrist, {
                'right_wrist': (keypoints[10], confidences[10]),
                'right_elbow': (keypoints[8], confidences[8]),
                'right_shoulder': (keypoints[6], confidences[6])
            }

        return None, None, None, None



    def calculate_2d_distance(self, point1, point2):
        if point1 is None or point2 is None:
            return None
        x1, y1 = point1
        x2, y2 = point2
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
 
    def run(self):
        last_fps_time = time.time()
        frame_count = 0
 
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to capture frame. Exiting.")
                break
 
            # Get keypoints with debug info
            head, shoulder, right_wrist, debug_points = self.get_keypoints(frame)
 
            # Draw points
            if head:
                cv2.circle(frame, head, 5, (0, 255, 0), -1)  # Green for head
            if shoulder:
                cv2.circle(frame, shoulder, 5, (255, 255, 0), -1)  # Yellow for shoulder
            if right_wrist:
                cv2.circle(frame, right_wrist, 5, (0, 0, 255), -1)  # Blue for right wrist
 
            # Debug visualization
            if debug_points:
                # Draw confidence values for right arm points
                for name, (point, conf) in debug_points.items():
                    x, y = point
                    pos = (int(x), int(y))
                    # Draw point
                    cv2.circle(frame, pos, 3, (128, 128, 128), -1)
                    # Draw confidence value
                    cv2.putText(frame, f"{name}: {conf:.2f}",
                              (int(x) + 10, int(y)),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
 
            # Calculate distances
            head_to_right = self.calculate_2d_distance(head, right_wrist)
            head_to_shoulder = self.calculate_2d_distance(head, shoulder)
 
            # Display distances and check condition
            if head_to_shoulder and right_wrist:
                cv2.line(frame, head, right_wrist, (0, 0, 255), 2)
                cv2.line(frame, head, shoulder, (0, 255, 255), 2)
 
                text = f"H-RW: {head_to_right:.1f}px | H-S: {head_to_shoulder:.1f}px"
                cv2.putText(frame, text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
 
                if 2 * head_to_shoulder > head_to_right:
                    warning_text = "Warning: Right wrist too close"
                    cv2.putText(frame, warning_text, (10, 90),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    send_data(180)
                else :
                    send_data(0)
                   
 
            # Calculate FPS
            frame_count += 1
            current_time = time.time()
            if current_time - last_fps_time >= 1.0:
                fps = frame_count / (current_time - last_fps_time)
                frame_count = 0
                last_fps_time = current_time
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
 
            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
 
        self.cleanup()
 
    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()
 
if __name__ == "__main__":
    estimator = FastYoloDistanceEstimator()
    estimator.run()