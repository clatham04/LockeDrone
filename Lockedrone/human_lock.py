import time
import cv2
import os
from camera import BackgroundCamera
from detector import HumanDetector
from telemetry import TelemetryCalculator

def clear_screen():
    """Clears the terminal screen for a clean, live-updating dashboard look."""
    os.system('cls' if os.name == 'nt' else 'clear')

def main():
    print("[INFO] Initializing Locke Drone autonomous tracking system...")
    
    cam = BackgroundCamera(src=0, width=640, height=480)
    detector = HumanDetector(model_path="yolo11n_ncnn_model", device="cpu")
    telemetry = TelemetryCalculator(known_width=50.0, known_height=170.0, focal_length=640.0, camera_tilt=15.0)
    
    print("[INFO] System arming... Warm-up phase complete.")
    time.sleep(1.5) 

    fps_start_time = time.time()
    fps_counter = 0
    current_fps = 0.0
    lost_frame_counter = 0
    MAX_LOST_FRAMES = 5  

    try:
        while True:
            time.sleep(0.005) # Pacing delay for Windows stability

            ret, frame = cam.read()
            if not ret or frame is None:
                continue

            target = detector.detect_primary_target(frame, conf_threshold=0.30, imgsz=256)
            
            # --- Initialize Dashboard Status Variables ---
            status_msg = "[SEARCHING] Looking for human target..."
            extra_telemetry = ""

            if target:
                lost_frame_counter = 0
                x1, y1, x2, y2, confidence = target
                
                box_width, box_height = (x2 - x1), (y2 - y1)
                distance_to_target = telemetry.get_distance_to_human(box_width, box_height)
                distance_to_ground = telemetry.get_distance_to_ground(distance_to_target, y2, frame_height=480)
                
                horizontal_offset = (x1 + (box_width / 2)) - (frame.shape[1] / 2)
                head_x, head_y = int(x1 + (box_width / 2)), int(y1 + (box_height * 0.10))

                # Update Status Strings for Dashboard
                status_msg = "TRACKING TARGET"
                extra_telemetry = (
                    f" Model Confidence: {confidence*100:.1f}%\n"
                    f" Dist to Target:   {distance_to_target:.1f} cm\n"
                    f" Estimated Alt:    {distance_to_ground:.1f} cm\n"
                    f" Center Offset X:  {horizontal_offset:.1f} px\n"
                    f" Target Lock:      Head Coordinates ({head_x}, {head_y})\n"
                )
                
                # Visual Overlays
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (head_x, head_y), 7, (0, 0, 255), -1)
                cv2.putText(frame, "LOCK ON", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                lost_frame_counter += 1
                if lost_frame_counter < MAX_LOST_FRAMES and 'distance_to_target' in locals():
                    status_msg = f"[TRACKING - HOLD] Filtering frame drop... ({lost_frame_counter}/{MAX_LOST_FRAMES})"

            # --- Unified Live Updating Dashboard ---
            if fps_counter % 10 == 0:
                clear_screen()
                print("================ LOCKE DRONE TELEMETRY ================")
                print(f" Status:           {status_msg}")
                print(f" System Speed:     {current_fps:.1f} FPS")
                if extra_telemetry:
                    print(extra_telemetry.strip())
                print("=======================================================")
                print(" Press 'q' on the camera window to exit.")

            # Track operational FPS accurately
            fps_counter += 1
            elapsed_time = time.time() - fps_start_time
            if elapsed_time >= 1.0:
                current_fps = fps_counter / elapsed_time
                fps_counter, fps_start_time = 0, time.time()

            # Render frame window safely
            cv2.imshow("Locke Drone Vision System", frame)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[INFO] Manual termination intercepted.")
        
    finally:
        print("[INFO] Disarming systems and closing camera pipelines...")
        cam.release()
        cv2.destroyAllWindows()
        print("[INFO] Drone tracking engine offline.")

if __name__ == "__main__":
    main()