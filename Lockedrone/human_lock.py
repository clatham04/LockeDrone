import time
import cv2
from camera import BackgroundCamera
from detector import HumanDetector
from telemetry import TelemetryCalculator

def main():
    print("[INFO] Initializing Locke Drone autonomous tracking system...")
    
    # 1. Initialize our optimized modules
    # Frame size 640x480 provides a good balance for telemetry calculations
    cam = BackgroundCamera(src=0, width=640, height=480)
    
    # Points to your optimized CPU/NCNN model directory (or "yolo11n.pt" for testing)
    detector = HumanDetector(model_path="yolo11n.pt", device="cpu")
    
    # Telemetry assumes average human dimensions (cm) and your camera's physical setup
    telemetry = TelemetryCalculator(known_width=50.0, known_height=170.0, focal_length=640.0, camera_tilt=15.0)
    
    print("[INFO] System arming... Warm-up phase complete.")
    print("[INFO] Press 'q' in the console or frame window to exit.")

    # Variables to track exact operational FPS
    fps_start_time = time.time()
    fps_counter = 0
    current_fps = 0.0

    # --- Grace Period / Hysteresis Tracking Logic ---
    lost_frame_counter = 0
    MAX_LOST_FRAMES = 5  # Allow up to 5 blurry/bad frames before giving up entirely

    try:
        while True:
            # 2. Capture the latest frame from the background thread
            ret, frame = cam.read()
            if not ret or frame is None:
                continue

            # 3. Perform human tracking inference (Confidence set to 0.30 for stability)
            target = detector.detect_primary_target(frame, conf_threshold=0.30, imgsz=256)
            
            if target:
                # Reset our lost frame counter because we see the target clearly
                lost_frame_counter = 0
                x1, y1, x2, y2, confidence = target
                
                # Calculate bounding box dimensions
                box_width = x2 - x1
                box_height = y2 - y1
                
                # 4. Extract telemetry and spatial distance configurations
                distance_to_target = telemetry.get_distance_to_human(box_width, box_height)
                distance_to_ground = telemetry.get_distance_to_ground(distance_to_target, y2, frame_height=480)
                
                # Calculate center offset (for drone yaw/pan alignment)
                frame_center_x = frame.shape[1] / 2
                target_center_x = x1 + (box_width / 2)
                horizontal_offset = target_center_x - frame_center_x

                # 5. Output true tracking telemetry vectors (Throttled to once every 10 frames)
                if fps_counter % 10 == 0:
                    print(f"[TRACKING] FPS: {current_fps:.1f} | Conf: {confidence:.2f} | "
                          f"Dist: {distance_to_target:.1f}cm | Alt: {distance_to_ground:.1f}cm | "
                          f"Offset X: {horizontal_offset:.1f}px")
                
                # Optional visual overlay for bench-testing
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Human: {distance_to_target:.1f}cm", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                # We didn't see the target this frame. Increment our counter.
                lost_frame_counter += 1
                
                if lost_frame_counter < MAX_LOST_FRAMES and 'distance_to_target' in locals():
                    # If we just recently lost the target, hold onto the last known telemetry data smoothly
                    if fps_counter % 10 == 0:
                        print(f"[TRACKING - HOLD] FPS: {current_fps:.1f} | Filtering frame drop ({lost_frame_counter}/{MAX_LOST_FRAMES})")
                else:
                    # Truly lost the target after 5 consecutive failed frames
                    if fps_counter % 10 == 0:
                        print(f"[SEARCHING] FPS: {current_fps:.1f} | No human target detected in frame context.")

            # Calculate actual performance frames per second
            fps_counter += 1
            elapsed_time = time.time() - fps_start_time
            if elapsed_time >= 1.0:
                current_fps = fps_counter / elapsed_time
                fps_counter = 0
                fps_start_time = time.time()

            # Display window (Note: running GUI windows on a headless Pi reduces FPS. 
            # Comment out these two lines below during actual autonomous flights)
            cv2.imshow("Locke Drone Vision System", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[INFO] Manual termination intercepted.")
        
    finally:
        # Clean shutdown sequence
        print("[INFO] Disarming systems and closing camera pipelines...")
        cam.release()
        cv2.destroyAllWindows()
        print("[INFO] Drone tracking engine offline.")

if __name__ == "__main__":
    main()