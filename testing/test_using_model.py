import time
import cv2
from ultralytics import YOLO

# Load the thermal human detection model (model.pt in the same folder)
model = YOLO("model.pt")

# Open default camera (0). Use 1, 2... for other cameras.
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open camera")

print("Press 'q' to quit")

prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference (conf=0.6 as recommended on the model card)
    results = model(frame, conf=0.6, verbose=False)

    # Draw boxes + labels on the frame
    annotated = results[0].plot()

    # Calculate FPS from time between frames
    curr_time = time.time()
    fps = 1.0 / (curr_time - prev_time)
    prev_time = curr_time

    # Draw FPS in the top-left corner
    cv2.putText(
        annotated,
        f"FPS: {fps:.1f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
    )

    cv2.imshow("Thermal Human Detection", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()