import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Convert frame to HSV — better for color detection than BGR
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define blue color range in HSV
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([130, 255, 255])

    # Create a mask that isolates blue pixels
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Clean up the mask with morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # Find contours of detected blue regions
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Track the largest blue contour
        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) > 500:  # Ignore tiny blobs
            x, y, w, h = cv2.boundingRect(largest)

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Draw centroid
            cx, cy = x + w // 2, y + h // 2
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # Label with position info
            cv2.putText(
                frame,
                f"Blue Object ({cx}, {cy})",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

    cv2.imshow("Blue Object Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()