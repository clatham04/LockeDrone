import cv2

# Try index 0, then index 1 if it fails
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW) 

while True:
    ret, frame = cap.read()
    if not ret:
        print("Still cannot grab frame...")
        break
    cv2.imshow("Raw WebCam Test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()