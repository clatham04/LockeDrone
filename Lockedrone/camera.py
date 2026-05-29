import cv2
import threading
import time
import platform

class BackgroundCamera:
    """Threaded camera capture handling with automated Windows/Linux driver fallbacks."""
    def __init__(self, src=0, width=640, height=480):
        self.running = True
        self.lock = threading.Lock()
        self.frame = None
        self.ret = False

        if platform.system() == "Windows":
            # Try Media Foundation first (highly compatible with modern Windows 11 / Alienware)
            print(f"[CAMERA] Attempting Windows MSMF backend on index {src}...")
            self.cap = cv2.VideoCapture(src, cv2.CAP_MSMF)
            
            # Fallback to standard initialization if MSMF fails to open
            if not self.cap.isOpened():
                print("[CAMERA] MSMF failed. Falling back to universal auto-detect backend...")
                self.cap = cv2.VideoCapture(src)
        else:
            # Native Linux configuration for the Raspberry Pi
            print(f"[CAMERA] Initializing Linux V4L2 backend on index {src}...")
            self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
        # Set frame parameters
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # Double check if hardware hooked successfully
        if not self.cap.isOpened():
            print(f"[WARNING] Camera hardware could not be opened on index {src}.")
            self.ret = False
        else:
            time.sleep(1.0)  # Warm-up delay for lens exposure adjustment
            self.ret, self.frame = self.cap.read()

        # Start the background thread loop
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while self.running:
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    with self.lock:
                        self.ret = ret
                        self.frame = frame
            time.sleep(0.01)

    def read(self):
        with self.lock:
            if self.frame is not None:
                return self.ret, self.frame.copy()
            return self.ret, None

    def release(self):
        self.running = False
        time.sleep(0.1)
        if self.cap.isOpened():
            self.cap.release()