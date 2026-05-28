import cv2
import time
import threading
import platform # Added to check system OS

class BackgroundCamera:
    """Reads webcam frames in a separate thread optimized for environment context."""
    def __init__(self, src=0, width=640, height=480):
        # Checks if you are on Linux (Raspberry Pi) or Windows/macOS (Laptop)
        if platform.system() == "Linux":
            self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            # Safe default fallback for your Laptop
            self.cap = cv2.VideoCapture(src)
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        self.ret, self.frame = self.cap.read()
        self.running = True
        self.lock = threading.Lock()
        
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret = ret
                    self.frame = frame
            time.sleep(0.005)

    def read(self):
        with self.lock:
            return self.ret, self.frame

    def release(self):
        self.running = False
        self.cap.release()