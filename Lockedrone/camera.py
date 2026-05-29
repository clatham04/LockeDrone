import cv2
import platform

class BackgroundCamera:
    """Handles camera streams cleanly. Uses threading on Linux (Pi), and direct stream on Windows."""
    def __init__(self, src=0, width=640, height=480):
        self.is_linux = platform.system() == "Linux"
        
        if self.is_linux:
            # High-speed background thread capture for the Pi
            import threading
            self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            self.ret, self.frame = self.cap.read()
            self.running = True
            self.lock = threading.Lock()
            threading.Thread(target=self._update, daemon=True).start()
        else:
            # Bulletproof DirectShow driver for Windows Laptops to prevent freezes
            self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def _update(self):
        # This only runs on Linux/Raspberry Pi
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret = ret
                    self.frame = frame

    def read(self):
        if self.is_linux:
            with self.lock:
                return self.ret, self.frame
        else:
            # On Windows, read directly from the camera stream to avoid thread deadlocks
            return self.cap.read()

    def release(self):
        if self.is_linux:
            self.running = False
        self.cap.release()