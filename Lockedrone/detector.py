from ultralytics import YOLO
import numpy as np

class HumanDetector:
    """Handles initialization and prediction optimized for Raspberry Pi CPU execution."""
    def __init__(self, model_path="yolo11n_ncnn_model", device="cpu"):
        # Note: 'yolo11n_ncnn_model' is a compiled directory format that leverages 
        # ARM NEON instructions natively on the Pi 4B CPU.
        self.model = YOLO(model_path, task="detect")
        self.device = device

    def detect_primary_target(self, frame, conf_threshold=0.40, imgsz=256):
        """
        Runs highly optimized inference on ARM architecture.
        Returns: (x1, y1, x2, y2, confidence) or None if no target found.
        """
        # Lowering default imgsz to 256 yields a vast speedup while retaining target identification
        results = self.model(
            frame, 
            conf=conf_threshold, 
            classes=[0], 
            verbose=False, 
            imgsz=imgsz, 
            device=self.device
        )
        
        boxes = results[0].boxes

        if boxes is not None and len(boxes) > 0:
            # Handle native Tensor format or Numpy arrays cleanly without breaking CPU memory
            confs = boxes.conf.cpu().numpy() if hasattr(boxes.conf, 'cpu') else boxes.conf
            best_idx = int(np.argmax(confs))
            
            xyxy = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, 'cpu') else boxes.xyxy
            x1, y1, x2, y2 = map(int, xyxy[best_idx])
            conf = float(confs[best_idx])
            
            return x1, y1, x2, y2, conf
            
        return None