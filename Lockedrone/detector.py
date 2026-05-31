from ultralytics import YOLO
from pathlib import Path
import numpy as np

class HumanDetector:
    """Handles initialization and prediction optimized for Raspberry Pi CPU execution."""
    def __init__(self, model_path="yolo11n_ncnn_model", device="cpu"):
        # Note: 'yolo11n_ncnn_model' is a compiled directory format (holding
        # model.ncnn.param + model.ncnn.bin) that leverages ARM NEON instructions
        # natively on the Pi 4B CPU.
        #
        # Resolve to an absolute path relative to the repo root (one level up from
        # this file) so it works no matter which directory you launch from. Without
        # this, Ultralytics silently falls back to downloading yolo11n.pt when the
        # relative path isn't found from the current working directory.
        model_dir = Path(model_path)
        if not model_dir.is_absolute():
            model_dir = (Path(__file__).resolve().parent.parent / model_path)
        if not model_dir.exists():
            raise FileNotFoundError(f"NCNN model not found at: {model_dir}")

        self.model = YOLO(str(model_dir), task="detect")
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