from ultralytics import YOLO

class HumanDetector:
    """Handles initialization and prediction using YOLOv11."""
    def __init__(self, model_path="yolo11n.pt", device="cuda"):
        self.model = YOLO(model_path)
        self.device = device
        self.model.to(self.device)

    def detect_primary_target(self, frame, conf_threshold=0.45, imgsz=320):
        """
        Runs inference and returns data for the highest confidence person.
        Returns: (x1, y1, x2, y2, confidence) or None if no target found.
        """
        results = self.model(frame, conf=conf_threshold, classes=[0], verbose=False, imgsz=imgsz, device=self.device)
        boxes = results[0].boxes

        if len(boxes) > 0:
            best_idx = int(boxes.conf.argmax())
            x1, y1, x2, y2 = map(int, boxes.xyxy[best_idx])
            conf = float(boxes.conf[best_idx])
            return x1, y1, x2, y2, conf
            
        return None