import math

class TelemetryCalculator:
    """Calculates spatial metrics relative to the target human and ground geometry."""
    def __init__(self, known_width=50.0, known_height=170.0, focal_length=640.0, camera_tilt=15.0):
        self.known_width = known_width
        self.known_height = known_height
        self.focal_length = focal_length
        self.camera_tilt = camera_tilt

    def get_distance_to_human(self, w, h):
        """Calculates optical distance estimating based on target pixel size dimensions."""
        if w <= 0 or h <= 0:
            return None
            
        d_width = (self.known_width * self.focal_length) / w
        d_height = (self.known_height * self.focal_length) / h
        
        # Simple mathematical average is low cost for real-time tracking execution
        return (d_width + d_height) / 2.0

    def get_distance_to_ground(self, dist_human, y2, frame_height=480):
        """Calculates distance to ground using physical downward pitch layout vectors."""
        if not dist_human:
            return None
            
        # Distance from screen center down to the feet pixel coordinate
        px_from_center = y2 - (frame_height / 2.0) 
        angle_offset = (px_from_center / (frame_height / 2.0)) * (45.0 / 2.0) 
        total_angle = self.camera_tilt + angle_offset
        
        if 0 < total_angle < 85:
            return dist_human * math.sin(math.radians(total_angle))
            
        return None