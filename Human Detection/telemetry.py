"""Pure math: turn pixel measurements into real-world distance and altitude.

No state, no classes. Every value a function needs is passed in explicitly, so you
can read each formula top-to-bottom without chasing `self.` attributes around.
"""
import math


def distance_to_human(box_w, box_h, known_width, known_height, focal_length):
    """Estimate distance (cm) to a person from their bounding-box size.

    Uses the pinhole-camera model: the bigger the box, the closer the person.
    Averages the width-based and height-based estimates. Returns None if the box
    is empty.
    """
    if box_w <= 0 or box_h <= 0:
        return None

    by_width = (known_width * focal_length) / box_w
    by_height = (known_height * focal_length) / box_h
    return (by_width + by_height) / 2.0


def distance_to_ground(dist_human, feet_y, frame_height, camera_tilt):
    """Estimate altitude (cm) from the distance and where the feet sit in frame.

    Combines the known camera tilt with how far below centre the target's feet are,
    then projects the distance onto the vertical. Returns None if the geometry is
    out of a sensible range.
    """
    if dist_human is None:
        return None

    px_from_center = feet_y - (frame_height / 2.0)
    angle_offset = (px_from_center / (frame_height / 2.0)) * (45.0 / 2.0)
    total_angle = camera_tilt + angle_offset

    if 0 < total_angle < 85:
        return dist_human * math.sin(math.radians(total_angle))
    return None
