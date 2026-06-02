"""Flight telemetry layer for managing stable hover loops."""
import time
from pid import PIDController

# Standard RC byte centering: 127 is neutral zero-velocity command
NEUTRAL_BYTE = 127


def generate_hover_commands(sensor_drift=None):
    """Calculates stabilization signals required to maintain zero-velocity.

    sensor_drift expects a dictionary: {'x_drift': float, 'y_drift': float}
    representing movement away from the starting point.
    """
    # Instantiate Pitch and Roll loops to target 0.0 drift
    pitch_pid = PIDController(kp=1.2, ki=0.05, kd=0.1, setpoint=0.0)
    roll_pid = PIDController(kp=1.2, ki=0.05, kd=0.1, setpoint=0.0)

    if sensor_drift is None:
        # Without telemetry tracking feedback, output perfect default neutral hold
        return {
            "roll": NEUTRAL_BYTE,
            "pitch": NEUTRAL_BYTE,
            "throttle": NEUTRAL_BYTE,
            "yaw": NEUTRAL_BYTE,
        }

    # Extract current orientation drift measurements
    current_x = sensor_drift.get("x_drift", 0.0)
    current_y = sensor_drift.get("y_drift", 0.0)

    # Compute raw stabilization outputs
    roll_adjustment = roll_pid.update(current_x)
    pitch_adjustment = pitch_pid.update(current_y)

    # Translate the outputs into centered 0-254 bytes for transmission hardware
    return {
        "roll": int(NEUTRAL_BYTE + roll_adjustment),
        "pitch": int(NEUTRAL_BYTE + pitch_adjustment),
        "throttle": NEUTRAL_BYTE,  # Assumes altitude hold is active onboard
        "yaw": NEUTRAL_BYTE,  # Maintain current facing heading
    }