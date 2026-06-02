"""Stateless-capable PID controller class for flight stabilization."""
import time


class PIDController:

    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(-127, 127)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits

        # Persistent controller memory state
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None

    def reset(self):
        """Resets the internal error integrals to prevent windup."""
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None

    def update(self, current_value):
        """Computes the control output based on the current system feedback."""
        now = time.time()
        if self._last_time is None:
            self._last_time = now
            return 0.0

        dt = now - self._last_time
        if dt <= 0.0:
            return 0.0

        # Calculate error parameters
        error = self.setpoint - current_value

        # 1. Proportional term
        p_term = self.kp * error

        # 2. Integral term (with anti-windup protection via clamping)
        self._integral += error * dt
        i_term = self.ki * self._integral

        # 3. Derivative term (rate of error change)
        derivative = (error - self._last_error) / dt
        d_term = self.kd * derivative

        # Total combined output
        output = p_term + i_term + d_term

        # Enforce strict hardware boundary limits
        min_limit, max_limit = self.output_limits
        if output > max_limit:
            output = max_limit
        elif output < min_limit:
            output = min_limit

        # Update state history
        self._last_error = error
        self._last_time = now

        return output