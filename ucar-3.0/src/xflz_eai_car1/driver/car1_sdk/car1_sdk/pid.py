"""PID controller implementation."""


class PID:
    """Discrete PID controller."""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def update(self, error, dt=1.0):
        """Compute PID output given current error and time step."""
        self._integral += error * dt

        if self._first_call:
            derivative = 0.0
            self._first_call = False
        else:
            derivative = (error - self._prev_error) / dt if dt > 0 else 0.0

        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))
            # Anti-windup: clamp integral if output is saturated
            if abs(output) >= self.output_limit:
                self._integral -= error * dt

        return output

    def reset(self):
        """Reset PID state."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def set_gains(self, kp=None, ki=None, kd=None):
        """Update PID gains."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
