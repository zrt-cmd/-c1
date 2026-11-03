"""
Common utilities for car1_app ROS2 nodes.
"""

import time
from typing import Callable, Optional, Tuple
import numpy as np


class PID:
    """Simple PID controller."""

    def __init__(self, kp: float, ki: float, kd: float, limit: float):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            limit: Output limit (absolute value)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit

        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        """
        Update PID controller with new error.

        Args:
            error: Current error value
            dt: Time delta since last update

        Returns:
            Control output
        """
        if dt <= 0:
            dt = 0.001

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Apply output limit
        output = max(-self.limit, min(self.limit, output))

        self.prev_error = error

        return output

    def reset(self):
        """Reset PID controller state."""
        self.integral = 0.0
        self.prev_error = 0.0


class ColorPicker:
    """Accumulates LAB color samples from clicked point over multiple frames."""

    def __init__(self, num_samples: int = 10):
        """
        Initialize ColorPicker.

        Args:
            num_samples: Number of samples to accumulate
        """
        self.num_samples = num_samples
        self.samples = []
        self.click_point: Optional[Tuple[int, int]] = None

    def set_click_point(self, x: int, y: int):
        """Set the point to sample from."""
        self.click_point = (x, y)
        self.samples = []

    def add_sample(self, lab_image: np.ndarray) -> bool:
        """
        Add a sample from the current frame.

        Args:
            lab_image: Image in LAB color space

        Returns:
            True if enough samples collected
        """
        if self.click_point is None:
            return False

        x, y = self.click_point
        if 0 <= y < lab_image.shape[0] and 0 <= x < lab_image.shape[1]:
            self.samples.append(lab_image[y, x].copy())

        return len(self.samples) >= self.num_samples

    def get_average_lab(self) -> Optional[Tuple[float, float, float]]:
        """
        Get averaged LAB values from collected samples.

        Returns:
            Tuple of (L, A, B) values or None if not enough samples
        """
        if len(self.samples) < self.num_samples:
            return None

        avg = np.mean(self.samples, axis=0)
        return tuple(avg.tolist())

    def reset(self):
        """Reset the color picker."""
        self.samples = []
        self.click_point = None


class Heart:
    """Heartbeat watchdog that calls timeout callback if no heartbeat received."""

    def __init__(self, timeout: float, timeout_callback: Callable[[], None]):
        """
        Initialize Heart watchdog.

        Args:
            timeout: Timeout duration in seconds
            timeout_callback: Function to call on timeout
        """
        self.timeout = timeout
        self.timeout_callback = timeout_callback
        self.last_heartbeat = time.time()
        self.enabled = True

    def beat(self):
        """Register a heartbeat."""
        self.last_heartbeat = time.time()

    def check(self):
        """Check if timeout occurred and call callback if needed."""
        if not self.enabled:
            return

        if time.time() - self.last_heartbeat > self.timeout:
            self.enabled = False
            self.timeout_callback()

    def reset(self):
        """Reset the watchdog."""
        self.last_heartbeat = time.time()
        self.enabled = True

