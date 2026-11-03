"""Common utility functions for car1 robot."""
import time


def set_range(value, min_val, max_val):
    """Clamp value to [min_val, max_val]."""
    return max(min_val, min(max_val, value))


def map_value(value, in_min, in_max, out_min, out_max):
    """Map value from one range to another."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class FPS:
    """Frame rate counter."""

    def __init__(self, buffer_size=30):
        self._buffer_size = buffer_size
        self._timestamps = []
        self._fps = 0.0

    def update(self):
        """Call once per frame to update FPS calculation."""
        now = time.time()
        self._timestamps.append(now)
        if len(self._timestamps) > self._buffer_size:
            self._timestamps.pop(0)
        if len(self._timestamps) >= 2:
            elapsed = self._timestamps[-1] - self._timestamps[0]
            if elapsed > 0:
                self._fps = (len(self._timestamps) - 1) / elapsed

    @property
    def fps(self):
        """Get current FPS value."""
        return self._fps
