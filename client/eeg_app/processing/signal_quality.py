"""Signal quality tracking — RMS, railed detection, flatline detection."""
from collections import deque
from typing import Dict, Tuple
import numpy as np

from eeg_app.core.constants import LSB_UV


class SignalQualityTracker:
    """Tracks per-channel signal quality from raw ADC samples.

    Quality heuristics (no impedance mode required):
      - Railed: channel stuck at +/- full scale (2^23 - 1)
      - Flatline: no variance (50+ consecutive identical values)
      - RMS amplitude: very low = shorted, very high = loose electrode

    Each channel gets a quality score: "good", "marginal", "bad", "railed"
    """

    RAIL_THRESHOLD = (2**23 - 100)  # Near full-scale ADC value
    FLATLINE_COUNT = 50             # Consecutive identical values
    WINDOW_SIZE = 250               # 1 second at 250 Hz

    def __init__(self, num_channels: int):
        self.num_channels = num_channels
        self._buffers: list[deque] = [deque(maxlen=self.WINDOW_SIZE) for _ in range(num_channels)]
        self._last_values: list[int] = [0] * num_channels
        self._consecutive_same: list[int] = [0] * num_channels
        self._quality: list[str] = ["unknown"] * num_channels

    def push_sample(self, channels: list):
        """Push one sample's worth of channel data."""
        for i, val in enumerate(channels):
            if i >= self.num_channels:
                break
            self._buffers[i].append(val)

            # Flatline detection
            if val == self._last_values[i]:
                self._consecutive_same[i] += 1
            else:
                self._consecutive_same[i] = 0
                self._last_values[i] = val

    def compute_quality(self) -> Tuple[list, float, int, int]:
        """Compute per-channel quality and summary stats.

        Returns:
            (quality_list, overall_pct, usable_count, railed_count)
        """
        usable = 0
        railed = 0

        for i in range(self.num_channels):
            buf = self._buffers[i]
            if len(buf) < 10:
                self._quality[i] = "unknown"
                continue

            arr = np.array(buf, dtype=np.int32)

            # Check railed
            abs_vals = np.abs(arr)
            if np.any(abs_vals > self.RAIL_THRESHOLD):
                railed_pct = np.mean(abs_vals > self.RAIL_THRESHOLD)
                if railed_pct > 0.5:
                    self._quality[i] = "railed"
                    railed += 1
                    continue

            # Check flatline
            if self._consecutive_same[i] >= self.FLATLINE_COUNT:
                self._quality[i] = "bad"
                continue

            # Check RMS
            rms_uv = np.std(arr) * LSB_UV
            if rms_uv < 0.5:
                self._quality[i] = "marginal"  # Very low — possibly shorted
            elif rms_uv > 500:
                self._quality[i] = "marginal"  # Very high — loose electrode
            else:
                self._quality[i] = "good"
                usable += 1

        overall_pct = (usable / self.num_channels * 100) if self.num_channels > 0 else 0
        return self._quality.copy(), overall_pct, usable, railed

    def reset(self, num_channels: int = None):
        if num_channels is not None:
            self.num_channels = num_channels
        self._buffers = [deque(maxlen=self.WINDOW_SIZE) for _ in range(self.num_channels)]
        self._last_values = [0] * self.num_channels
        self._consecutive_same = [0] * self.num_channels
        self._quality = ["unknown"] * self.num_channels
