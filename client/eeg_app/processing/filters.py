"""Standard filter chain: 60Hz notch + 50Hz LPF + 1Hz HPF (always on)."""
import numpy as np
from scipy import signal as sig

from eeg_app.core.constants import FS


class FilterBank:
    """Per-channel stateful filter bank.

    Standard chain (always on):
      1. 60 Hz notch (Q=30)
      2. 50 Hz low-pass (10th-order Butterworth)
      3. 1 Hz high-pass (10th-order Butterworth)

    Optional:
      - CMF (common-mode filter): subtract mean across channels
    """

    def __init__(self, num_channels: int = 8, fs: float = FS):
        self.num_channels = num_channels
        self.fs = fs

        # 60 Hz notch
        self.notch_b, self.notch_a = sig.iirnotch(60.0, Q=30.0, fs=fs)
        self._notch_zi_template = sig.lfilter_zi(self.notch_b, self.notch_a)

        # 50 Hz low-pass (SOS)
        self.lpf_sos = sig.butter(10, 50.0, btype="low", fs=fs, output="sos")
        self._lpf_zi_template = sig.sosfilt_zi(self.lpf_sos)

        # 1 Hz high-pass (SOS)
        self.hpf_sos = sig.butter(10, 1.0, btype="high", fs=fs, output="sos")
        self._hpf_zi_template = sig.sosfilt_zi(self.hpf_sos)

        # Per-channel filter states
        self.reset(num_channels)

    def reset(self, num_channels: int = None):
        """Reset filter states for N channels."""
        if num_channels is not None:
            self.num_channels = num_channels
        n = self.num_channels
        self.notch_states = [self._notch_zi_template.copy() for _ in range(n)]
        self.lpf_states = [self._lpf_zi_template.copy() for _ in range(n)]
        self.hpf_states = [self._hpf_zi_template.copy() for _ in range(n)]

    def apply(self, data: np.ndarray, cmf: bool = False) -> np.ndarray:
        """Apply the full filter chain to data.

        Args:
            data: (num_samples, num_channels) array in uV
            cmf: if True, subtract common-mode (mean across channels)

        Returns:
            Filtered data (num_samples, num_channels)
        """
        if data.shape[0] == 0:
            return data

        num_ch = min(data.shape[1], self.num_channels)
        out = data.copy()

        for c in range(num_ch):
            # 60 Hz notch
            out[:, c], self.notch_states[c] = sig.lfilter(
                self.notch_b, self.notch_a, out[:, c], zi=self.notch_states[c]
            )
            # 50 Hz LPF
            out[:, c], self.lpf_states[c] = sig.sosfilt(
                self.lpf_sos, out[:, c], zi=self.lpf_states[c]
            )
            # 1 Hz HPF
            out[:, c], self.hpf_states[c] = sig.sosfilt(
                self.hpf_sos, out[:, c], zi=self.hpf_states[c]
            )

        # Common-mode filter (optional)
        if cmf and num_ch > 1:
            common = np.mean(out[:, :num_ch], axis=1, keepdims=True)
            out[:, :num_ch] -= common

        return out
