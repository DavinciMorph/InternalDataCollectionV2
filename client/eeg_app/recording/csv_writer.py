"""CSVWriterThread — async CSV writer for EEG data (from simpleviz.py)."""
import os
import threading
from datetime import datetime
from queue import Queue, Empty


class CSVWriterThread:
    """Async CSV writer that runs in a dedicated thread.

    Consumes raw sample dicts from a queue and writes them to CSV.
    Non-blocking: callers do put_nowait(); if queue full, sample dropped.
    """

    def __init__(self, port_configs, csv_dir="."):
        self._port_configs = port_configs
        self._csv_dir = csv_dir
        self._queue = Queue(maxsize=10000)
        self._running = False
        self._thread = None
        self._total_written = 0
        self._filename = None

        # Build channel names: PortN_devM_chK
        self._channel_names = []
        for pc in port_configs:
            port_name = pc["name"] if isinstance(pc, dict) else pc.name
            num_devices = pc["num_devices"] if isinstance(pc, dict) else pc.num_devices
            for d in range(num_devices):
                for c in range(8):
                    self._channel_names.append(f"{port_name}_dev{d+1}_ch{c+1}")
        self._num_channels = len(self._channel_names)

    @property
    def filename(self):
        return self._filename

    @property
    def total_written(self):
        return self._total_written

    def start(self, filename: str = None):
        """Open CSV file and start writer thread."""
        if self._running:
            return

        if filename:
            self._filename = filename
        else:
            os.makedirs(self._csv_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y-%m-%d_%H%M%S")
            self._filename = os.path.join(self._csv_dir, f"eeg_data_{ts}.csv")

        self._running = True
        self._total_written = 0
        self._thread = threading.Thread(target=self._run, name="csv-writer", daemon=True)
        self._thread.start()

    def push(self, sample):
        """Non-blocking enqueue. Drops if full."""
        if not self._running:
            return
        try:
            self._queue.put_nowait(sample)
        except Exception:
            pass

    def stop(self):
        """Drain, flush, and close."""
        if not self._running:
            return
        self._running = False
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def _run(self):
        f = None
        try:
            f = open(self._filename, "w", buffering=65536)
            header = "timestamp,sample_number," + ",".join(self._channel_names) + "\n"
            f.write(header)

            while True:
                try:
                    sample = self._queue.get(timeout=0.1)
                except Empty:
                    if not self._running:
                        break
                    continue
                if sample is None:
                    break
                self._write_sample(f, sample)
                self._total_written += 1

                # Drain burst
                for _ in range(500):
                    try:
                        sample = self._queue.get_nowait()
                    except Empty:
                        break
                    if sample is None:
                        break
                    self._write_sample(f, sample)
                    self._total_written += 1

            # Final drain
            while True:
                try:
                    sample = self._queue.get_nowait()
                except Empty:
                    break
                if sample is None:
                    continue
                self._write_sample(f, sample)
                self._total_written += 1

        except Exception as e:
            print(f"[csv] Writer error: {e}")
        finally:
            if f is not None:
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass

    def _write_sample(self, f, sample):
        channels = sample["channels"]
        num_ch = min(len(channels), self._num_channels)
        parts = [f"{sample['timestamp']:.6f}", str(sample["sample_number"])]
        for i in range(num_ch):
            parts.append(str(channels[i]))
        for i in range(num_ch, self._num_channels):
            parts.append("0")
        f.write(",".join(parts))
        f.write("\n")
