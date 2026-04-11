"""SampleBus — fans out samples from a single TCPClient to N subscribers."""
from PyQt5.QtCore import QObject, pyqtSlot


class SampleBus(QObject):
    """Distributes samples from TCPClient to multiple consumers.

    All dispatch happens on the main thread (via Qt signal auto-connection).
    At 250 Hz, main-thread dispatch takes <0.1ms per sample.

    Subscribers are callables that accept a sample dict:
        {'timestamp': float, 'sample_number': int, 'channels': [int]}
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._subscribers: list = []
        self._active = True

    def subscribe(self, callback):
        """Add a subscriber callback."""
        if callback not in self._subscribers:
            self._subscribers.append(callback)

    def unsubscribe(self, callback):
        """Remove a subscriber callback."""
        try:
            self._subscribers.remove(callback)
        except ValueError:
            pass

    def set_active(self, active: bool):
        """Enable/disable sample dispatch."""
        self._active = active

    @pyqtSlot(dict)
    def on_sample(self, sample: dict):
        """Connected to TCPClient.sample_received. Runs on main thread."""
        if not self._active:
            return
        for cb in self._subscribers:
            try:
                cb(sample)
            except Exception:
                pass
