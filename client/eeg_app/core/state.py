"""Application state machine."""
from enum import Enum, auto
from PyQt5.QtCore import QObject, pyqtSignal


class AppState(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    STARTING = auto()
    SIGNAL_CHECK = auto()
    RECORDING = auto()
    STOPPING = auto()
    ERROR = auto()


# Valid state transitions
_TRANSITIONS = {
    AppState.DISCONNECTED: {AppState.CONNECTING},
    AppState.CONNECTING: {AppState.CONNECTED, AppState.ERROR, AppState.DISCONNECTED},
    AppState.CONNECTED: {AppState.STARTING, AppState.DISCONNECTED},
    AppState.STARTING: {AppState.SIGNAL_CHECK, AppState.ERROR, AppState.CONNECTED, AppState.DISCONNECTED},
    AppState.SIGNAL_CHECK: {AppState.RECORDING, AppState.STOPPING, AppState.ERROR, AppState.DISCONNECTED},
    AppState.RECORDING: {AppState.SIGNAL_CHECK, AppState.STOPPING, AppState.ERROR, AppState.DISCONNECTED},
    AppState.STOPPING: {AppState.CONNECTED, AppState.STARTING, AppState.DISCONNECTED, AppState.ERROR},
    AppState.ERROR: {AppState.CONNECTED, AppState.DISCONNECTED, AppState.STARTING},
}


class StateMachine(QObject):
    """Thread-safe state machine with validated transitions."""
    state_changed = pyqtSignal(object, object)  # old_state, new_state

    def __init__(self, parent=None):
        super().__init__(parent)
        self._state = AppState.DISCONNECTED

    @property
    def state(self) -> AppState:
        return self._state

    def transition(self, new_state: AppState) -> bool:
        old = self._state
        if new_state not in _TRANSITIONS.get(old, set()):
            return False
        self._state = new_state
        self.state_changed.emit(old, new_state)
        return True

    def force(self, new_state: AppState):
        old = self._state
        self._state = new_state
        self.state_changed.emit(old, new_state)
