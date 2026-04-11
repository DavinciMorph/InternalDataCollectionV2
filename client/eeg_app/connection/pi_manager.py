"""PiManager — orchestrates SSH connection, binary lifecycle, and auto-retry."""
from PyQt5.QtCore import QObject, pyqtSignal, QTimer

from eeg_app.core.config import AppConfig
from eeg_app.core.state import StateMachine, AppState
from eeg_app.core.types import InitProgress
from eeg_app.connection.ssh_worker import SSHWorker
from eeg_app.connection.stdout_parser import ParsedEvent


class PiManager(QObject):
    """High-level Pi management: connect SSH, start/stop binary, handle retries.

    Signals:
        init_progress(InitProgress): Updated init progress for UI
        system_ready(int, int): Binary ready (ports, channels)
        error(str): Error message for display
        stdout_line(str): Raw stdout for log display
    """
    init_progress = pyqtSignal(object)
    system_ready = pyqtSignal(int, int)
    error = pyqtSignal(str)
    stdout_line = pyqtSignal(str)
    ssh_connected = pyqtSignal(str)
    binary_stopped = pyqtSignal()

    MAX_RETRIES = 3
    RETRY_DELAY_MS = 2000

    def __init__(self, config: AppConfig, state_machine: StateMachine, parent=None):
        super().__init__(parent)
        self.config = config
        self.sm = state_machine
        self._progress = InitProgress()
        self._retry_count = 0

        # SSH worker thread
        self._worker = SSHWorker(parent=self)
        self._worker.configure(config.pi_host, config.ssh_user, config.ssh_password)
        self._worker.ssh_connected.connect(self._on_ssh_connected)
        self._worker.ssh_error.connect(self._on_ssh_error)
        self._worker.binary_started.connect(self._on_binary_started)
        self._worker.binary_exited.connect(self._on_binary_exited)
        self._worker.system_ready.connect(self._on_system_ready)
        self._worker.stdout_event.connect(self._on_stdout_event)
        self._worker.stdout_line.connect(self.stdout_line.emit)

        # Retry timer
        self._retry_timer = QTimer(self)
        self._retry_timer.setSingleShot(True)
        self._retry_timer.timeout.connect(self._do_retry)

    def connect(self):
        """Initiate SSH connection to Pi."""
        self.sm.transition(AppState.CONNECTING)
        self._progress = InitProgress()
        self._progress.phase = "Connecting via SSH..."
        self.init_progress.emit(self._progress)
        self._worker.request_connect()

    def start_binary(self):
        """Start the acquisition binary on Pi."""
        self._retry_count = 0
        self._start_binary_internal()

    def _start_binary_internal(self):
        """Internal: start binary (called for initial start and retries)."""
        self.sm.transition(AppState.STARTING)
        self._progress.phase = "Starting acquisition binary..."
        self._progress.retry_count = self._retry_count
        self._progress.percent = 0.1
        self.init_progress.emit(self._progress)

        cmd = self._build_command()
        self._worker.request_start_binary(cmd)

    def stop_binary(self):
        """Stop the acquisition binary on Pi."""
        self.sm.transition(AppState.STOPPING)
        self._worker.request_stop_binary()

    def shutdown(self):
        """Shut down everything."""
        self._retry_timer.stop()
        self._worker.request_shutdown()
        self._worker.wait(5000)

    def _build_command(self) -> str:
        """Build the SSH command to start the binary."""
        binary_dir = self.config.binary_dir
        binary_name = self.config.binary_name
        extra = self.config.extra_args.strip()

        cmd = (
            f"cd {binary_dir} && "
            f"sudo stdbuf -oL ./{binary_name} "
            f"--host 0.0.0.0 --port {self.config.tcp_port}"
        )
        if extra:
            cmd += f" {extra}"
        return cmd

    # --- SSH Worker signal handlers ---

    def _on_ssh_connected(self, ip: str):
        self.sm.transition(AppState.CONNECTED)
        self._progress.phase = "SSH connected"
        self._progress.percent = 0.05
        self.init_progress.emit(self._progress)
        self.ssh_connected.emit(ip)
        # Auto-start binary
        self.start_binary()

    def _on_ssh_error(self, msg: str):
        self.sm.transition(AppState.ERROR)
        self.error.emit(msg)

    def _on_binary_started(self):
        self._progress.phase = "Binary started, initializing ports..."
        self._progress.percent = 0.1
        self.init_progress.emit(self._progress)

    def _on_binary_exited(self, exit_code: int):
        if self.sm.state == AppState.STOPPING:
            # Expected shutdown
            self.sm.transition(AppState.CONNECTED)
            self.binary_stopped.emit()
            return

        if self.sm.state in (AppState.STARTING, AppState.SIGNAL_CHECK):
            # Unexpected exit — retry
            if self._retry_count < self.MAX_RETRIES:
                self._retry_count += 1
                self._progress.phase = f"Init failed (exit {exit_code}), retrying ({self._retry_count}/{self.MAX_RETRIES})..."
                self._progress.retry_count = self._retry_count
                self.init_progress.emit(self._progress)
                self._retry_timer.start(self.RETRY_DELAY_MS)
            else:
                self.sm.transition(AppState.ERROR)
                if exit_code == 2:
                    self.error.emit(
                        "Hardware initialization failed after 3 attempts.\n\n"
                        "Power cycle the EEG hardware:\n"
                        "1. Turn off the power supply\n"
                        "2. Wait 10 seconds\n"
                        "3. Turn it back on\n"
                        "4. Click Retry"
                    )
                else:
                    self.error.emit(f"Binary exited with code {exit_code} after {self.MAX_RETRIES} retries.")

    def _on_system_ready(self, num_ports: int, num_channels: int):
        self._progress.phase = "System ready"
        self._progress.percent = 1.0
        self.init_progress.emit(self._progress)
        self.sm.transition(AppState.SIGNAL_CHECK)
        self.system_ready.emit(num_ports, num_channels)

    def _on_stdout_event(self, event: ParsedEvent):
        """Update progress based on parsed stdout events."""
        t = event.type
        d = event.data

        if t == "phase1_start":
            self._progress.phase = "Phase 1: Configuring devices..."
            self._progress.percent = 0.15
        elif t == "port_configured":
            self._progress.phase = f"All {d.get('count', '?')} ports configured"
            self._progress.percent = 0.3
        elif t == "phase2_start":
            self._progress.phase = "Phase 2: Starting conversions..."
            self._progress.percent = 0.35
        elif t == "initial_health":
            healthy, total = d.get("healthy", 0), d.get("total", 0)
            self._progress.phase = f"Initial health: {healthy}/{total} ports healthy"
            self._progress.percent = 0.4
        elif t == "tier1_start":
            self._progress.phase = "Tier 1 recovery: RDATAC cycling..."
            self._progress.percent = 0.45
        elif t in ("tier1_attempt", "tier1_recover"):
            port = d.get("port", "?")
            attempt = d.get("attempt", 0)
            self._progress.phase = f"Tier 1: {port} attempt {attempt}"
            self._progress.percent = min(0.65, 0.45 + attempt * 0.002)
        elif t == "tier2_start":
            self._progress.phase = "Tier 2 recovery: software RESET..."
            self._progress.percent = 0.65
        elif t in ("tier2_attempt", "tier2_recover"):
            port = d.get("port", "?")
            attempt = d.get("attempt", 0)
            self._progress.phase = f"Tier 2: {port} attempt {attempt}"
            self._progress.percent = min(0.8, 0.65 + attempt * 0.03)
        elif t == "tier3_start":
            self._progress.phase = "Tier 3 recovery: full re-init..."
            self._progress.percent = 0.8
        elif t == "warmup":
            self._progress.phase = "Warmup: discarding settling samples..."
            self._progress.percent = 0.85
        elif t == "final_health":
            port = d.get("port", "?")
            pct = d.get("valid_pct", 0)
            self._progress.phase = f"Final health: {port} {pct}% valid"
            self._progress.percent = 0.9
        elif t == "stream_listen":
            self._progress.phase = "TCP server listening, connecting..."
            self._progress.percent = 0.95
        elif t == "acquisition_run":
            self._progress.phase = "Acquisition running"
            self._progress.percent = 0.98
        else:
            return

        self.init_progress.emit(self._progress)

    def _do_retry(self):
        """Retry starting the binary after a failed init."""
        self.sm.force(AppState.CONNECTED)
        self._start_binary_internal()
