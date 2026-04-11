"""SSHWorker — QThread that handles all blocking paramiko SSH operations."""
import time
import paramiko
from PyQt5.QtCore import QThread, pyqtSignal

from eeg_app.connection.stdout_parser import StdoutParser, ParsedEvent


class SSHWorker(QThread):
    """Manages SSH connection and remote process lifecycle in a background thread.

    Signals:
        ssh_connected(str): SSH session established (ip)
        ssh_error(str): SSH connection or command error
        stdout_event(ParsedEvent): Structured event parsed from binary stdout
        stdout_line(str): Raw stdout line (for log display)
        binary_started(): Binary process launched on Pi
        binary_exited(int): Binary process exited (exit code)
        system_ready(int, int): System ready (ports, channels)
    """
    ssh_connected = pyqtSignal(str)
    ssh_error = pyqtSignal(str)
    stdout_event = pyqtSignal(object)
    stdout_line = pyqtSignal(str)
    binary_started = pyqtSignal()
    binary_exited = pyqtSignal(int)
    system_ready = pyqtSignal(int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._client: paramiko.SSHClient | None = None
        self._channel = None
        self._running = False
        self._parser = StdoutParser()

        # Command queue (set by main thread, consumed by worker)
        self._pending_command: str | None = None
        self._host: str = ""
        self._user: str = ""
        self._password: str = ""

    def configure(self, host: str, user: str, password: str):
        self._host = host
        self._user = user
        self._password = password

    def request_connect(self):
        """Request SSH connection (called from main thread)."""
        self._pending_command = "connect"
        if not self.isRunning():
            self._running = True
            self.start()

    def request_start_binary(self, command: str):
        """Request binary start (called from main thread)."""
        self._binary_command = command
        self._pending_command = "start_binary"

    def request_stop_binary(self):
        """Request binary stop (called from main thread)."""
        self._pending_command = "stop_binary"

    def request_shutdown(self):
        """Request thread shutdown."""
        self._running = False
        self._pending_command = "shutdown"

    def run(self):
        self._running = True
        while self._running:
            cmd = self._pending_command
            self._pending_command = None

            if cmd == "connect":
                self._do_connect()
            elif cmd == "start_binary":
                self._do_start_binary()
            elif cmd == "stop_binary":
                self._do_stop_binary()
            elif cmd == "shutdown":
                self._do_disconnect()
                break
            elif self._channel is not None:
                # Read stdout while binary is running
                self._read_stdout()
            else:
                self.msleep(100)

    def _do_connect(self):
        try:
            self._client = paramiko.SSHClient()
            self._client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self._client.connect(
                self._host,
                username=self._user,
                password=self._password,
                timeout=10,
            )
            self._client.get_transport().set_keepalive(10)
            self.ssh_connected.emit(self._host)
        except Exception as e:
            self.ssh_error.emit(f"SSH connection failed: {e}")

    def _do_disconnect(self):
        if self._channel:
            try:
                self._channel.close()
            except Exception:
                pass
            self._channel = None
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None

    def _do_start_binary(self):
        if not self._client:
            self.ssh_error.emit("Not connected via SSH")
            return

        # Kill any stale process first
        try:
            self._exec_simple(
                "sudo pkill -INT -f ads1299_acquire 2>/dev/null; sleep 2; "
                "sudo pkill -9 -f ads1299_acquire 2>/dev/null; sleep 1"
            )
        except Exception:
            pass

        # Start the binary
        try:
            transport = self._client.get_transport()
            self._channel = transport.open_session()
            self._channel.get_pty()
            self._channel.exec_command(self._binary_command)
            self.binary_started.emit()
        except Exception as e:
            self.ssh_error.emit(f"Failed to start binary: {e}")
            self._channel = None

    def _do_stop_binary(self):
        if not self._client:
            return

        # Send SIGINT via separate SSH command
        try:
            self._exec_simple("sudo kill -INT $(pgrep -f ads1299_acquire) 2>/dev/null")
        except Exception:
            pass

        # Wait for channel to close
        if self._channel:
            for _ in range(150):  # 15 seconds
                if self._channel.exit_status_ready():
                    break
                time.sleep(0.1)

            if not self._channel.exit_status_ready():
                # Force kill
                try:
                    self._exec_simple("sudo kill -9 $(pgrep -f ads1299_acquire) 2>/dev/null")
                except Exception:
                    pass
                time.sleep(1)

            exit_code = self._channel.recv_exit_status() if self._channel.exit_status_ready() else -1
            self._channel.close()
            self._channel = None
            self.binary_exited.emit(exit_code)
        else:
            self.binary_exited.emit(0)

    def _read_stdout(self):
        """Read available stdout from the running binary."""
        if not self._channel:
            return

        # Check if process exited
        if self._channel.exit_status_ready():
            exit_code = self._channel.recv_exit_status()
            self._channel.close()
            self._channel = None
            self.binary_exited.emit(exit_code)
            return

        # Read available data
        if self._channel.recv_ready():
            try:
                data = self._channel.recv(65536).decode("utf-8", errors="replace")
                for line in data.splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    self.stdout_line.emit(line)
                    event = self._parser.parse_line(line)
                    if event:
                        self.stdout_event.emit(event)
                        if event.type == "system_ready":
                            self.system_ready.emit(
                                event.data["ports"],
                                event.data["channels"],
                            )
            except Exception:
                pass
        else:
            self.msleep(50)

    def _exec_simple(self, cmd: str, timeout: float = 15.0):
        """Execute a simple command and wait for completion."""
        if not self._client:
            return
        _, stdout, _ = self._client.exec_command(cmd, timeout=timeout)
        stdout.channel.recv_exit_status()

    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._client.get_transport() is not None
