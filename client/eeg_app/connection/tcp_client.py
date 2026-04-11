"""TCPClient — QThread that receives binary EEG samples from the Pi."""
import json
import socket
import struct
import time

from PyQt5.QtCore import QThread, pyqtSignal

from eeg_app.core.types import StreamMetadata


class TCPClient(QThread):
    """Single persistent TCP client that connects to the Pi streaming server.

    Emits samples as dicts: {'timestamp': float, 'sample_number': int, 'channels': [int]}

    Signals:
        connected(StreamMetadata): TCP connected, metadata received
        disconnected(str): TCP disconnected (reason)
        sample_received(dict): One sample received
    """
    connected = pyqtSignal(object)       # StreamMetadata
    disconnected = pyqtSignal(str)       # reason
    sample_received = pyqtSignal(dict)   # sample dict

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._host = ""
        self._port = 8888
        self._should_connect = False
        self._should_disconnect = False
        self._is_connected = False
        self._buffer = b""

    def connect_to(self, host: str, port: int):
        """Request connection (thread-safe)."""
        self._host = host
        self._port = port
        self._should_connect = True
        if not self.isRunning():
            self._running = True
            self.start()

    def disconnect_from(self):
        """Request disconnection (thread-safe)."""
        self._should_disconnect = True

    def shutdown(self):
        """Stop the thread entirely."""
        self._running = False
        self._should_disconnect = True
        self.wait(5000)

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def run(self):
        self._running = True
        while self._running:
            if self._should_connect:
                self._should_connect = False
                self._do_connect()
            elif self._should_disconnect:
                self._should_disconnect = False
                self._do_disconnect()
            else:
                self.msleep(100)

    def _do_connect(self):
        sock = None
        try:
            host = self._host
            if host.endswith(".local"):
                host = socket.gethostbyname(host)

            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10.0)
            sock.connect((host, self._port))
            sock.settimeout(1.0)

            # Receive metadata line
            self._buffer = b""
            line = self._recv_line(sock)
            metadata_dict = json.loads(line)
            metadata = StreamMetadata.from_json(metadata_dict)

            sample_struct = struct.Struct(metadata.sample_struct)
            sample_size = sample_struct.size
            header_struct = struct.Struct("<II")

            self._is_connected = True
            self.connected.emit(metadata)

            # Main receive loop
            while self._running and not self._should_disconnect:
                try:
                    header_data = self._recv_exact(sock, 8)
                    payload_size, sample_count = header_struct.unpack(header_data)
                    raw = self._recv_exact(sock, payload_size)

                    for i in range(sample_count):
                        offset = i * sample_size
                        unpacked = sample_struct.unpack_from(raw, offset)
                        sample = {
                            "timestamp": unpacked[0],
                            "sample_number": unpacked[1],
                            "channels": list(unpacked[2:]),
                        }
                        self.sample_received.emit(sample)
                except socket.timeout:
                    continue

        except Exception as e:
            if self._running and not self._should_disconnect:
                reason = f"{type(e).__name__}: {e}"
                self.disconnected.emit(reason)
        finally:
            self._is_connected = False
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

    def _do_disconnect(self):
        self._is_connected = False
        # Socket will be closed by the _do_connect finally block

    def _recv_exact(self, sock, n: int) -> bytes:
        while len(self._buffer) < n:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                if not self._running or self._should_disconnect:
                    raise ConnectionError("Shutting down")
                continue
            if not chunk:
                raise ConnectionError("Connection closed")
            self._buffer += chunk
        data = self._buffer[:n]
        self._buffer = self._buffer[n:]
        return data

    def _recv_line(self, sock) -> str:
        while True:
            pos = self._buffer.find(b"\n")
            if pos != -1:
                line = self._buffer[:pos]
                self._buffer = self._buffer[pos + 1 :]
                return line.decode("utf-8")
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("Connection closed")
            self._buffer += chunk
