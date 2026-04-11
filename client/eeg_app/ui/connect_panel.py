"""Stage 1: Connection & Initialization panel."""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QProgressBar,
    QFrame, QPushButton, QLineEdit, QFormLayout, QDialog,
    QTextEdit,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont

from eeg_app.core.config import AppConfig
from eeg_app.core.state import StateMachine, AppState
from eeg_app.core.types import InitProgress
from eeg_app.core.constants import (
    DARK_BG, SURFACE, SURFACE_RAISED, BORDER, TEXT_PRIMARY,
    TEXT_SECONDARY, TEXT_MUTED, ACCENT, ACCENT_DIM,
    STATUS_SUCCESS, STATUS_WARNING, STATUS_ERROR, STATUS_CAUTION,
    FONT_FAMILY,
)
from eeg_app.connection.pi_manager import PiManager


class PortStatusRow(QWidget):
    """Single row showing status of one SPI port."""

    def __init__(self, port_name: str, num_devices: int, bus_label: str, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)

        # Status icon
        self.icon_label = QLabel("○")
        self.icon_label.setFixedWidth(20)
        self.icon_label.setAlignment(Qt.AlignCenter)
        self.icon_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 16px;")
        layout.addWidget(self.icon_label)

        # Port name + device count
        self.name_label = QLabel(f"{port_name} ({num_devices} devices)")
        self.name_label.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 14px;")
        layout.addWidget(self.name_label, 1)

        # Bus identifier
        self.bus_label = QLabel(bus_label)
        self.bus_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px;")
        self.bus_label.setFixedWidth(80)
        layout.addWidget(self.bus_label)

        # Status badge
        self.status_label = QLabel("WAITING")
        self.status_label.setFixedWidth(100)
        self.status_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.status_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px; font-weight: bold;")
        layout.addWidget(self.status_label)

    def set_status(self, status: str):
        colors = {
            "waiting": TEXT_MUTED,
            "initializing": STATUS_WARNING,
            "recovering": STATUS_CAUTION,
            "ready": STATUS_SUCCESS,
            "failed": STATUS_ERROR,
        }
        icons = {
            "waiting": "○",
            "initializing": "◌",
            "recovering": "◎",
            "ready": "●",
            "failed": "✕",
        }
        color = colors.get(status, TEXT_MUTED)
        icon = icons.get(status, "○")
        self.icon_label.setText(icon)
        self.icon_label.setStyleSheet(f"color: {color}; font-size: 16px;")
        self.status_label.setText(status.upper())
        self.status_label.setStyleSheet(f"color: {color}; font-size: 12px; font-weight: bold;")


# Default port layout
_DEFAULT_PORTS = [
    ("Port1", 8, "SPI0 CE0"),
    ("Port2", 7, "SPI0 CE1"),
    ("Port3", 5, "SPI3 CE0"),
    ("Port4", 5, "SPI3 CE1"),
    ("Port5", 5, "SPI4 CE0"),
    ("Port6", 5, "SPI4 CE1"),
    ("Port7", 7, "SPI5 CE0"),
]


class ConnectPanel(QWidget):
    """Stage 1: Connection & Init screen.

    Auto-connects to Pi via SSH, starts acquisition binary,
    shows per-port init progress, and auto-advances when ready.
    """
    system_ready = pyqtSignal(int, int)  # ports, channels

    def __init__(self, config: AppConfig, state_machine: StateMachine, parent=None):
        super().__init__(parent)
        self.config = config
        self.sm = state_machine

        self._pi_manager = PiManager(config, state_machine, parent=self)
        self._pi_manager.init_progress.connect(self._on_init_progress)
        self._pi_manager.system_ready.connect(self._on_system_ready)
        self._pi_manager.error.connect(self._on_error)
        self._pi_manager.stdout_line.connect(self._on_stdout_line)
        self._pi_manager.ssh_connected.connect(self._on_ssh_connected)

        self._build_ui()

    @property
    def pi_manager(self) -> PiManager:
        return self._pi_manager

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(20)
        layout.setContentsMargins(40, 40, 40, 40)

        # App header
        title = QLabel("EEG ACQUISITION SYSTEM")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 28px; font-weight: bold;")
        layout.addWidget(title)

        subtitle = QLabel("336 Channel  |  7 Port  |  ADS1299")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet(f"color: {TEXT_SECONDARY}; font-size: 16px;")
        layout.addWidget(subtitle)

        layout.addSpacing(20)

        # Progress card
        card = QFrame()
        card.setStyleSheet(f"""
            QFrame {{
                background-color: {SURFACE};
                border: 1px solid {BORDER};
                border-radius: 8px;
            }}
        """)
        card.setMaximumWidth(650)
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(20, 20, 20, 20)
        card_layout.setSpacing(12)

        # Status label
        self.status_label = QLabel("Connecting to Pi...")
        self.status_label.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 16px; font-weight: bold; border: none;")
        card_layout.addWidget(self.status_label)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setFixedHeight(6)
        card_layout.addWidget(self.progress_bar)

        # Port status rows
        self.port_rows: dict[str, PortStatusRow] = {}
        for port_name, num_devices, bus_label in _DEFAULT_PORTS:
            row = PortStatusRow(port_name, num_devices, bus_label)
            row.setStyleSheet("border: none;")
            card_layout.addWidget(row)
            self.port_rows[port_name] = row

        # Recovery / retry label
        self.detail_label = QLabel("")
        self.detail_label.setStyleSheet(f"color: {STATUS_WARNING}; font-size: 13px; border: none;")
        self.detail_label.setWordWrap(True)
        card_layout.addWidget(self.detail_label)

        layout.addWidget(card, 0, Qt.AlignCenter)

        # Error area (hidden by default)
        self.error_frame = QFrame()
        self.error_frame.setStyleSheet(f"""
            QFrame {{
                background-color: #1a0a0a;
                border: 1px solid {STATUS_ERROR};
                border-radius: 8px;
            }}
        """)
        self.error_frame.setMaximumWidth(650)
        error_layout = QVBoxLayout(self.error_frame)
        error_layout.setContentsMargins(20, 15, 20, 15)

        self.error_label = QLabel("")
        self.error_label.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 14px; border: none;")
        self.error_label.setWordWrap(True)
        error_layout.addWidget(self.error_label)

        error_btn_layout = QHBoxLayout()
        self.retry_btn = QPushButton("Retry")
        self.retry_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {ACCENT_DIM};
                color: #0d0d0d;
                border: none;
                border-radius: 6px;
                padding: 8px 24px;
                font-weight: bold;
            }}
            QPushButton:hover {{ background-color: {ACCENT}; }}
        """)
        self.retry_btn.clicked.connect(self._retry)
        error_btn_layout.addWidget(self.retry_btn)

        self.settings_btn = QPushButton("Settings")
        self.settings_btn.clicked.connect(self._open_settings)
        error_btn_layout.addWidget(self.settings_btn)
        error_btn_layout.addStretch()
        error_layout.addLayout(error_btn_layout)

        self.error_frame.hide()
        layout.addWidget(self.error_frame, 0, Qt.AlignCenter)

        # Log viewer (collapsed by default)
        self.log_toggle = QPushButton("Show Log")
        self.log_toggle.setStyleSheet(f"color: {TEXT_MUTED}; border: none; font-size: 12px;")
        self.log_toggle.setFlat(True)
        self.log_toggle.clicked.connect(self._toggle_log)
        layout.addWidget(self.log_toggle, 0, Qt.AlignCenter)

        self.log_viewer = QTextEdit()
        self.log_viewer.setReadOnly(True)
        self.log_viewer.setMaximumWidth(650)
        self.log_viewer.setMaximumHeight(150)
        self.log_viewer.setStyleSheet(f"""
            QTextEdit {{
                background-color: {SURFACE};
                color: {TEXT_MUTED};
                border: 1px solid {BORDER};
                border-radius: 4px;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 11px;
            }}
        """)
        self.log_viewer.hide()
        layout.addWidget(self.log_viewer, 0, Qt.AlignCenter)

        # Settings link
        settings_link = QPushButton("Connection Settings")
        settings_link.setStyleSheet(f"color: {TEXT_MUTED}; border: none; font-size: 12px;")
        settings_link.setFlat(True)
        settings_link.setCursor(Qt.PointingHandCursor)
        settings_link.clicked.connect(self._open_settings)
        layout.addWidget(settings_link, 0, Qt.AlignCenter)

        layout.addStretch()

    def start_connection(self):
        """Start the connection process."""
        self.error_frame.hide()
        self.status_label.setText("Connecting to Pi...")
        self.progress_bar.setValue(0)
        self.detail_label.setText("")
        for row in self.port_rows.values():
            row.set_status("waiting")
        self._pi_manager.connect()

    def _retry(self):
        """Retry connection from scratch."""
        self.start_connection()

    # --- Signal handlers ---

    def _on_ssh_connected(self, ip: str):
        self.status_label.setText(f"SSH connected to {ip}")

    def _on_init_progress(self, progress: InitProgress):
        self.status_label.setText(progress.phase)
        self.progress_bar.setValue(int(progress.percent * 100))
        if progress.retry_count > 0:
            self.detail_label.setText(
                f"Retry {progress.retry_count}/{progress.max_retries}"
            )
        # Update port rows based on progress phase
        phase = progress.phase.lower()
        if "tier" in phase:
            # Extract port name if present
            for port_name, row in self.port_rows.items():
                if port_name.lower() in phase.lower():
                    row.set_status("recovering")
        elif "system ready" in phase or progress.percent >= 1.0:
            for row in self.port_rows.values():
                row.set_status("ready")

    def _on_system_ready(self, num_ports: int, num_channels: int):
        self.status_label.setText(f"SYSTEM READY — {num_ports} ports, {num_channels} channels")
        self.status_label.setStyleSheet(
            f"color: {STATUS_SUCCESS}; font-size: 16px; font-weight: bold; border: none;"
        )
        self.progress_bar.setValue(100)
        for row in self.port_rows.values():
            row.set_status("ready")
        self.detail_label.setText("")
        # Auto-advance after brief delay
        QTimer.singleShot(1500, lambda: self.system_ready.emit(num_ports, num_channels))

    def _on_error(self, message: str):
        self.error_label.setText(message)
        self.error_frame.show()
        self.status_label.setText("Error")
        self.status_label.setStyleSheet(
            f"color: {STATUS_ERROR}; font-size: 16px; font-weight: bold; border: none;"
        )

    def _on_stdout_line(self, line: str):
        self.log_viewer.append(line)
        # Auto-scroll
        sb = self.log_viewer.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _toggle_log(self):
        if self.log_viewer.isVisible():
            self.log_viewer.hide()
            self.log_toggle.setText("Show Log")
        else:
            self.log_viewer.show()
            self.log_toggle.setText("Hide Log")

    def _open_settings(self):
        dlg = ConnectionSettingsDialog(self.config, parent=self)
        if dlg.exec_() == QDialog.Accepted:
            self.config.save()
            self._pi_manager._worker.configure(
                self.config.pi_host, self.config.ssh_user, self.config.ssh_password
            )


class ConnectionSettingsDialog(QDialog):
    """Dialog for editing Pi connection settings."""

    def __init__(self, config: AppConfig, parent=None):
        super().__init__(parent)
        self.config = config
        self.setWindowTitle("Connection Settings")
        self.setMinimumWidth(400)

        layout = QFormLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(20, 20, 20, 20)

        self.host_edit = QLineEdit(config.pi_host)
        layout.addRow("Pi Host:", self.host_edit)

        self.user_edit = QLineEdit(config.ssh_user)
        layout.addRow("SSH User:", self.user_edit)

        self.pass_edit = QLineEdit(config.ssh_password)
        self.pass_edit.setEchoMode(QLineEdit.Password)
        layout.addRow("SSH Password:", self.pass_edit)

        self.port_edit = QLineEdit(str(config.tcp_port))
        layout.addRow("TCP Port:", self.port_edit)

        self.binary_edit = QLineEdit(config.binary_path)
        layout.addRow("Binary Path:", self.binary_edit)

        self.args_edit = QLineEdit(config.extra_args)
        layout.addRow("Extra Args:", self.args_edit)

        btn_layout = QHBoxLayout()
        save_btn = QPushButton("Save")
        save_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {ACCENT_DIM};
                color: #0d0d0d;
                font-weight: bold;
                border: none;
                border-radius: 6px;
                padding: 8px 24px;
            }}
        """)
        save_btn.clicked.connect(self._save)
        btn_layout.addWidget(save_btn)

        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        btn_layout.addWidget(cancel_btn)
        layout.addRow(btn_layout)

    def _save(self):
        self.config.pi_host = self.host_edit.text().strip()
        self.config.ssh_user = self.user_edit.text().strip()
        self.config.ssh_password = self.pass_edit.text()
        self.config.tcp_port = int(self.port_edit.text().strip() or "8888")
        self.config.binary_path = self.binary_edit.text().strip()
        self.config.extra_args = self.args_edit.text().strip()
        self.accept()
