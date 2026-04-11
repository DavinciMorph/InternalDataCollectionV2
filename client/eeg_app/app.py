"""Main application window -- wizard flow through acquisition stages.

Stage 1: Connection & Init   (ConnectPanel)
Stage 2: Signal Preview       (SignalPanel)
Stage 3: Session Config        (ConfigPanel)
Stage 3b: Instructions         (InstructionScreen)
Stage 4: Ready / Experiment    (ReadyScreen / ExperimentScreen)
Stage 5: Break / Pause         (BreakScreen / PauseScreen overlay)
Stage 6: Summary               (SummaryPanel)
"""
import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QMainWindow, QStackedWidget, QLabel, QMessageBox,
)

from eeg_app.core.state import StateMachine, AppState
from eeg_app.core.config import AppConfig
from eeg_app.core.types import StreamMetadata, SessionConfig
from eeg_app.connection.tcp_client import TCPClient
from eeg_app.connection.sample_bus import SampleBus
from eeg_app.recording.csv_writer import CSVWriterThread
from eeg_app.experiment.engine import ExperimentEngine, ExperimentState
from eeg_app.ui.connect_panel import ConnectPanel
from eeg_app.ui.signal_panel import SignalPanel
from eeg_app.ui.config_panel import ConfigPanel
from eeg_app.ui.experiment_screens import (
    ExperimentScreen, BreakScreen, PauseScreen, ReadyScreen, InstructionScreen,
)
from eeg_app.ui.summary_panel import SummaryPanel


# Stack indices
PAGE_CONNECT = 0
PAGE_SIGNAL = 1
PAGE_CONFIG = 2
PAGE_INSTRUCTION = 3
PAGE_READY = 4
PAGE_EXPERIMENT = 5
PAGE_BREAK = 6
PAGE_SUMMARY = 7


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EEG Acquisition System")
        self.resize(1200, 800)

        self.config = AppConfig.load()
        self.state_machine = StateMachine(parent=self)
        self.state_machine.state_changed.connect(self._on_state_changed)

        # TCP client + sample distribution
        self.tcp_client = TCPClient(parent=self)
        self.sample_bus = SampleBus(parent=self)
        self.tcp_client.sample_received.connect(self.sample_bus.on_sample)
        self.tcp_client.connected.connect(self._on_tcp_connected)
        self.tcp_client.disconnected.connect(self._on_tcp_disconnected)

        # Metadata (set when TCP connects)
        self.metadata: StreamMetadata | None = None

        # Session state
        self._session_config: SessionConfig | None = None
        self._csv_writer: CSVWriterThread | None = None

        # Experiment engine (signal-based, no direct app reference)
        self.engine = ExperimentEngine(parent=self)
        self.engine.phase_changed.connect(self._on_phase_changed)
        self.engine.iti_started.connect(self._on_iti_started)
        self.engine.trial_info_changed.connect(self._on_trial_info_changed)
        self.engine.progress_changed.connect(self._on_progress_changed)
        self.engine.break_started.connect(self._on_break_started)
        self.engine.pause_requested.connect(self._on_engine_pause)
        self.engine.resume_requested.connect(self._on_engine_resume)
        self.engine.practice_completed.connect(self._on_practice_complete)
        self.engine.session_finished.connect(self._on_session_finished)

        # Central stacked widget for wizard flow
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        # Stage 1: Connection & Init
        self.connect_panel = ConnectPanel(self.config, self.state_machine, parent=self)
        self.connect_panel.system_ready.connect(self._on_system_ready)
        self.stack.addWidget(self.connect_panel)  # 0

        # Stage 2: Signal Preview (created after metadata arrives)
        self.signal_panel: SignalPanel | None = None
        # Placeholder at index 1 until signal panel is built
        self._signal_placeholder = QLabel("Connecting to EEG stream...")
        self._signal_placeholder.setAlignment(Qt.AlignCenter)
        self._signal_placeholder.setStyleSheet("font-size: 18px; color: #666;")
        self.stack.addWidget(self._signal_placeholder)  # 1

        # Stage 3: Config panel
        self.config_panel = ConfigPanel(self.config, parent=self)
        self.config_panel.start_requested.connect(self._on_config_done)
        self.config_panel.back_requested.connect(self._on_back_to_signals)
        self.stack.addWidget(self.config_panel)  # 2

        # Stage 3b: Instructions
        self.instruction_screen = InstructionScreen(parent=self)
        self.instruction_screen.begin_practice.connect(self._on_begin_practice)
        self.instruction_screen.skip_practice.connect(self._on_skip_practice)
        self.instruction_screen.back_to_config.connect(self._on_back_to_config)
        self.stack.addWidget(self.instruction_screen)  # 3

        # Stage 4: Ready screen
        self.ready_screen = ReadyScreen(parent=self)
        self.ready_screen.start_requested.connect(self._start_real_experiment)
        self.stack.addWidget(self.ready_screen)  # 4

        # Stage 4/5: Experiment screen
        self.experiment_screen = ExperimentScreen(parent=self)
        self.stack.addWidget(self.experiment_screen)  # 5

        # Stage 5: Break screen
        self.break_screen = BreakScreen(parent=self)
        self.break_screen.resume_requested.connect(self._on_break_resume)
        self.stack.addWidget(self.break_screen)  # 6

        # Stage 6: Summary
        self.summary_panel = SummaryPanel(parent=self)
        self.summary_panel.new_session.connect(self._on_new_session)
        self.summary_panel.new_participant.connect(self._on_new_participant)
        self.summary_panel.quit_requested.connect(self.close)
        self.stack.addWidget(self.summary_panel)  # 7

        # Pause overlay (sits on top of experiment screen)
        self.pause_screen = PauseScreen(self.experiment_screen)
        self.pause_screen.hide()
        self.pause_screen.resume_requested.connect(self._on_pause_resume)
        self.pause_screen.end_session_requested.connect(self._on_pause_end)

        # Start on connection panel
        self.stack.setCurrentWidget(self.connect_panel)

        # Auto-start connection on launch
        self.connect_panel.start_connection()

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _on_state_changed(self, old_state: AppState, new_state: AppState):
        pass

    # ------------------------------------------------------------------
    # Stage 1 -> Stage 2: connection
    # ------------------------------------------------------------------
    def _on_system_ready(self, num_ports: int, num_channels: int):
        """Binary is ready -- connect TCP to start receiving data."""
        self.tcp_client.connect_to(self.config.pi_host, self.config.tcp_port)

    def _on_tcp_connected(self, metadata: StreamMetadata):
        """TCP connected and metadata received -- build signal panel and show it."""
        self.metadata = metadata

        # Create or rebuild signal panel with new metadata
        if self.signal_panel is not None:
            self.sample_bus.unsubscribe(self.signal_panel.on_sample)
            self.stack.removeWidget(self.signal_panel)
            self.signal_panel.deleteLater()

        self.signal_panel = SignalPanel(metadata, self.config, parent=self)
        self.signal_panel.proceed_clicked.connect(self._on_proceed_to_config)

        # Replace placeholder at index 1
        if self._signal_placeholder is not None:
            idx = self.stack.indexOf(self._signal_placeholder)
            if idx >= 0:
                self.stack.removeWidget(self._signal_placeholder)
                self._signal_placeholder.deleteLater()
                self._signal_placeholder = None
        self.stack.insertWidget(PAGE_SIGNAL, self.signal_panel)

        # Subscribe signal panel to sample bus
        self.sample_bus.subscribe(self.signal_panel.on_sample)

        # Show signal panel
        self.stack.setCurrentWidget(self.signal_panel)

    def _on_tcp_disconnected(self, reason: str):
        """TCP connection lost."""
        pass

    # ------------------------------------------------------------------
    # Stage 2 -> Stage 3
    # ------------------------------------------------------------------
    def _on_proceed_to_config(self):
        """User clicked 'Signals Look Good' -- proceed to session config."""
        self.stack.setCurrentIndex(PAGE_CONFIG)

    def _on_back_to_signals(self):
        """Back from config to signal panel."""
        if self.signal_panel:
            self.stack.setCurrentWidget(self.signal_panel)
        else:
            self.stack.setCurrentIndex(PAGE_SIGNAL)

    # ------------------------------------------------------------------
    # Stage 3 -> Stage 3b (instructions)
    # ------------------------------------------------------------------
    def _on_config_done(self, config: SessionConfig):
        self._session_config = config
        self.instruction_screen.set_config(config)
        self.stack.setCurrentIndex(PAGE_INSTRUCTION)

    def _on_back_to_config(self):
        self.stack.setCurrentIndex(PAGE_CONFIG)

    # ------------------------------------------------------------------
    # Instructions -> Practice or Real experiment
    # ------------------------------------------------------------------
    def _on_begin_practice(self):
        self.stack.setCurrentIndex(PAGE_EXPERIMENT)
        self.experiment_screen.set_practice(True)
        self.showFullScreen()
        self.engine.start_session(self._session_config, practice=True)

    def _on_skip_practice(self):
        self._show_ready_screen()

    def _show_ready_screen(self):
        self.showFullScreen()
        self.stack.setCurrentIndex(PAGE_READY)

    def _start_real_experiment(self):
        """Start actual experiment with EEG recording."""
        # Start CSV writer for EEG data
        self._start_eeg_recording()

        self.stack.setCurrentIndex(PAGE_EXPERIMENT)
        self.experiment_screen.set_practice(False)
        self.engine.start_session(self._session_config, practice=False)

    def _start_eeg_recording(self):
        """Start the CSVWriterThread for EEG sample recording."""
        if self.metadata is None:
            return

        port_configs = [
            {"name": pc.name, "num_devices": pc.num_devices}
            for pc in self.metadata.port_config
        ]
        csv_dir = self._session_config.output_dir if self._session_config else "."
        self._csv_writer = CSVWriterThread(port_configs, csv_dir=csv_dir)
        self._csv_writer.start()

        # Subscribe writer to sample bus
        self.sample_bus.subscribe(self._csv_writer_on_sample)

        # Tell the engine where EEG data lives
        self.engine.set_eeg_info(
            csv_path=self._csv_writer.filename or "",
            sample_count_fn=lambda: self._csv_writer.total_written if self._csv_writer else 0,
        )

    def _csv_writer_on_sample(self, sample: dict):
        """Forward SampleBus samples to the CSV writer."""
        if self._csv_writer:
            self._csv_writer.push(sample)

    def _stop_eeg_recording(self):
        """Stop the CSVWriterThread."""
        if self._csv_writer:
            self.sample_bus.unsubscribe(self._csv_writer_on_sample)
            self._csv_writer.stop()
            self._csv_writer = None

    # ------------------------------------------------------------------
    # Practice complete
    # ------------------------------------------------------------------
    def _on_practice_complete(self):
        self.showNormal()
        self.resize(1200, 800)
        msg = QMessageBox(self)
        msg.setWindowTitle("Practice Complete")
        msg.setText("Practice session is complete.")
        msg.setInformativeText("What would you like to do?")
        begin_btn = msg.addButton("Begin Experiment", QMessageBox.AcceptRole)
        repeat_btn = msg.addButton("Repeat Practice", QMessageBox.ActionRole)
        back_btn = msg.addButton("Back to Config", QMessageBox.RejectRole)
        msg.exec_()
        clicked = msg.clickedButton()
        if clicked is begin_btn:
            self._show_ready_screen()
        elif clicked is repeat_btn:
            self._on_begin_practice()
        else:
            self.stack.setCurrentIndex(PAGE_CONFIG)

    # ------------------------------------------------------------------
    # Engine signal handlers -> UI updates
    # ------------------------------------------------------------------
    def _on_phase_changed(self, phase, gesture, subtext,
                          show_video, go_stop_text, go_stop_color):
        self.experiment_screen.set_phase(
            phase=phase,
            gesture=gesture,
            subtext=subtext,
            show_video=show_video,
            go_stop_text=go_stop_text,
            go_stop_color=go_stop_color,
        )

    def _on_iti_started(self):
        self.experiment_screen.set_iti()

    def _on_trial_info_changed(self, trial, total_trials, block, total_blocks):
        self.experiment_screen.set_trial_info(
            trial, total_trials, block, total_blocks
        )

    def _on_progress_changed(self, current, total):
        self.experiment_screen.set_progress(current, total)

    def _on_break_started(self, duration, block_just_done, total_blocks):
        self.stack.setCurrentIndex(PAGE_BREAK)
        self.break_screen.start_break(duration, block_just_done, total_blocks)

    def _on_break_resume(self):
        self.stack.setCurrentIndex(PAGE_EXPERIMENT)
        self.engine.resume_from_break()

    # ------------------------------------------------------------------
    # Pause / Resume
    # ------------------------------------------------------------------
    def _on_engine_pause(self, trial, total_trials, block, total_blocks):
        self.pause_screen.set_position(trial, total_trials, block, total_blocks)
        self.pause_screen.setGeometry(self.experiment_screen.rect())
        self.pause_screen.raise_()
        self.pause_screen.show()

    def _on_engine_resume(self):
        self.pause_screen.hide()

    def _on_pause_resume(self):
        self.engine.resume()

    def _on_pause_end(self):
        self.pause_screen.hide()
        self.engine.end_early()

    # ------------------------------------------------------------------
    # Session finished
    # ------------------------------------------------------------------
    def _on_session_finished(self, trials_completed, total_trials,
                             duration_s, csv_path, interrupted,
                             eeg_csv_path, eeg_samples):
        self._stop_eeg_recording()
        self.showNormal()
        self.resize(1200, 800)

        # Update EEG path from writer if available
        if self._csv_writer and not eeg_csv_path:
            eeg_csv_path = self._csv_writer.filename or ""
            eeg_samples = self._csv_writer.total_written

        self.summary_panel.set_results(
            trials_completed, total_trials, duration_s,
            csv_path, interrupted, eeg_csv_path, eeg_samples,
        )
        self.stack.setCurrentIndex(PAGE_SUMMARY)

    # ------------------------------------------------------------------
    # Summary navigation
    # ------------------------------------------------------------------
    def _on_new_session(self):
        """New Session -> back to config (Stage 3)."""
        self.stack.setCurrentIndex(PAGE_CONFIG)

    def _on_new_participant(self):
        """New Participant -> back to electrode setup (Stage 2)."""
        if self.signal_panel:
            self.stack.setCurrentWidget(self.signal_panel)
        else:
            self.stack.setCurrentIndex(PAGE_SIGNAL)

    # ------------------------------------------------------------------
    # Keyboard handling
    # ------------------------------------------------------------------
    def keyPressEvent(self, event):
        key = event.key()
        current_page = self.stack.currentIndex()

        # Spacebar or Escape: pause/resume during experiment, breaks, any phase
        if key in (Qt.Key_Space, Qt.Key_Escape):
            # Ready screen: spacebar starts experiment
            if key == Qt.Key_Space and current_page == PAGE_READY:
                self._start_real_experiment()
                return

            # Paused -> resume
            if self.engine.state == ExperimentState.PAUSED:
                self.engine.resume()
                return

            # Any active experiment state (including breaks) -> pause
            if self.engine.state not in (
                ExperimentState.IDLE, ExperimentState.FINISHED,
                ExperimentState.PAUSED,
            ) and current_page in (PAGE_EXPERIMENT, PAGE_BREAK):
                self.engine.pause()
                return

        super().keyPressEvent(event)

    # ------------------------------------------------------------------
    # Window events
    # ------------------------------------------------------------------
    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Keep pause overlay sized to experiment screen
        if self.pause_screen.isVisible():
            self.pause_screen.setGeometry(self.experiment_screen.rect())

    def closeEvent(self, event):
        # Check if experiment is running
        if self.engine.state not in (
            ExperimentState.IDLE, ExperimentState.FINISHED
        ):
            reply = QMessageBox.question(
                self, "Confirm Exit",
                "An experiment is in progress. Data recorded so far is saved.\n\n"
                "Are you sure you want to exit?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                event.ignore()
                return

            # Clean up logger
            if self.engine.logger:
                duration = time.perf_counter() - self.engine.session_start_time
                self.engine.logger.write_metadata_json(
                    session_completed=False,
                    total_trials=self.engine.active_trial_count,
                    duration_s=duration,
                )
                self.engine.logger.close()

        # Stop EEG recording
        self._stop_eeg_recording()

        # Shutdown networking
        self.sample_bus.set_active(False)
        self.tcp_client.shutdown()
        self.connect_panel.pi_manager.shutdown()
        super().closeEvent(event)
