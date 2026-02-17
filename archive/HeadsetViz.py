#!/usr/bin/env python3
"""
HeadsetViz - EEG Headset Visualizer with SPI Port Organization
==============================================================

A sleek dark-themed visualizer for ADS1299 EEG data with:
- Black and neon color scheme
- Channels organized by SPI port (matching server configuration)
- Clusters of 8 channels per daisy-chained device
- Toggleable filters (CMF, Bandpass, Notch)
- Adjustable fixed axis range

Default Configuration (matches HeadsetServer.py):
  Port1: 3 devices (24 channels)
  Port2: 5 devices (40 channels)
  Port3: 5 devices (40 channels)
  Port4: 5 devices (40 channels)
  Port5: 3 devices (24 channels)
  Total: 21 devices, 168 channels @ 250 Hz

USAGE:
======
  # Use default configuration (matches HeadsetServer.py defaults)
  python HeadsetViz.py --host 192.168.1.100
  
  # Custom SPI port configuration
  python HeadsetViz.py --host 192.168.1.100 --spi-ports "Port1,3 Port2,5"
"""

import sys
import socket
import json
import struct
import argparse
import time
import subprocess
import platform
import threading
import traceback
from collections import deque
from queue import Queue, Empty, Full
from typing import Optional, List, Dict, Tuple
import numpy as np
from scipy import signal

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg




# ============================================================================
# DARK NEON THEME
# ============================================================================
DARK_BG = "#0d0d0d"
PANEL_BG = "#1a1a1a"
PLOT_BG = "#0a0a0a"
GRID_COLOR = (60, 60, 60)
TEXT_COLOR = "#e0e0e0"
ACCENT_COLOR = "#00ff88"

# Neon colors for channels (cycling through)
NEON_COLORS = [
    "#00ffff",  # Cyan
    "#ff00ff",  # Magenta
    "#00ff00",  # Lime
    "#ffff00",  # Yellow
    "#ff6600",  # Orange
    "#ff0066",  # Hot pink
    "#6600ff",  # Purple
    "#00ffaa",  # Mint
]

# Port header colors (cycling through)
PORT_COLORS = [
    "#ff6b6b",  # Coral red
    "#4ecdc4",  # Teal
    "#ffe66d",  # Yellow
    "#95e1d3",  # Mint
    "#f38181",  # Salmon
    "#aa96da",  # Lavender
    "#fcbad3",  # Pink
    "#a8d8ea",  # Light blue
]

STYLESHEET = f"""
QMainWindow {{
    background-color: {DARK_BG};
}}
QWidget {{
    background-color: {DARK_BG};
    color: {TEXT_COLOR};
    font-family: 'Segoe UI', 'Arial', sans-serif;
}}
QLabel {{
    color: {TEXT_COLOR};
    font-size: 11pt;
}}
QPushButton {{
    background-color: #2a2a2a;
    color: {TEXT_COLOR};
    border: 1px solid #444;
    border-radius: 4px;
    padding: 8px 16px;
    font-size: 10pt;
    min-width: 80px;
}}
QPushButton:hover {{
    background-color: #3a3a3a;
    border-color: {ACCENT_COLOR};
}}
QPushButton:checked {{
    background-color: #00aa55;
    border-color: {ACCENT_COLOR};
    color: white;
}}
QPushButton:!checked {{
    background-color: #333;
}}
QLineEdit {{
    background-color: #2a2a2a;
    color: {TEXT_COLOR};
    border: 1px solid #444;
    border-radius: 4px;
    padding: 6px;
    font-size: 10pt;
}}
QLineEdit:focus {{
    border-color: {ACCENT_COLOR};
}}
QScrollArea {{
    border: none;
    background-color: {DARK_BG};
}}
QScrollBar:vertical {{
    background-color: #1a1a1a;
    width: 12px;
    border-radius: 6px;
}}
QScrollBar::handle:vertical {{
    background-color: #444;
    border-radius: 6px;
    min-height: 30px;
}}
QScrollBar::handle:vertical:hover {{
    background-color: {ACCENT_COLOR};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}
QScrollBar:horizontal {{
    background-color: #1a1a1a;
    height: 12px;
    border-radius: 6px;
}}
QScrollBar::handle:horizontal {{
    background-color: #444;
    border-radius: 6px;
    min-width: 30px;
}}
QScrollBar::handle:horizontal:hover {{
    background-color: {ACCENT_COLOR};
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0px;
}}
QGroupBox {{
    background-color: {PANEL_BG};
    border: 1px solid #333;
    border-radius: 8px;
    margin-top: 12px;
    padding-top: 10px;
    font-size: 11pt;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 15px;
    padding: 0 8px;
    color: {ACCENT_COLOR};
}}
"""


# ============================================================================
# SPI Port Configuration
# ============================================================================

from dataclasses import dataclass

# Default headset port configuration (matches HeadsetServer.py)
DEFAULT_SPI_PORTS = "Port1,1 Port2,5 Port3,5 Port4,5 Port5,3"
# Total: 21 devices, 168 channels

@dataclass
class SPIPortConfig:
    """Configuration for a single SPI port from the server"""
    name: str
    num_devices: int  # Number of daisy-chained ADS1299 devices
    
    @property
    def num_channels(self) -> int:
        """Each device has 8 channels"""
        return self.num_devices * 8
    
    @property
    def num_clusters(self) -> int:
        """Each device = 1 cluster of 8 channels"""
        return self.num_devices


class GlobalCMF:
    """Global Common Mode Follower for motion artifact rejection."""
    
    def __init__(self, num_channels, sample_rate=250, smoothing_samples=1):
        self.num_channels = num_channels
        self.sample_rate = sample_rate
        self.smoothing_samples = smoothing_samples
        self.cmf_buffer = deque(maxlen=smoothing_samples)
        self.cmf_history = deque(maxlen=int(sample_rate * 5))
        
    def compute_cmf(self, channels):
        cmf_value = np.mean(channels)
        if self.smoothing_samples > 1:
            self.cmf_buffer.append(cmf_value)
            cmf_value = np.mean(self.cmf_buffer)
        self.cmf_history.append(cmf_value)
        return cmf_value
    
    def remove_cmf(self, channels, cmf_value=None):
        if cmf_value is None:
            cmf_value = self.compute_cmf(channels)
        cleaned = [ch - cmf_value for ch in channels]
        return cleaned, cmf_value
    
    def get_cmf_stats(self):
        if len(self.cmf_history) < 10:
            return {'mean': 0, 'std': 0, 'max': 0, 'min': 0, 'range': 0}
        cmf_array = np.array(self.cmf_history)
        return {
            'mean': np.mean(cmf_array),
            'std': np.std(cmf_array),
            'max': np.max(cmf_array),
            'min': np.min(cmf_array),
            'range': np.max(cmf_array) - np.min(cmf_array)
        }


class ChannelPlot(QtWidgets.QWidget):
    """Single channel plot widget with toggleable filters."""

    def __init__(self, channel_num, global_channel_idx, window_seconds, sample_rate, color, parent_visualizer=None):
        super().__init__()
        self.channel_num = channel_num  # Display number (1-8 within cluster)
        self.global_channel_idx = global_channel_idx  # Index in the full channel array
        self.window_seconds = window_seconds
        self.sample_rate = sample_rate
        self.max_samples = int(sample_rate * window_seconds)
        self.parent_visualizer = parent_visualizer
        self.color = color

        # Data buffers
        self.time_buffer = deque(maxlen=self.max_samples)
        self.data_buffer = deque(maxlen=self.max_samples)

        # Filter states (controlled by parent)
        self.setup_bandpass_filter()
        self.setup_notch_filter()
        self.setup_ui()

    def setup_notch_filter(self):
        """Setup 60Hz notch filter."""
        notch_freq = 60.0
        quality_factor = 30.0
        b, a = signal.iirnotch(notch_freq, quality_factor, self.sample_rate)
        self.notch_b = b
        self.notch_a = a
        self.notch_zi = signal.lfilter_zi(b, a)
        self.notch_initialized = False

    def setup_bandpass_filter(self):
        """Setup bandpass filter (1-50Hz)."""
        lowcut = 1.0
        highcut = 50.0
        filter_order = 4
        nyquist = self.sample_rate / 2.0
        low = lowcut / nyquist
        high = highcut / nyquist
        b, a = signal.butter(filter_order, [low, high], btype='bandpass')
        self.filter_b = b
        self.filter_a = a
        self.filter_zi = signal.lfilter_zi(b, a)
        self.filter_initialized = False

    def setup_ui(self):
        """Setup the plot widget UI."""
        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(10)

        # Channel label
        label = QtWidgets.QLabel(f"Ch {self.channel_num}")
        label.setFixedWidth(50)
        label.setStyleSheet(f"color: {self.color}; font-weight: bold; font-size: 10pt;")
        layout.addWidget(label)

        # Plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setMinimumHeight(80)
        self.plot_widget.setMaximumHeight(100)
        self.plot_widget.setBackground(PLOT_BG)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.getAxis('left').setPen(pg.mkPen(color='#555'))
        self.plot_widget.getAxis('bottom').setPen(pg.mkPen(color='#555'))
        self.plot_widget.getAxis('left').setTextPen(pg.mkPen(color='#888'))
        self.plot_widget.getAxis('bottom').setTextPen(pg.mkPen(color='#888'))
        self.plot_widget.disableAutoRange()
        self.plot_widget.setXRange(0, self.window_seconds, padding=0)

        self.curve = self.plot_widget.plot(pen=pg.mkPen(color=self.color, width=1))
        layout.addWidget(self.plot_widget)

        self.setLayout(layout)
        self.setFixedHeight(105)

    def add_sample(self, timestamp, value):
        """Add a new data sample with filtering applied based on parent settings."""
        filtered_value = value

        # Apply notch filter if enabled
        if self.parent_visualizer and self.parent_visualizer.notch_enabled:
            if not self.notch_initialized:
                self.notch_zi = self.notch_zi * value
                self.notch_initialized = True
            notched_value, self.notch_zi = signal.lfilter(
                self.notch_b, self.notch_a, [filtered_value], zi=self.notch_zi
            )
            filtered_value = notched_value[0]

        # Apply bandpass filter if enabled
        if self.parent_visualizer and self.parent_visualizer.bandpass_enabled:
            if not self.filter_initialized:
                self.filter_zi = self.filter_zi * filtered_value
                self.filter_initialized = True
            bp_value, self.filter_zi = signal.lfilter(
                self.filter_b, self.filter_a, [filtered_value], zi=self.filter_zi
            )
            filtered_value = bp_value[0]

        self.time_buffer.append(timestamp)
        self.data_buffer.append(filtered_value)

    def update_plot(self, current_time):
        """Update the plot with latest data."""
        if len(self.time_buffer) == 0:
            return

        time_array = np.array(self.time_buffer)
        data_array = np.array(self.data_buffer)

        mask = time_array >= (current_time - self.window_seconds)
        time_array = time_array[mask]
        data_array = data_array[mask]

        if len(time_array) == 0:
            return

        # Batch all plot changes into a single repaint
        self.plot_widget.setUpdatesEnabled(False)

        self.curve.setData(time_array, data_array)
        self.plot_widget.setXRange(
            max(0, current_time - self.window_seconds),
            current_time,
            padding=0
        )

        if self.parent_visualizer and not self.parent_visualizer.autoscale_enabled:
            self.plot_widget.setYRange(
                self.parent_visualizer.fixed_y_min,
                self.parent_visualizer.fixed_y_max,
                padding=0
            )
        elif len(data_array) > 0:
            y_min, y_max = float(data_array.min()), float(data_array.max())
            y_range = y_max - y_min
            pad = y_range * 0.1 if y_range > 0 else 100
            self.plot_widget.setYRange(y_min - pad, y_max + pad, padding=0)

        self.plot_widget.setUpdatesEnabled(True)


class ClusterWidget(QtWidgets.QWidget):
    """Collapsible cluster containing 8 channels (one ADS1299 device)."""
    
    def __init__(self, cluster_num, start_channel_idx, window_seconds, sample_rate, parent_visualizer, start_expanded=False):
        super().__init__()
        self.cluster_num = cluster_num
        self.start_channel_idx = start_channel_idx  # Global channel index for first channel
        self.expanded = start_expanded
        self.channel_plots = []
        self.parent_visualizer = parent_visualizer
        
        self.setup_ui(window_seconds, sample_rate)
    
    def setup_ui(self, window_seconds, sample_rate):
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Header button - start with collapsed arrow
        arrow = "▼" if self.expanded else "▶"
        self.header_btn = QtWidgets.QPushButton(f"{arrow} Cluster {self.cluster_num}")
        self.header_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: #1f1f1f;
                color: {ACCENT_COLOR};
                border: 1px solid #333;
                border-radius: 6px;
                padding: 10px 20px;
                font-size: 12pt;
                font-weight: bold;
                text-align: left;
            }}
            QPushButton:hover {{
                background-color: #2a2a2a;
                border-color: {ACCENT_COLOR};
            }}
        """)
        self.header_btn.clicked.connect(self.toggle_expand)
        layout.addWidget(self.header_btn)
        
        # Content widget (channels)
        self.content_widget = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout()
        content_layout.setContentsMargins(10, 5, 10, 10)
        content_layout.setSpacing(3)
        
        # Create 8 channel plots for this cluster
        for i in range(8):
            color = NEON_COLORS[i % len(NEON_COLORS)]
            global_idx = self.start_channel_idx + i
            channel_plot = ChannelPlot(
                i + 1,  # Display as Ch 1-8
                global_idx,
                window_seconds, 
                sample_rate, 
                color, 
                self.parent_visualizer
            )
            self.channel_plots.append(channel_plot)
            content_layout.addWidget(channel_plot)
        
        self.content_widget.setLayout(content_layout)
        self.content_widget.setVisible(self.expanded)
        layout.addWidget(self.content_widget)
        
        self.setLayout(layout)
    
    def toggle_expand(self):
        self.expanded = not self.expanded
        self.content_widget.setVisible(self.expanded)
        arrow = "▼" if self.expanded else "▶"
        self.header_btn.setText(f"{arrow} Cluster {self.cluster_num}")
    
    def update_plots(self, current_time):
        if self.expanded:
            for plot in self.channel_plots:
                plot.update_plot(current_time)


class SPIPortColumn(QtWidgets.QWidget):
    """A column representing a single SPI port with its clusters stacked vertically."""
    
    def __init__(self, port_config: SPIPortConfig, start_channel_idx: int, color: str, parent_visualizer):
        super().__init__()
        self.port_config = port_config
        self.start_channel_idx = start_channel_idx
        self.color = color
        self.parent_visualizer = parent_visualizer
        self.clusters = []
        
        # Map global channel indices to plots
        self.channel_to_plot: Dict[int, ChannelPlot] = {}
        
        self.setup_ui()
    
    def setup_ui(self):
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)
        
        # Port header with name and channel info
        header_text = f"{self.port_config.name}"
        subtitle_text = f"{self.port_config.num_devices} devices • {self.port_config.num_channels} ch"
        
        header_widget = QtWidgets.QWidget()
        header_layout = QtWidgets.QVBoxLayout()
        header_layout.setContentsMargins(12, 8, 12, 8)
        header_layout.setSpacing(2)
        
        header_label = QtWidgets.QLabel(header_text)
        header_label.setAlignment(QtCore.Qt.AlignCenter)
        header_label.setStyleSheet(f"""
            QLabel {{
                color: {self.color};
                font-size: 16pt;
                font-weight: bold;
            }}
        """)
        header_layout.addWidget(header_label)
        
        subtitle_label = QtWidgets.QLabel(subtitle_text)
        subtitle_label.setAlignment(QtCore.Qt.AlignCenter)
        subtitle_label.setStyleSheet(f"""
            QLabel {{
                color: #888;
                font-size: 10pt;
            }}
        """)
        header_layout.addWidget(subtitle_label)
        
        header_widget.setLayout(header_layout)
        header_widget.setStyleSheet(f"""
            QWidget {{
                background-color: #1a1a1a;
                border: 2px solid {self.color};
                border-radius: 8px;
            }}
        """)
        layout.addWidget(header_widget)
        
        # Scroll area for clusters
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        scroll_area.setMinimumWidth(420)
        
        self.scroll_widget = QtWidgets.QWidget()
        self.cluster_layout = QtWidgets.QVBoxLayout()
        self.cluster_layout.setSpacing(5)
        self.cluster_layout.setContentsMargins(2, 2, 2, 2)
        
        # Create clusters (one per daisy-chained device)
        current_channel_idx = self.start_channel_idx
        for cluster_idx in range(self.port_config.num_clusters):
            cluster = ClusterWidget(
                cluster_idx + 1,
                current_channel_idx,
                self.parent_visualizer.window_seconds,
                self.parent_visualizer.sample_rate,
                self.parent_visualizer,
                start_expanded=False
            )
            self.clusters.append(cluster)
            self.cluster_layout.addWidget(cluster)
            
            # Map channels to plots
            for plot in cluster.channel_plots:
                self.channel_to_plot[plot.global_channel_idx] = plot
            
            current_channel_idx += 8
        
        self.cluster_layout.addStretch()
        self.scroll_widget.setLayout(self.cluster_layout)
        scroll_area.setWidget(self.scroll_widget)
        layout.addWidget(scroll_area)
        
        self.setLayout(layout)
    
    def update_plots(self, current_time):
        for cluster in self.clusters:
            cluster.update_plots(current_time)


class HeadsetVisualizer(QtWidgets.QMainWindow):
    """Main window for the headset visualizer."""

    def __init__(self, host: str, port: int, spi_ports: List[SPIPortConfig],
                 window_seconds=5.0, update_rate_hz=30, corruption_enabled=True):
        super().__init__()

        self.host = host
        self.port = port
        self.spi_ports = spi_ports
        self.window_seconds = window_seconds
        self.update_rate_hz = update_rate_hz
        self.corruption_enabled = corruption_enabled
        self.sample_rate = 250

        # Thread-safe queue for incoming samples
        self.sample_queue = Queue(maxsize=500)

        # Calculate total channels
        self.total_channels = sum(p.num_channels for p in spi_ports)

        # Filter states
        self.cmf_enabled = False
        self.bandpass_enabled = False
        self.notch_enabled = False


        # Axis settings
        self.autoscale_enabled = True
        self.fixed_y_min = -5000
        self.fixed_y_max = 5000

        # CMF (global across all channels)
        self.cmf = GlobalCMF(self.total_channels, self.sample_rate)

        # Connection state
        self.sample_count = 0
        self.bytes_received = 0
        self.start_time = None
        self.latest_server_timestamp = 0  # Track latest timestamp from server
        self.server_timestamp_offset = None  # Offset to make server timestamps start from 0

        # Corruption tracking
        self.corruption_count = 0
        self.corruption_log = []  # (sample_num, port_name, device_num, channel_num, value)

        # Port columns
        self.port_columns: List[SPIPortColumn] = []
        
        # Global channel index to plot mapping
        self.channel_to_plot: Dict[int, ChannelPlot] = {}

        # Setup UI
        self.setup_ui()

        # Start network thread
        self.network_thread = NetworkThread(self, host=self.host, port=self.port)
        self.network_thread.start()

        # Start background ping thread (prints to console even if GUI freezes)
        self.ping_thread = PingThread(self.host, interval=5)
        self.ping_thread.start()

        # Start watchdog (prints main thread stack trace if frozen)
        self.watchdog = WatchdogThread(threading.main_thread().ident, timeout=3)
        self.watchdog.start()

        # Start update timer
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(int(1000 / self.update_rate_hz))

    def setup_ui(self):
        """Setup the main window UI."""
        self.setWindowTitle('HeadsetViz - EEG Visualizer')
        self.setGeometry(100, 100, 1800, 950)
        self.setStyleSheet(STYLESHEET)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QtWidgets.QVBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        central_widget.setLayout(main_layout)

        # Top control bar
        control_bar = self.create_control_bar()
        main_layout.addLayout(control_bar)

        # Horizontal scroll area for SPI port columns
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        scroll_area.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        self.ports_widget = QtWidgets.QWidget()
        self.ports_layout = QtWidgets.QHBoxLayout()
        self.ports_layout.setSpacing(15)
        self.ports_layout.setContentsMargins(5, 5, 5, 5)
        self.ports_widget.setLayout(self.ports_layout)

        # Create SPI port columns
        current_channel_idx = 0
        for idx, port_config in enumerate(self.spi_ports):
            color = PORT_COLORS[idx % len(PORT_COLORS)]
            port_col = SPIPortColumn(port_config, current_channel_idx, color, self)
            self.port_columns.append(port_col)
            self.ports_layout.addWidget(port_col)
            
            # Update global channel mapping
            self.channel_to_plot.update(port_col.channel_to_plot)
            
            current_channel_idx += port_config.num_channels

        self.ports_layout.addStretch()

        scroll_area.setWidget(self.ports_widget)
        main_layout.addWidget(scroll_area)

    def create_control_bar(self):
        """Create the top control bar with filter toggles and axis settings."""
        layout = QtWidgets.QHBoxLayout()
        layout.setSpacing(15)

        # Status label
        self.status_label = QtWidgets.QLabel('Connecting...')
        self.status_label.setStyleSheet(f"color: {ACCENT_COLOR}; font-size: 11pt; padding: 5px;")
        layout.addWidget(self.status_label, stretch=1)

        # Filter toggles
        filter_group = QtWidgets.QHBoxLayout()
        filter_group.setSpacing(8)
        
        filter_label = QtWidgets.QLabel("Filters:")
        filter_label.setStyleSheet("font-weight: bold;")
        filter_group.addWidget(filter_label)

        self.cmf_btn = self.create_toggle_button("CMF", self.cmf_enabled, self.toggle_cmf)
        filter_group.addWidget(self.cmf_btn)

        self.bandpass_btn = self.create_toggle_button("1-50Hz", self.bandpass_enabled, self.toggle_bandpass)
        filter_group.addWidget(self.bandpass_btn)

        self.notch_btn = self.create_toggle_button("60Hz Notch", self.notch_enabled, self.toggle_notch)
        filter_group.addWidget(self.notch_btn)

        layout.addLayout(filter_group)

        # Separator
        sep = QtWidgets.QFrame()
        sep.setFrameShape(QtWidgets.QFrame.VLine)
        sep.setStyleSheet("color: #444;")
        layout.addWidget(sep)

        # Axis settings
        axis_group = QtWidgets.QHBoxLayout()
        axis_group.setSpacing(8)

        self.autoscale_btn = self.create_toggle_button("Autoscale", self.autoscale_enabled, self.toggle_autoscale)
        axis_group.addWidget(self.autoscale_btn)

        axis_group.addWidget(QtWidgets.QLabel("Y Range:"))
        
        self.y_min_input = QtWidgets.QLineEdit(str(self.fixed_y_min))
        self.y_min_input.setFixedWidth(70)
        self.y_min_input.setPlaceholderText("Min")
        self.y_min_input.returnPressed.connect(self.update_fixed_range)
        axis_group.addWidget(self.y_min_input)

        axis_group.addWidget(QtWidgets.QLabel("to"))

        self.y_max_input = QtWidgets.QLineEdit(str(self.fixed_y_max))
        self.y_max_input.setFixedWidth(70)
        self.y_max_input.setPlaceholderText("Max")
        self.y_max_input.returnPressed.connect(self.update_fixed_range)
        axis_group.addWidget(self.y_max_input)

        apply_btn = QtWidgets.QPushButton("Apply")
        apply_btn.setFixedWidth(60)
        apply_btn.clicked.connect(self.update_fixed_range)
        axis_group.addWidget(apply_btn)

        layout.addLayout(axis_group)

        return layout

    def create_toggle_button(self, text, initial_state, callback):
        """Create a toggle button."""
        btn = QtWidgets.QPushButton(text)
        btn.setCheckable(True)
        btn.setChecked(initial_state)
        btn.clicked.connect(callback)
        return btn

    def toggle_cmf(self):
        self.cmf_enabled = self.cmf_btn.isChecked()
        print(f"CMF {'enabled' if self.cmf_enabled else 'disabled'}")

    def toggle_bandpass(self):
        self.bandpass_enabled = self.bandpass_btn.isChecked()
        print(f"Bandpass filter {'enabled' if self.bandpass_enabled else 'disabled'}")

    def toggle_notch(self):
        self.notch_enabled = self.notch_btn.isChecked()
        print(f"60Hz Notch filter {'enabled' if self.notch_enabled else 'disabled'}")

    def toggle_autoscale(self):
        self.autoscale_enabled = self.autoscale_btn.isChecked()
        self.y_min_input.setEnabled(not self.autoscale_enabled)
        self.y_max_input.setEnabled(not self.autoscale_enabled)
        print(f"Autoscale {'enabled' if self.autoscale_enabled else 'disabled'}")

    def update_fixed_range(self):
        """Update the fixed Y-axis range from input fields."""
        try:
            new_min = float(self.y_min_input.text())
            new_max = float(self.y_max_input.text())
            if new_min < new_max:
                self.fixed_y_min = new_min
                self.fixed_y_max = new_max
                print(f"Fixed Y range set to [{self.fixed_y_min}, {self.fixed_y_max}]")
            else:
                print("Error: Min must be less than Max")
        except ValueError:
            print("Error: Invalid number format")

    def process_sample(self, sample):
        """Process a received sample."""
        if self.start_time is None:
            self.start_time = time.time()

        # Use server's timestamp (when data was acquired), not client time (when received)
        # Offset so timestamps start from 0 when client connects
        raw_server_timestamp = sample['timestamp']
        if self.server_timestamp_offset is None:
            self.server_timestamp_offset = raw_server_timestamp

        server_timestamp = raw_server_timestamp - self.server_timestamp_offset
        self.latest_server_timestamp = server_timestamp  # Track for plot window
        channels = list(sample['channels'])  # Make a mutable copy

        # Corruption detection - any value outside normal EEG range is suspicious
        if self.corruption_enabled:
            CORRUPTION_THRESHOLD = 6000  # Values beyond ±5000 indicate corruption

            raw_channels = sample['channels']  # Use raw data for corruption check
            sample_num = sample.get('sample_number', self.sample_count)

            # Check each channel for out-of-range values (corruption)
            ch_idx = 0
            for port_idx, port_config in enumerate(self.spi_ports):
                for device_idx in range(port_config.num_devices):
                    for ch_in_device in range(8):
                        if ch_idx < len(raw_channels):
                            value = raw_channels[ch_idx]
                            if value > CORRUPTION_THRESHOLD or value < -CORRUPTION_THRESHOLD:
                                self.corruption_count += 1
                                event = (sample_num, port_config.name, device_idx + 1, ch_in_device + 1, value)
                                self.corruption_log.append(event)
                                print(f"CORRUPTION #{self.corruption_count}: Sample {sample_num}, "
                                      f"{port_config.name} Device {device_idx+1} Ch {ch_in_device+1}, "
                                      f"Value: {value}")
                        ch_idx += 1

        # Apply Global CMF if enabled
        if self.cmf_enabled and self.cmf is not None:
            channels, _ = self.cmf.remove_cmf(channels)

        # Clamp values to prevent extreme data from overwhelming pyqtgraph rendering
        CLAMP_LIMIT = 500000
        for i in range(len(channels)):
            if channels[i] > CLAMP_LIMIT:
                channels[i] = CLAMP_LIMIT
            elif channels[i] < -CLAMP_LIMIT:
                channels[i] = -CLAMP_LIMIT

        # Send to appropriate channel plots
        for ch_idx, value in enumerate(channels):
            if ch_idx in self.channel_to_plot:
                self.channel_to_plot[ch_idx].add_sample(server_timestamp, value)

        # Update stats
        self.sample_count += 1
        if self.sample_count % 250 == 0:
            elapsed = time.time() - self.start_time
            rate = self.sample_count / elapsed if elapsed > 0 else 0

            filters_active = []
            if self.cmf_enabled:
                filters_active.append("CMF")
            if self.bandpass_enabled:
                filters_active.append("BP")
            if self.notch_enabled:
                filters_active.append("Notch")
            filter_str = "+".join(filters_active) if filters_active else "None"

            port_summary = " | ".join([f"{p.name}({p.num_channels}ch)" for p in self.spi_ports])
            self.status_label.setText(
                f'{port_summary} | Filters: {filter_str} | '
                f'Rate: {rate:.1f} Hz | Corruptions: {self.corruption_count} | Time: {elapsed:.1f}s'
            )

    def update_plots(self):
        """Drain sample queue and update all plots."""
        self.watchdog.heartbeat()

        # Process pending samples (cap per tick to keep event loop responsive)
        drained = 0
        while drained < 30:
            try:
                sample = self.sample_queue.get_nowait()
                self.process_sample(sample)
                drained += 1
            except Empty:
                break

        if self.start_time is None or drained == 0:
            return

        current_time = self.latest_server_timestamp
        for port_col in self.port_columns:
            port_col.update_plots(current_time)

    def closeEvent(self, event):
        """Handle window close."""
        print("\nClosing visualizer...")

        self.watchdog.running = False
        self.ping_thread.stop()

        if self.network_thread.isRunning():
            self.network_thread.stop()
            self.network_thread.wait()

        if self.start_time:
            elapsed = time.time() - self.start_time
            print(f"\nStreamed {self.sample_count} samples in {elapsed:.2f}s")
            if elapsed > 0:
                print(f"Average rate: {self.sample_count/elapsed:.2f} Hz")
            print(f"Total data: {self.bytes_received/1e6:.2f} MB")

        event.accept()


class NetworkThread(QtCore.QThread):
    """Thread for network communication."""

    def __init__(self, visualizer, host, port):
        super().__init__()
        self.visualizer = visualizer
        self.host = host
        self.port = port
        self.running = True
        self.recv_buffer = b''
        self.socket = None
        self.format = None

    def run(self):
        """Thread main loop."""
        try:
            print(f"Connecting to {self.host}:{self.port}...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(1.0)
            self.socket.connect((self.host, self.port))
            print(f"Connected!")

            metadata_line = self._recv_line()
            metadata = json.loads(metadata_line)
            self.format = metadata['format']
            print(f"Stream format: {self.format}")
            print(f"Server reports: {metadata.get('num_channels', '?')} channels @ {metadata.get('sample_rate', '?')} Hz")


            while self.running:
                try:
                    sample = self.receive_sample()
                    if sample:
                        try:
                            self.visualizer.sample_queue.put_nowait(sample)
                        except Full:
                            try:
                                self.visualizer.sample_queue.get_nowait()
                            except Empty:
                                pass
                            self.visualizer.sample_queue.put_nowait(sample)
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"Connection lost: {e}")
                    break

        except Exception as e:
            print(f"Network error: {e}")
        finally:
            if self.socket:
                self.socket.close()

    def stop(self):
        """Stop the thread."""
        self.running = False

    def _recv_line(self):
        """Receive a line of text (for JSON format)."""
        while True:
            newline_pos = self.recv_buffer.find(b'\n')
            if newline_pos != -1:
                line = self.recv_buffer[:newline_pos]
                self.recv_buffer = self.recv_buffer[newline_pos + 1:]
                self.visualizer.bytes_received += len(line) + 1
                return line.decode('utf-8')

            try:
                chunk = self.socket.recv(4096)
                if not chunk:
                    raise ConnectionError("Connection closed")
                self.recv_buffer += chunk
            except socket.timeout:
                if len(self.recv_buffer) > 0:
                    continue
                raise

    def _recv_exact(self, n):
        """Receive exactly n bytes."""
        data = b''
        while len(data) < n:
            chunk = self.socket.recv(n - len(data))
            if not chunk:
                raise ConnectionError("Connection closed")
            data += chunk
        self.visualizer.bytes_received += len(data)
        return data

    def _recv_binary_sample(self):
        """Receive a binary sample."""
        size_bytes = self._recv_exact(4)
        packet_size = struct.unpack('<I', size_bytes)[0]
        packet = self._recv_exact(packet_size)

        offset = 0
        timestamp = struct.unpack('<d', packet[offset:offset+8])[0]
        offset += 8
        sample_number = struct.unpack('<I', packet[offset:offset+4])[0]
        offset += 4
        num_channels = struct.unpack('<H', packet[offset:offset+2])[0]
        offset += 2

        channels = []
        for _ in range(num_channels):
            ch = struct.unpack('<i', packet[offset:offset+4])[0]
            offset += 4
            channels.append(ch)

        return {
            'timestamp': timestamp,
            'sample_number': sample_number,
            'channels': channels
        }

    def receive_sample(self):
        """Receive one sample."""
        if self.format == 'json':
            line = self._recv_line()
            data = json.loads(line)
            if data['type'] == 'sample':
                return data
            return None
        else:
            return self._recv_binary_sample()


class WatchdogThread(threading.Thread):
    """Monitors main thread responsiveness. Prints stack trace on freeze."""

    def __init__(self, main_thread_id, timeout=3):
        super().__init__(daemon=True)
        self.main_thread_id = main_thread_id
        self.timeout = timeout
        self.last_heartbeat = time.time()
        self.running = True

    def heartbeat(self):
        self.last_heartbeat = time.time()

    def run(self):
        while self.running:
            time.sleep(1)
            elapsed = time.time() - self.last_heartbeat
            if elapsed > self.timeout:
                print(f"\n[WATCHDOG] Main thread frozen for {elapsed:.1f}s! Stack trace:")
                frames = sys._current_frames()
                if self.main_thread_id in frames:
                    traceback.print_stack(frames[self.main_thread_id])
                else:
                    print("[WATCHDOG] Could not get main thread frame")
                # Reset so we print again if still frozen after another timeout
                self.last_heartbeat = time.time()


class PingThread(QtCore.QThread):
    """Background thread that pings the server periodically during streaming."""

    def __init__(self, host, interval=5):
        super().__init__()
        self.host = host
        self.interval = interval
        self.running = True

    def run(self):
        """Ping server every N seconds, print result to console."""
        if platform.system() == "Windows":
            ping_cmd = ["ping", "-n", "1", "-w", "1000", self.host]
        else:
            ping_cmd = ["ping", "-c", "1", "-W", "1", self.host]

        while self.running:
            try:
                t0 = time.time()
                result = subprocess.run(ping_cmd, capture_output=True, text=True, timeout=5)
                elapsed = (time.time() - t0) * 1000
                if result.returncode == 0:
                    # Extract round-trip time
                    output = result.stdout
                    if "time=" in output:
                        # Parse "time=Xms" from output
                        rtt = output.split("time=")[-1].split("ms")[0].split("m")[0].strip()
                        print(f"[PING] {self.host}: {rtt}ms")
                    else:
                        print(f"[PING] {self.host}: OK ({elapsed:.0f}ms)")
                else:
                    print(f"[PING] {self.host}: FAILED - no response")
            except subprocess.TimeoutExpired:
                print(f"[PING] {self.host}: TIMEOUT")
            except Exception as e:
                print(f"[PING] {self.host}: ERROR - {e}")

            # Sleep in small increments so stop() is responsive
            for _ in range(self.interval * 10):
                if not self.running:
                    break
                time.sleep(0.1)

    def stop(self):
        self.running = False


def parse_spi_ports(ports_string: str) -> List[SPIPortConfig]:
    """
    Parse SPI ports string into configurations.
    
    Format: "Port1,3 Port2,5 Port3,5 Port4,5 Port5,3"
    Each entry is: name,num_devices
    """
    ports = []
    for entry in ports_string.split():
        parts = entry.split(',')
        if len(parts) != 2:
            raise ValueError(f"Invalid port format '{entry}'. Expected 'Name,NumDevices'")
        name = parts[0]
        num_devices = int(parts[1])
        ports.append(SPIPortConfig(name=name, num_devices=num_devices))
    return ports


def main():
    parser = argparse.ArgumentParser(
        description="HeadsetViz - EEG Headset Visualizer with SPI Port Organization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Default Configuration (matches HeadsetServer.py):
  Port1: 3 devices (24 channels)
  Port2: 5 devices (40 channels)
  Port3: 5 devices (40 channels)
  Port4: 5 devices (40 channels)
  Port5: 3 devices (24 channels)
  Total: 21 devices, 168 channels @ 250 Hz

Examples:
  # Use defaults (168 channels)
  python HeadsetViz.py --host 192.168.1.100
  
  # Custom SPI port configuration
  python HeadsetViz.py --host 192.168.1.100 --spi-ports "Port1,3 Port2,5"
  
  # Single port with 6 daisy-chained devices
  python HeadsetViz.py --host 192.168.1.100 --spi-ports "MainPort,6"

SPI Port Format: "Name,NumDevices Name2,NumDevices2 ..."
  - Name: Display name for the SPI port (e.g., Port1, FrontHead)
  - NumDevices: Number of ADS1299 devices daisy-chained on this SPI bus
  - Each device has 8 channels, displayed as one cluster
        """
    )

    parser.add_argument("--host", type=str, required=True,
                       help="IP address of the streaming server")
    parser.add_argument("--port", type=int, default=8888,
                       help="Network port of the streaming server (default: 8888)")
    parser.add_argument("--spi-ports", type=str, default=DEFAULT_SPI_PORTS,
                       help=f"SPI port configuration (default: '{DEFAULT_SPI_PORTS}')")
    parser.add_argument("--window", type=float, default=5.0,
                       help="Time window to display in seconds (default: 5.0)")
    parser.add_argument("--update-rate", type=float, default=30,
                       help="Visualization update rate in Hz (default: 30)")
    parser.add_argument("--corruption", choices=["on", "off"], default="on",
                       help="Enable or disable corruption detection (default: on)")

    args = parser.parse_args()

    # Parse SPI port configuration
    try:
        spi_ports = parse_spi_ports(args.spi_ports)
    except ValueError as e:
        print(f"Error parsing SPI ports: {e}")
        sys.exit(1)

    # Calculate totals
    total_devices = sum(p.num_devices for p in spi_ports)
    total_channels = sum(p.num_channels for p in spi_ports)
    total_clusters = sum(p.num_clusters for p in spi_ports)

    # Create Qt application
    app = QtWidgets.QApplication(sys.argv)

    # Create and show visualizer
    visualizer = HeadsetVisualizer(
        args.host,
        args.port,
        spi_ports,
        window_seconds=args.window,
        update_rate_hz=args.update_rate,
        corruption_enabled=(args.corruption == 'on')
    )
    visualizer.show()

    print("\n" + "=" * 70)
    print("HeadsetViz - EEG Visualizer")
    print("=" * 70)
    print(f"Server: {args.host}:{args.port}")
    print(f"\nSPI Port Configuration:")
    channel_offset = 0
    for p in spi_ports:
        print(f"  {p.name}: {p.num_devices} devices, {p.num_channels} channels (ch {channel_offset+1}-{channel_offset+p.num_channels})")
        channel_offset += p.num_channels
    print(f"\nTotal: {total_devices} devices, {total_channels} channels, {total_clusters} clusters")
    print("\nFeatures:")
    print("  • SPI ports displayed horizontally")
    print("  • Clusters (8 ch each) stacked under each port (collapsed by default)")
    print("  • Toggleable filters: CMF, Bandpass (1-50Hz), 60Hz Notch")
    print("  • Adjustable Y-axis range (disable Autoscale first)")
    print("=" * 70 + "\n")

    # Run application
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
