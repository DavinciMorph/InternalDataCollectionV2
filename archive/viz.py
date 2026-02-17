#!/usr/bin/env python3
"""
viz.py — Full-Screen EEG Visualizer (matplotlib + tkinter)

Single matplotlib Figure with GridSpec(8, N_ports) embedded in tkinter.
One column per SPI port, 8 channel rows. Device dropdown per port selects
which ADS1299 cluster is displayed. All channels are always buffered so
dropdown switches are instant.

Usage:
    python viz.py --host 192.168.1.99 --spi-ports "Port1,1"
    python viz.py --host 192.168.1.99 --spi-ports "Port1,1 Port2,5"
"""

import argparse
import collections
import json
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass
from queue import Empty, Full, Queue
from typing import List, Optional

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.gridspec import GridSpec
import numpy as np
from scipy import signal

# ── Constants ────────────────────────────────────────────────────────────────

DEFAULT_HOST = "192.168.1.99"
DEFAULT_PORT = 8888
DEFAULT_SPI_PORTS = "Port1,1 Port2,5 Port3,5 Port4,5 Port5,3"
DEFAULT_WINDOW = 5.0
SAMPLE_RATE = 250
UPDATE_INTERVAL_MS = 66       # ~15 fps
QUEUE_MAXSIZE = 500
STATUS_INTERVAL = 1.0
DIAG_INTERVAL = 5.0           # diagnostics print interval
RECV_STALL_TIMEOUT = 10.0     # force reconnect if no data for this long

# ── Colors (from HeadsetViz.py) ──────────────────────────────────────────────

DARK_BG = "#0d0d0d"
PANEL_BG = "#1a1a1a"
PLOT_BG = "#0a0a0a"
TEXT_COLOR = "#e0e0e0"
ACCENT_COLOR = "#00ff88"

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

TOGGLE_ON_BG = "#00aa55"
TOGGLE_OFF_BG = "#2a2a2a"

# ── SPI Port Config ──────────────────────────────────────────────────────────

@dataclass
class SPIPortConfig:
    name: str
    num_devices: int

    @property
    def num_channels(self) -> int:
        return self.num_devices * 8


def parse_spi_ports(spec: str) -> List[SPIPortConfig]:
    """Parse 'Port1,1 Port2,5' → list of SPIPortConfig."""
    ports = []
    for token in spec.strip().split():
        name, count = token.split(",")
        ports.append(SPIPortConfig(name=name, num_devices=int(count)))
    return ports


# ── Filters ──────────────────────────────────────────────────────────────────

class Filters:
    """IIR filter coefficients (shared); per-channel state stored separately."""

    def __init__(self, sample_rate: int = SAMPLE_RATE):
        # 60 Hz notch
        self.notch_b, self.notch_a = signal.iirnotch(60.0, 30.0, sample_rate)
        self.notch_zi_template = signal.lfilter_zi(self.notch_b, self.notch_a)

        # 1-50 Hz bandpass (order 4 Butterworth)
        nyq = sample_rate / 2.0
        self.bp_b, self.bp_a = signal.butter(4, [1.0 / nyq, 50.0 / nyq], btype="bandpass")
        self.bp_zi_template = signal.lfilter_zi(self.bp_b, self.bp_a)

    @staticmethod
    def make_channel_state() -> dict:
        return {"notch_zi": None, "bp_zi": None}


# ── Network Thread ───────────────────────────────────────────────────────────

class NetworkThread(threading.Thread):
    """Reads JSON samples from TCP → Queue (drop-oldest on full)."""

    def __init__(self, host: str, port: int, queue: Queue):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.queue = queue
        self.running = True
        self.metadata: Optional[dict] = None
        self.connected = False
        self._buf = b""

        # ── Diagnostics ──────────────────────────────────────────────────
        self.samples_queued = 0        # total samples pushed to queue
        self.queue_drops = 0           # times we had to drop oldest
        self.last_recv_time = 0.0      # wall time of last successful recv
        self.non_sample_count = 0      # lines that weren't type=sample
        self.json_errors = 0           # JSON parse failures
        self.reconnect_count = 0       # number of reconnection attempts

    def _recv_line(self, sock: socket.socket) -> str:
        while True:
            pos = self._buf.find(b"\n")
            if pos != -1:
                line = self._buf[:pos]
                self._buf = self._buf[pos + 1:]
                return line.decode("utf-8")
            chunk = sock.recv(8192)
            if not chunk:
                raise ConnectionError("Connection closed by server")
            self._buf += chunk

    def run(self):
        while self.running:
            sock = None
            try:
                self.reconnect_count += 1
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                print(f"[net] Connecting to {self.host}:{self.port} … "
                      f"(attempt #{self.reconnect_count})")
                sock.connect((self.host, self.port))
                self.connected = True
                print("[net] Connected")
                self._buf = b""

                meta_line = self._recv_line(sock)
                self.metadata = json.loads(meta_line)
                print(f"[net] Metadata: {self.metadata}")
                self.last_recv_time = time.time()

                while self.running:
                    try:
                        line = self._recv_line(sock)
                    except socket.timeout:
                        # Check for server-side stall
                        silent = time.time() - self.last_recv_time
                        if silent > RECV_STALL_TIMEOUT:
                            raise ConnectionError(
                                f"Server silent for {silent:.1f}s — forcing reconnect")
                        continue
                    self.last_recv_time = time.time()
                    try:
                        data = json.loads(line)
                    except json.JSONDecodeError:
                        self.json_errors += 1
                        continue
                    if data.get("type") != "sample":
                        self.non_sample_count += 1
                        continue
                    try:
                        self.queue.put_nowait(data)
                    except Full:
                        try:
                            self.queue.get_nowait()
                        except Empty:
                            pass
                        self.queue.put_nowait(data)
                        self.queue_drops += 1
                    self.samples_queued += 1

            except Exception as exc:
                self.connected = False
                if self.running:
                    print(f"[net] Error: {type(exc).__name__}: {exc}  "
                          f"— retrying in 2 s")
                    time.sleep(2)
            finally:
                self.connected = False
                if sock:
                    try:
                        sock.close()
                    except OSError:
                        pass

    def stop(self):
        self.running = False


# ── EEG Visualizer ───────────────────────────────────────────────────────────

class EEGVisualizer:
    """Full-screen EEG visualizer — single matplotlib Figure, GridSpec layout."""

    def __init__(self, args):
        self.host = args.host
        self.port = args.port
        self.spi_ports = parse_spi_ports(args.spi_ports)
        self.window_sec = args.window
        self.sample_rate = SAMPLE_RATE
        self.n_ports = len(self.spi_ports)

        self.total_channels = sum(p.num_channels for p in self.spi_ports)

        # Per-port channel offset (global index of first channel in each port)
        self.port_ch_offset: List[int] = []
        offset = 0
        for p in self.spi_ports:
            self.port_ch_offset.append(offset)
            offset += p.num_channels

        # Selected device index per port (0-based)
        self.selected_device: List[int] = [0] * self.n_ports

        # Data buffers — one deque per global channel
        maxlen = self.sample_rate * int(self.window_sec)
        self.buf_maxlen = maxlen
        self.time_buf: collections.deque = collections.deque(maxlen=maxlen)
        self.ch_bufs: List[collections.deque] = [
            collections.deque(maxlen=maxlen) for _ in range(self.total_channels)
        ]

        # Filters
        self.filters = Filters(self.sample_rate)
        self.ch_filter_state = [Filters.make_channel_state()
                                for _ in range(self.total_channels)]
        self.notch_enabled = False
        self.bandpass_enabled = False
        self.cmf_enabled = False

        # Y-axis
        self.autoscale_enabled = True
        self.fixed_y_min = -5000.0
        self.fixed_y_max = 5000.0

        # Queue + network
        self.queue: Queue = Queue(maxsize=QUEUE_MAXSIZE)
        self.net = NetworkThread(self.host, self.port, self.queue)

        # Stats
        self.samples_received = 0
        self.start_time = time.time()
        self.last_status_time = 0.0

        # ── Diagnostics state ────────────────────────────────────────────
        self.last_diag_time = 0.0
        self.diag_samples_prev = 0          # samples_received at last diag
        self.diag_frame_count = 0           # _update() calls since last diag
        self.diag_draw_count = 0            # successful draws since last diag
        self.diag_draw_total_ms = 0.0       # cumulative draw time
        self.diag_draw_max_ms = 0.0         # worst draw time this interval
        self.diag_skip_count = 0            # channels skipped (len mismatch)
        self.diag_zero_samples = 0          # samples where ALL channels == 0
        self.diag_null_channels = 0         # individual channel values that are exactly 0.0
        self.diag_last_sample_time = 0.0    # wall time when last sample was processed
        self.diag_stall_warned = False       # whether we've printed a stall warning
        self.diag_drain_counts: List[int] = []  # how many samples drained each frame
        self.diag_rate_history: List[float] = []  # instantaneous Hz per diag interval

        # Build UI
        self._build_ui()

    # ── UI construction ──────────────────────────────────────────────────

    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("viz.py \u2014 Full-Screen EEG Visualizer")
        self.root.configure(bg=DARK_BG)
        self.root.state("zoomed")

        # Dark ttk style
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Vertical.TScrollbar",
                        background="#333333", troughcolor=DARK_BG,
                        arrowcolor=TEXT_COLOR)
        style.configure("Dark.TCombobox",
                        fieldbackground=PANEL_BG, background=PANEL_BG,
                        foreground=TEXT_COLOR, arrowcolor=TEXT_COLOR,
                        selectbackground=PANEL_BG, selectforeground=TEXT_COLOR)
        style.map("Dark.TCombobox",
                  fieldbackground=[("readonly", PANEL_BG)],
                  foreground=[("readonly", TEXT_COLOR)])

        self._build_control_bar()
        self._build_figure()

        # Keyboard bindings
        self.root.bind("<Key>", self._on_key)
        self.root.protocol("WM_DELETE_WINDOW", self.stop)

    def _build_control_bar(self):
        bar = tk.Frame(self.root, bg=PANEL_BG)
        bar.pack(fill="x", padx=0, pady=0)

        # ── Status label (left) ──────────────────────────────────────────
        self.status_var = tk.StringVar(value="Connecting\u2026")
        tk.Label(bar, textvariable=self.status_var,
                 font=("Consolas", 10), fg=ACCENT_COLOR, bg=PANEL_BG,
                 anchor="w").pack(side="left", padx=10, pady=6)

        # ── Separator ────────────────────────────────────────────────────
        tk.Frame(bar, bg="#444444", width=1).pack(side="left", fill="y",
                                                   padx=8, pady=4)

        # ── Filter toggles ───────────────────────────────────────────────
        self.cmf_btn = self._make_toggle(bar, "CMF", self.cmf_enabled,
                                          lambda: self._toggle_filter("cmf"))
        self.cmf_btn.pack(side="left", padx=3, pady=4)

        self.bp_btn = self._make_toggle(bar, "1-50Hz", self.bandpass_enabled,
                                         lambda: self._toggle_filter("bp"))
        self.bp_btn.pack(side="left", padx=3, pady=4)

        self.notch_btn = self._make_toggle(bar, "60Hz Notch", self.notch_enabled,
                                            lambda: self._toggle_filter("notch"))
        self.notch_btn.pack(side="left", padx=3, pady=4)

        # ── Separator ────────────────────────────────────────────────────
        tk.Frame(bar, bg="#444444", width=1).pack(side="left", fill="y",
                                                   padx=8, pady=4)

        # ── Autoscale toggle ─────────────────────────────────────────────
        self.autoscale_btn = self._make_toggle(
            bar, "Autoscale", self.autoscale_enabled, self._toggle_autoscale)
        self.autoscale_btn.pack(side="left", padx=3, pady=4)

        # ── Y Range ──────────────────────────────────────────────────────
        tk.Label(bar, text="Y:", font=("Consolas", 10),
                 fg=TEXT_COLOR, bg=PANEL_BG).pack(side="left", padx=(8, 2))

        self.y_min_entry = tk.Entry(bar, width=7, font=("Consolas", 10),
                                     bg="#222222", fg=TEXT_COLOR,
                                     insertbackground=TEXT_COLOR,
                                     relief="flat", bd=2)
        self.y_min_entry.insert(0, str(int(self.fixed_y_min)))
        self.y_min_entry.pack(side="left", padx=2, pady=4)

        tk.Label(bar, text="to", font=("Consolas", 10),
                 fg=TEXT_COLOR, bg=PANEL_BG).pack(side="left", padx=2)

        self.y_max_entry = tk.Entry(bar, width=7, font=("Consolas", 10),
                                     bg="#222222", fg=TEXT_COLOR,
                                     insertbackground=TEXT_COLOR,
                                     relief="flat", bd=2)
        self.y_max_entry.insert(0, str(int(self.fixed_y_max)))
        self.y_max_entry.pack(side="left", padx=2, pady=4)

        apply_btn = tk.Button(bar, text="Apply", font=("Consolas", 9),
                               bg="#333333", fg=TEXT_COLOR,
                               activebackground="#444444",
                               activeforeground=TEXT_COLOR,
                               relief="flat", bd=1, padx=8,
                               command=self._apply_y_range)
        apply_btn.pack(side="left", padx=4, pady=4)
        self.y_min_entry.bind("<Return>", lambda _: self._apply_y_range())
        self.y_max_entry.bind("<Return>", lambda _: self._apply_y_range())

        # ── Separator ────────────────────────────────────────────────────
        tk.Frame(bar, bg="#444444", width=1).pack(side="left", fill="y",
                                                   padx=8, pady=4)

        # ── Per-port device dropdowns (only for ports with >1 device) ────
        self.device_combos: List[Optional[ttk.Combobox]] = []
        for port_idx, port_config in enumerate(self.spi_ports):
            if port_config.num_devices > 1:
                tk.Label(bar, text=f"{port_config.name}:",
                         font=("Consolas", 9), fg=PORT_COLORS[port_idx % len(PORT_COLORS)],
                         bg=PANEL_BG).pack(side="left", padx=(6, 2), pady=4)

                values = [f"Dev {d+1}" for d in range(port_config.num_devices)]
                combo = ttk.Combobox(bar, values=values, width=6,
                                      state="readonly", style="Dark.TCombobox",
                                      font=("Consolas", 9))
                combo.current(0)
                combo.pack(side="left", padx=2, pady=4)
                combo.bind("<<ComboboxSelected>>",
                           lambda _e, idx=port_idx: self._on_device_change(idx))
                self.device_combos.append(combo)
            else:
                self.device_combos.append(None)

    def _make_toggle(self, parent, text, initial_state, command):
        bg = TOGGLE_ON_BG if initial_state else TOGGLE_OFF_BG
        fg = "white" if initial_state else TEXT_COLOR
        btn = tk.Button(parent, text=text, font=("Consolas", 9, "bold"),
                        bg=bg, fg=fg,
                        activebackground=bg, activeforeground=fg,
                        relief="flat", bd=1, padx=10, pady=2,
                        command=command)
        return btn

    def _set_toggle_state(self, btn, on):
        if on:
            btn.config(bg=TOGGLE_ON_BG, fg="white",
                       activebackground=TOGGLE_ON_BG, activeforeground="white")
        else:
            btn.config(bg=TOGGLE_OFF_BG, fg=TEXT_COLOR,
                       activebackground=TOGGLE_OFF_BG, activeforeground=TEXT_COLOR)

    def _build_figure(self):
        # Frame to hold the matplotlib canvas
        self.fig_frame = tk.Frame(self.root, bg=DARK_BG)
        self.fig_frame.pack(fill="both", expand=True)

        self.fig = Figure(facecolor=PLOT_BG)
        self.fig.subplots_adjust(hspace=0.3, wspace=0.25,
                                  left=0.04, right=0.99,
                                  top=0.97, bottom=0.02)

        gs = GridSpec(8, self.n_ports, figure=self.fig)

        # axes[port_idx][ch] and lines[port_idx][ch]
        self.axes: List[List] = []
        self.lines: List[List] = []

        for port_idx in range(self.n_ports):
            port_axes = []
            port_lines = []
            port_color = PORT_COLORS[port_idx % len(PORT_COLORS)]
            port_name = self.spi_ports[port_idx].name

            for ch in range(8):
                ax = self.fig.add_subplot(gs[ch, port_idx])
                ax.set_facecolor(PLOT_BG)

                color = NEON_COLORS[ch % len(NEON_COLORS)]
                (line,) = ax.plot([], [], linewidth=0.8, color=color)

                # Channel label on Y axis
                ax.set_ylabel(f"Ch {ch+1}", fontsize=8, color=color,
                              rotation=0, labelpad=28)
                ax.tick_params(labelsize=6, colors="#888888")
                ax.set_xlim(0, self.window_sec)
                ax.set_ylim(self.fixed_y_min, self.fixed_y_max)
                ax.grid(True, alpha=0.2, color="#3c3c3c")
                for spine in ax.spines.values():
                    spine.set_color("#333333")

                # Port name as column title (top row only)
                if ch == 0:
                    ax.set_title(port_name, fontsize=11, fontweight="bold",
                                 color=port_color, pad=6)

                # Hide x tick labels except bottom row
                if ch < 7:
                    ax.set_xticklabels([])

                port_axes.append(ax)
                port_lines.append(line)

            self.axes.append(port_axes)
            self.lines.append(port_lines)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.fig_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ── Device dropdown ──────────────────────────────────────────────────

    def _on_device_change(self, port_idx):
        combo = self.device_combos[port_idx]
        if combo is not None:
            self.selected_device[port_idx] = combo.current()
            print(f"[ui] {self.spi_ports[port_idx].name}: "
                  f"switched to Dev {self.selected_device[port_idx]+1}")

    def _get_displayed_channel(self, port_idx, ch):
        """Global channel index for a displayed subplot."""
        return (self.port_ch_offset[port_idx]
                + self.selected_device[port_idx] * 8 + ch)

    # ── Filter toggles ───────────────────────────────────────────────────

    def _toggle_filter(self, which):
        if which == "cmf":
            self.cmf_enabled = not self.cmf_enabled
            self._set_toggle_state(self.cmf_btn, self.cmf_enabled)
            print(f"[filter] CMF: {'ON' if self.cmf_enabled else 'OFF'}")
        elif which == "bp":
            self.bandpass_enabled = not self.bandpass_enabled
            self._reset_filter_states()
            self._set_toggle_state(self.bp_btn, self.bandpass_enabled)
            print(f"[filter] Bandpass 1-50Hz: {'ON' if self.bandpass_enabled else 'OFF'}")
        elif which == "notch":
            self.notch_enabled = not self.notch_enabled
            self._reset_filter_states()
            self._set_toggle_state(self.notch_btn, self.notch_enabled)
            print(f"[filter] Notch 60Hz: {'ON' if self.notch_enabled else 'OFF'}")

    def _toggle_autoscale(self):
        self.autoscale_enabled = not self.autoscale_enabled
        self._set_toggle_state(self.autoscale_btn, self.autoscale_enabled)
        print(f"[ui] Autoscale: {'ON' if self.autoscale_enabled else 'OFF'}")

    def _apply_y_range(self):
        try:
            new_min = float(self.y_min_entry.get())
            new_max = float(self.y_max_entry.get())
            if new_min < new_max:
                self.fixed_y_min = new_min
                self.fixed_y_max = new_max
                print(f"[ui] Y range: [{self.fixed_y_min}, {self.fixed_y_max}]")
            else:
                print("[ui] Error: min must be less than max")
        except ValueError:
            print("[ui] Error: invalid number format")

    def _reset_filter_states(self):
        self.ch_filter_state = [Filters.make_channel_state()
                                for _ in range(self.total_channels)]

    # ── Keyboard ─────────────────────────────────────────────────────────

    def _on_key(self, event):
        key = event.char.lower() if event.char else ""
        if key == "n":
            self._toggle_filter("notch")
        elif key == "b":
            self._toggle_filter("bp")
        elif key == "c":
            self._toggle_filter("cmf")
        elif key == "q":
            self.stop()

    # ── Sample processing ────────────────────────────────────────────────

    def _process_sample(self, sample: dict):
        channels = sample["channels"]
        ts = sample.get("timestamp", 0.0)

        # Pad if fewer channels than expected
        while len(channels) < self.total_channels:
            channels.append(0.0)

        # ── Diagnostics: detect null/zero data ───────────────────────────
        raw_slice = channels[:self.total_channels]
        zero_count = sum(1 for v in raw_slice if v == 0.0)
        self.diag_null_channels += zero_count
        if zero_count == self.total_channels:
            self.diag_zero_samples += 1

        # CMF (subtract global mean)
        if self.cmf_enabled:
            mean_val = float(np.mean(raw_slice))
            channels = [v - mean_val for v in channels]

        # Per-channel filter + buffer (ALL channels, not just displayed)
        for i in range(self.total_channels):
            val = float(channels[i])
            state = self.ch_filter_state[i]

            if self.notch_enabled:
                zi = state["notch_zi"]
                if zi is None:
                    zi = self.filters.notch_zi_template * val
                out, zi = signal.lfilter(
                    self.filters.notch_b, self.filters.notch_a, [val], zi=zi)
                val = out[0]
                state["notch_zi"] = zi

            if self.bandpass_enabled:
                zi = state["bp_zi"]
                if zi is None:
                    zi = self.filters.bp_zi_template * val
                out, zi = signal.lfilter(
                    self.filters.bp_b, self.filters.bp_a, [val], zi=zi)
                val = out[0]
                state["bp_zi"] = zi

            self.ch_bufs[i].append(val)

        self.time_buf.append(ts)
        self.samples_received += 1
        self.diag_last_sample_time = time.time()

    # ── Diagnostics ──────────────────────────────────────────────────────

    def _print_diagnostics(self):
        now = time.time()
        dt = now - self.last_diag_time if self.last_diag_time > 0 else DIAG_INTERVAL
        elapsed = now - self.start_time

        samples_this = self.samples_received - self.diag_samples_prev
        inst_rate = samples_this / dt if dt > 0 else 0
        self.diag_rate_history.append(inst_rate)
        if len(self.diag_rate_history) > 12:
            self.diag_rate_history.pop(0)

        # Rate trend detection
        trend = ""
        if len(self.diag_rate_history) >= 3:
            recent = self.diag_rate_history[-3:]
            if recent[-1] < recent[0] * 0.5 and recent[0] > 10:
                trend = " *** RATE DROPPING ***"
            elif recent[-1] == 0 and recent[0] > 0:
                trend = " *** DATA STOPPED ***"

        # Stall detection
        stall_sec = now - self.diag_last_sample_time if self.diag_last_sample_time > 0 else 0
        stall_warn = ""
        if stall_sec > 3.0 and self.samples_received > 0:
            stall_warn = f" *** STALL: no sample for {stall_sec:.1f}s ***"

        # Avg/max draw time
        avg_draw = (self.diag_draw_total_ms / self.diag_draw_count
                    if self.diag_draw_count > 0 else 0)

        # Drain stats
        avg_drain = (sum(self.diag_drain_counts) / len(self.diag_drain_counts)
                     if self.diag_drain_counts else 0)
        max_drain = max(self.diag_drain_counts) if self.diag_drain_counts else 0
        empty_drains = sum(1 for d in self.diag_drain_counts if d == 0)

        # Network thread state
        net_alive = self.net.is_alive()
        net_ago = now - self.net.last_recv_time if self.net.last_recv_time > 0 else -1

        # Buffer lengths
        time_len = len(self.time_buf)
        ch_lens = [len(self.ch_bufs[i]) for i in range(self.total_channels)]
        ch_len_min = min(ch_lens) if ch_lens else 0
        ch_len_max = max(ch_lens) if ch_lens else 0
        len_mismatch = "MISMATCH" if ch_len_min != time_len or ch_len_max != time_len else "ok"

        print(f"\n{'='*80}")
        print(f"[DIAG] t={elapsed:.0f}s | interval={dt:.1f}s")
        print(f"  FLOW: {samples_this} samples ({inst_rate:.1f} Hz) "
              f"| total={self.samples_received} "
              f"| avg={self.samples_received/elapsed:.1f} Hz{trend}{stall_warn}")
        print(f"  NET:  thread={'ALIVE' if net_alive else 'DEAD'} "
              f"| conn={self.net.connected} "
              f"| queued={self.net.samples_queued} "
              f"| drops={self.net.queue_drops} "
              f"| json_err={self.net.json_errors} "
              f"| non_sample={self.net.non_sample_count} "
              f"| reconnects={self.net.reconnect_count}"
              f"| last_recv={net_ago:.1f}s ago" if net_ago >= 0 else "| last_recv=never")
        print(f"  QUEUE: depth={self.queue.qsize()}/{QUEUE_MAXSIZE}")
        print(f"  DRAW: frames={self.diag_frame_count} "
              f"| draws={self.diag_draw_count} "
              f"| avg={avg_draw:.1f}ms "
              f"| max={self.diag_draw_max_ms:.1f}ms "
              f"| skipped_ch={self.diag_skip_count}")
        print(f"  DRAIN: avg={avg_drain:.1f}/frame "
              f"| max={max_drain} "
              f"| empty_frames={empty_drains}/{self.diag_frame_count}")
        print(f"  BUFS: time_buf={time_len} "
              f"| ch_range=[{ch_len_min}..{ch_len_max}] "
              f"| maxlen={self.buf_maxlen} "
              f"| sync={len_mismatch}")
        print(f"  ZEROS: all_zero_samples={self.diag_zero_samples} "
              f"| null_ch_values={self.diag_null_channels} "
              f"(of {samples_this * self.total_channels} total)")

        # ── Channel data snapshot ─────────────────────────────────────────
        if self.time_buf:
            print(f"  SNAPSHOT (latest value per displayed channel):")
            for port_idx in range(self.n_ports):
                port_name = self.spi_ports[port_idx].name
                dev = self.selected_device[port_idx]
                for ch in range(8):
                    global_ch = self._get_displayed_channel(port_idx, ch)
                    if global_ch >= self.total_channels:
                        continue
                    buf = self.ch_bufs[global_ch]
                    val = buf[-1] if buf else 0.0
                    print(f"    {port_name} D{dev+1} Ch{ch+1} "
                          f"(g={global_ch:>3d}): {val:>14.2f}")
                if port_idx == 0:
                    break  # only snapshot first port to keep output readable
            # Timestamp
            ts = self.time_buf[-1]
            print(f"    last_ts={ts:.4f}s")

        print(f"{'='*80}\n")

        # Reset interval counters
        self.last_diag_time = now
        self.diag_samples_prev = self.samples_received
        self.diag_frame_count = 0
        self.diag_draw_count = 0
        self.diag_draw_total_ms = 0.0
        self.diag_draw_max_ms = 0.0
        self.diag_skip_count = 0
        self.diag_zero_samples = 0
        self.diag_null_channels = 0
        self.diag_drain_counts.clear()

    # ── Periodic update ──────────────────────────────────────────────────

    def _update(self):
        self.diag_frame_count += 1

        # Drain queue (up to 500 samples)
        count = 0
        while count < 500:
            try:
                sample = self.queue.get_nowait()
            except Empty:
                break
            self._process_sample(sample)
            count += 1
        self.diag_drain_counts.append(count)

        # Stall detection — real-time warning (only once per stall)
        now = time.time()
        if (self.diag_last_sample_time > 0
                and now - self.diag_last_sample_time > 3.0
                and not self.diag_stall_warned
                and self.samples_received > 0):
            stall_dur = now - self.diag_last_sample_time
            print(f"[WARN] Data stall detected: no new samples for {stall_dur:.1f}s "
                  f"| net.connected={self.net.connected} "
                  f"| net.alive={self.net.is_alive()} "
                  f"| queue={self.queue.qsize()}")
            self.diag_stall_warned = True
        elif self.diag_last_sample_time > 0 and now - self.diag_last_sample_time < 1.0:
            self.diag_stall_warned = False

        # Update plots if we have data
        if self.time_buf:
            t = np.array(self.time_buf)
            n_pts = len(t)
            t_max = t[-1]
            t_min = t_max - self.window_sec

            for port_idx in range(self.n_ports):
                for ch in range(8):
                    global_ch = self._get_displayed_channel(port_idx, ch)
                    if global_ch >= self.total_channels:
                        continue

                    line = self.lines[port_idx][ch]
                    ax = self.axes[port_idx][ch]
                    y_deque = self.ch_bufs[global_ch]

                    if len(y_deque) != n_pts:
                        self.diag_skip_count += 1
                        continue

                    y = np.array(y_deque)
                    line.set_data(t, y)
                    ax.set_xlim(t_min, t_max)

                    # Y-axis limits
                    if self.autoscale_enabled:
                        mask = t >= t_min
                        visible = y[mask]
                        if len(visible) > 0:
                            ymin = float(np.min(visible))
                            ymax = float(np.max(visible))
                            margin = max(abs(ymax - ymin) * 0.1, 10)
                            ax.set_ylim(ymin - margin, ymax + margin)
                    else:
                        ax.set_ylim(self.fixed_y_min, self.fixed_y_max)

            draw_start = time.perf_counter()
            try:
                self.canvas.draw()
                draw_ms = (time.perf_counter() - draw_start) * 1000
                self.diag_draw_count += 1
                self.diag_draw_total_ms += draw_ms
                if draw_ms > self.diag_draw_max_ms:
                    self.diag_draw_max_ms = draw_ms
            except Exception as exc:
                draw_ms = (time.perf_counter() - draw_start) * 1000
                print(f"[viz] Draw FAILED after {draw_ms:.1f}ms: "
                      f"{type(exc).__name__}: {exc}")

        # Status label (~1 second interval)
        if now - self.last_status_time >= STATUS_INTERVAL:
            elapsed = now - self.start_time
            rate = self.samples_received / elapsed if elapsed > 0 else 0
            conn = "Connected" if self.net.connected else "Disconnected"
            alive = "" if self.net.is_alive() else " [THREAD DEAD]"

            port_summary = " | ".join(
                f"{p.name}({p.num_channels}ch)" for p in self.spi_ports)

            filt_parts = []
            if self.cmf_enabled:
                filt_parts.append("CMF")
            if self.bandpass_enabled:
                filt_parts.append("BP")
            if self.notch_enabled:
                filt_parts.append("Notch")
            filt_str = ", ".join(filt_parts) if filt_parts else "None"

            self.status_var.set(
                f"{conn}{alive} | {port_summary} | Filters: {filt_str} | "
                f"Rate: {rate:.1f} Hz | Q: {self.queue.qsize()} | "
                f"Time: {elapsed:.0f}s")
            self.last_status_time = now

        # Diagnostics print
        if now - self.last_diag_time >= DIAG_INTERVAL:
            self._print_diagnostics()

        # Schedule next frame
        self.root.after(UPDATE_INTERVAL_MS, self._update)

    # ── Run / Stop ───────────────────────────────────────────────────────

    def run(self):
        self.net.start()
        self.last_diag_time = time.time()
        self.root.after(UPDATE_INTERVAL_MS, self._update)
        print("[viz] Window open — press n/b/c to toggle filters, q to quit")
        print(f"[viz] Diagnostics printing every {DIAG_INTERVAL:.0f}s")
        self.root.mainloop()

    def stop(self):
        self.net.stop()
        print("\n[viz] Final diagnostics:")
        self._print_diagnostics()
        try:
            self.root.destroy()
        except tk.TclError:
            pass
        print("[viz] Stopped")


# ── CLI ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Full-Screen EEG Visualizer (matplotlib + tkinter)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            '  python viz.py --host 192.168.1.99 --spi-ports "Port1,1"\n'
            '  python viz.py --host 192.168.1.99 --spi-ports "Port1,1 Port2,5"\n'
        ),
    )
    parser.add_argument("--host", type=str, default=DEFAULT_HOST,
                        help=f"Streaming server IP (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"Streaming server port (default: {DEFAULT_PORT})")
    parser.add_argument("--spi-ports", type=str, default=DEFAULT_SPI_PORTS,
                        help=f"SPI port config (default: '{DEFAULT_SPI_PORTS}')")
    parser.add_argument("--window", type=float, default=DEFAULT_WINDOW,
                        help=f"Display window in seconds (default: {DEFAULT_WINDOW})")
    args = parser.parse_args()

    print(f"[viz] host={args.host}  port={args.port}")
    print(f"[viz] spi-ports: {args.spi_ports}")
    print(f"[viz] window={args.window}s")

    viz = EEGVisualizer(args)
    viz.run()


if __name__ == "__main__":
    main()
