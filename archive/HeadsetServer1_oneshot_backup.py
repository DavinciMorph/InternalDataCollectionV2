#!/usr/bin/env python3
"""
ADS1299 Headset Server - V2 Board (Per-Device Retry)
=====================================================

This module provides a streaming server for the EEG headset with 5 ports and
21 daisy-chained ADS1299 devices (168 channels total).

V2 Board Features:
- No GPIO hardware reset (RESET/PWDN tied HIGH on board)
- Uses software reset via SPI command only
- 500ms delay after software reset

Per-Device Retry Strategy:
- Each device gets up to 5 internal retry attempts
- Special handling for CONFIG3 (reference buffer power-up)
- If CONFIG3 doesn't stick, re-writes it with extra settling time
- System-level retry only for catastrophic failures

Default Configuration:
  Port1: SPI 0.0, 3 daisy-chained devices (24 channels)
  Port2: SPI 0.1, 5 daisy-chained devices (40 channels)
  Port3: SPI 3.0, 5 daisy-chained devices (40 channels)
  Port4: SPI 5.0, 5 daisy-chained devices (40 channels)
  Port5: SPI 4.0, 3 daisy-chained devices (24 channels)
  
  Total: 21 devices, 168 channels @ 250 Hz

Usage:
    python3 HeadsetServer.py
    python3 HeadsetServer.py --test-mode normal
"""

import spidev
import time
import RPi.GPIO as GPIO
import smbus2
import socket
import json
import threading
import queue
import os
from queue import Queue
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import IntEnum


# --- Thread Priority Helper ---

def set_realtime_priority(priority: int = 50) -> bool:
    """Set thread to real-time FIFO scheduling. Requires root or CAP_SYS_NICE."""
    try:
        # SCHED_FIFO = 1, SCHED_RR = 2
        SCHED_FIFO = 1
        import ctypes
        libc = ctypes.CDLL('libc.so.6', use_errno=True)

        class sched_param(ctypes.Structure):
            _fields_ = [('sched_priority', ctypes.c_int)]

        param = sched_param(priority)
        # 0 = current thread
        result = libc.sched_setscheduler(0, SCHED_FIFO, ctypes.byref(param))
        if result == 0:
            return True
        else:
            return False
    except Exception:
        return False


def set_high_priority() -> bool:
    """Set high priority using nice value (fallback when RT scheduling unavailable)."""
    try:
        os.nice(-10)  # Higher priority (negative nice = higher priority)
        return True
    except PermissionError:
        return False
    except Exception:
        return False


# --- ADS1299 Register and Command Definitions ---

class ADS1299_Cmd(IntEnum):
    """ADS1299 SPI command definitions"""
    WAKEUP = 0x02
    STANDBY = 0x04
    RESET = 0x06
    START = 0x08
    STOP = 0x0A
    RDATAC = 0x10
    SDATAC = 0x11
    RDATA = 0x12


class ADS1299_Reg(IntEnum):
    """ADS1299 register addresses"""
    ID = 0x00
    CONFIG1 = 0x01
    CONFIG2 = 0x02
    CONFIG3 = 0x03
    LOFF = 0x04
    CH1SET = 0x05
    CH2SET = 0x06
    CH3SET = 0x07
    CH4SET = 0x08
    CH5SET = 0x09
    CH6SET = 0x0A
    CH7SET = 0x0B
    CH8SET = 0x0C
    BIAS_SENSP = 0x0D
    BIAS_SENSN = 0x0E
    LOFF_SENSP = 0x0F
    LOFF_SENSN = 0x10
    LOFF_FLIP = 0x11
    MISC1 = 0x15
    CONFIG4 = 0x17


# --- Hardware Configuration ---

# GPIO pins for global RESET and PWDN (connected to all ADS1299 devices)
GPIO_RESET_PIN = 26
GPIO_PWDN_PIN = 27

# I2C bus for TCA9534 expanders
I2C_BUS = 6

# TCA9534 register addresses
TCA9534_INPUT_PORT = 0x00
TCA9534_OUTPUT_PORT = 0x01
TCA9534_POLARITY_INV = 0x02
TCA9534_CONFIG = 0x03

# SPI configuration
SPI_MODE = 0b01  # Mode 1: CPOL=0, CPHA=1
SPI_SPEED_HZ = 6_000_000  # 6 MHz - need speed for 21 devices at 250 Hz
                          # At 250 Hz sample rate with 21 devices × 27 bytes = 567 bytes/sample
                          # 4 MHz can transfer 567 bytes in ~1.1ms (well under 4ms budget)
                          # 2 MHz was causing sample rate to drop to 130 Hz due to read time

# Default headset port configuration
DEFAULT_PORTS = [
    "0,0,Port1,3",   # SPI 0.0, 3 daisy devices (24 channels)
    "0,1,Port2,5",   # SPI 0.1, 5 daisy devices (40 channels)
    "3,0,Port3,5",   # SPI 3.0, 5 daisy devices (40 channels)
    "5,0,Port4,5",   # SPI 5.0, 5 daisy devices (40 channels)
    "4,0,Port5,3",   # SPI 4.0, 3 daisy devices (24 channels)
]


# --- Register Configuration Values ---

@dataclass
class ADS1299_Config:
    """Standard register configuration for ADS1299"""
    # Global settings
    config1: int = 0x96   # Daisy-chain mode, 250 SPS, external clock
    config2: int = 0xD0   # Yes test signals, 1mV square wave operation
    config3: int = 0xE0   # Int ref enabled, BIAS off, external BIASREF
    config4: int = 0x00   # Continuous conversion, lead-off comp off
    
    # Channel settings (individual configuration for each channel)
    ch1set: int = 0x05    # CH1: Channel on, gain=1, test signal
    ch2set: int = 0x05    # CH2: Channel on, gain=1, test signal
    ch3set: int = 0x05    # CH3: Channel on, gain=1, test signal
    ch4set: int = 0x05    # CH4: Channel on, gain=1, test signal
    ch5set: int = 0x05    # CH5: Channel on, gain=1, test signal
    ch6set: int = 0x05    # CH6: Channel on, gain=1, test signal
    ch7set: int = 0x05    # CH7: Channel on, gain=1, test signal
    ch8set: int = 0x05    # CH8: Channel on, gain=1, test signal
    
    # BIAS and lead-off (disabled by default)
    bias_sensp: int = 0x00
    bias_sensn: int = 0x00
    loff_sensp: int = 0x00
    loff_sensn: int = 0x00
    loff_flip: int = 0x00
    
    # Miscellaneous
    misc1: int = 0x20
    
    def get_channel_settings(self) -> List[int]:
        """Get all channel settings as a list [CH1, CH2, ..., CH8]"""
        return [self.ch1set, self.ch2set, self.ch3set, self.ch4set,
                self.ch5set, self.ch6set, self.ch7set, self.ch8set]
    
    def set_all_channels(self, value: int):
        """Set all channels to the same value"""
        self.ch1set = value
        self.ch2set = value
        self.ch3set = value
        self.ch4set = value
        self.ch5set = value
        self.ch6set = value
        self.ch7set = value
        self.ch8set = value
    
    @classmethod
    def test_signal_config(cls):
        """Configuration for internal test signal (1mV square wave)"""
        config = cls()
        config.config2 = 0xD0  # Enable test signal
        config.set_all_channels(0x05)  # Test signal input for all channels
        return config
    
    @classmethod
    def normal_config(cls):
        """Configuration for normal EEG acquisition"""
        config = cls()
        config.config2 = 0xC0  # No test signals
        config.set_all_channels(0x00)  # Normal input, gain=24
        return config
    
    @classmethod
    def input_short_config(cls):
        """Configuration for input short (noise measurement)"""
        config = cls()
        config.set_all_channels(0x01)  # Input shorted for all channels
        return config


# --- SPI Bus Configuration Mapping ---

@dataclass
class SPIBusConfig:
    """Configuration for a single SPI bus"""
    bus_num: int
    device_num: int
    port_id: str
    num_daisy_devices: int
    drdy_i2c_addr: int
    drdy_pin: int
    start_i2c_addr: int
    start_pin: int


def get_bus_config(bus: int, device: int, port_id: str, num_daisy: int = 1) -> SPIBusConfig:
    """
    Get the complete configuration for an SPI bus including DRDY/START mappings.
    
    This implements the hardware mapping from your custom CM4 expansion board.
    """
    # DRDY pin mapping
    if bus <= 3:
        drdy_addr = 0x20
        drdy_pin = bus * 2 + device
    elif bus == 4:
        drdy_addr = 0x22
        drdy_pin = 0 if device == 0 else 1
    elif bus == 5:
        drdy_addr = 0x22
        drdy_pin = 2
    else:
        raise ValueError(f"Unsupported SPI bus: {bus}")
    
    # START pin mapping
    if bus <= 3:
        start_addr = 0x21
        start_pin = bus * 2 + device
    elif bus == 4:
        start_addr = 0x22
        start_pin = 4 if device == 0 else 5
    elif bus == 5:
        start_addr = 0x22
        start_pin = 6
    else:
        raise ValueError(f"Unsupported SPI bus: {bus}")
    
    return SPIBusConfig(
        bus_num=bus,
        device_num=device,
        port_id=port_id,
        num_daisy_devices=num_daisy,
        drdy_i2c_addr=drdy_addr,
        drdy_pin=drdy_pin,
        start_i2c_addr=start_addr,
        start_pin=start_pin
    )


# --- TCA9534 I2C GPIO Expander Control ---

class TCA9534_Pin:
    """Controller for a single pin on a TCA9534 I2C GPIO expander."""
    
    def __init__(self, i2c_bus: int, i2c_addr: int, pin: int, is_input: bool):
        self.bus = smbus2.SMBus(i2c_bus)
        self.addr = i2c_addr
        self.pin = pin
        self.pin_mask = 1 << pin
        
        # Configure pin direction
        config = self.bus.read_byte_data(i2c_addr, TCA9534_CONFIG)
        if is_input:
            config |= self.pin_mask
        else:
            config &= ~self.pin_mask
        self.bus.write_byte_data(i2c_addr, TCA9534_CONFIG, config)
        
        if not is_input:
            self.set_low()
    
    def read(self) -> bool:
        value = self.bus.read_byte_data(self.addr, TCA9534_INPUT_PORT)
        return bool(value & self.pin_mask)
    
    def set_high(self):
        value = self.bus.read_byte_data(self.addr, TCA9534_OUTPUT_PORT)
        value |= self.pin_mask
        self.bus.write_byte_data(self.addr, TCA9534_OUTPUT_PORT, value)
    
    def set_low(self):
        value = self.bus.read_byte_data(self.addr, TCA9534_OUTPUT_PORT)
        value &= ~self.pin_mask
        self.bus.write_byte_data(self.addr, TCA9534_OUTPUT_PORT, value)


class TCA9534_MultiPin:
    """Controller for setting multiple pins atomically."""
    
    def __init__(self, i2c_bus: int = I2C_BUS):
        self.bus = smbus2.SMBus(i2c_bus)
        self.pin_masks: Dict[int, int] = {}
    
    def add_pin(self, i2c_addr: int, pin: int):
        pin_mask = 1 << pin
        if i2c_addr not in self.pin_masks:
            self.pin_masks[i2c_addr] = 0
        self.pin_masks[i2c_addr] |= pin_mask
    
    def set_all_high(self):
        for addr, mask in self.pin_masks.items():
            current = self.bus.read_byte_data(addr, TCA9534_OUTPUT_PORT)
            new_value = current | mask
            self.bus.write_byte_data(addr, TCA9534_OUTPUT_PORT, new_value)
    
    def set_all_low(self):
        for addr, mask in self.pin_masks.items():
            current = self.bus.read_byte_data(addr, TCA9534_OUTPUT_PORT)
            new_value = current & ~mask
            self.bus.write_byte_data(addr, TCA9534_OUTPUT_PORT, new_value)


# --- ADS1299 SPI Communication Layer ---

# Per-bus locks to prevent contention when multiple devices share the same SPI bus
# (e.g., Port4 and Port5 both use SPI bus 4 with different chip selects)
_spi_bus_locks: Dict[int, threading.Lock] = {}

def get_bus_lock(bus_num: int) -> threading.Lock:
    """Get or create a lock for the given SPI bus number."""
    if bus_num not in _spi_bus_locks:
        _spi_bus_locks[bus_num] = threading.Lock()
    return _spi_bus_locks[bus_num]


class ADS1299_SPI:
    """Low-level SPI communication with ADS1299."""

    def __init__(self, bus_config: SPIBusConfig, interpolation_enabled: bool = True):
        self.config = bus_config
        self.initialized = False
        self.running = False
        self.interpolation_enabled = interpolation_enabled

        # Open SPI device
        self.spi = spidev.SpiDev()
        self.spi.open(bus_config.bus_num, bus_config.device_num)
        self.spi.mode = SPI_MODE
        self.spi.max_speed_hz = SPI_SPEED_HZ

        # Initialize control pins
        self.drdy = TCA9534_Pin(I2C_BUS, bus_config.drdy_i2c_addr,
                               bus_config.drdy_pin, is_input=True)
        self.start = TCA9534_Pin(I2C_BUS, bus_config.start_i2c_addr,
                                bus_config.start_pin, is_input=False)

        # Corruption tracking for debug logging
        self.corruption_count = 0
        self.total_reads = 0

        # Window-based corruption tracking for per-port detection
        self.window_start_reads = 0
        self.window_start_corruptions = 0

        # Interpolation tracking - store last valid sample per device/channel
        self.last_valid_samples = [[0] * 8 for _ in range(bus_config.num_daisy_devices)]
        self.interpolation_count = 0

    def reset_stats(self):
        """Reset corruption counters - call after initialization to ignore startup errors."""
        self.corruption_count = 0
        self.total_reads = 0
        self.interpolation_count = 0
        self.reset_window()

    def reset_window(self):
        """Reset the sliding window - call periodically to check recent corruption rate."""
        self.window_start_reads = self.total_reads
        self.window_start_corruptions = self.corruption_count

    def get_window_corruption_rate(self) -> float:
        """Get corruption rate since last window reset."""
        reads_in_window = self.total_reads - self.window_start_reads
        if reads_in_window < 100:  # Need minimum samples for meaningful rate
            return 0.0
        corruptions_in_window = self.corruption_count - self.window_start_corruptions
        return corruptions_in_window / reads_in_window
    
    @staticmethod
    def wait_4tclk():
        # Minimum wait after SPI commands per ADS1299 datasheet: 4 tCLK (~2μs at 2.048 MHz)
        # Keep this minimal during data acquisition - timing is critical at 250 Hz.
        time.sleep(0.000002)  # 2 microseconds - datasheet minimum
    
    def flush_spi(self):
        self.spi.xfer2([0x00] * 32)
        time.sleep(0.001)
    
    def send_command(self, command: int):
        self.spi.xfer2([command])
        self.wait_4tclk()
    
    def read_registers(self, start_reg: int, num_regs: int) -> List[int]:
        cmd_byte1 = 0x20 | (start_reg & 0x1F)
        cmd_byte2 = (num_regs - 1) & 0x1F
        packet = [cmd_byte1, cmd_byte2] + ([0x00] * num_regs)
        response = self.spi.xfer2(packet)
        self.wait_4tclk()
        return response[2:]

    def write_registers(self, start_reg: int, values: List[int]):
        num_regs = len(values)
        cmd_byte1 = 0x40 | (start_reg & 0x1F)
        cmd_byte2 = (num_regs - 1) & 0x1F
        packet = [cmd_byte1, cmd_byte2] + values
        self.spi.xfer2(packet)
        self.wait_4tclk()
    
    def is_data_ready(self) -> bool:
        return not self.drdy.read()
    
    def wait_for_drdy(self, timeout: float = 0.1) -> bool:
        start_time = time.time()
        while not self.is_data_ready():
            if time.time() - start_time > timeout:
                return False
            time.sleep(0.0001)
        return True

    def read_data(self) -> Tuple[List[int], List[List[int]]]:
        """Read data from all daisy-chained devices. Returns (status_bytes, channel_data)."""
        bytes_per_device = 27
        total_bytes = bytes_per_device * self.config.num_daisy_devices

        # Pre-read settling delay for daisy-chain data propagation
        # The last device in a chain has the longest signal path and is most
        # prone to bit-shift corruption. A small delay allows data to stabilize.
        # At 250 Hz (4ms budget), 10-20μs is <1% overhead.
        if self.config.num_daisy_devices >= 3:
            time.sleep(0.000015)  # 15 microseconds for 3+ device chains

        raw_data = self.spi.xfer2([0x00] * total_bytes)

        channel_data = []
        first_status = None
        self.total_reads += 1

        # Track if any corruption occurred in this read for detailed logging
        corruption_in_this_read = False
        corruption_details = []

        for device_idx in range(self.config.num_daisy_devices):
            device_offset = device_idx * bytes_per_device
            device_status = raw_data[device_offset:device_offset + 3]

            # Validate status byte - upper nibble should be 0xC (1100 binary)
            # This is defined by ADS1299 - indicates valid conversion data
            status_byte = device_status[0]
            status_valid = (status_byte & 0xF0) == 0xC0

            if not status_valid:
                self.corruption_count += 1
                corruption_in_this_read = True

                # Determine corruption type for better diagnostics
                if status_byte == 0x00:
                    corruption_type = "NO_DATA"
                elif status_byte == 0xFF:
                    corruption_type = "FLOATING"
                else:
                    corruption_type = "BIT_SHIFT"

                # Get raw bytes for this device for forensic analysis
                device_raw = raw_data[device_offset:device_offset + bytes_per_device]
                raw_hex = ' '.join([f'{b:02X}' for b in device_raw[:12]])  # First 12 bytes

                corruption_details.append({
                    'device_idx': device_idx,
                    'status_byte': status_byte,
                    'corruption_type': corruption_type,
                    'raw_hex': raw_hex,
                    'full_status': device_status
                })

                if self.corruption_count <= 10 or self.corruption_count % 100 == 0:
                    print(f"CORRUPTION: {self.config.port_id} Device {device_idx+1}/{self.config.num_daisy_devices} "
                          f"status=0x{status_byte:02X} - {corruption_type} "
                          f"(read #{self.total_reads}, corruption #{self.corruption_count})")

            if device_idx == 0:
                first_status = device_status

            # If status byte is invalid, use interpolated (last valid) values for entire device
            # Only interpolate if interpolation is enabled
            if not status_valid:
                if self.interpolation_enabled:
                    device_channels = self.last_valid_samples[device_idx].copy()
                    self.interpolation_count += 8  # All 8 channels interpolated
                else:
                    # When interpolation disabled, still read raw data
                    device_channels = []
                    ch_data_offset = device_offset + 3

                    for ch in range(8):
                        byte_offset = ch_data_offset + (ch * 3)
                        ch_bytes = raw_data[byte_offset:byte_offset + 3]
                        value = (ch_bytes[0] << 16) | (ch_bytes[1] << 8) | ch_bytes[2]
                        if value & 0x800000:
                            value = value - 0x1000000
                        device_channels.append(value)
            else:
                device_channels = []
                ch_data_offset = device_offset + 3

                for ch in range(8):
                    byte_offset = ch_data_offset + (ch * 3)
                    ch_bytes = raw_data[byte_offset:byte_offset + 3]
                    value = (ch_bytes[0] << 16) | (ch_bytes[1] << 8) | ch_bytes[2]
                    if value & 0x800000:
                        value = value - 0x1000000
                    device_channels.append(value)

            channel_data.append(device_channels)

        # Enhanced logging: Check for out-of-range values even when status is valid
        # For test signal mode, values should be within ±5000 (square wave ~±2000)
        # Values beyond this threshold indicate data corruption even with valid status
        # INTERPOLATION: Replace OOR values with last valid sample
        VALUE_THRESHOLD = 5000  # Values beyond this indicate corruption
        out_of_range_detected = False
        oor_devices_this_read = set()  # Track which devices had OOR values

        for device_idx, device_channels in enumerate(channel_data):
            for ch_idx, value in enumerate(device_channels):
                if abs(value) > VALUE_THRESHOLD:
                    out_of_range_detected = True
                    oor_devices_this_read.add(device_idx)

                    # Only log first few occurrences to avoid spam
                    if not hasattr(self, '_out_of_range_count'):
                        self._out_of_range_count = 0
                    self._out_of_range_count += 1

                    # INTERPOLATION: Substitute with last valid value (only if enabled)
                    if self.interpolation_enabled:
                        original_value = value
                        channel_data[device_idx][ch_idx] = self.last_valid_samples[device_idx][ch_idx]
                        self.interpolation_count += 1

                        if self._out_of_range_count <= 20 or self._out_of_range_count % 100 == 0:
                            device_offset = device_idx * bytes_per_device
                            status_byte = raw_data[device_offset]
                            ch_offset = device_offset + 3 + (ch_idx * 3)
                            ch_raw = raw_data[ch_offset:ch_offset + 3]

                            print(f"OUT_OF_RANGE (interpolated): {self.config.port_id} Device {device_idx+1}/{self.config.num_daisy_devices} "
                                  f"Ch{ch_idx+1}={original_value}→{channel_data[device_idx][ch_idx]} "
                                  f"(raw: {ch_raw[0]:02X} {ch_raw[1]:02X} {ch_raw[2]:02X}) "
                                  f"status=0x{status_byte:02X} (read #{self.total_reads}, OOR #{self._out_of_range_count})")
                    else:
                        # Log without interpolation
                        if self._out_of_range_count <= 20 or self._out_of_range_count % 100 == 0:
                            device_offset = device_idx * bytes_per_device
                            status_byte = raw_data[device_offset]
                            ch_offset = device_offset + 3 + (ch_idx * 3)
                            ch_raw = raw_data[ch_offset:ch_offset + 3]

                            print(f"OUT_OF_RANGE (raw): {self.config.port_id} Device {device_idx+1}/{self.config.num_daisy_devices} "
                                  f"Ch{ch_idx+1}={value} "
                                  f"(raw: {ch_raw[0]:02X} {ch_raw[1]:02X} {ch_raw[2]:02X}) "
                                  f"status=0x{status_byte:02X} (read #{self.total_reads}, OOR #{self._out_of_range_count})")
                else:
                    # Update last valid sample for this channel (always track for logging purposes)
                    self.last_valid_samples[device_idx][ch_idx] = value

        # Count out-of-range as corruption even when status byte is valid
        # This catches data corruption that doesn't affect the status byte
        if out_of_range_detected and not corruption_in_this_read:
            # Only count once per read, regardless of how many OOR values
            self.corruption_count += 1
            corruption_in_this_read = True

            # Log detailed info for this type of corruption
            if self.corruption_count <= 5 or self.corruption_count % 50 == 0:
                print(f"  DATA_CORRUPTION: {self.config.port_id} - Valid status but corrupted values "
                      f"(read #{self.total_reads}, corruption #{self.corruption_count})")
                for dev_idx in oor_devices_this_read:
                    vals = channel_data[dev_idx]
                    val_str = ', '.join([f'{v:+d}' for v in vals])
                    print(f"    Device {dev_idx+1} channels: [{val_str}]")

        # If corruption occurred, log additional context
        if corruption_in_this_read and len(corruption_details) > 0:
            # Check if corruption is in last device (common pattern)
            last_device_corrupted = any(d['device_idx'] == self.config.num_daisy_devices - 1
                                       for d in corruption_details)

            # Log detailed info for first 5 corruptions and every 50th after
            if self.corruption_count <= 5 or self.corruption_count % 50 == 0:
                print(f"  DETAIL: Read #{self.total_reads} on {self.config.port_id}:")
                for detail in corruption_details:
                    print(f"    Device {detail['device_idx']+1}: status=0x{detail['status_byte']:02X} "
                          f"type={detail['corruption_type']} raw=[{detail['raw_hex']}]")

                # Also log channel values for corrupted devices
                for detail in corruption_details:
                    dev_idx = detail['device_idx']
                    if dev_idx < len(channel_data):
                        vals = channel_data[dev_idx]
                        val_str = ', '.join([f'{v:+d}' for v in vals])
                        print(f"    Device {dev_idx+1} channels: [{val_str}]")

                if last_device_corrupted:
                    print(f"    ⚠ Last device in chain corrupted (daisy-chain propagation issue)")

        if first_status is not None:
            return first_status, channel_data
        else:
            return [0, 0, 0], channel_data


def wait_for_all_drdy(spi_devices: List['ADS1299_SPI'], timeout: float = 0.1) -> bool:
    """Wait until ALL devices have DRDY active (LOW). Returns True if ready, False on timeout."""
    # Build lookup of which pins to check on each I2C address
    # This is done once and cached implicitly by Python
    drdy_by_addr: Dict[int, List[Tuple['ADS1299_SPI', int]]] = {}
    for spi_dev in spi_devices:
        addr = spi_dev.config.drdy_i2c_addr
        pin_mask = 1 << spi_dev.config.drdy_pin
        if addr not in drdy_by_addr:
            drdy_by_addr[addr] = []
        drdy_by_addr[addr].append((spi_dev, pin_mask))

    # Get I2C bus from first device (they all share the same bus)
    i2c_bus = spi_devices[0].drdy.bus

    start_time = time.time()
    while True:
        all_ready = True

        # Read each I2C expander once and check all relevant pins
        for addr, devices_and_masks in drdy_by_addr.items():
            try:
                port_value = i2c_bus.read_byte_data(addr, TCA9534_INPUT_PORT)
                for spi_dev, pin_mask in devices_and_masks:
                    # DRDY is active LOW, so bit=0 means ready
                    if port_value & pin_mask:
                        all_ready = False
                        break
                if not all_ready:
                    break
            except Exception:
                all_ready = False
                break

        if all_ready:
            return True

        if time.time() - start_time > timeout:
            return False

        time.sleep(0.00005)  # 50μs polling - faster than before


# --- ADS1299 Device Controller (V2 Board - Per-Device Retry) ---

class ADS1299_Controller:
    """High-level controller for ADS1299 initialization. V2 board uses software reset only."""

    @staticmethod
    def _write_verify_register(spi: 'ADS1299_SPI', reg: int, expected: int,
                               name: str, verbose: bool = True) -> Tuple[bool, int]:
        """Write a register value and verify readback. Returns (success, actual_value)."""
        spi.write_registers(reg, [expected])
        time.sleep(0.01)
        actual = spi.read_registers(reg, 1)[0]
        success = actual == expected
        if verbose:
            status = '✓' if success else '✗'
            print(f"  {name}: 0x{actual:02X} (expected 0x{expected:02X}) {status}")
        return success, actual

    @staticmethod
    def _broadcast_command(spi_devices: List['ADS1299_SPI'], command: int,
                           delay_ms: float = 10):
        """Send a command to all SPI devices with optional delay."""
        for spi_dev in spi_devices:
            spi_dev.send_command(command)
        if delay_ms > 0:
            time.sleep(delay_ms / 1000)

    @staticmethod
    def _create_start_controller(spi_devices: List['ADS1299_SPI']) -> TCA9534_MultiPin:
        """Create a TCA9534_MultiPin controller for all START pins."""
        start_controller = TCA9534_MultiPin()
        for spi_dev in spi_devices:
            start_controller.add_pin(
                spi_dev.config.start_i2c_addr,
                spi_dev.config.start_pin
            )
        return start_controller

    @staticmethod
    def _set_all_start_pins_low(spi_devices: List['ADS1299_SPI']):
        """Set all START pins LOW."""
        for spi_dev in spi_devices:
            spi_dev.start.set_low()
        time.sleep(0.01)

    @staticmethod
    def _restart_single_port(spi_dev: 'ADS1299_SPI', attempt: int = 0,
                             verbose: bool = True) -> bool:
        """Restart a single port by cycling STOP/SDATAC/RDATAC/START. Returns True if successful."""
        delay_scale = attempt + 1

        # Stop the port
        spi_dev.start.set_low()
        time.sleep(0.020 * delay_scale)

        spi_dev.send_command(ADS1299_Cmd.STOP)
        time.sleep(0.010)
        spi_dev.send_command(ADS1299_Cmd.SDATAC)
        time.sleep(0.020 * delay_scale)

        # Re-enter RDATAC (send twice for reliability)
        spi_dev.send_command(ADS1299_Cmd.RDATAC)
        time.sleep(0.010)
        spi_dev.send_command(ADS1299_Cmd.RDATAC)
        time.sleep(0.020 * delay_scale)

        # Assert START
        spi_dev.start.set_high()
        time.sleep(0.050 * delay_scale)

        # Verify DRDY and data
        if spi_dev.wait_for_drdy(timeout=0.1):
            test_status, _ = spi_dev.read_data()
            if (test_status[0] & 0xF0) == 0xC0:
                return True
            elif verbose:
                if test_status[0] == 0x00:
                    print(f"    {spi_dev.config.port_id}: DRDY active but returning zeros (attempt {attempt + 1})")
                else:
                    print(f"    {spi_dev.config.port_id}: DRDY active, status=0x{test_status[0]:02X} (attempt {attempt + 1})")
        elif verbose:
            print(f"    {spi_dev.config.port_id}: DRDY still not active (attempt {attempt + 1})")

        return False

    @staticmethod
    def initialize_gpio():
        """Initialize GPIO - minimal for V2 board (RESET/PWDN tied HIGH)."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        # V2: No need to control RESET/PWDN pins - they're hardwired HIGH
        print("GPIO initialized (V2 board - RESET/PWDN hardwired HIGH)")
    
    @staticmethod
    def force_all_start_pins_low(bus_configs: List[SPIBusConfig]):
        """Force ALL START pins LOW before initialization."""
        print("Forcing all START pins LOW...")
        
        # Group START pins by I2C expander address
        start_pins_by_addr: Dict[int, List[int]] = {}
        for cfg in bus_configs:
            addr = cfg.start_i2c_addr
            pin = cfg.start_pin
            if addr not in start_pins_by_addr:
                start_pins_by_addr[addr] = []
            start_pins_by_addr[addr].append(pin)
        
        # Configure and set each expander
        bus = smbus2.SMBus(I2C_BUS)
        try:
            for addr, pins in start_pins_by_addr.items():
                mask = 0
                for pin in pins:
                    mask |= (1 << pin)
                
                try:
                    # Configure pins as outputs
                    config_reg = bus.read_byte_data(addr, TCA9534_CONFIG)
                    config_reg &= ~mask
                    bus.write_byte_data(addr, TCA9534_CONFIG, config_reg)
                    
                    # Set all START pins LOW
                    output_reg = bus.read_byte_data(addr, TCA9534_OUTPUT_PORT)
                    output_reg &= ~mask
                    bus.write_byte_data(addr, TCA9534_OUTPUT_PORT, output_reg)
                    
                    print(f"  I2C 0x{addr:02X}: START pins {pins} set LOW ✓")
                except Exception as e:
                    print(f"  I2C 0x{addr:02X}: Failed to set START pins: {e}")
        finally:
            bus.close()
        
        time.sleep(0.01)
    
    @staticmethod
    def wait_for_power_on():
        """Wait for ADS1299 devices to be ready (VCAP1 charging)."""
        print("\nWaiting for VCAP1 charging and device ready (2s)...")
        time.sleep(2.0)
        print("Device power-on wait complete ✓")
    
    @staticmethod
    def initialize_device(spi: ADS1299_SPI, config: ADS1299_Config) -> Tuple[bool, int]:
        """
        Initialize ADS1299 with single-pass timing. No retries.

        Key insight: Bulk register writes fail in daisy-chain mode.
        Write each register individually with proper delays.
        """
        print(f"\nConfiguring {spi.config.port_id}...")

        num_devices = spi.config.num_daisy_devices

        # 1. Ensure START is low
        spi.start.set_low()
        time.sleep(0.05)

        # 2. Flush SPI and send STOP to clear any stale state
        spi.flush_spi()
        spi.send_command(ADS1299_Cmd.STOP)
        time.sleep(0.01)

        # 3. Software RESET
        #    Longer chains need more settling time
        reset_time = 2.0 + (num_devices * 0.2)  # Extra 200ms per device
        print(f"  Software reset ({reset_time:.1f}s for {num_devices} devices)...")
        spi.send_command(ADS1299_Cmd.RESET)
        time.sleep(reset_time)

        # 4. SDATAC to enable register access
        #    Send more times for longer daisy chains - each device needs to receive it
        sdatac_count = max(5, num_devices + 2)
        for _ in range(sdatac_count):
            spi.send_command(ADS1299_Cmd.SDATAC)
            time.sleep(0.02)
        time.sleep(0.1)  # Extra settling for long chains

        # 4. Verify SDATAC with LOFF write/read test - retry until it works
        test_val = 0xAA
        sdatac_ok = False
        for attempt in range(10):
            spi.write_registers(ADS1299_Reg.LOFF, [test_val])
            time.sleep(0.1)
            readback = spi.read_registers(ADS1299_Reg.LOFF, 1)[0]
            if readback == test_val:
                sdatac_ok = True
                if attempt > 0:
                    print(f"  SDATAC verified on attempt {attempt + 1} ✓")
                else:
                    print(f"  SDATAC verified ✓")
                break
            # Retry - send more SDATAC commands
            for _ in range(3):
                spi.send_command(ADS1299_Cmd.SDATAC)
                time.sleep(0.02)
            time.sleep(0.1)

        if not sdatac_ok:
            print(f"  SDATAC verification failed after 10 attempts: wrote 0x{test_val:02X}, read 0x{readback:02X}")
            return False, 1

        spi.write_registers(ADS1299_Reg.LOFF, [0x00])  # Restore
        time.sleep(0.1)

        # 5. Verify device ID
        id_value = spi.read_registers(ADS1299_Reg.ID, 1)[0]
        print(f"  Device ID: 0x{id_value:02X}")
        if id_value != 0x3E:
            print(f"  ✗ Expected ID 0x3E")
            return False, 1
        print(f"  ✓ ADS1299 detected")

        # 6. Write CONFIG registers - retry each until it sticks
        print(f"  Writing configuration...")

        def write_and_verify(reg, value, name, max_attempts=10):
            """Write register and retry until verified."""
            for attempt in range(max_attempts):
                spi.write_registers(reg, [value])
                time.sleep(0.1)
                readback = spi.read_registers(reg, 1)[0]
                if readback == value:
                    if attempt > 0:
                        print(f"    {name} verified on attempt {attempt + 1}")
                    return True
                time.sleep(0.05)  # Brief pause before retry
            print(f"  {name} failed after {max_attempts} attempts: wrote 0x{value:02X}, read 0x{readback:02X}")
            return False

        # CONFIG3 first - enables internal reference
        if not write_and_verify(ADS1299_Reg.CONFIG3, config.config3, "CONFIG3"):
            return False, 1

        # Wait for reference buffer to stabilize
        print(f"  Waiting for reference buffer...")
        time.sleep(0.5)

        # CONFIG1 - sample rate and daisy settings
        if not write_and_verify(ADS1299_Reg.CONFIG1, config.config1, "CONFIG1"):
            return False, 1

        # CONFIG2 - test signal settings
        if not write_and_verify(ADS1299_Reg.CONFIG2, config.config2, "CONFIG2"):
            return False, 1

        # 7. Write channel settings - retry each until it sticks
        channel_settings = config.get_channel_settings()
        for i, ch_val in enumerate(channel_settings):
            if not write_and_verify(ADS1299_Reg.CH1SET + i, ch_val, f"CH{i+1}SET"):
                return False, 1

        # 8. Write MISC1 and CONFIG4
        if not write_and_verify(ADS1299_Reg.MISC1, config.misc1, "MISC1"):
            return False, 1
        if not write_and_verify(ADS1299_Reg.CONFIG4, config.config4, "CONFIG4"):
            return False, 1

        # 9. Final verification - read each CONFIG register INDIVIDUALLY
        #    Bulk reads fail on long daisy chains (middle bytes corrupt)
        c1_read = spi.read_registers(ADS1299_Reg.CONFIG1, 1)[0]
        time.sleep(0.005)
        c2_read = spi.read_registers(ADS1299_Reg.CONFIG2, 1)[0]
        time.sleep(0.005)
        c3_read = spi.read_registers(ADS1299_Reg.CONFIG3, 1)[0]

        c1_ok = c1_read == config.config1
        c2_ok = c2_read == config.config2
        c3_ok = c3_read == config.config3

        c1 = '✓' if c1_ok else '✗'
        c2 = '✓' if c2_ok else '✗'
        c3 = '✓' if c3_ok else '✗'
        print(f"  CONFIG1=0x{c1_read:02X}{c1} CONFIG2=0x{c2_read:02X}{c2} CONFIG3=0x{c3_read:02X}{c3}")

        if c1_ok and c2_ok and c3_ok:
            print(f"  ✓ {spi.config.port_id} configured successfully")
            return True, 1

        print(f"  ✗ {spi.config.port_id} configuration failed")
        return False, 1

    @staticmethod
    def start_conversion(spi: ADS1299_SPI):
        """Start continuous data acquisition on a single device."""
        spi.send_command(ADS1299_Cmd.RDATAC)
        time.sleep(0.001)
        spi.start.set_high()
        time.sleep(0.01)
        print(f"  {spi.config.port_id}: Acquisition started ✓")
    
    @staticmethod
    def stop_conversion(spi: ADS1299_SPI):
        """Stop continuous data acquisition"""
        spi.start.set_low()
        spi.send_command(ADS1299_Cmd.STOP)
        time.sleep(0.001)
    
    @staticmethod
    def start_all_conversions_synchronized(spi_devices: List['ADS1299_SPI']) -> bool:
        """Start conversions on ALL devices with synchronized START. Returns True if valid data."""
        print("\n  Starting conversions with synchronized START...")

        print("    Stopping any ongoing conversions...")
        ADS1299_Controller._set_all_start_pins_low(spi_devices)
        ADS1299_Controller._broadcast_command(spi_devices, ADS1299_Cmd.STOP, delay_ms=10)

        print("    Ensuring all devices in SDATAC mode...")
        ADS1299_Controller._broadcast_command(spi_devices, ADS1299_Cmd.SDATAC, delay_ms=50)

        print("    Sending RDATAC to all devices...")
        for spi_dev in spi_devices:
            spi_dev.send_command(ADS1299_Cmd.RDATAC)
            time.sleep(0.005)
            spi_dev.send_command(ADS1299_Cmd.RDATAC)
            time.sleep(0.010)

        time.sleep(0.500)  # 500ms settling - longer delay helps synchronization

        print("    Verifying device communication after RDATAC...")
        rdatac_verify_failures = []
        for spi_dev in spi_devices:
            spi_dev.flush_spi()
            time.sleep(0.002)
            raw = spi_dev.spi.xfer2([0x00] * 3)
            if raw[0] == 0xFF and raw[1] == 0xFF and raw[2] == 0xFF:
                rdatac_verify_failures.append(spi_dev.config.port_id)
                print(f"      ⚠ {spi_dev.config.port_id}: SPI returns 0xFF - RDATAC may have failed")

        if rdatac_verify_failures:
            print(f"    Retrying RDATAC for failed ports: {', '.join(rdatac_verify_failures)}")
            for spi_dev in spi_devices:
                if spi_dev.config.port_id in rdatac_verify_failures:
                    spi_dev.send_command(ADS1299_Cmd.SDATAC)
                    time.sleep(0.010)
                    spi_dev.send_command(ADS1299_Cmd.RDATAC)
                    time.sleep(0.020)
                    spi_dev.send_command(ADS1299_Cmd.RDATAC)
                    time.sleep(0.020)

        time.sleep(0.05)

        start_controller = ADS1299_Controller._create_start_controller(spi_devices)
        start_controller.set_all_high()
        print(f"    [All START pins asserted simultaneously]")

        time.sleep(0.250)  # 250ms synchronization - longer delay for better sync

        for spi_dev in spi_devices:
            print(f"    {spi_dev.config.port_id}: START asserted ✓")

        # Verify DRDY is actually toggling on each port
        print(f"\n  Verifying DRDY signals are active...")
        drdy_failures = []
        for spi_dev in spi_devices:
            if spi_dev.wait_for_drdy(timeout=0.1):
                print(f"    {spi_dev.config.port_id}: DRDY active ✓")
            else:
                print(f"    {spi_dev.config.port_id}: DRDY NOT active ✗")
                drdy_failures.append(spi_dev.config.port_id)

        if drdy_failures:
            print(f"  ⚠ DRDY failed on: {', '.join(drdy_failures)} - attempting restart...")
            for spi_dev in spi_devices:
                if spi_dev.config.port_id in drdy_failures:
                    restart_success = False
                    for attempt in range(3):
                        if ADS1299_Controller._restart_single_port(spi_dev, attempt):
                            print(f"    {spi_dev.config.port_id}: DRDY now active ✓ (attempt {attempt + 1})")
                            restart_success = True
                            break

                    if not restart_success:
                        print(f"    {spi_dev.config.port_id}: Restart failed after 3 attempts ✗")

        print(f"\n  Waiting 1000ms for stabilization...")
        time.sleep(1.0)

        print(f"\n  Checking individual port DRDY signals...")
        ports_needing_restart = []
        for spi_dev in spi_devices:
            drdy_active = spi_dev.wait_for_drdy(timeout=0.05)
            if drdy_active:
                zero_count = 0
                valid_count = 0
                for _ in range(5):
                    status, _ = spi_dev.read_data()
                    if status[0] == 0x00:
                        zero_count += 1
                    elif (status[0] & 0xF0) == 0xC0:
                        valid_count += 1
                    time.sleep(0.004)

                if valid_count >= 3:
                    print(f"    {spi_dev.config.port_id}: DRDY ✓, valid status ✓ ({valid_count}/5 valid)")
                elif zero_count >= 3:
                    print(f"    {spi_dev.config.port_id}: DRDY ✓, but returning zeros ({zero_count}/5) ✗")
                    ports_needing_restart.append(spi_dev)
                else:
                    print(f"    {spi_dev.config.port_id}: DRDY ✓, mixed status (valid={valid_count}, zero={zero_count})")
                    if zero_count > valid_count:
                        ports_needing_restart.append(spi_dev)
            else:
                print(f"    {spi_dev.config.port_id}: DRDY not active ✗")
                ports_needing_restart.append(spi_dev)

        if ports_needing_restart:
            print(f"\n  Restarting {len(ports_needing_restart)} ports...")
            for spi_dev in ports_needing_restart:
                restart_success = False
                for attempt in range(3):
                    if ADS1299_Controller._restart_single_port(spi_dev, attempt, verbose=False):
                        print(f"    {spi_dev.config.port_id}: restart ok (attempt {attempt + 1}) ✓")
                        restart_success = True
                        break
                if not restart_success:
                    print(f"    {spi_dev.config.port_id}: restart failed")
            time.sleep(0.200)

        print(f"\n  Discarding 200 warmup samples...")
        reference_port = spi_devices[0]
        warmup_count = 0
        warmup_drdy_timeouts = 0
        warmup_corruptions = {spi_dev.config.port_id: 0 for spi_dev in spi_devices}
        consecutive_zeros = {spi_dev.config.port_id: 0 for spi_dev in spi_devices}
        port_restart_attempted = set()

        for i in range(200):
            if reference_port.wait_for_drdy(timeout=0.02):
                for spi_dev in spi_devices:
                    status, _ = spi_dev.read_data()
                    if (status[0] & 0xF0) != 0xC0:
                        warmup_corruptions[spi_dev.config.port_id] += 1
                        if status[0] == 0x00:
                            consecutive_zeros[spi_dev.config.port_id] += 1
                        else:
                            consecutive_zeros[spi_dev.config.port_id] = 0
                    else:
                        consecutive_zeros[spi_dev.config.port_id] = 0

                    if (consecutive_zeros[spi_dev.config.port_id] >= 20 and
                        spi_dev.config.port_id not in port_restart_attempted):
                        port_restart_attempted.add(spi_dev.config.port_id)
                        print(f"    ⚠ {spi_dev.config.port_id}: 20+ zeros at sample {i}, restarting...")
                        restart_success = False
                        for restart_attempt in range(3):
                            if ADS1299_Controller._restart_single_port(spi_dev, restart_attempt, verbose=False):
                                print(f"      {spi_dev.config.port_id}: restart ok (attempt {restart_attempt+1}) ✓")
                                consecutive_zeros[spi_dev.config.port_id] = 0
                                restart_success = True
                                break
                        if not restart_success:
                            print(f"      {spi_dev.config.port_id}: restart failed")

                warmup_count += 1
            else:
                warmup_drdy_timeouts += 1
                if warmup_drdy_timeouts <= 5:
                    print(f"    Warmup sample {i}: DRDY timeout on reference port")

        if warmup_drdy_timeouts > 0:
            print(f"  ⚠ {warmup_drdy_timeouts} DRDY timeouts during warmup ({warmup_drdy_timeouts/200*100:.1f}%)")

        total_warmup_corruptions = sum(warmup_corruptions.values())
        if total_warmup_corruptions > 0:
            print(f"  ⚠ Warmup corruption: {total_warmup_corruptions} total")
            for port_id, count in warmup_corruptions.items():
                if count > 0:
                    pct = count / warmup_count * 100 if warmup_count > 0 else 0
                    print(f"    {port_id}: {count} ({pct:.1f}%)")

        print(f"  Discarded {warmup_count} warmup samples")

        # Check for ports with elevated warmup corruption (>10%) and try aggressive re-init
        WARMUP_REINIT_THRESHOLD = 0.10  # 10% - trigger aggressive re-init
        WARMUP_ABORT_THRESHOLD = 0.50   # 50% - abort and request full system restart

        ports_needing_reinit = []
        for port_id, count in warmup_corruptions.items():
            if warmup_count > 0:
                corruption_rate = count / warmup_count
                if corruption_rate > WARMUP_REINIT_THRESHOLD:
                    # Find the spi_dev for this port
                    for spi_dev in spi_devices:
                        if spi_dev.config.port_id == port_id:
                            ports_needing_reinit.append((spi_dev, corruption_rate))
                            break

        # Aggressive re-initialization for ports with >10% corruption
        if ports_needing_reinit:
            print(f"\n  ⚠ {len(ports_needing_reinit)} port(s) with >10% warmup corruption - attempting aggressive re-init...")

            for spi_dev, rate in ports_needing_reinit:
                print(f"    {spi_dev.config.port_id}: {rate*100:.1f}% corruption - full re-init...")

                # Full re-initialization sequence
                spi_dev.start.set_low()
                time.sleep(0.050)
                spi_dev.send_command(ADS1299_Cmd.STOP)
                time.sleep(0.020)
                spi_dev.send_command(ADS1299_Cmd.SDATAC)
                time.sleep(0.100)

                # Re-enter RDATAC with longer settling
                spi_dev.send_command(ADS1299_Cmd.RDATAC)
                time.sleep(0.050)
                spi_dev.send_command(ADS1299_Cmd.RDATAC)  # Send twice for reliability
                time.sleep(0.200)

                # Re-assert START
                spi_dev.start.set_high()
                time.sleep(0.100)

                # Verify DRDY and do mini-warmup for this port
                if spi_dev.wait_for_drdy(timeout=0.1):
                    # Mini-warmup: read 50 samples and check corruption
                    mini_warmup_corruptions = 0
                    for _ in range(50):
                        if spi_dev.wait_for_drdy(timeout=0.02):
                            status, _ = spi_dev.read_data()
                            if (status[0] & 0xF0) != 0xC0:
                                mini_warmup_corruptions += 1

                    mini_rate = mini_warmup_corruptions / 50
                    if mini_rate < 0.05:  # Less than 5% after re-init = success
                        print(f"      {spi_dev.config.port_id}: re-init successful ({mini_rate*100:.1f}% corruption) ✓")
                        warmup_corruptions[spi_dev.config.port_id] = int(mini_rate * warmup_count)  # Update for threshold check
                    else:
                        print(f"      {spi_dev.config.port_id}: re-init helped but still {mini_rate*100:.1f}% corruption")
                        warmup_corruptions[spi_dev.config.port_id] = int(mini_rate * warmup_count)
                else:
                    print(f"      {spi_dev.config.port_id}: DRDY not active after re-init ✗")

        # Final check - abort if any port still has >50% corruption
        bad_start_detected = False
        for port_id, count in warmup_corruptions.items():
            if warmup_count > 0:
                corruption_rate = count / warmup_count
                if corruption_rate > WARMUP_ABORT_THRESHOLD:
                    print(f"  ✗ {port_id} has {corruption_rate*100:.0f}% warmup corruption (threshold: {WARMUP_ABORT_THRESHOLD*100:.0f}%)")
                    bad_start_detected = True

        if bad_start_detected:
            print(f"\n  ✗ High warmup corruption after re-init - requesting full system restart...")
            ADS1299_Controller._set_all_start_pins_low(spi_devices)
            ADS1299_Controller._broadcast_command(spi_devices, ADS1299_Cmd.STOP, delay_ms=0)
            return False

        print(f"  Resetting corruption counters...")
        for spi_dev in spi_devices:
            if spi_dev.corruption_count > 0:
                print(f"    {spi_dev.config.port_id}: {spi_dev.corruption_count} warmup corruptions (now cleared)")
            spi_dev.reset_stats()

        print(f"\n  Verifying data flow (20 reads)...")
        reference_port = spi_devices[0]
        valid_counts = {spi_dev.config.port_id: 0 for spi_dev in spi_devices}
        status_samples = {spi_dev.config.port_id: [] for spi_dev in spi_devices}
        total_reads = 0
        drdy_timeouts = 0

        for read_num in range(20):
            if reference_port.wait_for_drdy(timeout=0.02):
                total_reads += 1
                for spi_dev in spi_devices:
                    status, _ = spi_dev.read_data()
                    status_samples[spi_dev.config.port_id].append(status[0])
                    if (status[0] & 0xF0) == 0xC0:
                        valid_counts[spi_dev.config.port_id] += 1
            else:
                drdy_timeouts += 1
                if drdy_timeouts <= 3:
                    print(f"    Read {read_num}: DRDY timeout on reference port")

        if drdy_timeouts > 0:
            print(f"  ⚠ {drdy_timeouts} DRDY timeouts during verification")

        all_ok = True
        for spi_dev in spi_devices:
            port_id = spi_dev.config.port_id
            count = valid_counts[port_id]
            samples = status_samples[port_id]

            if count >= 10:
                print(f"    ✓ {port_id}: Data flowing ({count}/{total_reads} valid)")
            else:
                sample_hex = ' '.join([f'0x{s:02X}' for s in samples[:5]])
                print(f"    ⚠ {port_id}: Low valid ({count}/{total_reads}), samples: {sample_hex}...")
                if count == 0:
                    all_ok = False

        if all_ok:
            print(f"\n  ✓ All {len(spi_devices)} devices verified!")
        else:
            print(f"\n  ⚠ Some ports had issues but may self-correct during streaming")

        return all_ok


# --- WiFi Streaming Server ---

class StreamingServer:
    """WiFi streaming server for EEG data with client reconnection support."""
    
    def __init__(self, spi_devices: List[ADS1299_SPI],
                 host: str = '0.0.0.0', port: int = 8888):
        self.spi_devices = spi_devices
        self.host = host
        self.port = port

        self.data_queue = Queue(maxsize=10000)
        self.stop_event = threading.Event()
        self.client_connected = threading.Event()
        self.streaming_active = False

        self.server_socket = None
        self.client_socket = None
        self.client_lock = threading.Lock()

        self.samples_acquired = 0
        self.samples_sent = 0
        self.total_samples_sent = 0  # Across all client sessions
        self.start_time = None
        self.session_count = 0

    
    def _get_metadata(self) -> dict:
        """Generate metadata for client."""
        total_channels = sum(dev.config.num_daisy_devices * 8
                           for dev in self.spi_devices)
        # Check if any device has interpolation enabled (they should all be the same)
        server_interpolation = self.spi_devices[0].interpolation_enabled if self.spi_devices else True
        return {
            "format": "json",
            "sample_rate": 250,
            "num_channels": total_channels,
            "num_devices": sum(dev.config.num_daisy_devices
                              for dev in self.spi_devices),
            "ports": [dev.config.port_id for dev in self.spi_devices],
            "server_interpolation": server_interpolation
        }
    
    def start_server(self) -> bool:
        """Start the server socket (but don't wait for client yet)."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)  # Non-blocking accept with timeout
            
            total_channels = sum(dev.config.num_daisy_devices * 8 for dev in self.spi_devices)
            print(f"\n{'='*60}\nServer: {self.host}:{self.port}, {total_channels}ch @ 250Hz\n{'='*60}")
            
            return True
            
        except Exception as e:
            print(f"Error starting server: {e}")
            return False
    
    def wait_for_client(self) -> bool:
        """Wait for a client to connect. Returns True if connected, False if stop requested."""
        print(f"\nWaiting for client connection...")
        
        while not self.stop_event.is_set():
            try:
                self.client_socket, client_addr = self.server_socket.accept()
                self.client_socket.setblocking(False)
                
                self.session_count += 1
                print(f"\n{'='*60}\nCLIENT CONNECTED (Session #{self.session_count}) from {client_addr}\n{'='*60}")

                # Send metadata
                metadata = self._get_metadata()
                self.client_socket.send((json.dumps(metadata) + '\n').encode('utf-8'))
                print(f"Streaming {metadata['num_channels']} channels @ 250 Hz")
                
                # Reset session stats
                self.samples_sent = 0
                
                # Clear any old data from queue
                while not self.data_queue.empty():
                    try:
                        self.data_queue.get_nowait()
                    except queue.Empty:
                        break
                
                self.client_connected.set()
                return True
                
            except socket.timeout:
                continue
            except Exception as e:
                if not self.stop_event.is_set():
                    print(f"Accept error: {e}")
                return False
        
        return False
    
    def disconnect_client(self):
        """Cleanly disconnect the current client."""
        try:
            with self.client_lock:
                self.client_connected.clear()
                if self.client_socket:
                    try:
                        self.client_socket.close()
                    except:
                        pass
                    self.client_socket = None
        except KeyboardInterrupt:
            # Handle Ctrl+C during cleanup gracefully
            self.client_connected.clear()
            self.client_socket = None

    def _trigger_recovery(self):
        """Trigger recovery to re-synchronize all devices after timing violations."""
        if not hasattr(self, '_recovery_count'):
            self._recovery_count = 0
        self._recovery_count += 1

        print(f"  Recovery #{self._recovery_count}: Stopping all conversions...")

        # Stop all conversions using helper methods
        ADS1299_Controller._set_all_start_pins_low(self.spi_devices)
        ADS1299_Controller._broadcast_command(self.spi_devices, ADS1299_Cmd.STOP, delay_ms=10)
        ADS1299_Controller._broadcast_command(self.spi_devices, ADS1299_Cmd.SDATAC, delay_ms=50)

        # Re-enter RDATAC mode
        print(f"  Recovery #{self._recovery_count}: Re-entering RDATAC mode...")
        for spi_dev in self.spi_devices:
            spi_dev.send_command(ADS1299_Cmd.RDATAC)
            time.sleep(0.002)
        time.sleep(0.05)  # Settling time

        # Re-assert START simultaneously using helper
        print(f"  Recovery #{self._recovery_count}: Re-asserting START pins...")
        start_controller = ADS1299_Controller._create_start_controller(self.spi_devices)
        start_controller.set_all_high()
        time.sleep(0.05)

        # Verify DRDY is active
        if self.spi_devices[0].wait_for_drdy(timeout=0.1):
            print(f"  Recovery #{self._recovery_count}: DRDY active, resuming acquisition ✓")
        else:
            print(f"  Recovery #{self._recovery_count}: WARNING - DRDY not active after recovery")

        # Discard a few samples to allow settling
        for _ in range(10):
            if self.spi_devices[0].wait_for_drdy(timeout=0.02):
                for spi_dev in self.spi_devices:
                    spi_dev.read_data()

        print(f"  Recovery #{self._recovery_count}: Complete, continuing acquisition")
    
    def acquisition_thread(self):
        """Thread for acquiring data from ADS1299 devices. Uses reference port DRDY only."""
        # Try to set real-time priority for this thread
        if set_realtime_priority(50):
            print("  Acquisition thread: Real-time priority set (SCHED_FIFO, priority 50)")
        elif set_high_priority():
            print("  Acquisition thread: High priority set (nice -10)")
        else:
            print("  Acquisition thread: Running at normal priority (run as root for better timing)")

        sample_number = 0
        consecutive_drdy_timeouts = 0
        consecutive_slow_reads = 0  # Track consecutive slow reads for recovery

        # Use Port1 as the reference for DRDY timing (it has fewest devices, fastest read)
        reference_port = self.spi_devices[0]

        while not self.stop_event.is_set():
            try:
                # Wait for reference port's DRDY only - others should be ready simultaneously
                # since START pins are synchronized. This minimizes I2C overhead.
                if not reference_port.wait_for_drdy(timeout=0.008):  # 8ms timeout (2 sample periods)
                    consecutive_drdy_timeouts += 1

                    # Log first timeout and periodically after
                    if consecutive_drdy_timeouts == 1:
                        print(f"WARNING: DRDY timeout on {reference_port.config.port_id}")
                    elif consecutive_drdy_timeouts % 50 == 0:
                        print(f"WARNING: {consecutive_drdy_timeouts} consecutive DRDY timeouts")
                    continue

                # Reset timeout counter on success
                consecutive_drdy_timeouts = 0

                timestamp = time.time()
                if self.start_time is None:
                    self.start_time = timestamp

                # Small post-DRDY settling delay for daisy-chain data to stabilize
                # This helps prevent last-device corruption by allowing signal propagation
                # At 250 Hz (4ms budget), 20μs is 0.5% overhead but helps data integrity
                time.sleep(0.000020)  # 20 microseconds

                # Read all ports sequentially - NO delays between ports
                # Each port is on a separate SPI bus, so no contention issues.
                # Speed is critical: we have only 4ms per sample at 250 Hz.
                all_channels = []
                read_times = []  # Track timing of each port read
                corruption_this_sample = []
                read_start_time = time.time()

                for spi_dev in self.spi_devices:
                    port_read_start = time.time()
                    status, channel_data = spi_dev.read_data()
                    port_read_end = time.time()
                    read_times.append((spi_dev.config.port_id, (port_read_end - port_read_start) * 1000))  # ms

                    # Check if this read had corruption (status byte check)
                    if (status[0] & 0xF0) != 0xC0:
                        corruption_this_sample.append(spi_dev.config.port_id)

                    for device_channels in channel_data:
                        all_channels.extend(device_channels)

                # Track total read time and warn if approaching limit
                total_read_time = (time.time() - read_start_time) * 1000
                if total_read_time > 3.5:  # Getting close to 4ms limit
                    if not hasattr(self, '_slow_read_count'):
                        self._slow_read_count = 0
                    self._slow_read_count += 1
                    consecutive_slow_reads += 1
                    if self._slow_read_count <= 10 or self._slow_read_count % 100 == 0:
                        timing_str = ', '.join([f'{p}:{t:.2f}ms' for p, t in read_times])
                        print(f"SLOW READ #{self._slow_read_count}: Total {total_read_time:.2f}ms (limit 4ms)")
                        print(f"  Per-port: [{timing_str}]")

                    # If we have 5+ consecutive slow reads with corruption, trigger recovery
                    if consecutive_slow_reads >= 5 and len(corruption_this_sample) > 0:
                        print(f"\n⚠ TIMING VIOLATION: {consecutive_slow_reads} consecutive slow reads with corruption")
                        print(f"  Triggering automatic recovery to re-synchronize devices...")
                        self._trigger_recovery()
                        consecutive_slow_reads = 0
                        continue
                else:
                    consecutive_slow_reads = 0  # Reset counter on good read

                # Log if multiple ports corrupted in same sample (indicates systemic issue)
                if len(corruption_this_sample) > 1:
                    if not hasattr(self, '_multi_port_corruption_count'):
                        self._multi_port_corruption_count = 0
                    self._multi_port_corruption_count += 1

                    if self._multi_port_corruption_count <= 10 or self._multi_port_corruption_count % 20 == 0:
                        timing_str = ', '.join([f'{p}:{t:.2f}ms' for p, t in read_times])
                        print(f"MULTI-PORT CORRUPTION #{self._multi_port_corruption_count} at sample {sample_number}: "
                              f"Ports affected: {corruption_this_sample}")
                        print(f"  Read timings: [{timing_str}]")
                        print(f"  Time since DRDY: {(time.time() - timestamp) * 1000:.2f}ms")

                sample = {
                    "type": "sample",
                    "timestamp": timestamp - self.start_time,
                    "sample_number": sample_number,
                    "channels": all_channels
                }

                # Only queue data if client is connected
                if self.client_connected.is_set():
                    try:
                        self.data_queue.put_nowait(sample)
                    except queue.Full:
                        # Drop oldest sample to make room
                        try:
                            self.data_queue.get_nowait()
                            self.data_queue.put_nowait(sample)
                        except queue.Empty:
                            pass

                self.samples_acquired += 1
                sample_number += 1

                # Periodic status logging (every 5 seconds at 250 Hz)
                if sample_number % 1250 == 0:
                    elapsed = time.time() - self.start_time
                    rate = sample_number / elapsed if elapsed > 0 else 0
                    client_status = "connected" if self.client_connected.is_set() else "waiting"

                    # Calculate total corruption across all ports
                    total_corruptions = sum(dev.corruption_count for dev in self.spi_devices)
                    total_reads = sum(dev.total_reads for dev in self.spi_devices)
                    overall_rate = (total_corruptions / total_reads * 100) if total_reads > 0 else 0

                    print(f"Acquired: {sample_number}, Rate: {rate:.1f} Hz, Client: {client_status}, "
                          f"Total corruptions: {total_corruptions} ({overall_rate:.3f}%)")

                    # Log per-port corruption stats (only if there are corruptions)
                    for spi_dev in self.spi_devices:
                        if spi_dev.corruption_count > 0:
                            corruption_rate = (spi_dev.corruption_count / spi_dev.total_reads * 100) if spi_dev.total_reads > 0 else 0

                            # Warn if corruption rate is high (potential cascade failure)
                            if corruption_rate > 1.0:
                                status = "⚠ CASCADE WARNING"
                            elif corruption_rate > 0.1:
                                status = "elevated"
                            else:
                                status = "low"

                            # Also show out-of-range count if available
                            oor_str = ""
                            if hasattr(spi_dev, '_out_of_range_count') and spi_dev._out_of_range_count > 0:
                                oor_str = f", OOR: {spi_dev._out_of_range_count}"

                            print(f"  {spi_dev.config.port_id}: {spi_dev.corruption_count} corruptions "
                                  f"({corruption_rate:.2f}% of {spi_dev.total_reads} reads) [{status}]{oor_str}")

                    # Log multi-port corruption summary if it occurred
                    if hasattr(self, '_multi_port_corruption_count') and self._multi_port_corruption_count > 0:
                        print(f"  Multi-port corruption events: {self._multi_port_corruption_count}")

                    # Log slow read warnings
                    if hasattr(self, '_slow_read_count') and self._slow_read_count > 0:
                        print(f"  Slow reads (>3.5ms): {self._slow_read_count}")

                    # Log interpolation stats
                    total_interpolations = sum(dev.interpolation_count for dev in self.spi_devices)
                    if total_interpolations > 0:
                        print(f"  Interpolated samples: {total_interpolations}")

                    # Log recovery count
                    if hasattr(self, '_recovery_count') and self._recovery_count > 0:
                        print(f"  Automatic recoveries: {self._recovery_count}")

            except Exception as e:
                print(f"Acquisition error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    def streaming_thread(self):
        """Thread for streaming data to network client."""
        while not self.stop_event.is_set():
            # Wait for client to be connected
            if not self.client_connected.wait(timeout=0.5):
                continue
            
            try:
                sample = self.data_queue.get(timeout=0.1)
                sample_line = json.dumps(sample) + '\n'
                
                with self.client_lock:
                    if self.client_socket and self.client_connected.is_set():
                        if not self._send_nonblocking(sample_line.encode('utf-8')):
                            print(f"\n{'='*60}\nCLIENT DISCONNECTED\n{'='*60}")
                            print(f"Session #{self.session_count}: {self.samples_sent} samples sent")
                            self.total_samples_sent += self.samples_sent
                            self.disconnect_client()
                            continue
                        
                        self.samples_sent += 1
                
            except queue.Empty:
                continue
            except Exception as e:
                if self.client_connected.is_set():
                    print(f"Streaming error: {e}")
                    self.disconnect_client()
    
    def _send_nonblocking(self, data: bytes, timeout: float = 0.1) -> bool:
        """Send data on non-blocking socket with timeout."""
        total_sent = 0
        data_len = len(data)
        start_time = time.time()
        
        while total_sent < data_len:
            if time.time() - start_time > timeout:
                return False
            
            try:
                sent = self.client_socket.send(data[total_sent:])
                total_sent += sent
            except BlockingIOError:
                time.sleep(0.0001)
                continue
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                return False
        
        return True
    
    def run(self):
        """Main server loop with reconnection support."""
        print(f"\n{'='*60}\nSTARTING DATA ACQUISITION\n{'='*60}\n")
        
        # Verify all devices are converting - check each port's DRDY individually
        print("  Verifying all devices are converting...")
        for spi_dev in self.spi_devices:
            if spi_dev.wait_for_drdy(timeout=0.1):
                print(f"    {spi_dev.config.port_id}: DRDY active ✓")
            else:
                print(f"    {spi_dev.config.port_id}: DRDY not active, restarting...")
                spi_dev.start.set_low()
                time.sleep(0.01)
                spi_dev.send_command(ADS1299_Cmd.RDATAC)
                time.sleep(0.001)
                spi_dev.start.set_high()
                time.sleep(0.01)

        # Do a synchronized read to clear any pending data
        if wait_for_all_drdy(self.spi_devices, timeout=0.1):
            for spi_dev in self.spi_devices:
                spi_dev.read_data()

        # Reset corruption counters so startup errors don't affect steady-state stats
        print("  Resetting corruption counters...")
        for spi_dev in self.spi_devices:
            spi_dev.reset_stats()

        self.streaming_active = True
        
        # Start acquisition thread (runs continuously)
        acq_thread = threading.Thread(target=self.acquisition_thread, daemon=True)
        acq_thread.start()
        
        # Start streaming thread (handles client connections)
        stream_thread = threading.Thread(target=self.streaming_thread, daemon=True)
        stream_thread.start()
        
        print(f"\n{'='*60}\nSERVER RUNNING - Ctrl+C to stop, clients reconnect anytime\n{'='*60}\n")
        
        # Main loop - wait for clients and handle reconnection
        try:
            while not self.stop_event.is_set():
                if not self.client_connected.is_set():
                    # Wait for new client
                    self.wait_for_client()
                else:
                    # Client connected, just wait
                    time.sleep(1)
                    
        except KeyboardInterrupt:
            print("\n\nShutdown requested...")
        
        # Shutdown
        print("\nStopping server...")
        self.stop_event.set()
        self.disconnect_client()
        
        acq_thread.join(timeout=2)
        stream_thread.join(timeout=2)
        
        # Stop ADS1299 devices
        for spi_dev in self.spi_devices:
            spi_dev.start.set_low()
            spi_dev.send_command(ADS1299_Cmd.STOP)
        
        # Final statistics
        print(f"\n{'='*60}\nFINAL STATISTICS\n{'='*60}")
        elapsed = time.time() - self.start_time if self.start_time else 0
        rate = self.samples_acquired / elapsed if elapsed > 0 else 0
        print(f"Sessions: {self.session_count}, Acquired: {self.samples_acquired}, Sent: {self.total_samples_sent + self.samples_sent}")
        print(f"Runtime: {elapsed:.1f}s, Rate: {rate:.1f}Hz\n{'='*60}\n")
    
    def cleanup(self):
        """Clean up resources."""
        self.disconnect_client()
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        GPIO.cleanup()

# --- Main Entry Point ---

def main():
    """Main entry point for ADS1299 streaming system."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="ADS1299 EEG Streaming System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Default Configuration:
  Port1: SPI 0.0, 3 daisy-chained devices (24 channels)
  Port2: SPI 0.1, 5 daisy-chained devices (40 channels)
  Port3: SPI 3.0, 5 daisy-chained devices (40 channels)
  Port4: SPI 4.1, 5 daisy-chained devices (40 channels)
  Port5: SPI 4.0, 3 daisy-chained devices (24 channels)
  
  Total: 21 devices, 168 channels @ 250 Hz

Examples:
  # Stream with default headset configuration
  python3 HeadsetServer.py
  
  # Stream from three separate SPI buses
  python3 HeadsetServer.py --ports 3,0,Port1,1 4,1,Port2,1 5,0,Port3,1
  
Port format: bus,device,name,num_daisy
  - bus: SPI bus number (0-5)
  - device: SPI device number (0-1)  
  - name: Descriptive name (e.g. Port1, FrontHead, etc)
  - num_daisy: Number of ADS1299s daisy-chained on this bus
"""
    )
    
    parser.add_argument("--ports", nargs='+', 
                       default=DEFAULT_PORTS,
                       help="SPI port configurations (format: bus,device,name,num_daisy)")
    parser.add_argument("--host", type=str, default='0.0.0.0',
                       help="Server host address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8888,
                       help="Server port number (default: 8888)")
    parser.add_argument("--test-mode", type=str, choices=['normal', 'test-signal', 'input-short'],
                       default='test-signal',
                       help="Operating mode: normal, test-signal (1mV square wave), or input-short (noise test)")
    parser.add_argument("--debug-channels", action='store_true',
                       help="Enable channel debugging output (shows first few samples)")
    parser.add_argument("--no-interpolation", action='store_true',
                       help="Send raw data without interpolation (for client-side processing)")

    args = parser.parse_args()
    
    # Parse port configurations
    bus_configs = []
    for port_str in args.ports:
        parts = port_str.split(',')
        if len(parts) != 4:
            print(f"Error: Invalid port format '{port_str}'")
            print("Expected format: bus,device,name,num_daisy")
            return
        
        try:
            bus = int(parts[0])
            device = int(parts[1])
            name = parts[2]
            num_daisy = int(parts[3])
            
            config = get_bus_config(bus, device, name, num_daisy)
            bus_configs.append(config)
            
        except (ValueError, IndexError) as e:
            print(f"Error parsing port '{port_str}': {e}")
            return
    
    total_devices = sum(cfg.num_daisy_devices for cfg in bus_configs)
    total_channels = total_devices * 8
    print(f"\n{'='*60}\nADS1299 EEG STREAMING SYSTEM\n{'='*60}")
    print(f"Config: {len(bus_configs)} buses, {total_devices} devices, {total_channels} ch @ 250Hz, {args.host}:{args.port}\n")
    
    # Initialize GPIO
    ADS1299_Controller.initialize_gpio()
    
    # CRITICAL: Force all START pins LOW before initialization
    ADS1299_Controller.force_all_start_pins_low(bus_configs)
    
    # Wait for devices to be ready (V2: VCAP1 charging, no GPIO reset)
    ADS1299_Controller.wait_for_power_on()
    
    # Initialize SPI devices
    interpolation_enabled = not args.no_interpolation
    print(f"\nInitializing SPI devices (interpolation: {'disabled' if args.no_interpolation else 'enabled'})...")
    spi_devices = []

    for bus_cfg in bus_configs:
        spi_dev = ADS1299_SPI(bus_cfg, interpolation_enabled=interpolation_enabled)
        spi_devices.append(spi_dev)
    
    # Configure all devices
    print(f"\n{'='*60}\nCONFIGURING DEVICES\n{'='*60}")

    # Select configuration based on test mode
    if args.test_mode == 'test-signal':
        config = ADS1299_Config.test_signal_config()
        print("Using TEST SIGNAL mode (1mV square wave)")
    elif args.test_mode == 'input-short':
        config = ADS1299_Config.input_short_config()
        print("Using INPUT SHORT mode (noise measurement)")
    else:
        config = ADS1299_Config.normal_config()
        print("Using NORMAL mode")
    
    # Configure each device (single attempt each - no retries)
    for spi_dev in spi_devices:
        success, _ = ADS1299_Controller.initialize_device(spi_dev, config)
        if not success:
            print(f"\n✗ {spi_dev.config.port_id} failed to initialize")
            GPIO.cleanup()
            return
        time.sleep(0.1)  # Gap between ports

    print(f"\n✓ All devices configured successfully!")

    # Now verify data flow
    print(f"\n{'='*60}\nVERIFYING DATA FLOW\n{'='*60}")

    data_flow_ok = ADS1299_Controller.start_all_conversions_synchronized(spi_devices)

    if not data_flow_ok:
        print(f"\n⚠ Data flow verification failed")

        # Check if DRDY is at least active - if so, devices may still work
        drdy_active = False
        for spi_dev in spi_devices:
            if spi_dev.wait_for_drdy(timeout=0.1):
                drdy_active = True
                break

        if drdy_active:
            print(f"DRDY active - devices may work. Continuing with monitoring...")
        else:
            print(f"DRDY NOT active - possible: connection, clock, power, or chip issue")
            response = input("Continue anyway? (y/n): ").strip().lower()
            if response != 'y':
                print("Aborting.")
                GPIO.cleanup()
                return
    else:
        print(f"\n{'='*60}\n✓ SYSTEM INITIALIZATION COMPLETE!\n{'='*60}")
    
    # Create streaming server
    server = StreamingServer(spi_devices, host=args.host, port=args.port)
    
    try:
        # Start server and wait for client
        if not server.start_server():
            print("Failed to start server")
            return
        
        # Run acquisition and streaming
        server.run()
        
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        server.cleanup()
        print("System shutdown complete")


if __name__ == "__main__":
    main()