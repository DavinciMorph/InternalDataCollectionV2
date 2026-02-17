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
    python3 controller.py
    python3 controller.py --ports 0,0,Port1,1
"""

import spidev
import time
import RPi.GPIO as GPIO
import smbus2
import socket
import json
import struct
import threading
import queue
import os
import csv
import gc
import lz4.frame
from collections import deque
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
SPI_SPEED_HZ = 6_000_000  # 6 MHz - faster reads for more timing margin
                          # At 250 Hz sample rate with 21 devices x 27 bytes = 567 bytes/sample
                          # 4 MHz was tested but increased slow reads significantly
                          # 2 MHz was causing sample rate to drop to 130 Hz due to read time

# ============================================================
# PORT CONFIGURATION — Modify device counts here
# Format: "bus,device,PortName,num_daisy_devices"
# Change the last number to set how many daisy-chained ADS1299s per port
# ============================================================
DEFAULT_PORTS = [
    "0,0,Port1,4",   # SPI 0.0, OUT0 — 1 device (change to 3 for daisy chain of 3)
    "0,1,Port2,4",   # SPI 0.1, OUT1
    "3,0,Port3,4",   # SPI 3.0, OUT2
    "3,1,Port4,4",   # SPI 3.1, OUT3
    "4,0,Port5,4",   # SPI 4.0, OUT4
    "4,1,Port6,4",   # SPI 4.1, OUT5
    "5,0,Port7,4",   # SPI 5.0, OUT6
]


# --- Register Configuration Values ---

@dataclass
class ADS1299_Config:
    """Standard register configuration for ADS1299"""
    # Global settings
    config1: int = 0x96   # Daisy-chain mode, 250 SPS, external clock
    config2: int = 0xD0   # TEST MODE: internal test signal ON (standard: 0xC0 = test signal off)
    config3: int = 0xE0   # Internal BIASREF, BIAS buffer off
    config4: int = 0x00   # Continuous conversion, lead-off comp off

    # Channel settings (individual configuration for each channel)
    ch1set: int = 0x05    # TEST MODE: gain=1, test signal input (standard: 0x60 = gain=24, normal input)
    ch2set: int = 0x05    # TEST MODE: gain=1, test signal input (standard: 0x60 = gain=24, normal input)
    ch3set: int = 0x05    # TEST MODE: gain=1, test signal input (standard: 0x60 = gain=24, normal input)
    ch4set: int = 0x05    # TEST MODE: gain=1, test signal input (standard: 0x60 = gain=24, normal input)
    ch5set: int = 0x05    # TEST MODE: gain=1, test signal input (standard: 0x60 = gain=24, normal input)
    ch6set: int = 0x05    # TEST MODE: gain=1, test signal input (standard: 0x60 = gain=24, normal input)
    ch7set: int = 0x05    # TEST MODE: gain=1, test signal input (standard: 0x60 = gain=24, normal input)
    ch8set: int = 0x05    # TEST MODE: gain=1, test signal input (standard: 0x60 = gain=24, normal input)
    
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
    
    This implements the hardware mapping from the custom CM4 expansion board V2.
    
    Output Port Mapping (directly to sensor connectors):
        OUT0 -> SPI0.CE0 (bus=0, device=0) -> DRDY_0, START_0
        OUT1 -> SPI0.CE1 (bus=0, device=1) -> DRDY_1, START_1
        OUT2 -> SPI3.CE0 (bus=3, device=0) -> DRDY_2, START_2
        OUT3 -> SPI3.CE1 (bus=3, device=1) -> DRDY_3, START_3
        OUT4 -> SPI4.CE0 (bus=4, device=0) -> DRDY_4, START_4
        OUT5 -> SPI4.CE1 (bus=4, device=1) -> DRDY_5, START_5
        OUT6 -> SPI5.CE0 (bus=5, device=0) -> DRDY_6, START_6
    
    TCA9534 Pin Mapping:
        DRDY (0x20):  P0=DRDY_0, P1=DRDY_6, P2=DRDY_5, P3=DRDY_4, P4=DRDY_3, P5=DRDY_2, P6=DRDY_1, P7=GND
        START (0x21): P0=START_0, P1=START_6, P2=START_5, P3=START_4, P4=START_3, P5=START_2, P6=START_1, P7=GND
    """
    # Map (bus, device) to output number
    bus_device_to_output = {
        (0, 0): 0,  # OUT0
        (0, 1): 1,  # OUT1
        (3, 0): 2,  # OUT2
        (3, 1): 3,  # OUT3
        (4, 0): 4,  # OUT4
        (4, 1): 5,  # OUT5
        (5, 0): 6,  # OUT6
    }
    
    # Map output number to TCA9534 pin number
    # P0=OUT0, P1=OUT6, P2=OUT5, P3=OUT4, P4=OUT3, P5=OUT2, P6=OUT1
    output_to_pin = {
        0: 0,  # OUT0 -> P0
        1: 6,  # OUT1 -> P6
        2: 5,  # OUT2 -> P5
        3: 4,  # OUT3 -> P4
        4: 3,  # OUT4 -> P3
        5: 2,  # OUT5 -> P2
        6: 1,  # OUT6 -> P1
    }
    
    key = (bus, device)
    if key not in bus_device_to_output:
        raise ValueError(f"Unsupported SPI bus/device combination: SPI{bus}.{device}")
    
    output_num = bus_device_to_output[key]
    pin_num = output_to_pin[output_num]
    
    # Both DRDY and START use the same pin number on their respective expanders
    drdy_addr = 0x20
    start_addr = 0x21
    
    return SPIBusConfig(
        bus_num=bus,
        device_num=device,
        port_id=port_id,
        num_daisy_devices=num_daisy,
        drdy_i2c_addr=drdy_addr,
        drdy_pin=pin_num,
        start_i2c_addr=start_addr,
        start_pin=pin_num
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

    def __init__(self, bus_config: SPIBusConfig):
        self.config = bus_config
        self.initialized = False
        self.running = False

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

    @staticmethod
    def wait_4tclk():
        # Minimum wait after SPI commands per ADS1299 datasheet: 4 tCLK (~2us at 2.048 MHz)
        # Keep this minimal during data acquisition - timing is critical at 250 Hz.
        time.sleep(0.000002)  # 2 microseconds - datasheet minimum
    
    def flush_spi(self):
        frame_size = self.config.num_daisy_devices * 27  # Full daisy-chain frame
        self.spi.xfer2([0x00] * frame_size)
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

    def read_raw(self) -> list:
        """SPI transfer only — returns raw bytes, no parsing.

        Use this in time-critical paths, then call parse_raw() afterwards.
        """
        return self.spi.xfer2([0x00] * (self.config.num_daisy_devices * 27))

    @staticmethod
    def parse_raw(raw_data: list, num_devices: int) -> Tuple[List[int], List[List[int]]]:
        """Parse raw SPI bytes into status and channel data.

        Returns (status_bytes, channel_data) where channel_data is a list of
        per-device channel lists.
        """
        bytes_per_device = 27
        channel_data = []
        first_status = raw_data[0:3]

        for device_idx in range(num_devices):
            device_offset = device_idx * bytes_per_device
            ch_data_offset = device_offset + 3

            device_channels = []
            for ch in range(8):
                byte_offset = ch_data_offset + (ch * 3)
                value = (raw_data[byte_offset] << 16) | (raw_data[byte_offset + 1] << 8) | raw_data[byte_offset + 2]
                if value & 0x800000:
                    value = value - 0x1000000
                device_channels.append(value)

            channel_data.append(device_channels)

        return first_status, channel_data

    def read_data(self) -> Tuple[List[int], List[List[int]]]:
        """Read and parse data from all daisy-chained devices.

        Returns (status_bytes, channel_data). For time-critical paths,
        use read_raw() + parse_raw() separately to defer parsing.
        """
        return self.parse_raw(self.read_raw(), self.config.num_daisy_devices)


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

        time.sleep(0.00005)  # 50us polling - faster than before


# --- Parallel SPI Bus Worker ---

class SPIBusWorker:
    """Persistent thread that reads all ports on a single SPI bus.

    Multiple SPIBusWorkers run in parallel — one per physical SPI bus.
    spidev.xfer2() releases the GIL during the kernel ioctl, enabling
    true parallel SPI transfers across independent buses (SPI0, SPI3, SPI4, SPI5).

    Ports on the SAME bus are read sequentially (shared SCLK/MOSI/MISO),
    but ports on DIFFERENT buses are read simultaneously.
    """

    def __init__(self, bus_num: int, spi_devices: List[ADS1299_SPI]):
        self.bus_num = bus_num
        self.spi_devices = spi_devices
        self.raw_buffers = [None] * len(spi_devices)
        self._request = threading.Event()
        self._done = threading.Event()
        self._done.set()  # Start in "done" state (ready for first trigger)
        self._running = True
        self._thread = threading.Thread(
            target=self._run, daemon=True, name=f"spi-bus{bus_num}-worker")
        self._thread.start()

    def _run(self):
        # Set real-time priority (slightly below main acquisition thread)
        set_realtime_priority(49)

        while True:
            self._request.wait()
            self._request.clear()
            if not self._running:
                return
            for i, dev in enumerate(self.spi_devices):
                self.raw_buffers[i] = dev.read_raw()
            self._done.set()

    def trigger(self):
        """Signal worker to start reading (non-blocking)."""
        self._done.clear()
        self._request.set()

    def wait(self):
        """Block until reads are complete."""
        self._done.wait()

    def stop(self):
        """Shut down the worker thread."""
        self._running = False
        self._request.set()  # Unblock _run() if waiting
        self._thread.join(timeout=2.0)


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
            status = '[OK]' if success else '[FAIL]'
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
            test_status, _ = spi_dev.read_data()  # Suppress verification logging
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
                    
                    print(f"  I2C 0x{addr:02X}: START pins {pins} set LOW [OK]")
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
        print("Device power-on wait complete [OK]")
    
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

        # 3. Multiple software RESETs to ensure all devices settle properly
        #    Some devices may need multiple reset cycles to reach a stable state
        reset_time = 1.0 + (num_devices * 0.1)  # Per-reset settling time
        num_resets = 5
        print(f"  Performing {num_resets} software resets ({reset_time:.1f}s each for {num_devices} devices)...")
        for reset_num in range(num_resets):
            spi.send_command(ADS1299_Cmd.RESET)
            time.sleep(reset_time)
            # Send SDATAC between resets to clear any pending data mode
            spi.send_command(ADS1299_Cmd.SDATAC)
            time.sleep(0.05)
        print(f"  Reset cycles complete")

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
        for attempt in range(100):
            spi.write_registers(ADS1299_Reg.LOFF, [test_val])
            time.sleep(0.1)
            readback = spi.read_registers(ADS1299_Reg.LOFF, 1)[0]
            if readback == test_val:
                sdatac_ok = True
                if attempt > 0:
                    print(f"  SDATAC verified on attempt {attempt + 1} [OK]")
                else:
                    print(f"  SDATAC verified [OK]")
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

        # 5. Verify device ID - retry if corrupted read
        id_ok = False
        for attempt in range(100):
            id_value = spi.read_registers(ADS1299_Reg.ID, 1)[0]
            if id_value == 0x3E:
                id_ok = True
                if attempt > 0:
                    print(f"  Device ID: 0x{id_value:02X} (attempt {attempt + 1})")
                else:
                    print(f"  Device ID: 0x{id_value:02X}")
                break
            time.sleep(0.05)

        if not id_ok:
            print(f"  Device ID: 0x{id_value:02X}")
            print(f"  [FAIL] Expected ID 0x3E after 10 attempts")
            return False, 1
        print(f"  [OK] ADS1299 detected")

        # 6. Write CONFIG registers - retry each until it sticks
        print(f"  Writing configuration...")

        def write_and_verify(reg, value, name, max_attempts=100):
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

        # 9. Final verification - re-verify each CONFIG with retry if needed
        #    Device may slip back to RDATAC, so retry reads
        def verify_register(reg, expected, name):
            for attempt in range(5):
                val = spi.read_registers(reg, 1)[0]
                if val == expected:
                    return val, True
                time.sleep(0.02)
                # Send SDATAC in case device slipped back
                spi.send_command(ADS1299_Cmd.SDATAC)
                time.sleep(0.02)
            return val, False

        c1_read, c1_ok = verify_register(ADS1299_Reg.CONFIG1, config.config1, "CONFIG1")
        c2_read, c2_ok = verify_register(ADS1299_Reg.CONFIG2, config.config2, "CONFIG2")
        c3_read, c3_ok = verify_register(ADS1299_Reg.CONFIG3, config.config3, "CONFIG3")

        c1 = '[OK]' if c1_ok else '[FAIL]'
        c2 = '[OK]' if c2_ok else '[FAIL]'
        c3 = '[OK]' if c3_ok else '[FAIL]'
        print(f"  CONFIG1=0x{c1_read:02X}{c1} CONFIG2=0x{c2_read:02X}{c2} CONFIG3=0x{c3_read:02X}{c3}")

        if c1_ok and c2_ok and c3_ok:
            print(f"  [OK] {spi.config.port_id} configured successfully")
            return True, 1

        print(f"  [FAIL] {spi.config.port_id} configuration failed")
        return False, 1

    @staticmethod
    def start_conversion(spi: ADS1299_SPI):
        """Start continuous data acquisition on a single device."""
        spi.send_command(ADS1299_Cmd.RDATAC)
        time.sleep(0.001)
        spi.start.set_high()
        time.sleep(0.01)
        print(f"  {spi.config.port_id}: Acquisition started [OK]")
    
    @staticmethod
    def stop_conversion(spi: ADS1299_SPI):
        """Stop continuous data acquisition"""
        spi.start.set_low()
        spi.send_command(ADS1299_Cmd.STOP)
        time.sleep(0.001)
    
    @staticmethod
    def start_all_conversions_synchronized(spi_devices: List['ADS1299_SPI'], config: 'ADS1299_Config' = None) -> bool:
        """Start conversions on ALL devices with synchronized START. Returns True if valid data."""
        print("\n  Starting conversions with synchronized START...")

        print("    Stopping any ongoing conversions...")
        ADS1299_Controller._set_all_start_pins_low(spi_devices)
        ADS1299_Controller._broadcast_command(spi_devices, ADS1299_Cmd.STOP, delay_ms=10)

        print("    Ensuring all devices in SDATAC mode...")
        ADS1299_Controller._broadcast_command(spi_devices, ADS1299_Cmd.SDATAC, delay_ms=50)

        print("    Sending RDATAC to all devices (fast sequential)...")
        # Send RDATAC as quickly as possible to minimize desync between ports
        for spi_dev in spi_devices:
            spi_dev.send_command(ADS1299_Cmd.RDATAC)
        time.sleep(0.050)  # Brief settle
        # Second RDATAC pass to ensure all devices received it
        for spi_dev in spi_devices:
            spi_dev.send_command(ADS1299_Cmd.RDATAC)

        time.sleep(0.750)  # 750ms settling after RDATAC

        print("    Verifying device communication after RDATAC...")
        rdatac_verify_failures = []
        for spi_dev in spi_devices:
            spi_dev.flush_spi()
            time.sleep(0.002)
            raw = spi_dev.spi.xfer2([0x00] * 3)
            if raw[0] == 0xFF and raw[1] == 0xFF and raw[2] == 0xFF:
                rdatac_verify_failures.append(spi_dev.config.port_id)
                print(f"      [WARN] {spi_dev.config.port_id}: SPI returns 0xFF - RDATAC may have failed")

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

        time.sleep(0.500)  # 500ms synchronization - longer delay for better sync

        for spi_dev in spi_devices:
            print(f"    {spi_dev.config.port_id}: START asserted [OK]")

        # Verify DRDY is actually toggling on each port
        print(f"\n  Verifying DRDY signals are active...")
        drdy_failures = []
        for spi_dev in spi_devices:
            if spi_dev.wait_for_drdy(timeout=0.1):
                print(f"    {spi_dev.config.port_id}: DRDY active [OK]")
            else:
                print(f"    {spi_dev.config.port_id}: DRDY NOT active [FAIL]")
                drdy_failures.append(spi_dev.config.port_id)

        if drdy_failures:
            print(f"  [WARN] DRDY failed on: {', '.join(drdy_failures)} - attempting restart...")
            for spi_dev in spi_devices:
                if spi_dev.config.port_id in drdy_failures:
                    restart_success = False
                    for attempt in range(3):
                        if ADS1299_Controller._restart_single_port(spi_dev, attempt):
                            print(f"    {spi_dev.config.port_id}: DRDY now active [OK] (attempt {attempt + 1})")
                            restart_success = True
                            break

                    if not restart_success:
                        print(f"    {spi_dev.config.port_id}: Restart failed after 3 attempts [FAIL]")

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
                    status, _ = spi_dev.read_data()  # Suppress warmup logging
                    if status[0] == 0x00:
                        zero_count += 1
                    elif (status[0] & 0xF0) == 0xC0:
                        valid_count += 1
                    time.sleep(0.004)

                if valid_count >= 3:
                    print(f"    {spi_dev.config.port_id}: DRDY [OK], valid status [OK] ({valid_count}/5 valid)")
                elif zero_count >= 3:
                    print(f"    {spi_dev.config.port_id}: DRDY [OK], but returning zeros ({zero_count}/5) [FAIL]")
                    ports_needing_restart.append(spi_dev)
                else:
                    print(f"    {spi_dev.config.port_id}: DRDY [OK], mixed status (valid={valid_count}, zero={zero_count})")
                    if zero_count > valid_count:
                        ports_needing_restart.append(spi_dev)
            else:
                print(f"    {spi_dev.config.port_id}: DRDY not active [FAIL]")
                ports_needing_restart.append(spi_dev)

        if ports_needing_restart:
            print(f"\n  Restarting {len(ports_needing_restart)} ports...")
            for spi_dev in ports_needing_restart:
                restart_success = False
                for attempt in range(3):
                    if ADS1299_Controller._restart_single_port(spi_dev, attempt, verbose=False):
                        print(f"    {spi_dev.config.port_id}: restart ok (attempt {attempt + 1}) [OK]")
                        restart_success = True
                        break
                if not restart_success:
                    print(f"    {spi_dev.config.port_id}: restart failed")
            time.sleep(0.200)

        print(f"\n  Discarding 500 warmup samples...")
        # Use port with longest daisy chain as reference for consistent DRDY timing
        reference_port = max(spi_devices, key=lambda d: d.config.num_daisy_devices)
        warmup_count = 0
        warmup_drdy_timeouts = 0
        warmup_corruptions = {spi_dev.config.port_id: 0 for spi_dev in spi_devices}
        consecutive_zeros = {spi_dev.config.port_id: 0 for spi_dev in spi_devices}
        port_restart_attempted = set()

        for i in range(500):
            if reference_port.wait_for_drdy(timeout=0.02):
                for spi_dev in spi_devices:
                    port_id = spi_dev.config.port_id
                    status, channel_data = spi_dev.read_data()

                    # Check status byte
                    if (status[0] & 0xF0) != 0xC0:
                        warmup_corruptions[port_id] += 1
                        if status[0] == 0x00:
                            consecutive_zeros[port_id] += 1
                        else:
                            consecutive_zeros[port_id] = 0
                    else:
                        consecutive_zeros[port_id] = 0

                    if (consecutive_zeros[port_id] >= 20 and
                        port_id not in port_restart_attempted):
                        port_restart_attempted.add(port_id)
                        print(f"    [WARN] {port_id}: 20+ zeros at sample {i}, restarting...")
                        restart_success = False
                        for restart_attempt in range(3):
                            if ADS1299_Controller._restart_single_port(spi_dev, restart_attempt, verbose=False):
                                print(f"      {port_id}: restart ok (attempt {restart_attempt+1}) [OK]")
                                consecutive_zeros[port_id] = 0
                                restart_success = True
                                break
                        if not restart_success:
                            print(f"      {port_id}: restart failed")

                warmup_count += 1
            else:
                warmup_drdy_timeouts += 1
                if warmup_drdy_timeouts <= 5:
                    print(f"    Warmup sample {i}: DRDY timeout on reference port")

        if warmup_drdy_timeouts > 0:
            print(f"  [WARN] {warmup_drdy_timeouts} DRDY timeouts during warmup ({warmup_drdy_timeouts/500*100:.1f}%)")

        total_warmup_corruptions = sum(warmup_corruptions.values())

        if total_warmup_corruptions > 0:
            print(f"  [WARN] Warmup STATUS corruption: {total_warmup_corruptions} total")
            for port_id, count in warmup_corruptions.items():
                if count > 0:
                    pct = count / warmup_count * 100 if warmup_count > 0 else 0
                    print(f"    {port_id}: {count} ({pct:.1f}%)")

        print(f"  Discarded {warmup_count} warmup samples")

        # Check for ports with elevated warmup corruption (>10%) and try aggressive re-init
        WARMUP_REINIT_THRESHOLD = 0.10  # 10% - trigger aggressive re-init

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
        # This does a FULL re-init: STOP, flush, RESET, SDATAC verify, register rewrite, RDATAC, START
        if ports_needing_reinit:
            print(f"\n  [WARN] {len(ports_needing_reinit)} port(s) with >10% warmup corruption - attempting aggressive re-init...")

            for spi_dev, rate in ports_needing_reinit:
                print(f"    {spi_dev.config.port_id}: {rate*100:.1f}% corruption - full re-init...")

                # 1. Stop the port completely
                spi_dev.start.set_low()
                time.sleep(0.050)
                spi_dev.send_command(ADS1299_Cmd.STOP)
                time.sleep(0.020)

                # 2. Full re-initialization: RESET + SDATAC verify + register rewrite
                if config is not None:
                    reinit_ok, _ = ADS1299_Controller.initialize_device(spi_dev, config)
                else:
                    # Fallback: just cycle commands (same as old behavior)
                    reinit_ok = False
                    spi_dev.flush_spi()
                    spi_dev.send_command(ADS1299_Cmd.SDATAC)
                    time.sleep(0.100)

                if reinit_ok:
                    # 3. Enter RDATAC and start conversion
                    spi_dev.send_command(ADS1299_Cmd.RDATAC)
                    time.sleep(0.010)
                    spi_dev.send_command(ADS1299_Cmd.RDATAC)
                    time.sleep(0.050)
                    spi_dev.start.set_high()
                    time.sleep(0.200)

                    # 4. Mini-warmup: verify data is clean
                    if spi_dev.wait_for_drdy(timeout=0.1):
                        mini_warmup_corruptions = 0
                        for _ in range(50):
                            if spi_dev.wait_for_drdy(timeout=0.02):
                                status, _ = spi_dev.read_data()
                                if (status[0] & 0xF0) != 0xC0:
                                    mini_warmup_corruptions += 1

                        mini_rate = mini_warmup_corruptions / 50
                        if mini_rate < 0.05:
                            print(f"      {spi_dev.config.port_id}: re-init successful ({mini_rate*100:.1f}% corruption) [OK]")
                            warmup_corruptions[spi_dev.config.port_id] = int(mini_rate * warmup_count)
                        else:
                            print(f"      {spi_dev.config.port_id}: re-init helped but still {mini_rate*100:.1f}% corruption")
                            warmup_corruptions[spi_dev.config.port_id] = int(mini_rate * warmup_count)
                    else:
                        print(f"      {spi_dev.config.port_id}: DRDY not active after re-init [FAIL]")
                else:
                    print(f"      {spi_dev.config.port_id}: re-initialization failed [FAIL]")

        print(f"\n  Verifying data flow (20 reads)...")
        # Use same reference as warmup and acquisition for consistency
        reference_port = max(spi_devices, key=lambda d: d.config.num_daisy_devices)
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
            print(f"  [WARN] {drdy_timeouts} DRDY timeouts during verification")

        all_ok = True
        for spi_dev in spi_devices:
            port_id = spi_dev.config.port_id
            count = valid_counts[port_id]
            samples = status_samples[port_id]

            if count >= 10:
                print(f"    [OK] {port_id}: Data flowing ({count}/{total_reads} valid)")
            else:
                sample_hex = ' '.join([f'0x{s:02X}' for s in samples[:5]])
                print(f"    [WARN] {port_id}: Low valid ({count}/{total_reads}), samples: {sample_hex}...")
                if count == 0:
                    all_ok = False

        if all_ok:
            print(f"\n  [OK] All {len(spi_devices)} devices verified!")
        else:
            print(f"\n  [WARN] Some ports had issues but may self-correct during streaming")

        # Final re-synchronization: stop and restart all devices together
        # This ensures all devices begin acquisition in perfect sync
        print(f"\n  Final re-synchronization...")

        # Stop all conversions
        start_controller = ADS1299_Controller._create_start_controller(spi_devices)
        start_controller.set_all_low()
        time.sleep(0.100)

        # Brief pause to let all devices settle
        ADS1299_Controller._broadcast_command(spi_devices, ADS1299_Cmd.STOP, delay_ms=10)
        time.sleep(0.200)

        # Flush all shift registers to clear stale data
        for spi_dev in spi_devices:
            spi_dev.flush_spi()

        # Re-enter RDATAC on all devices before restarting
        # Without this, a device that exited RDATAC during warmup would
        # not output data correctly after the re-sync restart
        ADS1299_Controller._broadcast_command(spi_devices, ADS1299_Cmd.SDATAC, delay_ms=20)
        ADS1299_Controller._broadcast_command(spi_devices, ADS1299_Cmd.RDATAC, delay_ms=10)
        ADS1299_Controller._broadcast_command(spi_devices, ADS1299_Cmd.RDATAC, delay_ms=20)

        # Restart all simultaneously
        start_controller.set_all_high()
        time.sleep(0.500)  # Allow devices to sync up

        # Discard first 100 samples after re-sync and collect diagnostics
        ref_port = max(spi_devices, key=lambda d: d.config.num_daisy_devices)

        # Discard first 100 post-resync samples and measure DRDY timing
        port_drdy_times = []

        print(f"\n  Post-resync settling (100 samples)...")

        for sample_idx in range(100):
            drdy_start = time.time()
            if ref_port.wait_for_drdy(timeout=0.02):
                drdy_elapsed = (time.time() - drdy_start) * 1000  # ms

                for spi_dev in spi_devices:
                    spi_dev.read_data()

                if sample_idx < 10:
                    port_drdy_times.append(drdy_elapsed)

        if port_drdy_times:
            avg_drdy = sum(port_drdy_times) / len(port_drdy_times)
            print(f"  DRDY timing (first 10): avg {avg_drdy:.2f}ms")

        print(f"  [OK] Re-synchronization complete")

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
        self.client_lock = threading.RLock()

        self.samples_acquired = 0
        self.samples_sent = 0
        self.total_samples_sent = 0  # Across all client sessions
        self.start_time = None
        self.session_count = 0

        # Binary LZ4 streaming constants
        self.batch_size = 10  # Samples per LZ4 frame
        self.num_channels = sum(dev.config.num_daisy_devices * 8
                                for dev in self.spi_devices)
        # Per-sample struct: <d = float64 timestamp, I = uint32 sample_number, Ni = N x int32 channels
        self.sample_struct_fmt = f'<dI{self.num_channels}i'
        self.sample_struct = struct.Struct(self.sample_struct_fmt)
        self.sample_size = self.sample_struct.size  # e.g. 684 bytes for 168 channels
        self.batch_buffer_size = self.sample_size * self.batch_size  # e.g. 6840 bytes
        # Frame header: uint32 compressed_size + uint32 sample_count
        self.header_struct = struct.Struct('<II')

    
    def _get_metadata(self) -> dict:
        """Generate metadata for client."""
        return {
            "format": "binary_lz4",
            "batch_size": self.batch_size,
            "sample_rate": 250,
            "num_channels": self.num_channels,
            "num_devices": sum(dev.config.num_daisy_devices
                              for dev in self.spi_devices),
            "ports": [dev.config.port_id for dev in self.spi_devices],
            "port_config": [
                {"name": dev.config.port_id,
                 "num_devices": dev.config.num_daisy_devices}
                for dev in self.spi_devices
            ],
            "sample_size": self.sample_size,
            "sample_struct": self.sample_struct_fmt
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
            print(f"\n{'='*60}\nServer: {self.host}:{self.port}, {total_channels}ch @ 250Hz (binary_lz4, batch={self.batch_size})\n{'='*60}")
            
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
                self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)

                self.session_count += 1
                print(f"\n{'='*60}\nCLIENT CONNECTED (Session #{self.session_count}) from {client_addr}\n{'='*60}")

                # Send metadata
                metadata = self._get_metadata()
                self.client_socket.send((json.dumps(metadata) + '\n').encode('utf-8'))
                print(f"Streaming {metadata['num_channels']} channels @ 250 Hz (binary_lz4, {self.sample_size}B/sample, batch={self.batch_size})")
                
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

    def acquisition_thread(self):
        """Thread for acquiring data from ADS1299 devices. LEAN VERSION - minimal overhead."""
        # Pin to isolated core 3 if available
        try:
            os.sched_setaffinity(0, {3})
            print("  Acquisition thread: Pinned to isolated core 3")
        except Exception:
            print("  Acquisition thread: Could not pin to core 3")

        # Try to set real-time priority for this thread
        if set_realtime_priority(50):
            print("  Acquisition thread: Real-time priority set (SCHED_FIFO, priority 50)")
        elif set_high_priority():
            print("  Acquisition thread: High priority set (nice -10)")
        else:
            print("  Acquisition thread: Running at normal priority (run as root for better timing)")

        sample_number = 0

        # Build list of all devices across all ports for ch1 logging
        # Each entry: (port_name, device_index_within_port, ch1_offset_in_all_channels)
        device_info = []
        ch_offset = 0
        for spi_dev in self.spi_devices:
            port_name = spi_dev.config.port_id.lower()
            for dev_idx in range(spi_dev.config.num_daisy_devices):
                device_info.append((port_name, dev_idx + 1, ch_offset))
                ch_offset += 8

        # CSV writing runs in a separate thread to keep SD card I/O
        # out of the time-critical acquisition loop.
        csv_path = 'all_ports_ch1_data.csv'
        csv_queue = deque(maxlen=50000)  # lock-free ring buffer for CSV rows
        csv_stop = threading.Event()
        csv_header = ['timestamp', 'sample_number'] + [
            f'{pn}_dev{di}_ch1' for pn, di, _ in device_info
        ]

        def csv_writer_thread():
            try:
                with open(csv_path, 'w', newline='', buffering=1048576) as csv_file:
                    writer = csv.writer(csv_file)
                    writer.writerow(csv_header)
                    while not csv_stop.is_set():
                        # Drain all pending rows
                        rows_written = 0
                        while csv_queue:
                            try:
                                row = csv_queue.popleft()
                                writer.writerow(row)
                                rows_written += 1
                            except IndexError:
                                break
                        if rows_written == 0:
                            time.sleep(0.05)  # 50ms idle sleep when no data
                    # Flush remaining rows on shutdown
                    while csv_queue:
                        try:
                            writer.writerow(csv_queue.popleft())
                        except IndexError:
                            break
            except Exception as e:
                print(f"  CSV writer thread error: {e}")

        csv_thread = threading.Thread(target=csv_writer_thread, daemon=True)
        csv_thread.start()
        print(f"  Logging ch1 from {len(device_info)} devices to {csv_path} (async writer)")

        # Pre-build DRDY lookup, excluding dead ports whose DRDY never goes active
        drdy_bus = self.spi_devices[0].drdy.bus
        drdy_masks: Dict[int, int] = {}
        for spi_dev in self.spi_devices:
            if spi_dev.wait_for_drdy(timeout=0.1):
                addr = spi_dev.config.drdy_i2c_addr
                pin_mask = 1 << spi_dev.config.drdy_pin
                drdy_masks[addr] = drdy_masks.get(addr, 0) | pin_mask
            else:
                print(f"  [WARN] {spi_dev.config.port_id}: DRDY not active — excluding from acquisition")

        if not drdy_masks:
            print(f"  [FAIL] No ports have active DRDY — cannot acquire data")
            return

        # Create parallel SPI bus workers — one persistent thread per physical bus.
        # Ports on the same bus are read sequentially by their worker;
        # workers on different buses run in parallel (GIL released during ioctl).
        bus_groups: Dict[int, List[ADS1299_SPI]] = {}
        for spi_dev in self.spi_devices:
            bus_num = spi_dev.config.bus_num
            if bus_num not in bus_groups:
                bus_groups[bus_num] = []
            bus_groups[bus_num].append(spi_dev)

        bus_workers: List[SPIBusWorker] = []
        port_worker_map: Dict[int, Tuple[SPIBusWorker, int]] = {}
        for bus_num in sorted(bus_groups.keys()):
            devices = bus_groups[bus_num]
            worker = SPIBusWorker(bus_num, devices)
            bus_workers.append(worker)
            for idx, dev in enumerate(devices):
                port_worker_map[id(dev)] = (worker, idx)

        bus_summary = ', '.join(
            f"SPI{w.bus_num}({len(w.spi_devices)})" for w in bus_workers)
        print(f"  Parallel SPI workers: {len(bus_workers)} buses [{bus_summary}]")

        # Force a full GC sweep now (before real-time loop starts) to
        # clean up all setup allocations, then disable GC so it can't
        # pause the acquisition loop (50-200ms pauses miss DRDY windows).
        gc.collect()
        gc.disable()
        print("  Acquisition thread: GC collected and disabled for real-time loop")

        try:
            while not self.stop_event.is_set():
                try:
                    # Wait for ALL ports' DRDY to go low (active)
                    drdy_deadline = time.time() + 0.008
                    while True:
                        all_ready = True
                        for addr, mask in drdy_masks.items():
                            val = drdy_bus.read_byte_data(addr, TCA9534_INPUT_PORT)
                            if val & mask:  # Any bit set = that port not ready
                                all_ready = False
                                break
                        if all_ready:
                            break
                        if time.time() > drdy_deadline:
                            break
                        time.sleep(0.0001)
                    if not all_ready:
                        continue

                    timestamp = time.time()
                    if self.start_time is None:
                        self.start_time = timestamp

                    # Phase 1: Parallel SPI transfers across independent buses
                    # Each worker reads its bus's ports sequentially, but all
                    # workers run simultaneously (GIL released during ioctl).
                    # ~288μs (2 ports on slowest bus) vs ~1008μs sequential.
                    for worker in bus_workers:
                        worker.trigger()
                    for worker in bus_workers:
                        worker.wait()

                    # Phase 2: Parse all raw buffers (no longer time-critical)
                    # Iterate in original port order to maintain channel ordering
                    all_channels = []
                    for spi_dev in self.spi_devices:
                        worker, idx = port_worker_map[id(spi_dev)]
                        _, channel_data = ADS1299_SPI.parse_raw(
                            worker.raw_buffers[idx],
                            spi_dev.config.num_daisy_devices)
                        for device_channels in channel_data:
                            all_channels.extend(device_channels)

                    # Queue sample as tuple for efficient binary packing
                    # (timestamp_relative, sample_number, channels_list)
                    sample_tuple = (timestamp - self.start_time, sample_number, all_channels)

                    # Only queue data if client is connected
                    if self.client_connected.is_set():
                        try:
                            self.data_queue.put_nowait(sample_tuple)
                        except queue.Full:
                            try:
                                self.data_queue.get_nowait()
                                self.data_queue.put_nowait(sample_tuple)
                            except queue.Empty:
                                pass

                    self.samples_acquired += 1
                    sample_number += 1

                    # Queue ch1 row for async CSV writer (non-blocking deque append)
                    rel_time = timestamp - self.start_time
                    row = [rel_time, sample_number]
                    for _, _, ch1_offset in device_info:
                        row.append(all_channels[ch1_offset] if len(all_channels) > ch1_offset else "NULL")
                    csv_queue.append(row)



                except Exception as e:
                    print(f"Acquisition error: {e}")
                    time.sleep(0.1)
        finally:
            gc.enable()
            gc.collect()
            for worker in bus_workers:
                worker.stop()
            csv_stop.set()
            csv_thread.join(timeout=5)
            print(f"  Acquisition thread: CSV writer stopped ({csv_path})")
    
    def streaming_thread(self):
        """Thread for streaming binary LZ4-compressed batched data to network client.
        
        Protocol (after JSON metadata handshake):
          - Accumulate batch_size samples (default 10)
          - Pack each sample as: float64 timestamp + uint32 sample_num + N×int32 channels
          - Compress batch with lz4.frame.compress()
          - Send 8-byte header (uint32 compressed_size + uint32 sample_count)
          - Send compressed payload
        
        This reduces bandwidth ~6x vs JSON and send() calls from 250/s to 25/s.
        """
        batch = []

        while not self.stop_event.is_set():
            # Wait for client to be connected
            if not self.client_connected.wait(timeout=0.5):
                continue
            
            try:
                sample_tuple = self.data_queue.get(timeout=0.1)
                batch.append(sample_tuple)

                # Accumulate until we have a full batch
                if len(batch) < self.batch_size:
                    continue

                # Pack all samples into a single binary buffer
                buf = bytearray(self.batch_buffer_size)
                for i, (ts, sn, channels) in enumerate(batch):
                    offset = i * self.sample_size
                    self.sample_struct.pack_into(buf, offset, ts, sn, *channels)

                # Compress with LZ4
                compressed = lz4.frame.compress(bytes(buf))
                compressed_size = len(compressed)
                sample_count = len(batch)

                # Build frame: 8-byte header + compressed payload
                header = self.header_struct.pack(compressed_size, sample_count)
                frame = header + compressed

                batch.clear()

                with self.client_lock:
                    if self.client_socket and self.client_connected.is_set():
                        if not self._send_nonblocking(frame):
                            print(f"\n{'='*60}\nCLIENT DISCONNECTED\n{'='*60}")
                            print(f"Session #{self.session_count}: {self.samples_sent} samples sent")
                            self.total_samples_sent += self.samples_sent
                            self.disconnect_client()
                            batch.clear()
                            continue
                        
                        self.samples_sent += sample_count
                
            except queue.Empty:
                # Flush partial batch if we have samples waiting (prevents stale data)
                if batch and self.client_connected.is_set():
                    partial_count = len(batch)
                    partial_buf = bytearray(self.sample_size * partial_count)
                    for i, (ts, sn, channels) in enumerate(batch):
                        offset = i * self.sample_size
                        self.sample_struct.pack_into(partial_buf, offset, ts, sn, *channels)
                    
                    compressed = lz4.frame.compress(bytes(partial_buf))
                    header = self.header_struct.pack(len(compressed), partial_count)
                    frame = header + compressed

                    with self.client_lock:
                        if self.client_socket and self.client_connected.is_set():
                            if not self._send_nonblocking(frame):
                                print(f"\n{'='*60}\nCLIENT DISCONNECTED\n{'='*60}")
                                print(f"Session #{self.session_count}: {self.samples_sent} samples sent")
                                self.total_samples_sent += self.samples_sent
                                self.disconnect_client()
                            else:
                                self.samples_sent += partial_count
                    batch.clear()
                continue
            except Exception as e:
                if self.client_connected.is_set():
                    print(f"Streaming error: {e}")
                    self.disconnect_client()
                batch.clear()
    
    def _send_nonblocking(self, data: bytes, timeout: float = 2.0) -> bool:
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
                print(f"    {spi_dev.config.port_id}: DRDY active [OK]")
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
                spi_dev.read_data()  # Suppress startup clearing

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
  Port4: SPI 5.0, 5 daisy-chained devices (40 channels)
  Port5: SPI 4.0, 3 daisy-chained devices (24 channels)
  
  Total: 21 devices, 168 channels @ 250 Hz

Valid SPI bus/device combinations (V2 board):
  OUT0: SPI0.CE0 (bus=0, device=0) -> pin P0
  OUT1: SPI0.CE1 (bus=0, device=1) -> pin P6
  OUT2: SPI3.CE0 (bus=3, device=0) -> pin P5
  OUT3: SPI3.CE1 (bus=3, device=1) -> pin P4
  OUT4: SPI4.CE0 (bus=4, device=0) -> pin P3
  OUT5: SPI4.CE1 (bus=4, device=1) -> pin P2
  OUT6: SPI5.CE0 (bus=5, device=0) -> pin P1

Examples:
  # Stream with default headset configuration (all 5 ports)
  python3 controller.py
  
  # Stream from a single port
  python3 controller.py --ports 0,0,Port1,1
  
  # Stream from three separate SPI buses
  python3 controller.py --ports 3,0,Port1,1 4,1,Port2,1 5,0,Port3,1
  
Port format: bus,device,name,num_daisy
  - bus: SPI bus number (0, 3, 4, or 5)
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
    parser.add_argument("--debug-channels", action='store_true',
                       help="Enable channel debugging output (shows first few samples)")
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
    print(f"\nInitializing SPI devices...")
    spi_devices = []

    for bus_cfg in bus_configs:
        spi_dev = ADS1299_SPI(bus_cfg)
        spi_devices.append(spi_dev)
    
    # Configure all devices
    print(f"\n{'='*60}\nCONFIGURING DEVICES\n{'='*60}")

    config = ADS1299_Config()
    
    # Configure each device (single attempt each - no retries)
    for spi_dev in spi_devices:
        success, _ = ADS1299_Controller.initialize_device(spi_dev, config)
        if not success:
            print(f"\n[FAIL] {spi_dev.config.port_id} failed to initialize")
            GPIO.cleanup()
            return
        time.sleep(0.1)  # Gap between ports

    print(f"\n[OK] All devices configured successfully!")

    # Now verify data flow
    print(f"\n{'='*60}\nVERIFYING DATA FLOW\n{'='*60}")

    data_flow_ok = ADS1299_Controller.start_all_conversions_synchronized(spi_devices, config)

    if not data_flow_ok:
        print(f"\n[WARN] Data flow verification failed")

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
        print(f"\n{'='*60}\n[OK] SYSTEM INITIALIZATION COMPLETE!\n{'='*60}")
    
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