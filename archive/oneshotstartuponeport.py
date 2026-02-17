#!/usr/bin/env python3
"""
ADS1299 EEG Acquisition System - Clean Implementation
=====================================================

This module provides a complete, well-structured system for acquiring EEG data
from multiple ADS1299 devices configured in daisy-chain mode across multiple
SPI buses, with WiFi streaming capability.

Key Features:
- Proper hardware reset sequence via GPIO
- Support for multiple SPI buses with daisy-chained devices
- I2C GPIO expander control for DRDY and START signals
- WiFi streaming server compatible with PyQt5 visualization client
- Clean, modular architecture with proper error handling

Hardware Requirements:
- Raspberry Pi CM4 with custom expansion board
- ADS1299 devices with RESET/PWDN connected to GPIO pins 26/27
- TCA9534 I2C expanders for DRDY/START control
- 2.048 MHz clock source for ADS1299 CLK pins

Author: Clean rewrite based on working prototype

FIX APPLIED: Corrected daisy-chain data format handling.
Per ADS1299 datasheet section 10.1.4.2: Each device outputs its own
3 status bytes + 24 channel bytes = 27 bytes per device.
"""

import spidev
import time
import RPi.GPIO as GPIO
import smbus2
import socket
import json
import threading
import queue
from queue import Queue
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import IntEnum


# ==============================================================================
# ADS1299 Register and Command Definitions
# ==============================================================================

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


# ==============================================================================
# Hardware Configuration
# ==============================================================================

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
SPI_SPEED_HZ = 4_000_000  # 4 MHz (default, can be overridden per-port)


# ==============================================================================
# Register Configuration Values
# ==============================================================================

@dataclass
class ADS1299_Config:
    """Standard register configuration for ADS1299"""
    # Global settings
    config1: int = 0x96   # Daisy-chain mode, 250 SPS, external clock
    config2: int = 0xC2   # No test signals, normal operation
    config3: int = 0xE0   # Int ref enabled, BIAS off, external BIASREF
    config4: int = 0x00   # Continuous conversion, lead-off comp off
    
    # Channel settings (individual configuration for each channel)
    ch1set: int = 0x00    # CH1: Channel on, gain=24, normal input
    ch2set: int = 0x00    # CH2: Channel on, gain=24, normal input
    ch3set: int = 0x00    # CH3: Channel on, gain=24, normal input
    ch4set: int = 0x00    # CH4: Channel on, gain=24, normal input
    ch5set: int = 0x00    # CH5: Channel on, gain=24, normal input
    ch6set: int = 0x00    # CH6: Channel on, gain=24, normal input
    ch7set: int = 0x00    # CH7: Channel on, gain=24, normal input
    ch8set: int = 0x00    # CH8: Channel on, gain=24, normal input
    
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
    def input_short_config(cls):
        """Configuration for input short (noise measurement)"""
        config = cls()
        config.set_all_channels(0x01)  # Input shorted for all channels
        return config


# ==============================================================================
# SPI Bus Configuration Mapping
# ==============================================================================

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
    spi_speed_hz: int = 4_000_000  # Default 4 MHz
    settling_delay_us: int = 0     # Microseconds to wait after DRDY goes low


def get_bus_config(bus: int, device: int, port_id: str, num_daisy: int = 1,
                   spi_speed_hz: int = 4_000_000, settling_delay_us: int = 0) -> SPIBusConfig:
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
        start_pin=start_pin,
        spi_speed_hz=spi_speed_hz,
        settling_delay_us=settling_delay_us
    )


# ==============================================================================
# TCA9534 I2C GPIO Expander Control
# ==============================================================================

class TCA9534_Pin:
    """
    Controller for a single pin on a TCA9534 I2C GPIO expander.
    
    This provides a simple interface for controlling DRDY (input) and
    START (output) signals.
    """
    
    def __init__(self, i2c_bus: int, i2c_addr: int, pin: int, is_input: bool):
        """
        Initialize pin control.
        
        Args:
            i2c_bus: I2C bus number (6 for CM4)
            i2c_addr: TCA9534 I2C address (0x20, 0x21, or 0x22)
            pin: Pin number (0-7)
            is_input: True for input (DRDY), False for output (START)
        """
        self.bus = smbus2.SMBus(i2c_bus)
        self.addr = i2c_addr
        self.pin = pin
        self.pin_mask = 1 << pin
        
        # Configure pin direction
        config = self.bus.read_byte_data(i2c_addr, TCA9534_CONFIG)
        if is_input:
            config |= self.pin_mask   # Set bit = input
        else:
            config &= ~self.pin_mask  # Clear bit = output
        self.bus.write_byte_data(i2c_addr, TCA9534_CONFIG, config)
        
        # If output, set initial state to LOW
        if not is_input:
            self.set_low()
    
    def read(self) -> bool:
        """Read pin state (for DRDY input)"""
        value = self.bus.read_byte_data(self.addr, TCA9534_INPUT_PORT)
        return bool(value & self.pin_mask)
    
    def set_high(self):
        """Set pin HIGH (for START output)"""
        value = self.bus.read_byte_data(self.addr, TCA9534_OUTPUT_PORT)
        value |= self.pin_mask
        self.bus.write_byte_data(self.addr, TCA9534_OUTPUT_PORT, value)
    
    def set_low(self):
        """Set pin LOW (for START output)"""
        value = self.bus.read_byte_data(self.addr, TCA9534_OUTPUT_PORT)
        value &= ~self.pin_mask
        self.bus.write_byte_data(self.addr, TCA9534_OUTPUT_PORT, value)


class TCA9534_MultiPin:
    """
    Controller for setting multiple pins on TCA9534 I2C GPIO expanders atomically.
    
    This is used to synchronize START signals across multiple ADS1299 devices
    by setting all START pins HIGH in a single I2C transaction per expander.
    """
    
    def __init__(self, i2c_bus: int = I2C_BUS):
        """Initialize with I2C bus"""
        self.bus = smbus2.SMBus(i2c_bus)
        # Track pins by expander address: {addr: combined_mask}
        self.pin_masks: Dict[int, int] = {}
    
    def add_pin(self, i2c_addr: int, pin: int):
        """Register a pin to be controlled"""
        pin_mask = 1 << pin
        if i2c_addr not in self.pin_masks:
            self.pin_masks[i2c_addr] = 0
        self.pin_masks[i2c_addr] |= pin_mask
    
    def set_all_high(self):
        """Set ALL registered pins HIGH atomically (one I2C write per expander)"""
        for addr, mask in self.pin_masks.items():
            # Read current output state
            current = self.bus.read_byte_data(addr, TCA9534_OUTPUT_PORT)
            # Set all our pins HIGH
            new_value = current | mask
            self.bus.write_byte_data(addr, TCA9534_OUTPUT_PORT, new_value)
    
    def set_all_low(self):
        """Set ALL registered pins LOW atomically (one I2C write per expander)"""
        for addr, mask in self.pin_masks.items():
            # Read current output state
            current = self.bus.read_byte_data(addr, TCA9534_OUTPUT_PORT)
            # Clear all our pins
            new_value = current & ~mask
            self.bus.write_byte_data(addr, TCA9534_OUTPUT_PORT, new_value)


# ==============================================================================
# ADS1299 SPI Communication Layer
# ==============================================================================

class ADS1299_SPI:
    """
    Low-level SPI communication with ADS1299.
    
    Handles command sending, register read/write, and data acquisition
    with proper timing for daisy-chain configurations.
    """
    
    def __init__(self, bus_config: SPIBusConfig):
        """Initialize SPI interface for one bus"""
        self.config = bus_config
        
        # Open SPI device
        self.spi = spidev.SpiDev()
        self.spi.open(bus_config.bus_num, bus_config.device_num)
        self.spi.mode = SPI_MODE
        self.spi.max_speed_hz = bus_config.spi_speed_hz
        
        # Initialize control pins
        self.drdy = TCA9534_Pin(I2C_BUS, bus_config.drdy_i2c_addr, 
                               bus_config.drdy_pin, is_input=True)
        self.start = TCA9534_Pin(I2C_BUS, bus_config.start_i2c_addr,
                                bus_config.start_pin, is_input=False)
        
        # Corruption detection statistics
        self.corruption_retries = 0
        self.corruption_unrecovered = 0
        self.spikes_repaired = 0  # Spikes detected and repaired via delta check
        
        # Spike forensics - circular buffer of recent raw reads
        self.raw_history = []  # List of (sample_num, raw_bytes, parsed_values)
        self.raw_history_size = 5  # Keep last 5 samples
        self.spike_events = []  # Captured spike forensic data
        self.sample_counter = 0
        self.previous_values = None  # For delta detection
        
        print(f"  {bus_config.port_id}: SPI {bus_config.bus_num}.{bus_config.device_num} "
              f"@ {bus_config.spi_speed_hz/1e6:.1f} MHz")
        print(f"    DRDY: 0x{bus_config.drdy_i2c_addr:02X}:P{bus_config.drdy_pin}")
        print(f"    START: 0x{bus_config.start_i2c_addr:02X}:P{bus_config.start_pin}")
    
    @staticmethod
    def wait_4tclk():
        """Wait for 4 clock cycles at 2.048 MHz (approximately 2 microseconds)"""
        time.sleep(0.000002)
    
    def flush_spi(self):
        """
        Flush any stale data from the SPI bus.
        
        This sends dummy bytes to clear the shift register and ensure
        the bus is in a known state before critical operations.
        """
        # Send 32 dummy bytes (enough for 2+ full data frames)
        self.spi.xfer2([0x00] * 32)
        time.sleep(0.001)
    
    def send_command(self, command: int, debug: bool = False):
        """Send a single-byte command"""
        if debug:
            cmd_names = {
                0x02: "WAKEUP", 0x04: "STANDBY", 0x06: "RESET",
                0x08: "START", 0x0A: "STOP", 0x10: "RDATAC",
                0x11: "SDATAC", 0x12: "RDATA"
            }
            cmd_name = cmd_names.get(command, f"0x{command:02X}")
            print(f"      → CMD: {cmd_name}")
        self.spi.xfer2([command])
        self.wait_4tclk()
    
    def read_registers(self, start_reg: int, num_regs: int, debug: bool = False) -> List[int]:
        """
        Read consecutive registers.
        
        Command format: [RREG|addr, num-1, dummy, dummy, ...]
        Response format: [dummy, dummy, reg0, reg1, ...]
        """
        cmd_byte1 = 0x20 | (start_reg & 0x1F)
        cmd_byte2 = (num_regs - 1) & 0x1F
        
        if debug:
            print(f"      → RREG: addr=0x{start_reg:02X}, count={num_regs}")
        
        # Send command and dummy bytes, receive response
        packet = [cmd_byte1, cmd_byte2] + ([0x00] * num_regs)
        response = self.spi.xfer2(packet)
        
        self.wait_4tclk()
        
        # Register data starts at byte 2
        result = response[2:]
        if debug:
            print(f"      ← Data: {[f'0x{v:02X}' for v in result]}")
        return result
    
    def write_registers(self, start_reg: int, values: List[int], debug: bool = False):
        """
        Write consecutive registers.
        
        Command format: [WREG|addr, num-1, value0, value1, ...]
        """
        num_regs = len(values)
        cmd_byte1 = 0x40 | (start_reg & 0x1F)
        cmd_byte2 = (num_regs - 1) & 0x1F
        
        if debug:
            print(f"      → WREG: addr=0x{start_reg:02X}, count={num_regs}, data={[f'0x{v:02X}' for v in values]}")
        
        # Send entire command in one transfer
        packet = [cmd_byte1, cmd_byte2] + values
        self.spi.xfer2(packet)
        
        self.wait_4tclk()
    
    def is_data_ready(self) -> bool:
        """Check if DRDY is low (data ready)"""
        return not self.drdy.read()
    
    def wait_for_drdy(self, timeout: float = 0.1) -> bool:
        """
        Wait for DRDY to go low (data ready), then apply settling delay.
        
        Returns True if data ready, False if timeout.
        """
        start_time = time.time()
        while not self.is_data_ready():
            if time.time() - start_time > timeout:
                return False
            time.sleep(0.0001)  # 100 microseconds
        
        # Apply settling delay if configured
        if self.config.settling_delay_us > 0:
            time.sleep(self.config.settling_delay_us / 1_000_000)
        
        return True
    
    def read_data(self, allow_retry: bool = True) -> Tuple[List[int], List[List[int]], bool]:
        """
        Read one sample from all devices in daisy chain with corruption detection.
        
        Data Structure from ADS1299 (per datasheet section 10.1.4.2):
        =============================================================
        Single device (no daisy-chain):
            [STATUS0, STATUS1, STATUS2, CH1_H, CH1_M, CH1_L, ..., CH8_H, CH8_M, CH8_L]
            3 status bytes + (8 channels × 3 bytes) = 27 bytes total
        
        Multiple devices (daisy-chain):
            EACH device outputs its own status + channel data in sequence.
            Per datasheet: "Status and data from device 1 appear first on DOUT,
            followed by the status and data from device 2."
            
            [DEV1_STATUS(3), DEV1_CH1-8(24), DEV2_STATUS(3), DEV2_CH1-8(24), ...]
            Total = 27 bytes × num_devices
        
        Status Byte Validation (per datasheet section 9.4.4.2):
        =============================================================
        The first status byte format is: 1100 + LOFF_STATP[7:4]
        So the upper nibble MUST be 0xC (binary 1100).
        Any other value indicates SPI corruption (e.g., kernel preemption).
        
        Returns:
            (status_bytes, channel_data, is_valid)
            - status_bytes: 3 status bytes from first device (for compatibility)
            - channel_data: List of [ch1, ch2, ..., ch8] for each device
            - is_valid: True if all status bytes were valid, False if corruption detected
        """
        # CRITICAL FIX: Each device outputs 27 bytes (3 status + 24 channel data)
        # The old code incorrectly assumed only 1 set of status bytes for all devices
        bytes_per_device = 27  # 3 status + (8 channels × 3 bytes)
        total_bytes = bytes_per_device * self.config.num_daisy_devices
        
        # Read data
        raw_data = self.spi.xfer2([0x00] * total_bytes)
        
        # Increment sample counter
        self.sample_counter += 1
        
        # Validate status bytes from each device BEFORE parsing
        # Per datasheet: status byte 0 must have upper nibble = 0xC (1100 binary)
        is_valid = True
        for device_idx in range(self.config.num_daisy_devices):
            device_offset = device_idx * bytes_per_device
            status_byte_0 = raw_data[device_offset]
            if (status_byte_0 & 0xF0) != 0xC0:
                is_valid = False
                break
        
        # If invalid and retry allowed, attempt immediate re-read
        if not is_valid and allow_retry:
            self.corruption_retries += 1
            return self.read_data(allow_retry=False)
        
        # If still invalid after retry, track it
        if not is_valid:
            self.corruption_unrecovered += 1
        
        # Parse data for each device
        channel_data = []
        first_status = None
        all_values = []  # Flat list for spike detection
        
        for device_idx in range(self.config.num_daisy_devices):
            # Each device's data starts at device_idx * 27
            device_offset = device_idx * bytes_per_device
            
            # Extract this device's status bytes (3 bytes)
            device_status = raw_data[device_offset:device_offset + 3]
            
            # Save first device's status for return value
            if device_idx == 0:
                first_status = device_status
            
            # Parse this device's 8 channels
            # Channel data starts 3 bytes after device_offset (after status)
            device_channels = []
            ch_data_offset = device_offset + 3
            
            for ch in range(8):
                # Each channel is 3 bytes (24-bit)
                byte_offset = ch_data_offset + (ch * 3)
                ch_bytes = raw_data[byte_offset:byte_offset + 3]
                
                # Convert 24-bit two's complement to signed integer
                # Format: [HIGH_BYTE, MID_BYTE, LOW_BYTE]
                value = (ch_bytes[0] << 16) | (ch_bytes[1] << 8) | ch_bytes[2]
                
                # Sign extension for 24-bit values
                if value & 0x800000:
                    value = value - 0x1000000
                
                device_channels.append(value)
                all_values.append(value)
            
            channel_data.append(device_channels)
        
        # SPIKE FORENSICS: Detect large jumps from previous sample
        # Only check if we have previous values and limit to 10 captured events
        spike_detected = False
        spike_channels = []
        if self.previous_values is not None and len(self.spike_events) < 10:
            for ch_idx, (curr, prev) in enumerate(zip(all_values, self.previous_values)):
                delta = abs(curr - prev)
                # Spike threshold: delta > 1 million counts (~120mV at gain=24)
                # Normal EEG should never jump this much in 4ms
                if delta > 1000000:
                    spike_detected = True
                    spike_channels.append((ch_idx, prev, curr, delta))
        
        # If spike detected, capture forensic data AND repair the values
        if spike_detected:
            forensic = {
                'sample_num': self.sample_counter,
                'port': self.config.port_id,
                'raw_bytes': list(raw_data),  # Copy of raw SPI data
                'status_valid': is_valid,
                'spike_channels': spike_channels,
                'history': list(self.raw_history),  # Previous samples
            }
            self.spike_events.append(forensic)
            
            # REPAIR: Replace corrupted channel values with previous sample's values
            for ch_idx, prev_val, curr_val, delta in spike_channels:
                all_values[ch_idx] = prev_val  # Use previous valid value
                # Also update channel_data (which is structured by device)
                device_idx = ch_idx // 8
                ch_in_device = ch_idx % 8
                channel_data[device_idx][ch_in_device] = prev_val
            
            # Track this as a repaired spike (don't mark is_valid=False since we fixed it)
            self.spikes_repaired += 1
        
        # Update history buffer (circular) - store REPAIRED values
        self.raw_history.append((self.sample_counter, list(raw_data), all_values.copy()))
        if len(self.raw_history) > self.raw_history_size:
            self.raw_history.pop(0)
        
        # Store current REPAIRED values for next comparison
        self.previous_values = all_values.copy()
        
        # Return status from first device, all channel data, and validity flag
        if first_status is not None:
            return first_status, channel_data, is_valid
        else:
            return [0, 0, 0], channel_data, is_valid


# ==============================================================================
# ADS1299 Device Controller
# ==============================================================================

class ADS1299_Controller:
    """
    High-level controller for ADS1299 system initialization and configuration.
    """
    
    @staticmethod
    def initialize_gpio():
        """Initialize GPIO for global RESET and PWDN control"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(GPIO_RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(GPIO_PWDN_PIN, GPIO.OUT, initial=GPIO.HIGH)
        print("GPIO initialized (RESET=GPIO26, PWDN=GPIO27)")
    
    @staticmethod
    def force_all_start_pins_low(bus_configs: List[SPIBusConfig]):
        """
        Force ALL START pins LOW before hardware reset.
        
        This is CRITICAL - if START pins are HIGH from a previous run,
        the devices will be in conversion mode and won't respond to
        configuration commands properly.
        """
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
                # Build mask for all START pins on this expander
                mask = 0
                for pin in pins:
                    mask |= (1 << pin)
                
                # Configure pins as outputs (clear bits in config register)
                try:
                    config_reg = bus.read_byte_data(addr, TCA9534_CONFIG)
                    config_reg &= ~mask  # Clear bits = output
                    bus.write_byte_data(addr, TCA9534_CONFIG, config_reg)
                    
                    # Set all START pins LOW
                    output_reg = bus.read_byte_data(addr, TCA9534_OUTPUT_PORT)
                    output_reg &= ~mask  # Clear bits = LOW
                    bus.write_byte_data(addr, TCA9534_OUTPUT_PORT, output_reg)
                    
                    print(f"  I2C 0x{addr:02X}: START pins {pins} set LOW ✓")
                except Exception as e:
                    print(f"  I2C 0x{addr:02X}: Failed to set START pins: {e}")
        finally:
            bus.close()
        
        time.sleep(0.01)  # Brief settling time
    
    @staticmethod
    def hardware_reset():
        """
        Perform complete hardware power-on reset sequence.
        
        This is CRITICAL for proper ADS1299 operation. The ADS1299 requires
        time for VCAP1 to charge to >1.1V before it can respond to commands.
        """
        print("\nPerforming hardware reset sequence...")
        
        # Step 1: Power down and reset
        GPIO.output(GPIO_PWDN_PIN, GPIO.LOW)
        GPIO.output(GPIO_RESET_PIN, GPIO.LOW)
        time.sleep(0.1)  # 100ms - let everything discharge
        
        # Step 2: Power up
        GPIO.output(GPIO_PWDN_PIN, GPIO.HIGH)
        time.sleep(0.2)  # 200ms - let power stabilize
        
        # Step 3: Release reset
        GPIO.output(GPIO_RESET_PIN, GPIO.HIGH)
        time.sleep(2.0)  # 2 seconds - allow VCAP1 charging and oscillator stabilization
        
        print("Hardware reset complete (VCAP1 charging time allowed) ✓")
    
    @staticmethod
    def initialize_device(spi: ADS1299_SPI, config: ADS1299_Config) -> bool:
        """
        Initialize and configure a single ADS1299 bus.
        
        Returns True if successful, False otherwise.
        """
        print(f"\nInitializing {spi.config.port_id}...")
        
        # Step 1: Ensure START is low (device must NOT be in conversion mode)
        spi.start.set_low()
        time.sleep(0.05)  # 50ms settling
        
        # Step 2: Flush SPI bus to clear any stale data
        print(f"  Flushing SPI bus...")
        spi.flush_spi()
        
        # Step 3: Send STOP command in case device is running
        spi.send_command(ADS1299_Cmd.STOP)
        time.sleep(0.01)
        
        # Step 4: Send software reset command
        print(f"  Sending software RESET...")
        spi.send_command(ADS1299_Cmd.RESET)
        time.sleep(0.3)  # 300ms for complete reset - be generous
        
        # Step 5: Exit RDATAC mode and VERIFY by attempting register write/read
        # This is the critical step - we MUST prove we can write registers
        print(f"  Exiting RDATAC mode...")
        max_sdatac_attempts = 10  # More attempts
        sdatac_success = False
        
        for attempt in range(max_sdatac_attempts):
            # Flush SPI before each attempt
            spi.flush_spi()
            
            # Send SDATAC multiple times
            for _ in range(5):
                spi.send_command(ADS1299_Cmd.SDATAC)
                time.sleep(0.005)
            
            time.sleep(0.05)  # Wait for command to take effect
            
            # CRITICAL: Verify SDATAC by doing a WRITE then READ test
            # If we're still in RDATAC mode, writes silently fail
            # Use LOFF register (0x04) as a test - it defaults to 0x00
            test_value = 0xAA  # Distinctive pattern
            try:
                # Write test value
                spi.write_registers(ADS1299_Reg.LOFF, [test_value])
                time.sleep(0.01)
                
                # Read it back
                readback = spi.read_registers(ADS1299_Reg.LOFF, 1)[0]
                
                if readback == test_value:
                    # SUCCESS! We can write registers, so SDATAC worked
                    # Restore LOFF to 0x00
                    spi.write_registers(ADS1299_Reg.LOFF, [0x00])
                    time.sleep(0.005)
                    print(f"  SDATAC verified (write/read test passed) ✓")
                    sdatac_success = True
                    break
                else:
                    print(f"  SDATAC attempt {attempt+1}: write/read test failed (wrote 0x{test_value:02X}, read 0x{readback:02X})")
                    
            except Exception as e:
                print(f"  SDATAC attempt {attempt+1} exception: {e}")
            
            # If not successful, try more aggressive reset
            if attempt >= 2:
                # Send additional STOP commands
                spi.send_command(ADS1299_Cmd.STOP)
                time.sleep(0.02)
                spi.send_command(ADS1299_Cmd.RESET)
                time.sleep(0.1)
        
        if not sdatac_success:
            print(f"  ✗ Failed to exit RDATAC mode after {max_sdatac_attempts} attempts")
            return False
        
        # Now verify ID register for device identification
        try:
            id_value = spi.read_registers(ADS1299_Reg.ID, 1)[0]
            print(f"  ID register: 0x{id_value:02X}")
            
            if id_value == 0x00:
                print(f"  ✗ ERROR: ID is 0x00 (no communication)")
                return False
            
            if id_value == 0xFF:
                print(f"  ✗ ERROR: ID is 0xFF (MISO disconnected)")
                return False
            
            dev_id = (id_value >> 2) & 0x03
            if dev_id == 3:
                print(f"  ✓ Valid ADS1299 detected")
            else:
                print(f"  ⚠ WARNING: Unexpected device ID = {dev_id}")
        
        except Exception as e:
            print(f"  ✗ ERROR: Failed to read ID register: {e}")
            return False
        
        # Step 6: Write configuration registers with verification loop
        print(f"  Writing configuration...")
        print(f"  ────────────────────────────────────────")
        
        # Write all config registers with retry
        max_config_attempts = 5
        config_success = False
        
        for config_attempt in range(max_config_attempts):
            # Ensure we're still in SDATAC mode
            spi.send_command(ADS1299_Cmd.SDATAC)
            time.sleep(0.01)
            
            # Write CONFIG1-CONFIG3
            spi.write_registers(ADS1299_Reg.CONFIG1, [
                config.config1,
                config.config2,
                config.config3
            ])
            time.sleep(0.02)
            
            # Write LOFF register (explicitly set to 0x00 to clear any test values)
            spi.write_registers(ADS1299_Reg.LOFF, [0x00])
            time.sleep(0.01)
            
            # Write channel registers
            channel_settings = config.get_channel_settings()
            spi.write_registers(ADS1299_Reg.CH1SET, channel_settings)
            time.sleep(0.02)
            
            # Write BIAS and lead-off registers
            spi.write_registers(ADS1299_Reg.BIAS_SENSP, [
                config.bias_sensp,
                config.bias_sensn,
                config.loff_sensp,
                config.loff_sensn,
                config.loff_flip
            ])
            time.sleep(0.01)
            
            # Write misc registers
            spi.write_registers(ADS1299_Reg.MISC1, [config.misc1])
            spi.write_registers(ADS1299_Reg.CONFIG4, [config.config4])
            time.sleep(0.02)
            
            # Verify all critical registers
            readback_config = spi.read_registers(ADS1299_Reg.CONFIG1, 3)
            readback_loff = spi.read_registers(ADS1299_Reg.LOFF, 1)[0]
            readback_ch = spi.read_registers(ADS1299_Reg.CH1SET, 8)
            readback_misc1 = spi.read_registers(ADS1299_Reg.MISC1, 1)[0]
            readback_config4 = spi.read_registers(ADS1299_Reg.CONFIG4, 1)[0]
            
            # Check all values
            all_match = True
            if readback_config[0] != config.config1:
                all_match = False
            if readback_config[1] != config.config2:
                all_match = False
            if readback_config[2] != config.config3:
                all_match = False
            if readback_loff != 0x00:  # LOFF should be 0x00
                all_match = False
            for i, ch_val in enumerate(readback_ch):
                if ch_val != channel_settings[i]:
                    all_match = False
            if readback_misc1 != config.misc1:
                all_match = False
            if readback_config4 != config.config4:
                all_match = False
            
            if all_match:
                config_success = True
                print(f"  Configuration verified on attempt {config_attempt + 1} ✓")
                break
            else:
                print(f"  Configuration attempt {config_attempt + 1} failed, retrying...")
                time.sleep(0.05)
        
        if not config_success:
            print(f"  ✗ Failed to configure registers after {max_config_attempts} attempts")
            return False
        
        # Wait for internal reference buffer to settle
        print(f"  Waiting for reference buffer to settle...")
        time.sleep(0.5)  # 500ms for complete reference settling
        
        # Final verification dump
        print(f"  ────────────────────────────────────────")
        print(f"  FINAL REGISTER STATE:")
        all_regs = spi.read_registers(0x00, 24)
        reg_names = [
            "ID", "CONFIG1", "CONFIG2", "CONFIG3", "LOFF",
            "CH1SET", "CH2SET", "CH3SET", "CH4SET", "CH5SET", "CH6SET", "CH7SET", "CH8SET",
            "BIAS_SENSP", "BIAS_SENSN", "LOFF_SENSP", "LOFF_SENSN", "LOFF_FLIP",
            "RES_0x12", "RES_0x13", "RES_0x14", "MISC1", "RES_0x16", "CONFIG4"
        ]
        
        expected_values = {
            "CONFIG1": config.config1, "CONFIG2": config.config2, "CONFIG3": config.config3,
            "LOFF": 0x00,  # Should always be 0x00 after our explicit write
            "CH1SET": config.ch1set, "CH2SET": config.ch2set, "CH3SET": config.ch3set,
            "CH4SET": config.ch4set, "CH5SET": config.ch5set, "CH6SET": config.ch6set,
            "CH7SET": config.ch7set, "CH8SET": config.ch8set,
            "MISC1": config.misc1, "CONFIG4": config.config4,
            "BIAS_SENSP": config.bias_sensp, "BIAS_SENSN": config.bias_sensn,
            "LOFF_SENSP": config.loff_sensp, "LOFF_SENSN": config.loff_sensn,
            "LOFF_FLIP": config.loff_flip
        }
        
        final_success = True
        for i, (name, value) in enumerate(zip(reg_names, all_regs)):
            expected = expected_values.get(name)
            if expected is not None:
                status = " ✓" if value == expected else " ✗"
                if value != expected:
                    final_success = False
            else:
                status = ""
            print(f"    0x{i:02X} {name:12s} = 0x{value:02X}{status}")
        print(f"  ────────────────────────────────────────")
        
        if final_success:
            print(f"  ✓ {spi.config.port_id} initialized successfully")
        else:
            print(f"  ✗ {spi.config.port_id} final verification failed")
        
        return final_success
    
    @staticmethod
    def start_conversion(spi: ADS1299_SPI):
        """Start continuous data acquisition"""
        # Put device in RDATAC mode
        spi.send_command(ADS1299_Cmd.RDATAC)
        time.sleep(0.001)
        
        # Assert START pin
        spi.start.set_high()
        time.sleep(0.01)
        
        print(f"  {spi.config.port_id}: Conversions started ✓")
    
    @staticmethod
    def stop_conversion(spi: ADS1299_SPI):
        """Stop continuous data acquisition"""
        # De-assert START pin
        spi.start.set_low()
        
        # Send STOP command
        spi.send_command(ADS1299_Cmd.STOP)
        time.sleep(0.001)
    
    @staticmethod
    def start_all_conversions_synchronized(spi_devices: List['ADS1299_SPI']) -> bool:
        """
        Start conversions on ALL devices with synchronized START signals.
        
        This is critical for multi-port operation - all START pins must be
        asserted simultaneously to ensure all devices begin converting at
        the same time, resulting in synchronized DRDY signals.
        
        Per ADS1299 datasheet Figure 69: "When using multiple devices, the 
        devices can be synchronized with the START signal."
        
        Returns True if all devices started successfully and produce valid data.
        """
        print("\n  Starting conversions with synchronized START signals...")
        
        # First, ensure all START pins are LOW and flush SPI
        for spi_dev in spi_devices:
            spi_dev.start.set_low()
            spi_dev.flush_spi()
        time.sleep(0.02)
        
        # Put all devices in RDATAC mode
        for spi_dev in spi_devices:
            spi_dev.send_command(ADS1299_Cmd.RDATAC)
        time.sleep(0.005)
        
        # Collect all START pins grouped by I2C expander address
        start_controller = TCA9534_MultiPin()
        for spi_dev in spi_devices:
            start_controller.add_pin(
                spi_dev.config.start_i2c_addr,
                spi_dev.config.start_pin
            )
        
        # Assert ALL START pins simultaneously (one I2C write per expander)
        # This synchronizes all devices to start converting at the same time
        start_controller.set_all_high()
        time.sleep(0.02)
        
        for spi_dev in spi_devices:
            print(f"    {spi_dev.config.port_id}: START asserted ✓")
        
        # CRITICAL: Verify data flow from ALL devices
        # We need VALID status bytes (0xCx) - not just any DRDY signal
        print(f"\n  Verifying data flow from all devices...")
        
        # Wait for first DRDY cycle (settling time + first conversion)
        # At 250 SPS, one cycle is 4ms, but settling takes longer
        time.sleep(0.1)  # 100ms for initial settling
        
        all_verified = True
        devices_with_bad_status = []
        
        for i, spi_dev in enumerate(spi_devices):
            # Wait for DRDY
            if not spi_dev.wait_for_drdy(timeout=0.5):
                print(f"    ✗ {spi_dev.config.port_id}: DRDY timeout - device not converting!")
                all_verified = False
                devices_with_bad_status.append(spi_dev)
                continue
            
            # Read data and check status bytes
            status, channel_data, _ = spi_dev.read_data()
            
            # Check status bytes - MUST start with 0xC0 for 8-channel ADS1299
            # Status format: 1100 + LOFF_STATP[7:0] + LOFF_STATN[7:0] + GPIO[4:7]
            # So first nibble MUST be 0xC (binary 1100)
            status_valid = (status[0] & 0xF0) == 0xC0
            
            if not status_valid:
                print(f"    ✗ {spi_dev.config.port_id}: INVALID status bytes: {[f'0x{b:02X}' for b in status]}")
                print(f"      Expected status[0] to start with 0xC0, got 0x{status[0]:02X}")
                all_verified = False
                devices_with_bad_status.append(spi_dev)
                continue
            
            # Check if all channels are zero (might indicate issue, but not always)
            all_zeros = True
            for device_channels in channel_data:
                for ch_val in device_channels:
                    if ch_val != 0:
                        all_zeros = False
                        break
            
            if all_zeros:
                print(f"    ⚠ {spi_dev.config.port_id}: Data flowing but all zeros (status=0x{status[0]:02X})")
            else:
                print(f"    ✓ {spi_dev.config.port_id}: Data flowing, status=0x{status[0]:02X}")
        
        # If any device has bad status, try to recover
        if devices_with_bad_status:
            print(f"\n  Attempting recovery for devices with invalid status...")
            
            for spi_dev in devices_with_bad_status:
                print(f"    Recovering {spi_dev.config.port_id}...")
                
                # Stop this device
                spi_dev.start.set_low()
                spi_dev.send_command(ADS1299_Cmd.STOP)
                time.sleep(0.05)
                
                # Flush SPI
                spi_dev.flush_spi()
                
                # Send SDATAC, then verify we can still write registers
                for _ in range(3):
                    spi_dev.send_command(ADS1299_Cmd.SDATAC)
                    time.sleep(0.01)
                
                # Quick register write/read test
                spi_dev.write_registers(ADS1299_Reg.LOFF, [0x55])
                time.sleep(0.01)
                test_read = spi_dev.read_registers(ADS1299_Reg.LOFF, 1)[0]
                spi_dev.write_registers(ADS1299_Reg.LOFF, [0x00])  # Restore
                time.sleep(0.005)
                
                if test_read != 0x55:
                    print(f"      ✗ Register access failed - device needs full reinit")
                    continue
                
                # Re-enter RDATAC and restart
                spi_dev.send_command(ADS1299_Cmd.RDATAC)
                time.sleep(0.005)
                spi_dev.start.set_high()
                time.sleep(0.05)
                
                # Check status again
                if spi_dev.wait_for_drdy(timeout=0.2):
                    status, _, _ = spi_dev.read_data()
                    if (status[0] & 0xF0) == 0xC0:
                        print(f"      ✓ Recovery successful, status=0x{status[0]:02X}")
                        devices_with_bad_status.remove(spi_dev)
                    else:
                        print(f"      ✗ Recovery failed, status still invalid: 0x{status[0]:02X}")
                else:
                    print(f"      ✗ Recovery failed, DRDY timeout")
        
        # Update verification status
        all_verified = len(devices_with_bad_status) == 0
        
        # Do verification reads only if all devices have valid status
        if all_verified:
            print(f"  Performing verification reads...")
            drdy_failures = [0] * len(spi_devices)
            
            for cycle in range(10):
                # Wait for DRDY on all devices (they should be synchronized)
                all_ready = True
                for i, spi_dev in enumerate(spi_devices):
                    if not spi_dev.wait_for_drdy(timeout=0.05):
                        drdy_failures[i] += 1
                        all_ready = False
                
                # Only read data if all devices are ready
                if all_ready:
                    for spi_dev in spi_devices:
                        spi_dev.read_data()
            
            for i, spi_dev in enumerate(spi_devices):
                if drdy_failures[i] > 2:
                    print(f"    ✗ {spi_dev.config.port_id}: {drdy_failures[i]}/10 DRDY failures")
                    all_verified = False
                else:
                    print(f"    ✓ {spi_dev.config.port_id}: DRDY stable ({10 - drdy_failures[i]}/10 successful)")
        
        if all_verified:
            print(f"\n  ✓ All {len(spi_devices)} devices verified and synchronized!")
        else:
            print(f"\n  ✗ FAILED: {len(devices_with_bad_status)} device(s) not producing valid data")
        
        return all_verified


# ==============================================================================
# WiFi Streaming Server
# ==============================================================================

class StreamingServer:
    """
    WiFi streaming server for EEG data.
    
    Acquires data from multiple ADS1299 buses and streams to network clients
    in JSON format compatible with PyQt5 visualization client.
    """
    
    def __init__(self, spi_devices: List[ADS1299_SPI], 
                 host: str = '0.0.0.0', port: int = 8888, debug_channels: bool = False,
                 hold_on_corruption: bool = True):
        """
        Initialize streaming server.
        
        Args:
            spi_devices: List of configured ADS1299_SPI objects
            host: Server IP address
            port: Server port number
            debug_channels: Enable channel debugging output
            hold_on_corruption: If True, hold previous valid sample when corruption detected
        """
        self.spi_devices = spi_devices
        self.host = host
        self.port = port
        self.debug_channels = debug_channels
        self.hold_on_corruption = hold_on_corruption
        
        # Data queue for inter-thread communication
        self.data_queue = Queue(maxsize=10000)
        
        # Control flags
        self.stop_event = threading.Event()
        self.streaming_active = False
        
        # Network
        self.server_socket = None
        self.client_socket = None
        
        # Statistics
        self.samples_acquired = 0
        self.samples_sent = 0
        self.corrupted_samples = 0
        self.start_time = None
        
        # Previous valid sample for corruption recovery
        self.previous_valid_channels = None
    
    def start_server(self) -> bool:
        """
        Start the network server and wait for a client connection.
        
        Returns True if client connected, False otherwise.
        """
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            
            print(f"\n{'='*60}")
            print(f"Server listening on {self.host}:{self.port}")
            print(f"Waiting for client connection...")
            print(f"{'='*60}")
            
            self.client_socket, client_addr = self.server_socket.accept()
            self.client_socket.setblocking(False)
            
            print(f"Client connected from {client_addr}")
            
            # Send metadata
            total_channels = sum(dev.config.num_daisy_devices * 8 
                               for dev in self.spi_devices)
            
            metadata = {
                "format": "json",
                "sample_rate": 250,
                "num_channels": total_channels,
                "num_devices": sum(dev.config.num_daisy_devices 
                                  for dev in self.spi_devices)
            }
            
            metadata_line = json.dumps(metadata) + '\n'
            self.client_socket.send(metadata_line.encode('utf-8'))
            
            print(f"Metadata sent: {total_channels} channels @ 250 Hz")
            
            return True
            
        except Exception as e:
            print(f"Error starting server: {e}")
            return False
    
    def acquisition_thread(self):
        """
        Thread for acquiring data from ADS1299 devices.
        
        CRITICAL: For multi-port operation, we must either:
        1. Ensure all devices have synchronized START signals (done in start_all_conversions_synchronized)
        2. Wait for each device's DRDY before reading from it
        
        We do both for maximum reliability.
        """
        sample_number = 0
        drdy_timeout_count = [0] * len(self.spi_devices)  # Track timeouts per device
        
        while not self.stop_event.is_set():
            try:
                # Wait for DRDY on ALL devices before reading any
                # This ensures all devices have completed their conversion
                all_ready = True
                for i, spi_dev in enumerate(self.spi_devices):
                    if not spi_dev.wait_for_drdy(timeout=0.02):  # 20ms timeout per device
                        all_ready = False
                        drdy_timeout_count[i] += 1
                        if drdy_timeout_count[i] % 250 == 1:  # Log every ~1 second at 250 SPS
                            print(f"WARNING: {spi_dev.config.port_id} DRDY timeout (count: {drdy_timeout_count[i]})")
                        break
                
                if not all_ready:
                    continue
                
                # Reset timeout counts on successful read
                drdy_timeout_count = [0] * len(self.spi_devices)
                
                # Record timestamp
                timestamp = time.time()
                if self.start_time is None:
                    self.start_time = timestamp
                
                # Read data from all devices
                # Since we verified all DRDY signals are low, all data should be ready
                all_channels = []
                sample_corrupted = False
                
                for spi_dev in self.spi_devices:
                    status, channel_data, is_valid = spi_dev.read_data()
                    
                    if not is_valid:
                        sample_corrupted = True
                    
                    # Flatten channel data from all daisy-chained devices
                    for device_channels in channel_data:
                        all_channels.extend(device_channels)
                
                # Handle corruption: hold previous valid sample if enabled
                if sample_corrupted and self.hold_on_corruption and self.previous_valid_channels is not None:
                    # Use previous valid sample to eliminate the spike
                    all_channels = self.previous_valid_channels.copy()
                elif not sample_corrupted:
                    # Store this valid sample for potential future recovery
                    self.previous_valid_channels = all_channels.copy()
                
                # Debug output for first few samples
                if self.debug_channels and sample_number < 5:
                    print(f"\n--- Sample {sample_number} Channel Debug ---")
                    for ch_idx, value in enumerate(all_channels):
                        print(f"  Ch{ch_idx:2d}: {value:10d}")
                    print(f"--- End Sample {sample_number} ---\n")
                
                # Create sample packet
                sample = {
                    "type": "sample",
                    "timestamp": timestamp - self.start_time,
                    "sample_number": sample_number,
                    "channels": all_channels
                }
                
                # Try to add to queue (non-blocking)
                try:
                    self.data_queue.put_nowait(sample)
                    self.samples_acquired += 1
                    if sample_corrupted:
                        self.corrupted_samples += 1
                except queue.Full:
                    # Queue full, sample dropped
                    pass
                
                sample_number += 1
                
                # Print stats every 5 seconds
                if sample_number % 1250 == 0:  # 1250 samples = 5 seconds at 250 Hz
                    elapsed = time.time() - self.start_time
                    rate = sample_number / elapsed if elapsed > 0 else 0
                    queue_usage = self.data_queue.qsize() / 10000 * 100
                    print(f"Acquired: {sample_number} samples, "
                          f"Rate: {rate:.1f} Hz, "
                          f"Queue: {queue_usage:.1f}%")
                
            except Exception as e:
                print(f"Acquisition error: {e}")
                import traceback
                traceback.print_exc()
                break
    
    def streaming_thread(self):
        """Thread for streaming data to network client"""
        while not self.stop_event.is_set():
            try:
                # Get sample from queue (with timeout)
                sample = self.data_queue.get(timeout=0.1)
                
                # Serialize to JSON
                sample_line = json.dumps(sample) + '\n'
                
                # Send to client (non-blocking with retry)
                if not self._send_nonblocking(sample_line.encode('utf-8')):
                    # Send failed, client probably disconnected
                    print("Client disconnected")
                    break
                
                self.samples_sent += 1
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Streaming error: {e}")
                break
    
    def _send_nonblocking(self, data: bytes, timeout: float = 0.1) -> bool:
        """
        Send data on non-blocking socket with timeout.
        
        Returns True if successful, False if failed/timeout.
        """
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
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return False
        
        return True
    
    def run(self):
        """
        Start acquisition and streaming.
        
        This is the main entry point - call this after initializing devices.
        Note: Conversions should already be started by start_all_conversions_synchronized()
        during the initialization verification phase.
        """
        print(f"\n{'='*60}")
        print(f"STARTING DATA ACQUISITION AND STREAMING")
        print(f"{'='*60}\n")
        
        # Conversions should already be running from initialization
        # Just verify DRDY is active on all devices
        print("  Verifying all devices are converting...")
        for spi_dev in self.spi_devices:
            if spi_dev.wait_for_drdy(timeout=0.1):
                print(f"    {spi_dev.config.port_id}: DRDY active ✓")
                # Read and discard to clear DRDY
                spi_dev.read_data()
            else:
                print(f"    {spi_dev.config.port_id}: DRDY not active, restarting...")
                # If not converting, restart this device
                spi_dev.start.set_low()
                time.sleep(0.01)
                spi_dev.send_command(ADS1299_Cmd.RDATAC)
                time.sleep(0.001)
                spi_dev.start.set_high()
                time.sleep(0.01)
        
        # Start threads
        self.streaming_active = True
        
        acq_thread = threading.Thread(target=self.acquisition_thread, daemon=True)
        stream_thread = threading.Thread(target=self.streaming_thread, daemon=True)
        
        acq_thread.start()
        stream_thread.start()
        
        print("\nSystem running. Press Ctrl+C to stop.\n")
        
        # Wait for stop signal
        try:
            while self.streaming_active and not self.stop_event.is_set():
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\nStopping...")
        
        # Stop threads
        self.stop_event.set()
        acq_thread.join(timeout=2)
        stream_thread.join(timeout=2)
        
        # Stop conversions
        for spi_dev in self.spi_devices:
            ADS1299_Controller.stop_conversion(spi_dev)
        
        # Print final stats
        print(f"\n{'='*60}")
        print(f"FINAL STATISTICS")
        print(f"{'='*60}")
        print(f"Samples acquired: {self.samples_acquired}")
        print(f"Samples sent: {self.samples_sent}")
        if self.start_time:
            elapsed = time.time() - self.start_time
            print(f"Elapsed time: {elapsed:.1f} seconds")
            if elapsed > 0:
                print(f"Average rate: {self.samples_acquired/elapsed:.1f} Hz")
        
        # Corruption statistics
        total_retries = sum(dev.corruption_retries for dev in self.spi_devices)
        total_unrecovered = sum(dev.corruption_unrecovered for dev in self.spi_devices)
        total_spikes_repaired = sum(dev.spikes_repaired for dev in self.spi_devices)
        
        if total_retries > 0 or self.corrupted_samples > 0 or total_spikes_repaired > 0:
            print(f"\nCorruption Detection:")
            print(f"  Status byte retries: {total_retries}")
            print(f"  Status byte unrecovered: {total_unrecovered}")
            print(f"  Spikes detected & repaired: {total_spikes_repaired}")
            print(f"  Corrupted samples (hold applied): {self.corrupted_samples}")
            for dev in self.spi_devices:
                if dev.corruption_retries > 0 or dev.corruption_unrecovered > 0 or dev.spikes_repaired > 0:
                    print(f"    {dev.config.port_id}: {dev.corruption_retries} retries, "
                          f"{dev.corruption_unrecovered} unrecovered, "
                          f"{dev.spikes_repaired} spikes fixed")
        else:
            print(f"\nNo corruption detected ✓")
        
        # Spike forensics report
        total_spikes = sum(len(dev.spike_events) for dev in self.spi_devices)
        if total_spikes > 0:
            print(f"\n{'='*60}")
            print(f"SPIKE FORENSICS ({total_spikes} events captured)")
            print(f"{'='*60}")
            
            for dev in self.spi_devices:
                for event in dev.spike_events:
                    print(f"\n--- Spike at sample {event['sample_num']} on {event['port']} ---")
                    print(f"Status byte valid: {event['status_valid']}")
                    print(f"Affected channels:")
                    for ch_idx, prev, curr, delta in event['spike_channels']:
                        print(f"  Ch{ch_idx}: {prev:+10d} -> {curr:+10d} (delta: {delta:,})")
                    
                    # Show raw bytes
                    raw = event['raw_bytes']
                    print(f"\nRaw bytes ({len(raw)} total):")
                    # Show status bytes
                    print(f"  Status: [{raw[0]:02X} {raw[1]:02X} {raw[2]:02X}]", end="")
                    if len(raw) > 27:
                        print(f"  [{raw[27]:02X} {raw[28]:02X} {raw[29]:02X}]", end="")
                    print()
                    
                    # Show channel bytes in groups
                    for dev_idx in range(len(raw) // 27):
                        base = dev_idx * 27 + 3  # Skip status
                        print(f"  Dev{dev_idx} channels: ", end="")
                        for ch in range(8):
                            offset = base + ch * 3
                            print(f"[{raw[offset]:02X}{raw[offset+1]:02X}{raw[offset+2]:02X}]", end=" ")
                        print()
                    
                    # Show history (previous samples)
                    if event['history']:
                        print(f"\nPrevious {len(event['history'])} samples raw status bytes:")
                        for sample_num, hist_raw, hist_vals in event['history']:
                            status_bytes = f"[{hist_raw[0]:02X} {hist_raw[1]:02X} {hist_raw[2]:02X}]"
                            if len(hist_raw) > 27:
                                status_bytes += f" [{hist_raw[27]:02X} {hist_raw[28]:02X} {hist_raw[29]:02X}]"
                            print(f"    Sample {sample_num}: {status_bytes}")
        
        print(f"{'='*60}\n")
    
    def cleanup(self):
        """Clean up resources"""
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        
        GPIO.cleanup()


# ==============================================================================
# Main Entry Point
# ==============================================================================

def main():
    """
    Main entry point for ADS1299 streaming system.
    
    Example usage:
        # Single device on SPI 3.0
        python3 ads1299_system.py
        
        # Multiple devices
        python3 ads1299_system.py --ports 3,0,Port1,1 4,1,Port2,1 5,0,Port3,1
    """
    import argparse
    
    parser = argparse.ArgumentParser(
        description="ADS1299 EEG Streaming System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Stream from SPI 3.0 with 6 daisy-chained devices
  python3 ads1299_system.py --ports 3,0,Port1,6
  
  # Stream from three separate SPI buses
  python3 ads1299_system.py --ports 3,0,Port1,1 4,1,Port2,1 5,0,Port3,1
  
  # Slow down SPI if seeing corruption on later channels
  python3 ads1299_system.py --ports 3,1,Port1,1 0,0,Port2,1 --spi-speed 2000000
  
  # Add settling delay after DRDY goes low
  python3 ads1299_system.py --ports 3,1,Port1,1 0,0,Port2,1 --settling-delay 20
  
Port format: bus,device,name,num_daisy
  - bus: SPI bus number (0-5)
  - device: SPI device number (0-1)  
  - name: Descriptive name (e.g. Port1, FrontHead, etc)
  - num_daisy: Number of ADS1299s daisy-chained on this bus
        """
    )
    
    parser.add_argument("--ports", nargs='+', 
                       default=["3,0,Port1,1", "4,1,Port2,1", "5,0,Port3,1"],
                       help="SPI port configurations (format: bus,device,name,num_daisy)")
    parser.add_argument("--host", type=str, default='0.0.0.0',
                       help="Server host address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8888,
                       help="Server port number (default: 8888)")
    parser.add_argument("--test-mode", type=str, choices=['normal', 'test-signal', 'input-short'],
                       default='normal',
                       help="Operating mode: normal, test-signal (1mV square wave), or input-short (noise test)")
    parser.add_argument("--debug-channels", action='store_true',
                       help="Enable channel debugging output (shows first few samples)")
    parser.add_argument("--spi-speed", type=int, default=4000000,
                       help="SPI clock speed in Hz (default: 4000000). Try 2000000 if seeing corruption.")
    parser.add_argument("--settling-delay", type=int, default=0,
                       help="Microseconds to wait after DRDY goes low before reading (default: 0)")
    parser.add_argument("--no-hold-on-corruption", action='store_true',
                       help="Disable holding previous sample on detected corruption (spikes will be visible)")
    
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
            
            config = get_bus_config(bus, device, name, num_daisy,
                                    spi_speed_hz=args.spi_speed,
                                    settling_delay_us=args.settling_delay)
            bus_configs.append(config)
            
        except (ValueError, IndexError) as e:
            print(f"Error parsing port '{port_str}': {e}")
            return
    
    print(f"\n{'='*60}")
    print(f"ADS1299 EEG STREAMING SYSTEM")
    print(f"{'='*60}")
    print(f"Configuration:")
    print(f"  SPI buses: {len(bus_configs)}")
    total_devices = sum(cfg.num_daisy_devices for cfg in bus_configs)
    total_channels = total_devices * 8
    print(f"  Total devices: {total_devices}")
    print(f"  Total channels: {total_channels}")
    print(f"  Sample rate: 250 Hz")
    print(f"  SPI speed: {args.spi_speed/1e6:.1f} MHz")
    if args.settling_delay > 0:
        print(f"  Settling delay: {args.settling_delay} µs")
    print(f"  Corruption protection: {'ON' if not args.no_hold_on_corruption else 'OFF'}")
    print(f"  Server: {args.host}:{args.port}")
    print(f"{'='*60}\n")
    
    # Initialize GPIO
    ADS1299_Controller.initialize_gpio()
    
    # CRITICAL: Force all START pins LOW before hardware reset
    # This ensures devices won't be in conversion mode during configuration
    ADS1299_Controller.force_all_start_pins_low(bus_configs)
    
    # Hardware reset
    ADS1299_Controller.hardware_reset()
    
    # Initialize SPI devices
    print(f"\nInitializing SPI devices...")
    spi_devices = []
    
    for bus_cfg in bus_configs:
        spi_dev = ADS1299_SPI(bus_cfg)
        spi_devices.append(spi_dev)
    
    # Configure all devices
    print(f"\n{'='*60}")
    print(f"CONFIGURING DEVICES")
    print(f"{'='*60}")
    
    # Select configuration based on test mode
    if args.test_mode == 'test-signal':
        config = ADS1299_Config.test_signal_config()
        print("Using TEST SIGNAL mode (1mV square wave)")
    elif args.test_mode == 'input-short':
        config = ADS1299_Config.input_short_config()
        print("Using INPUT SHORT mode (noise measurement)")
    else:
        config = ADS1299_Config()
        print("Using NORMAL mode")
    
    # Robust initialization with full retry including hardware reset
    max_init_attempts = 10  # Increased from 3 - keep trying until it works
    initialization_successful = False
    
    for init_attempt in range(max_init_attempts):
        print(f"\n{'='*60}")
        print(f"INITIALIZATION ATTEMPT {init_attempt + 1}/{max_init_attempts}")
        print(f"{'='*60}")
        
        all_config_success = True
        failed_ports = []
        
        # Configure each device
        for spi_dev in spi_devices:
            success = ADS1299_Controller.initialize_device(spi_dev, config)
            if not success:
                all_config_success = False
                failed_ports.append(spi_dev.config.port_id)
        
        if not all_config_success:
            print(f"\n⚠ Configuration failed for: {', '.join(failed_ports)}")
            if init_attempt < max_init_attempts - 1:
                print(f"Performing full reset and retry...")
                # Stop any running conversions
                for spi_dev in spi_devices:
                    spi_dev.start.set_low()
                    spi_dev.send_command(ADS1299_Cmd.STOP)
                time.sleep(0.1)
                # Force all START pins LOW again
                ADS1299_Controller.force_all_start_pins_low(bus_configs)
                # Hardware reset
                ADS1299_Controller.hardware_reset()
            continue
        
        print(f"\n✓ All devices configured successfully!")
        
        # Now verify data flow
        print(f"\n{'='*60}")
        print(f"VERIFYING DATA FLOW")
        print(f"{'='*60}")
        
        data_flow_ok = ADS1299_Controller.start_all_conversions_synchronized(spi_devices)
        
        if data_flow_ok:
            initialization_successful = True
            print(f"\n{'='*60}")
            print(f"✓ SYSTEM INITIALIZATION COMPLETE!")
            print(f"{'='*60}")
            break
        else:
            print(f"\n⚠ Data flow verification failed")
            if init_attempt < max_init_attempts - 1:
                print(f"Stopping conversions and retrying...")
                # Stop all conversions
                for spi_dev in spi_devices:
                    ADS1299_Controller.stop_conversion(spi_dev)
                time.sleep(0.1)
                # Force all START pins LOW again
                ADS1299_Controller.force_all_start_pins_low(bus_configs)
                # Hardware reset
                ADS1299_Controller.hardware_reset()
    
    if not initialization_successful:
        print(f"\n{'='*60}")
        print(f"✗ INITIALIZATION FAILED after {max_init_attempts} attempts")
        print(f"{'='*60}")
        print(f"Possible causes:")
        print(f"  - Hardware connection issues")
        print(f"  - Clock signal not reaching devices")
        print(f"  - Power supply issues")
        print(f"  - Damaged ADS1299 chip")
        # Ask user if they want to continue
        response = input("Continue anyway? (y/n): ").strip().lower()
        if response != 'y':
            print("Aborting.")
            GPIO.cleanup()
            return
    
    # Create streaming server
    server = StreamingServer(spi_devices, host=args.host, port=args.port, 
                            debug_channels=args.debug_channels,
                            hold_on_corruption=not args.no_hold_on_corruption)
    
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