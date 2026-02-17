#!/usr/bin/env python3
"""
ADS1299 Test Signal Server - V2 Board
======================================

Simple script that configures the ADS1299 to output its internal 1mV square wave
test signal and streams it over the network to the visualization client.

V2 Changes:
- Removed hardware RST and PWDN GPIO control (tied HIGH on board)
- Removed GPIO17 clock enable (hardwired HIGH on board)
- Uses software reset via SPI command only
- Updated TCA9534 pin mappings for V2 board

Usage:
    # On the Pi:
    python3 test_signal_visualizer_v2.py --port 3,1,OUT3
    
    # On your laptop:
    just test-client

Hardware:
    - Raspberry Pi CM4 with custom expansion board V2
    - RESET and PWDN tied HIGH on board
    - External clock hardwired enabled
    - TCA9534 I2C expanders at 0x20 (DRDY) and 0x21 (START)
"""

import spidev
import time
import smbus2
import socket
import json
import argparse
from enum import IntEnum

# ==============================================================================
# ADS1299 Definitions
# ==============================================================================

class Cmd(IntEnum):
    """ADS1299 commands"""
    WAKEUP = 0x02
    STANDBY = 0x04
    RESET = 0x06
    START = 0x08
    STOP = 0x0A
    RDATAC = 0x10
    SDATAC = 0x11
    RDATA = 0x12


class Reg(IntEnum):
    """ADS1299 register addresses"""
    ID = 0x00
    CONFIG1 = 0x01
    CONFIG2 = 0x02
    CONFIG3 = 0x03
    CH1SET = 0x05
    MISC1 = 0x15
    CONFIG4 = 0x17


# ==============================================================================
# Hardware Configuration
# ==============================================================================

# I2C
I2C_BUS = 6
TCA9534_DRDY_ADDR = 0x20
TCA9534_START_ADDR = 0x21
TCA9534_INPUT_REG = 0x00
TCA9534_OUTPUT_REG = 0x01
TCA9534_CONFIG_REG = 0x03

# SPI settings
SPI_MODE = 0b01
SPI_SPEED = 4_000_000

# Network
SERVER_HOST = '0.0.0.0'
SERVER_PORT = 8888


# ==============================================================================
# Test Signal Configuration
# ==============================================================================

# Register values for test signal mode
CONFIG1 = 0x96  # Daisy-chain mode, 250 SPS, external clock
CONFIG2 = 0xD0  # Enable internal test signal, 1mV amplitude at fCLK/2^21 Hz
CONFIG3 = 0xE8  # Internal reference enabled
CONFIG4 = 0x00  # Continuous conversion
CHNSET = 0x05   # Gain=1, test signal input (MUX=101)
MISC1 = 0x20    # SRB1 connected


# ==============================================================================
# V2 Board TCA9534 Pin Mapping
# ==============================================================================

def get_tca9534_pin(bus: int, device: int) -> int:
    """
    Get TCA9534 pin number for a given SPI bus/device combination.
    
    V2 Board Output Port Mapping:
        OUT0 → SPI0.CE0 (bus=0, device=0) → P0
        OUT1 → SPI0.CE1 (bus=0, device=1) → P6
        OUT2 → SPI3.CE0 (bus=3, device=0) → P5
        OUT3 → SPI3.CE1 (bus=3, device=1) → P4
        OUT4 → SPI4.CE0 (bus=4, device=0) → P3
        OUT5 → SPI4.CE1 (bus=4, device=1) → P2
        OUT6 → SPI5.CE0 (bus=5, device=0) → P1
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
    output_to_pin = {
        0: 0,  # OUT0 → P0
        1: 6,  # OUT1 → P6
        2: 5,  # OUT2 → P5
        3: 4,  # OUT3 → P4
        4: 3,  # OUT4 → P3
        5: 2,  # OUT5 → P2
        6: 1,  # OUT6 → P1
    }
    
    key = (bus, device)
    if key not in bus_device_to_output:
        raise ValueError(f"Unsupported SPI bus/device combination: SPI{bus}.{device}")
    
    output_num = bus_device_to_output[key]
    return output_to_pin[output_num]


# ==============================================================================
# ADS1299 Controller Class - V2
# ==============================================================================

class ADS1299_V2:
    """Simple ADS1299 controller for test signal streaming - V2 board"""
    
    def __init__(self, spi_bus: int, spi_device: int, port_name: str):
        """
        Initialize hardware interfaces.
        
        Args:
            spi_bus: SPI bus number (0, 3, 4, or 5)
            spi_device: SPI device/CE number (0 or 1)
            port_name: Descriptive name (e.g., OUT3)
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.port_name = port_name
        
        # Get TCA9534 pin for this SPI bus/device
        self.tca_pin = get_tca9534_pin(spi_bus, spi_device)
        print(f"Port {port_name}: SPI{spi_bus}.{spi_device} → TCA9534 P{self.tca_pin}")
        
        # Setup SPI
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.mode = SPI_MODE
        self.spi.max_speed_hz = SPI_SPEED
        
        # Setup I2C
        self.i2c = smbus2.SMBus(I2C_BUS)
        
        # Configure DRDY pin as input
        config = self.i2c.read_byte_data(TCA9534_DRDY_ADDR, TCA9534_CONFIG_REG)
        config |= (1 << self.tca_pin)
        self.i2c.write_byte_data(TCA9534_DRDY_ADDR, TCA9534_CONFIG_REG, config)
        
        # Configure START pin as output, initially LOW
        config = self.i2c.read_byte_data(TCA9534_START_ADDR, TCA9534_CONFIG_REG)
        config &= ~(1 << self.tca_pin)
        self.i2c.write_byte_data(TCA9534_START_ADDR, TCA9534_CONFIG_REG, config)
        self._set_start(False)
        
        print(f"  DRDY: 0x{TCA9534_DRDY_ADDR:02X}:P{self.tca_pin}")
        print(f"  START: 0x{TCA9534_START_ADDR:02X}:P{self.tca_pin}")
        print("Hardware initialized")
    
    def _set_start(self, high: bool):
        """Set START pin state"""
        value = self.i2c.read_byte_data(TCA9534_START_ADDR, TCA9534_OUTPUT_REG)
        if high:
            value |= (1 << self.tca_pin)
        else:
            value &= ~(1 << self.tca_pin)
        self.i2c.write_byte_data(TCA9534_START_ADDR, TCA9534_OUTPUT_REG, value)
    
    def _drdy_low(self) -> bool:
        """Check if DRDY is low (data ready)"""
        value = self.i2c.read_byte_data(TCA9534_DRDY_ADDR, TCA9534_INPUT_REG)
        return not (value & (1 << self.tca_pin))
    
    def _send_cmd(self, cmd: int):
        """Send a command"""
        self.spi.xfer2([cmd])
        time.sleep(0.000002)  # 4 tCLK
    
    def _read_regs(self, addr: int, count: int) -> list:
        """Read registers"""
        packet = [0x20 | (addr & 0x1F), count - 1] + [0x00] * count
        response = self.spi.xfer2(packet)
        time.sleep(0.000002)
        return response[2:]
    
    def _write_regs(self, addr: int, values: list):
        """Write registers"""
        packet = [0x40 | (addr & 0x1F), len(values) - 1] + values
        self.spi.xfer2(packet)
        time.sleep(0.000002)
    
    def wait_for_power_on(self):
        """
        Wait for ADS1299 to be ready after power-on.
        
        V2 board has RESET/PWDN tied HIGH, so we just need to wait for
        VCAP1 to charge on first use.
        """
        print("Waiting for VCAP1 charging and device ready (2s)...")
        time.sleep(2.0)
        print("  Device power-on wait complete ✓")
    
    def software_reset(self):
        """Perform software reset via SPI command"""
        print("Performing software reset...")
        self._send_cmd(Cmd.RESET)
        time.sleep(0.5)  # Wait for reset to complete
        print("  Software reset complete ✓")
    
    def configure_test_signal(self) -> bool:
        """Configure ADC for internal test signal output"""
        print("Configuring for test signal mode...")
        
        # Ensure START is low
        self._set_start(False)
        time.sleep(0.01)
        
        # Software reset
        self.software_reset()
        
        # Exit RDATAC mode
        self._send_cmd(Cmd.SDATAC)
        time.sleep(0.05)
        
        # Verify communication
        id_reg = self._read_regs(Reg.ID, 1)[0]
        print(f"Device ID: 0x{id_reg:02X}")
        
        if id_reg == 0x00 or id_reg == 0xFF:
            print("ERROR: No communication with ADS1299!")
            print("  Possible causes:")
            print(f"    1. No device connected to {self.port_name}")
            print(f"    2. SPI bus {self.spi_bus} not enabled")
            print(f"    3. Clock signal not reaching device")
            print(f"    4. Power supply issue")
            return False
        
        # Parse and display device ID
        dev_id = (id_reg >> 2) & 0x03
        if dev_id == 3:
            print("  ✓ Valid ADS1299 detected")
        else:
            print(f"  ⚠ Unexpected device ID = {dev_id}")
        
        # Write configuration
        self._write_regs(Reg.CONFIG1, [CONFIG1, CONFIG2, CONFIG3])
        time.sleep(0.01)
        
        # Configure all 8 channels for test signal
        self._write_regs(Reg.CH1SET, [CHNSET] * 8)
        time.sleep(0.01)
        
        # Write misc registers
        self._write_regs(Reg.MISC1, [MISC1])
        self._write_regs(Reg.CONFIG4, [CONFIG4])
        
        # Wait for reference to settle
        print("  Waiting for reference buffer to settle...")
        time.sleep(0.5)
        
        # Verify configuration
        readback = self._read_regs(Reg.CONFIG1, 3)
        print(f"  CONFIG1: 0x{readback[0]:02X} (expected 0x{CONFIG1:02X}) {'✓' if readback[0] == CONFIG1 else '✗'}")
        print(f"  CONFIG2: 0x{readback[1]:02X} (expected 0x{CONFIG2:02X}) {'✓' if readback[1] == CONFIG2 else '✗'}")
        print(f"  CONFIG3: 0x{readback[2]:02X} (expected 0x{CONFIG3:02X}) {'✓' if readback[2] == CONFIG3 else '✗'}")
        
        if readback[0] == CONFIG1 and readback[1] == CONFIG2 and readback[2] == CONFIG3:
            print("  ✓ Configuration successful!")
            return True
        else:
            print("  ⚠ Configuration mismatch, continuing anyway...")
            return True
    
    def start_acquisition(self):
        """Start continuous data acquisition"""
        self._send_cmd(Cmd.RDATAC)
        time.sleep(0.001)
        self._set_start(True)
        time.sleep(0.01)
        print("  ✓ Acquisition started")
    
    def stop_acquisition(self):
        """Stop data acquisition"""
        self._set_start(False)
        self._send_cmd(Cmd.STOP)
        print("Acquisition stopped")
    
    def read_sample(self) -> list:
        """Read one sample from all 8 channels"""
        # Read 27 bytes: 3 status + 8 channels * 3 bytes
        raw = self.spi.xfer2([0x00] * 27)
        
        channels = []
        for ch in range(8):
            offset = 3 + ch * 3
            value = (raw[offset] << 16) | (raw[offset + 1] << 8) | raw[offset + 2]
            # Sign extend 24-bit to 32-bit
            if value & 0x800000:
                value -= 0x1000000
            channels.append(value)
        
        return channels
    
    def wait_for_drdy(self, timeout: float = 0.1) -> bool:
        """Wait for data ready"""
        start = time.time()
        while not self._drdy_low():
            if time.time() - start > timeout:
                return False
            time.sleep(0.0001)
        return True
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_acquisition()
        print("Cleanup complete")


# ==============================================================================
# Streaming Server
# ==============================================================================

def run_server(adc: ADS1299_V2):
    """Run streaming server"""
    # Create server socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((SERVER_HOST, SERVER_PORT))
    server.listen(1)
    
    print(f"\n{'='*60}")
    print(f"Server listening on {SERVER_HOST}:{SERVER_PORT}")
    print(f"Waiting for client connection...")
    print(f"{'='*60}")
    
    client, addr = server.accept()
    client.setblocking(False)
    print(f"Client connected from {addr}")
    
    # Send metadata
    metadata = {
        "format": "json",
        "sample_rate": 250,
        "num_channels": 8,
        "num_devices": 1,
        "mode": "test_signal",
        "port": adc.port_name
    }
    client.send((json.dumps(metadata) + '\n').encode('utf-8'))
    print(f"Metadata sent: 8 channels @ 250 Hz (test signal mode)")
    
    # Start acquisition
    adc.start_acquisition()
    
    # Stream data
    sample_number = 0
    start_time = time.time()
    
    print(f"\n{'='*60}")
    print(f"STREAMING TEST SIGNAL DATA")
    print(f"{'='*60}\n")
    
    try:
        while True:
            # Wait for data ready
            if not adc.wait_for_drdy(timeout=0.1):
                continue
            
            # Read sample
            channels = adc.read_sample()
            timestamp = time.time() - start_time
            
            # Create sample packet
            sample = {
                "type": "sample",
                "timestamp": timestamp,
                "sample_number": sample_number,
                "channels": channels
            }
            
            # Send to client
            try:
                data = (json.dumps(sample) + '\n').encode('utf-8')
                total_sent = 0
                while total_sent < len(data):
                    try:
                        sent = client.send(data[total_sent:])
                        total_sent += sent
                    except BlockingIOError:
                        time.sleep(0.0001)
                        continue
            except (BrokenPipeError, ConnectionResetError):
                print("Client disconnected")
                break
            
            sample_number += 1
            
            # Print stats every 5 seconds
            if sample_number % 1250 == 0:
                elapsed = time.time() - start_time
                rate = sample_number / elapsed if elapsed > 0 else 0
                print(f"Samples: {sample_number}, Rate: {rate:.1f} Hz, "
                      f"Ch1: {channels[0]:8d}, Ch2: {channels[1]:8d}")
    
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        client.close()
        server.close()
        
        elapsed = time.time() - start_time
        print(f"\n{'='*60}")
        print(f"FINAL STATISTICS")
        print(f"{'='*60}")
        print(f"Samples streamed: {sample_number}")
        print(f"Elapsed time: {elapsed:.1f} seconds")
        if elapsed > 0:
            print(f"Average rate: {sample_number/elapsed:.1f} Hz")
        print(f"{'='*60}\n")


# ==============================================================================
# Main
# ==============================================================================

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="ADS1299 Test Signal Server - V2 Board",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test on OUT3 (SPI3.1)
  python3 test_signal_visualizer_v2.py --port 3,1,OUT3
  
  # Test on OUT0 (SPI0.0)
  python3 test_signal_visualizer_v2.py --port 0,0,OUT0

Valid SPI bus/device combinations (V2 board):
  OUT0: SPI0.CE0 (bus=0, device=0)
  OUT1: SPI0.CE1 (bus=0, device=1)
  OUT2: SPI3.CE0 (bus=3, device=0)
  OUT3: SPI3.CE1 (bus=3, device=1)
  OUT4: SPI4.CE0 (bus=4, device=0)
  OUT5: SPI4.CE1 (bus=4, device=1)
  OUT6: SPI5.CE0 (bus=5, device=0)
        """
    )
    
    parser.add_argument("--port", type=str, default="3,1,OUT3",
                       help="SPI port configuration (format: bus,device,name)")
    
    args = parser.parse_args()
    
    # Parse port configuration
    parts = args.port.split(',')
    if len(parts) != 3:
        print(f"Error: Invalid port format '{args.port}'")
        print("Expected format: bus,device,name (e.g., 3,1,OUT3)")
        return
    
    try:
        spi_bus = int(parts[0])
        spi_device = int(parts[1])
        port_name = parts[2]
    except ValueError as e:
        print(f"Error parsing port '{args.port}': {e}")
        return
    
    print("="*60)
    print("ADS1299 Test Signal Server - V2 Board")
    print("="*60)
    print(f"Port: {port_name} (SPI{spi_bus}.{spi_device})")
    print("="*60)
    
    adc = ADS1299_V2(spi_bus, spi_device, port_name)
    
    try:
        # Wait for device to be ready (VCAP1 charging)
        adc.wait_for_power_on()
        
        # Configure for test signal
        if not adc.configure_test_signal():
            print("Failed to configure ADS1299")
            return
        
        # Run server
        run_server(adc)
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        adc.cleanup()


if __name__ == "__main__":
    main()
