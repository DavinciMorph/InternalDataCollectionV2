# SPI4/SPI5 DMA on BCM2711 — Implementation Guide

## The Problem

Unlike SPI3 (which has dedicated DREQ lines), SPI4 and SPI5 use **alternate
DREQs** that share physical lines with UART3 and UART5 respectively. Enabling
DMA for SPI4/SPI5 requires:

1. **PACTL_CS register mux** — a runtime register write that routes DREQs from
   the UARTs to the SPIs
2. **Device tree overlay** — tells the spi-bcm2835 driver which DREQ numbers
   to use

A device tree overlay alone is NOT sufficient. The driver will accept the DREQ
configuration but DMA transfers will silently fail or hang because the hardware
mux is still pointing the DREQs at the UARTs.

## DREQ Number Mapping

**Confirmed (direct DREQs — no mux needed):**

| SPI  | TX DREQ | RX DREQ | Source              |
|------|---------|---------|---------------------|
| SPI0 | 6       | 7       | Datasheet           |
| SPI3 | 16      | 18      | Broadcom-confirmed  |
| SPI6 | 23      | 27      | Community-verified  |

**SPI4/SPI5 (alternate DREQs — PACTL_CS mux required):**

The exact DREQ numbers for SPI4/SPI5 must be read from the BCM2711 datasheet
section 4.2.1.3 (pages 61-62). The "Alternate" column shows which DREQ lines
can be switched from their default UART assignment to SPI.

Download the datasheet:
  https://datasheets.raspberrypi.com/bcm2711/bcm2711-peripherals.pdf

Look at the table rows where the "Alternate" column says SPI4 TX, SPI4 RX,
SPI5 TX, SPI5 RX. Note those DREQ numbers.

## PACTL_CS Register

**Address:** 0x7e204e00 (bus) / 0xfe204e00 (ARM physical on RPi 4)

**DMA_CNTRL_MUX bits:**
- Bit 23: DMA_CNTRL_MUX_2
- Bit 24: DMA_CNTRL_MUX_0
- Bit 25: DMA_CNTRL_MUX_1

These 3 bits form a mux selector. Different combinations route different DREQ
lines. The exact mapping is undocumented by Broadcom — it was reverse-
engineered by forum user abqjln who confirmed all 5 SPI channels working.

**Tradeoff:** When you mux DREQs to SPI4, UART3 loses DMA capability (and
vice versa). Same for SPI5/UART5.

## Step-by-Step Implementation

### Step 1: Read the Datasheet

Open bcm2711-peripherals.pdf, go to page 62, and find the DREQ table.
Identify the rows where SPI4 TX/RX and SPI5 TX/RX appear as alternates.
Note the DREQ numbers.

### Step 2: Create Device Tree Overlays

For SPI4 (replace NN with actual DREQ numbers from Step 1):

```dts
/dts-v1/;
/plugin/;
/ {
    compatible = "brcm,bcm2711";
    fragment@0 {
        target = <&spi4>;
        __overlay__ {
            dmas = <&dma NN>, <&dma NN>;  /* TX, RX DREQ numbers */
            dma-names = "tx", "rx";
        };
    };
};
```

For SPI5 (same pattern, different DREQ numbers).

Compile and install:
```bash
dtc -@ -I dts -O dtb -o spi4-dma.dtbo spi4-dma-overlay.dts
sudo cp spi4-dma.dtbo /boot/overlays/
```

### Step 3: Configure PACTL_CS Mux at Boot

Install devmem2:
```bash
sudo apt-get install devmem2
```

Test manually:
```bash
# Read current PACTL_CS value
sudo devmem2 0xfe204e00 w

# Set MUX_0 bit (start here, adjust if needed)
# Read the current value, OR in 0x01000000, write back
sudo devmem2 0xfe204e00 w 0x01000000
```

Once you find the right mux bit combination:
```bash
sudo cp spi45-dma-mux.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/spi45-dma-mux.sh
sudo cp spi45-dma-mux.service /etc/systemd/system/
sudo systemctl enable spi45-dma-mux.service
```

### Step 4: config.txt

```ini
# Enable SPI4 with chip selects
dtoverlay=spi4-2cs

# Add DMA configuration
dtoverlay=spi4-dma

# Repeat for SPI5 if needed
# dtoverlay=spi5-2cs
# dtoverlay=spi5-dma
```

### Step 5: Reboot and Verify

```bash
sudo reboot

# After reboot, check:
dmesg | grep -i spi
# Look for DMA channel allocation messages
# Should NOT see "no tx-dma configuration found"

# Check PACTL_CS was set
sudo devmem2 0xfe204e00 w
```

## Debugging Strategy

If DMA doesn't work after the overlay + mux:

1. **Wrong DREQ numbers** — Double-check against datasheet page 62
2. **Wrong mux bits** — Try all 8 combinations of bits 23/24/25:
   - 0x00000000 (none)
   - 0x00800000 (MUX_2 only)
   - 0x01000000 (MUX_0 only)
   - 0x01800000 (MUX_0 + MUX_2)
   - 0x02000000 (MUX_1 only)
   - 0x02800000 (MUX_1 + MUX_2)
   - 0x03000000 (MUX_0 + MUX_1)
   - 0x03800000 (all three)
3. **Driver probe order** — The SPI driver may probe before the mux script
   runs, falling back to PIO. Fix by reloading:
   ```bash
   sudo modprobe -r spi_bcm2835 && sudo modprobe spi_bcm2835
   ```
4. **Transfer size too small** — spi-bcm2835 has a ~96 byte minimum for DMA.
   Smaller transfers still use PIO regardless.

## Alternative: Kernel Module Approach

For production use, a proper kernel module that writes PACTL_CS during SPI
driver init would be more robust than a systemd service + devmem2. This
could be implemented as a small platform driver that the DT overlay loads
before the SPI driver probes.

## Key References

- BCM2711 Datasheet: https://datasheets.raspberrypi.com/bcm2711/bcm2711-peripherals.pdf
- DREQ table errata: https://github.com/raspberrypi/documentation/issues/3232
- abqjln's 5-SPI DMA work: https://forums.raspberrypi.com/viewtopic.php?t=357788
- PACTL_CS discussion: https://github.com/raspberrypi/documentation/issues/1903
- SPI4/SPI5 alternate DREQs: https://forums.raspberrypi.com/viewtopic.php?t=311899
