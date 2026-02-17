# Enabling SPI DMA on BCM2711 (Raspberry Pi 4) for SPI3, SPI4, and SPI5

## Problem

SPI3, SPI4, and SPI5 on the Raspberry Pi 4 run in PIO (programmed I/O) mode by default. The kernel has no device tree DMA configuration for these buses, confirmed by dmesg:

```
spi-bcm2835 fe204600.spi: error -ENODEV: no tx-dma configuration found - not using dma mode
spi-bcm2835 fe204800.spi: error -ENODEV: no tx-dma configuration found - not using dma mode
spi-bcm2835 fe204a00.spi: error -ENODEV: no tx-dma configuration found - not using dma mode
```

In PIO mode, the CPU manually feeds the SPI FIFO byte-by-byte. If the CPU is preempted during a transfer, the SPI clock stalls. For ADS1299 EEG devices in daisy-chain configuration at 250 SPS (4 ms DRDY period), a stall that extends past the next DRDY boundary causes the device to overwrite its shift register, corrupting data for downstream devices. This was the root cause of rare corruption events on ports 3-7.

SPI0 (ports 1-2) already had DMA configured in the default device tree and showed zero corruption.

## Solution Overview

| SPI Bus | Address      | DREQ TX | DREQ RX | DMA Controller | Extra Requirements |
|---------|-------------|---------|---------|----------------|-------------------|
| SPI0    | fe204000    | 6       | 7       | Legacy (`&dma`) | None (already working) |
| SPI3    | fe204600    | 16      | 18      | Legacy (`&dma`) | Device tree overlay only |
| SPI4    | fe204800    | 19      | 20      | Legacy (`&dma`) | Overlay + PACTL_CS mux |
| SPI5    | fe204a00    | 21      | 22      | DMA4 (`&dma40`) | Overlay + PACTL_CS mux + HDMI audio disabled |

### Key complications resolved

1. **SPI3 DREQs are mislabeled in the datasheet.** The BCM2711 datasheet (section 4.2.1.3, pages 61-62) lists DREQ 16 as "SPI1 TX" and DREQ 18 as "SPI1 RX". SPI1 is a mini SPI with no DMA support. These DREQs actually belong to SPI3 (confirmed by Broadcom: github.com/raspberrypi/documentation/issues/3232).

2. **SPI4 and SPI5 use alternate DREQs shared with UARTs.** DREQ 19/20 are shared with UART3, and DREQ 21/22 are shared with UART5. Routing them to SPI requires setting DMA_CNTRL_MUX bits in the PACTL_CS register (`0xfe204e00`) at boot via `devmem2`.

3. **Legacy DMA channel exhaustion.** The legacy DMA controller (`dma0`) has only 8 usable channels. With SPI0 (2), SPI3 (2), SPI4 (2), and MMC (1) allocated, only 1 channel remains — not enough for SPI5's TX + RX. SPI5 was moved to the DMA4 controller (`dma40`), which has separate channels freed by disabling HDMI audio.

4. **Expanding the legacy DMA channel mask breaks WiFi.** An early attempt to add GPU-reserved channels 1 and 3 to the legacy DMA controller's `brcm,dma-channel-mask` caused WiFi to fail on boot. The GPU uses these channels internally and they cannot be reclaimed.

## Final DMA Channel Allocation

```
dma0 (fe007000.dma-controller) — Legacy, 8 channels:
  dma0chan0  |  fe300000.mmcnr:rx-tx      (SD card)
  dma0chan1  |  fe204000.spi:tx            (SPI0 TX)
  dma0chan2  |  fe204000.spi:rx            (SPI0 RX)
  dma0chan3  |  fe204600.spi:tx            (SPI3 TX)
  dma0chan4  |  fe204600.spi:rx            (SPI3 RX)
  dma0chan5  |  fe204800.spi:tx            (SPI4 TX)
  dma0chan6  |  fe204800.spi:rx            (SPI4 RX)

dma1 (fe007b00.dma) — DMA4, 2 channels:
  dma1chan0  |  fe204a00.spi:tx            (SPI5 TX)
  dma1chan1  |  fe204a00.spi:rx            (SPI5 RX)
```

## Files

| File | Purpose |
|------|---------|
| `spi3-dma-overlay.dts` | Device tree overlay for SPI3 DMA (DREQ 16/18, legacy controller) |
| `spi4-dma-overlay.dts` | Device tree overlay for SPI4 DMA (DREQ 19/20, legacy controller) |
| `spi5-dma-overlay.dts` | Device tree overlay for SPI5 DMA (DREQ 21/22, DMA4 controller) |
| `spi45-dma-mux.sh` | Script to set PACTL_CS DMA_CNTRL_MUX bits for SPI4/SPI5 DREQ routing |
| `spi45-dma-mux.service` | Systemd unit to run the mux script early in boot |
| `verify-spi3-dma.sh` | Verification script for SPI3 DMA status |
| `check_corruption.py` | Fast CSV corruption analysis tool for validating acquisition data |

## Setup Instructions

### Prerequisites

Build `devmem2` from source (not in default Raspberry Pi OS repos):

```bash
cd ~
wget https://raw.githubusercontent.com/VCTLabs/devmem2/master/devmem2.c
gcc -o devmem2 devmem2.c
sudo cp devmem2 /usr/local/bin/
```

### Step 1: Copy files to the Pi

From the Windows development machine:

```
scp spi3-dma-overlay.dts spi4-dma-overlay.dts spi5-dma-overlay.dts \
    spi45-dma-mux.sh spi45-dma-mux.service verify-spi3-dma.sh \
    morph@192.168.1.99:~/
```

### Step 2: Compile and install overlays

On the Pi:

```bash
# Compile all three overlays
dtc -@ -I dts -O dtb -o spi3-dma.dtbo spi3-dma-overlay.dts
dtc -@ -I dts -O dtb -o spi4-dma.dtbo spi4-dma-overlay.dts
dtc -@ -I dts -O dtb -o spi5-dma.dtbo spi5-dma-overlay.dts

# Install to /boot/overlays/
sudo cp spi3-dma.dtbo spi4-dma.dtbo spi5-dma.dtbo /boot/overlays/
```

### Step 3: Install PACTL_CS mux service

```bash
sudo cp spi45-dma-mux.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/spi45-dma-mux.sh
sudo cp spi45-dma-mux.service /etc/systemd/system/
sudo systemctl enable spi45-dma-mux.service
```

### Step 4: Edit /boot/firmware/config.txt

Add the DMA overlays after their corresponding SPI overlays, and disable HDMI audio to free DMA4 channels for SPI5:

```ini
# Change this existing line:
#   dtoverlay=vc4-kms-v3d
# To:
dtoverlay=vc4-kms-v3d,noaudio

# SPI overlays — DMA overlay must come AFTER the SPI enable overlay
dtoverlay=spi3-2cs
dtoverlay=spi3-dma

dtoverlay=spi4-2cs
dtoverlay=spi4-dma

dtoverlay=spi5-1cs
dtoverlay=spi5-dma
```

**Order matters:** The `spi*-Ncs` overlay creates the SPI node; the `spi*-dma` overlay adds DMA properties to it.

### Step 5: Reboot and verify

```bash
sudo reboot
```

After reboot:

```bash
# Should show NO "no tx-dma" or "no rx-dma" errors
dmesg | grep -i spi

# Should show all SPI buses with DMA channels allocated
cat /sys/kernel/debug/dmaengine/summary

# Verify PACTL_CS mux is set (bits 24+25)
sudo devmem2 0xfe204e00 w

# Run SPI3 verification script
chmod +x verify-spi3-dma.sh
sudo ./verify-spi3-dma.sh
```

Expected `dmesg | grep -i spi` output: **no lines at all** (silence means all SPI buses initialized with DMA successfully, with no fallback to PIO).

Expected DMA summary:

```
dma0: 7 of 8 channels allocated (SPI0, SPI3, SPI4, MMC)
dma1: 2 of 2 channels allocated (SPI5 TX, SPI5 RX)
```

## DREQ Reference

From BCM2711 datasheet section 4.2.1.3 (page 62), with corrections:

| DREQ | Datasheet Label | Actual Function | Type |
|------|----------------|-----------------|------|
| 6    | SPI0 TX        | SPI0 TX         | Direct |
| 7    | SPI0 RX        | SPI0 RX         | Direct |
| 16   | SPI1 TX        | **SPI3 TX**     | Direct (mislabeled) |
| 18   | SPI1 RX        | **SPI3 RX**     | Direct (mislabeled) |
| 19   | UART3 TX       | SPI4 TX         | Alternate (via PACTL_CS mux) |
| 20   | UART3 RX       | SPI4 RX         | Alternate (via PACTL_CS mux) |
| 21   | UART5 TX       | SPI5 TX         | Alternate (via PACTL_CS mux) |
| 22   | UART5 RX       | SPI5 RX         | Alternate (via PACTL_CS mux) |

## PACTL_CS Register

- **Address:** `0xfe204e00` (ARM physical) / `0x7e204e00` (bus)
- **MUX_BITS set:** `0x03000000` (bit 24 + bit 25)
  - Bit 24 (MUX_0): routes DREQ 19/20 from UART3 to SPI4
  - Bit 25 (MUX_1): routes DREQ 21/22 from UART5 to SPI5

## Tradeoffs

- **UART3 DMA disabled** — DREQ 19/20 are rerouted to SPI4. UART3 is not used.
- **UART5 DMA disabled** — DREQ 21/22 are rerouted to SPI5. UART5 is not used.
- **HDMI audio disabled** — DMA4 channels freed for SPI5. HDMI audio is not needed for EEG acquisition.
- **PACTL_CS mux bits are poorly documented by Broadcom.** The `0x03000000` value was determined through community reverse engineering and confirmed working on our hardware.

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `no tx-dma configuration found` for SPI3 | Overlay not loaded or wrong order | Ensure `spi3-dma` comes after `spi3-2cs` in config.txt |
| `no tx-dma configuration found` for SPI4 | PACTL_CS mux not set | Check `systemctl status spi45-dma-mux.service` |
| `no rx-dma configuration found` for SPI5 | DMA4 channels exhausted by HDMI audio | Ensure `dtoverlay=vc4-kms-v3d,noaudio` is set |
| SPI4/5 errors after mux service runs | Driver probed before mux was set (race) | `sudo modprobe -r spi_bcm2835 && sudo modprobe spi_bcm2835` |
| WiFi broken after boot | Legacy DMA channel mask was expanded (channels 1/3) | Do NOT modify the legacy DMA `brcm,dma-channel-mask` |
| Pi won't boot / no SSH | Bad overlay or DMA conflict | Pull SD card, edit config.txt from another machine, remove offending `dtoverlay=` line |

## Validation

After enabling DMA on all SPI buses, run a long acquisition (700+ seconds) and analyze with:

```bash
python check_corruption.py all_ports_ch1_data.csv
```

**Results after DMA enablement:** 708 seconds, 177,230 samples, 4,962,440 data points. 1 isolated single-sample glitch (2 channels on port 5, corruption rate 0.00004%). Zero systematic corruption patterns. All 7 ports show identical noise floors (~1.9 counts RMS).
