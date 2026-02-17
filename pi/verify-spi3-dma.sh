#!/bin/bash
# verify-spi3-dma.sh — Check if SPI3 DMA is active on BCM2711
# Run after installing spi3-dma overlay and rebooting

set -e

echo "=== SPI3 DMA Verification ==="
echo ""

# 1. Check overlay is loaded
echo "[1] Checking loaded overlays..."
if vcgencmd get_config dtoverlay 2>/dev/null | grep -q "spi3-dma"; then
    echo "    ✓ spi3-dma overlay is loaded"
else
    echo "    ✗ spi3-dma overlay NOT found in loaded overlays"
    echo "      Check /boot/firmware/config.txt has: dtoverlay=spi3-dma"
fi
echo ""

# 2. Check device tree for DMA properties
echo "[2] Checking device tree DMA properties..."
SPI3_DT="/proc/device-tree/soc/spi@7e204600"
if [ -d "$SPI3_DT" ]; then
    if [ -f "$SPI3_DT/dma-names" ]; then
        echo "    ✓ dma-names present: $(cat $SPI3_DT/dma-names | tr '\0' ' ')"
    else
        echo "    ✗ dma-names NOT found in device tree node"
    fi
    if [ -f "$SPI3_DT/dmas" ]; then
        echo "    ✓ dmas property present"
    else
        echo "    ✗ dmas property NOT found"
    fi
else
    echo "    ✗ SPI3 device tree node not found at $SPI3_DT"
    echo "      Is spi3-1cs or spi3-2cs overlay loaded?"
fi
echo ""

# 3. Check kernel log for DMA vs PIO
echo "[3] Checking kernel log for SPI3 DMA status..."
SPI3_ADDR="fe204600"  # SPI3 physical address on BCM2711
if dmesg | grep -q "$SPI3_ADDR.*no.*dma"; then
    echo "    ✗ SPI3 fell back to PIO mode (no DMA)"
    echo "      Relevant log lines:"
    dmesg | grep "$SPI3_ADDR" | sed 's/^/      /'
elif dmesg | grep -q "$SPI3_ADDR"; then
    echo "    SPI3 kernel log:"
    dmesg | grep "$SPI3_ADDR" | sed 's/^/      /'
    if dmesg | grep -q "dma.*chan"; then
        echo "    ✓ DMA channels appear to be allocated"
    fi
else
    echo "    ? No SPI3 messages found in dmesg"
fi
echo ""

# 4. Check DMA channel allocation
echo "[4] Checking DMA channel status..."
if [ -d /sys/class/dma ]; then
    echo "    Available DMA channels:"
    ls /sys/class/dma/ | sed 's/^/      /'
else
    echo "    ? /sys/class/dma not found"
fi
echo ""

# 5. Show SPI devices
echo "[5] SPI device nodes:"
ls -la /dev/spidev3.* 2>/dev/null | sed 's/^/    /' || echo "    No /dev/spidev3.* found"
echo ""

echo "=== Done ==="
echo ""
echo "If DMA is NOT working, common issues:"
echo "  - Overlay load order: spi3-Ncs must load BEFORE spi3-dma"
echo "  - Wrong kernel: ensure you're on a BCM2711 RPi 4 / CM4"
echo "  - DMA channel conflict: another peripheral may have claimed channels"
echo "  - Try: sudo dtoverlay -l    (to see runtime overlay state)"
