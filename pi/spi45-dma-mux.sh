#!/bin/bash
# spi45-dma-mux.sh — Configure PACTL_CS to route DMA DREQs to SPI4/SPI5
#
# The BCM2711 PACTL_CS register controls whether certain DREQ lines
# are routed to UARTs or to SPI4/SPI5. This script must run at boot
# BEFORE the SPI driver probes (or you must reload the driver after).
#
# PACTL_CS register:
#   Bus address:  0x7e204e00
#   ARM physical: 0xfe204e00
#
# DMA_CNTRL_MUX bits (from datasheet page 62 + community RE):
#   Bit 23: DMA_CNTRL_MUX_2
#   Bit 24: DMA_CNTRL_MUX_0
#   Bit 25: DMA_CNTRL_MUX_1
#
# WARNING: The exact bit combinations and their effects are poorly
# documented by Broadcom. The information below is from community
# reverse engineering (forum user abqjln). Test carefully.
#
# PREREQUISITE: Install devmem2
#   sudo apt-get install devmem2
#
# USAGE:
#   1. Test manually first: sudo ./spi45-dma-mux.sh
#   2. Once working, install as a systemd service (see below)

set -e

PACTL_CS_ADDR="0xfe204e00"

# Read current value
echo "Reading PACTL_CS at $PACTL_CS_ADDR..."
CURRENT=$(devmem2 $PACTL_CS_ADDR w | grep "Read" | awk '{print $NF}')
echo "  Current value: $CURRENT"

# DMA_CNTRL_MUX bits
# Bit 23 = 0x00800000  (MUX_2)
# Bit 24 = 0x01000000  (MUX_0)
# Bit 25 = 0x02000000  (MUX_1)
#
# The exact combination needed depends on which alternates you want.
# From abqjln's experiments, various combinations of these bits
# switch different DREQ lines between UART and SPI routing.
#
# Strategy: Try setting MUX_0 first (bit 24) as a starting point,
# then test if SPI4/SPI5 DMA works. If not, try other combinations.
#
# Common combinations to try:
#   0x01000000  — MUX_0 only
#   0x02000000  — MUX_1 only
#   0x03000000  — MUX_0 + MUX_1
#   0x03800000  — all three MUX bits

MUX_BITS="0x03000000"  # MUX_0 (bit 24) + MUX_1 (bit 25) — routes UART3→SPI4 and UART5→SPI5

echo "Setting DMA_CNTRL_MUX bits: $MUX_BITS"

# OR the mux bits into the current register value
# (preserve other bits, just set the mux ones)
NEW_VAL=$(printf "0x%08X" $(( $CURRENT | $MUX_BITS )))
echo "  New value: $NEW_VAL"

devmem2 $PACTL_CS_ADDR w $NEW_VAL

# Verify
VERIFY=$(devmem2 $PACTL_CS_ADDR w | grep "Read" | awk '{print $NF}')
echo "  Verified: $VERIFY"

echo ""
echo "PACTL_CS mux configured. If SPI4/SPI5 DMA still doesn't work:"
echo "  1. Try different MUX_BITS combinations (edit this script)"
echo "  2. Verify DREQ numbers in the DT overlay match the datasheet"
echo "  3. Check dmesg for SPI DMA allocation messages"
echo ""
echo "NOTE: The SPI driver may have already probed in PIO mode."
echo "You may need to reload it:"
echo "  sudo modprobe -r spi_bcm2835 && sudo modprobe spi_bcm2835"
echo "Or place this script in a systemd unit that runs before"
echo "the device tree is fully processed (early boot)."
