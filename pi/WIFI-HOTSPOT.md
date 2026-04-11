# Pi WiFi Hotspot Mode

Turn the Pi into a WiFi access point so your laptop connects directly to it.
No router needed — useful for field recordings.

Pi IP when in hotspot mode: **10.42.0.1**

> **Note:** The Pi has one radio. When hotspot is on, it cannot connect to other WiFi networks (no internet unless Ethernet is plugged in).

---

## First-Time Setup

Run once on the Pi terminal to create the hotspot profile:

```bash
sudo nmcli device wifi hotspot ssid "MorphAP" password "morph123" ifname wlan0
```

This creates a connection profile called `Hotspot` and immediately starts it.

The Pi is configured to **boot into hotspot mode by default** (Hotspot priority=100, Morph priority=-1).

---

## Turn Hotspot ON

```bash
sudo nmcli connection up Hotspot
```

Then on your laptop, connect to **MorphAP** (password: `morph123`).

SSH into the Pi:
```
ssh morph@10.42.0.1
```

---

## Turn Hotspot OFF (back to normal WiFi)

```bash
sudo nmcli connection down Hotspot
sudo nmcli connection up Morph
sudo nmcli connection down Hotspot && sudo nmcli connection up Morph
```

The Pi reconnects to the Morph home network as a regular client.

To make Morph the default again:
```bash
sudo nmcli connection modify Morph connection.autoconnect-priority 100
sudo nmcli connection modify Hotspot connection.autoconnect-priority -1
```

---

## Quick Reference

| Action | Command |
|--------|---------|
| Start hotspot | `sudo nmcli connection up Hotspot` |
| Stop hotspot | `sudo nmcli connection down Hotspot` |
| Reconnect to Morph | `sudo nmcli connection up Morph` |
| Check current connection | `nmcli device status` |
| Pi IP in hotspot mode | `10.42.0.1` |
| SSID | `MorphAP` |
| Password | `morph123` |
