#!/usr/bin/env python3
"""Stream Pi SD card image over SSH with gzip compression."""
import paramiko
import sys
import time
import os

PI_HOST = "192.168.1.175"
PI_USER = "morph"
PI_PASS = "morph"
DEVICE = "/dev/mmcblk0"
OUTPUT = os.path.join(os.path.dirname(__file__), "pi", "pi_image_2026-03-07.img.gz")
BLOCK_SIZE = "4M"

print(f"Streaming {DEVICE} from {PI_HOST} -> {OUTPUT}")
print(f"This will take 15-30 minutes over WiFi...")
print()

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(PI_HOST, username=PI_USER, password=PI_PASS)

# Use sudo with piped password, dd + gzip, stream stdout
cmd = f"echo {PI_PASS} | sudo -S dd if={DEVICE} bs={BLOCK_SIZE} status=none 2>/dev/null | gzip -1"
print(f"Running: sudo dd if={DEVICE} bs={BLOCK_SIZE} | gzip -1")
print()

transport = ssh.get_transport()
channel = transport.open_session()
channel.exec_command(cmd)

start = time.time()
total_bytes = 0
chunk_size = 1024 * 1024  # 1MB read chunks

with open(OUTPUT, "wb") as f:
    while True:
        if channel.recv_ready():
            data = channel.recv(chunk_size)
            if not data:
                break
            f.write(data)
            total_bytes += len(data)
            elapsed = time.time() - start
            rate_mb = (total_bytes / 1024 / 1024) / elapsed if elapsed > 0 else 0
            print(f"\r  Received: {total_bytes / 1024 / 1024:.1f} MB  |  {rate_mb:.1f} MB/s  |  {elapsed:.0f}s elapsed", end="", flush=True)
        elif channel.exit_status_ready():
            # Drain remaining data
            while channel.recv_ready():
                data = channel.recv(chunk_size)
                if data:
                    f.write(data)
                    total_bytes += len(data)
            break
        else:
            time.sleep(0.1)

elapsed = time.time() - start
exit_status = channel.recv_exit_status()
ssh.close()

size_mb = total_bytes / 1024 / 1024
print(f"\n\nDone! Exit status: {exit_status}")
print(f"Image: {OUTPUT}")
print(f"Size: {size_mb:.1f} MB ({total_bytes} bytes)")
print(f"Time: {elapsed:.0f}s ({size_mb / (elapsed/60):.1f} MB/min)")
