#!/usr/bin/env python3
"""
Dummy EEG client — connects, receives, discards.

Measures receive rate and local memory to isolate whether streaming
degradation is caused by the GUI or the Pi server/WiFi stack.

Usage:
    python dummy_client.py              # default: 192.168.1.99:8888
    python dummy_client.py 10.0.0.5     # custom IP
    python dummy_client.py 10.0.0.5 9999  # custom IP and port
"""

import socket
import struct
import json
import time
import sys
import os

PI_HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.99"
PI_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8888

HEADER_SIZE = 8  # uint32 compressed_size + uint32 sample_count
header_struct = struct.Struct('<II')


def recv_exact(sock, n):
    """Read exactly n bytes from socket."""
    buf = bytearray(n)
    view = memoryview(buf)
    pos = 0
    while pos < n:
        nbytes = sock.recv_into(view[pos:])
        if nbytes == 0:
            raise ConnectionError("server closed connection")
        pos += nbytes
    return bytes(buf)


def main():
    print(f"Connecting to {PI_HOST}:{PI_PORT} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((PI_HOST, PI_PORT))
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    # Large receive buffer so we never bottleneck on our end
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)
    print("Connected.\n")

    # Read JSON metadata line
    meta_buf = b""
    while not meta_buf.endswith(b"\n"):
        meta_buf += sock.recv(4096)
    metadata = json.loads(meta_buf.decode("utf-8").strip())
    print(f"Metadata: {metadata['num_channels']}ch @ {metadata['sample_rate']}Hz, "
          f"batch={metadata['batch_size']}, sample_size={metadata['sample_size']}B")
    print(f"Ports: {metadata['ports']}\n")

    # Stats
    t_start = time.time()
    t_last_print = t_start
    samples_total = 0
    samples_window = 0
    bytes_total = 0
    frames_total = 0

    try:
        process = None
        try:
            import psutil
            process = psutil.Process(os.getpid())
        except ImportError:
            pass

        print(f"{'Elapsed':>8s}  {'Rate':>7s}  {'Avg':>7s}  {'Samples':>10s}  {'Bytes':>10s}  {'Frames':>8s}  {'RSS':>8s}")
        print("-" * 75)

        while True:
            # Read 8-byte frame header
            hdr = recv_exact(sock, HEADER_SIZE)
            compressed_size, sample_count = header_struct.unpack(hdr)

            # Read compressed payload (discard immediately)
            recv_exact(sock, compressed_size)

            samples_total += sample_count
            samples_window += sample_count
            bytes_total += HEADER_SIZE + compressed_size
            frames_total += 1

            # Print stats every 5 seconds
            now = time.time()
            if now - t_last_print >= 5.0:
                elapsed = now - t_start
                window_rate = samples_window / (now - t_last_print)
                avg_rate = samples_total / elapsed

                rss = ""
                if process:
                    rss = f"{process.memory_info().rss / 1024:.0f}KB"

                print(f"{elapsed:7.1f}s  {window_rate:6.1f}Hz  {avg_rate:6.1f}Hz  "
                      f"{samples_total:>10,}  {bytes_total:>10,}  {frames_total:>8,}  {rss:>8s}")

                samples_window = 0
                t_last_print = now

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        avg = samples_total / elapsed if elapsed > 0 else 0
        print(f"\n\nStopped after {elapsed:.1f}s — {samples_total:,} samples, avg {avg:.1f} Hz")
    except ConnectionError as e:
        elapsed = time.time() - t_start
        avg = samples_total / elapsed if elapsed > 0 else 0
        print(f"\nDisconnected: {e}")
        print(f"Ran {elapsed:.1f}s — {samples_total:,} samples, avg {avg:.1f} Hz")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
