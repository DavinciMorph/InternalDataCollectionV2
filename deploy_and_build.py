#!/usr/bin/env python3
"""Upload C++ source to Pi and build."""
import paramiko
import os
import sys

HOST = "192.168.1.99"
USER = "morph"
PASS = "morph"
LOCAL_BASE = os.path.join(os.path.dirname(__file__), "Cpp Implementation")
REMOTE_BASE = "/home/morph/ads1299-cpp"

def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, password=PASS, timeout=10)
    sftp = client.open_sftp()

    # Upload all source files
    count = 0
    for root, dirs, files in os.walk(LOCAL_BASE):
        if "build" in root:
            continue
        for f in files:
            if not f.endswith((".cpp", ".hpp", ".h", ".txt")):
                continue
            local_path = os.path.join(root, f)
            rel = os.path.relpath(local_path, LOCAL_BASE).replace("\\", "/")
            remote_path = REMOTE_BASE + "/" + rel
            remote_dir = os.path.dirname(remote_path)
            # mkdir -p
            parts = remote_dir.split("/")
            for i in range(2, len(parts) + 1):
                d = "/".join(parts[:i])
                try:
                    sftp.stat(d)
                except FileNotFoundError:
                    sftp.mkdir(d)
            sftp.put(local_path, remote_path)
            count += 1
            print(f"  {rel}")

    print(f"\nUploaded {count} files")
    sftp.close()

    # Build
    print("\n--- Building on Pi ---")
    build_cmd = (
        "cd ~/ads1299-cpp && "
        "mkdir -p build && cd build && "
        "cmake .. -DCMAKE_BUILD_TYPE=Release 2>&1 && "
        "make -j4 2>&1"
    )
    stdin, stdout, stderr = client.exec_command(build_cmd, timeout=120)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    rc = stdout.channel.recv_exit_status()
    if out:
        print(out)
    if err:
        print(err, file=sys.stderr)

    if rc == 0:
        print("BUILD SUCCESS")
    else:
        print(f"BUILD FAILED (exit code {rc})")

    client.close()
    return rc

if __name__ == "__main__":
    sys.exit(main())
