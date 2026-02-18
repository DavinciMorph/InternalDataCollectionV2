#!/bin/bash
set -e

# Ensure liblz4-dev is installed (needed for TCP streaming)
if ! dpkg -s liblz4-dev >/dev/null 2>&1; then
    echo "Installing liblz4-dev..."
    sudo apt-get install -y liblz4-dev
fi

# Check if files landed in nested dir
if [ -d ~/ads1299-cpp/Cpp\ Implementation ]; then
    echo "Moving files from nested directory..."
    mv ~/ads1299-cpp/Cpp\ Implementation/* ~/ads1299-cpp/
    rmdir ~/ads1299-cpp/Cpp\ Implementation
fi

echo "=== Files ==="
find ~/ads1299-cpp -type f | sort
echo ""

echo "=== Building ==="
cd ~/ads1299-cpp
rm -rf build
mkdir build
cd build
cmake .. 2>&1
echo ""
echo "=== Compiling ==="
make -j4 2>&1

echo ""
echo "=== Result ==="
ls -la ads1299_acquire 2>/dev/null && echo "BUILD SUCCESS" || echo "BUILD FAILED"
