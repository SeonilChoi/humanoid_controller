#!/usr/bin/env bash
set -euo pipefail

cat << 'EOF' | sudo tee /etc/udev/rules.d/99-humanoid-controller.rules
KERNEL=="ttyACM*", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", MODE="0666", SYMLINK+="CANable"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", SYMLINK+="Unitree"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="2639", MODE="0666", SYMLINK+="Xsens"
ACTION=="add", SUBSYSTEM=="usb-serial", ATTR{latency_timer}="1"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
