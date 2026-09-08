#!/usr/bin/env bash
set -euo pipefail

cat << 'EOF' | sudo tee /etc/udev/rules.d/99-humanoid-controller.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", SYMLINK+="unitree"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="2639", MODE="0666", SYMLINK+="xsens"
ACTION=="add", SUBSYSTEM=="usb-serial", ATTR{latency_timer}="1"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
