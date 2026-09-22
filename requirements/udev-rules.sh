#!/usr/bin/env bash
set -euo pipefail

cat << 'EOF' | sudo tee /etc/udev/rules.d/99-humanoid-controller.rules
ACTION=="add", KERNEL=="ttyACM*", SUBSYSTEM=="tty", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", ATTRS{serial}=="2061317B5230", MODE="0666", SYMLINK+="CANable"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FTBW2W11", MODE="0666", SYMLINK+="Unitree"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FTBENY4L", MODE="0666", SYMLINK+="Dynamixel"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="2639", MODE="0666", SYMLINK+="Xsens"
ACTION=="add", SUBSYSTEM=="usb-serial", ATTR{latency_timer}="1"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
sleep 1

for dev in /dev/Unitree /dev/Dynamixel /dev/CANable /dev/Xsens; do
    if [[ -e "$dev" ]]; then
        sudo chmod 777 "$dev"
    fi
done

# USB CANable2 is SLCAN over CDC ACM. Onboard mttcan already occupies can0,
# so the adapter is brought up as can1.
if [[ -e /dev/CANable ]] && command -v slcand >/dev/null 2>&1; then
    sudo modprobe slcan 2>/dev/null || true
    if ! ip link show can1 >/dev/null 2>&1; then
        sudo slcand -o -c -s8 /dev/CANable can1
        sleep 0.5
    fi
    sudo ip link set can1 up
fi
