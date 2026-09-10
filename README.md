# Humanoid Controller

This project is a hierarchical control system for a humanoid robot.
It includes motor control, sensor reading, joystick input, and robot-level control.

## Supported hardwares

### Motor drivers

| Status             | Hardware Name |
| ------------------ | ------------- |
| :heavy_check_mark: | `Unitree`     |
| :heavy_check_mark: | `Cubemars`    |

### Sensors

| Status             | Hardware Name |
| ------------------ | ------------- |
| :heavy_check_mark: | `Xsens MTi`   |

### Joysticks

| Status             | Hardware Name  |
| ------------------ | -------------- |
| :heavy_check_mark: | `Dualsense`    |

## Clone
```bash
cd ~/
git clone https://github.com/SeonilChoi/humanoid_controller.git
```

## Pre-build

1. Install the required packages.
```bash
cd ~/humanoid_controller
sudo apt update
xargs -a ./requirements/apt-packages.txt sudo apt install -y
```

2. Install Pinocchio from source. This project uses it to read the robot URDF and compute kinematics.
```bash
cd ~/
sudo apt install -y cmake build-essential git \
  libeigen3-dev libboost-all-dev \
  liburdfdom-dev liburdfdom-headers-dev \
  python3-dev python3-numpy
git clone --recursive https://github.com/stack-of-tasks/pinocchio
cd ~/pinocchio && mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/.local \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_TESTING=OFF
make -j2
make install
```

3. Set these environment variables. Add them to `~/.bashrc` if you want them in every new terminal.
```bash
export CMAKE_PREFIX_PATH=$HOME/.local:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig:$PKG_CONFIG_PATH
```

4. Install the udev rules so USB devices have the right permissions.
This script uses `sudo`. After it finishes, unplug and plug in your USB devices again.
If the devices are already plugged in, the script also sets permissions and brings up `can0`.
```bash
cd ~/humanoid_controller/requirements
./udev-rules.sh
```

## Build

Build the main project. CMake also builds `xspublic` for the Xsens MTi sensors.

Make sure the Pinocchio environment variables from Pre-build are still set.
```bash
cd ~/humanoid_controller
mkdir -p build && cd build
cmake ..
make -j2
```

## Run

After a successful build, you will find four programs in `build/`:

- `motor_manager` — test motors
- `sensor_manager` — test sensors
- `joy_test` — test the joystick
- `robot_test` — run the full robot example

### Motor test (you can also use `cubemars.yaml` or `olaf.yaml`):
```bash
cd ~/humanoid_controller/build
./motor_manager ../config/hardware/motor/unitree.yaml
```

### Sensor test:
```bash
cd ~/humanoid_controller/build
./sensor_manager ../config/hardware/sensor/xsens_mti.yaml
```

### Joystick test:

First, check the joystick driver.
```bash
udevadm info -a /dev/input/js0 | grep -E 'DRIVER'
```

You will see `hid-generic` or `hid-playstation`.
Then set `axis_layout` in `config/hardware/joy/dualsense.yaml`:

- `hid-generic` → `generic`
- `hid-playstation` → `playstation`

```yaml
type: dualsense
device: /dev/input/js0
period: 4000
axis_layout: generic
```

```bash
cd ~/humanoid_controller/build
./joy_test ../config/hardware/joy/dualsense.yaml
```

### Full robot test:
```bash
cd ~/humanoid_controller/build
./robot_test ../config/robot/olaf/olaf.yaml
```

## Troubleshooting

### Joystick axes look swapped

Ubuntu and Jetson often use different joystick drivers.
Check the driver and set `axis_layout` as shown in the Run section.

### Jetson Orin Nano: `slcand` is missing

On a Jetson Orin Nano, `slcand` is not available by default. You need to build extra kernel modules.

The steps below use L4T R36.4.4 kernel sources.

Download kernel sources:
```bash
mkdir -p ~/kernel-src && cd ~/kernel-src
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.4/sources/public_sources.tbz2
tar -xf public_sources.tbz2
cd Linux_for_Tegra/source
tar -xf kernel_src.tbz2
```

Build the modules:
```bash
mkdir -p ~/can-mods/slcan ~/can-mods/gs_usb

cp ~/kernel-src/Linux_for_Tegra/source/kernel/kernel-jammy-src/drivers/net/can/slcan.c \
   ~/can-mods/slcan/
echo 'obj-m := slcan.o' > ~/can-mods/slcan/Makefile

cp ~/kernel-src/Linux_for_Tegra/source/kernel/kernel-jammy-src/drivers/net/can/usb/gs_usb.c \
   ~/can-mods/gs_usb/
echo 'obj-m := gs_usb.o' > ~/can-mods/gs_usb/Makefile

make -C /lib/modules/$(uname -r)/build M=$HOME/can-mods/slcan modules
make -C /lib/modules/$(uname -r)/build M=$HOME/can-mods/gs_usb modules
```

Check vermagic. The string must match your running kernel (`uname -r`).
On Jetson Orin Nano it often looks like `5.15.148-tegra SMP preempt`.
```bash
modinfo ~/can-mods/slcan/slcan.ko | grep vermagic
modinfo ~/can-mods/gs_usb/gs_usb.ko | grep vermagic
```

Install the modules:
```bash
sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/net/can/usb
sudo cp ~/can-mods/slcan/slcan.ko  /lib/modules/$(uname -r)/kernel/drivers/net/can/
sudo cp ~/can-mods/gs_usb/gs_usb.ko /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/
sudo depmod -a

sudo modprobe slcan
sudo modprobe gs_usb
lsmod | grep -E 'slcan|gs_usb'

echo -e 'slcan\ngs_usb' | sudo tee /etc/modules-load.d/can-extra.conf
```

Bring up the CAN interface.
Use the same name as in your motor config. For example, `config/hardware/motor/olaf.yaml` uses `can1`.
After the udev rules, the CANable adapter may appear as `/dev/CANable`.
```bash
sudo slcand -o -c -s8 /dev/CANable can1
sudo ip link set can1 up
```
