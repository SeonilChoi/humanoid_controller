# Humanoid Controller

This project controls a humanoid robot.
It can talk to motors, sensors, and a joystick.

## Supported hardware

### Motors

| Status             | Hardware    |
| ------------------ | ----------- |
| :heavy_check_mark: | `Unitree`   |
| :heavy_check_mark: | `Cubemars`  |
| :heavy_check_mark: | `Dynamixel` |

### Sensors

| Status             | Hardware    |
| ------------------ | ----------- |
| :heavy_check_mark: | `Xsens MTi` |

### Joysticks

| Status             | Hardware   |
| ------------------ | ---------- |
| :heavy_check_mark: | `DualSense` |

## Clone
```bash
cd ~/
git clone https://github.com/SeonilChoi/humanoid_controller.git
```

## Setup

1. Install the required packages.
```bash
cd ~/humanoid_controller
sudo apt update
xargs -a ./requirements/apt-packages.txt sudo apt install -y
```

2. Install Pinocchio. The robot uses it to read the URDF.
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

3. Install ONNX Runtime. The robot controller needs it.
On Jetson, use the ARM64 package.
```bash
mkdir -p ~/.local
cd /tmp
wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-aarch64-1.20.1.tgz
tar -xzf onnxruntime-linux-aarch64-1.20.1.tgz
cp -r onnxruntime-linux-aarch64-1.20.1/include/. ~/.local/include/
cp -r onnxruntime-linux-aarch64-1.20.1/lib/. ~/.local/lib/
```

4. Set these environment variables. Add them to `~/.bashrc` if you want them in every new terminal.
```bash
export CMAKE_PREFIX_PATH=$HOME/.local:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig:$PKG_CONFIG_PATH
```

5. Install the udev rules so USB devices have the right names and permissions.
This script uses `sudo`. After it finishes, unplug and plug in your USB devices again.
If the devices are already plugged in, the script also sets permissions and brings up USB CANable as `can1`.
```bash
cd ~/humanoid_controller/requirements
./udev-rules.sh
```

## Build

Build the project. CMake also builds the Xsens library.

Make sure the environment variables from Setup are still set.
```bash
cd ~/humanoid_controller
mkdir -p build && cd build
cmake ..
make -j2
```

## Run

After a successful build, you will find these programs:

- `build/examples/example_unitree` — test Unitree motors
- `build/examples/example_cubemars` — test Cubemars motors
- `build/examples/example_dynamixel` — test Dynamixel motors
- `build/examples/example_xsens` — test the IMU
- `build/examples/example_dualsense` — test the joystick
- `build/ARTI-H1` — run the robot

Run the commands from `build/`. Press `Ctrl+C` to stop.

### Motor test
The last number is the goal position in radians.
```bash
cd ~/humanoid_controller/build
./examples/example_unitree ../config/hardware/motor/unitree.yaml 0.0
./examples/example_cubemars ../config/hardware/motor/cubemars.yaml 0.0
./examples/example_dynamixel ../config/hardware/motor/dynamixel.yaml 0.0
```

### Sensor test
```bash
cd ~/humanoid_controller/build
./examples/example_xsens ../config/hardware/sensor/xsens_mti.yaml
```

### Joystick test

First, check the joystick driver.
```bash
udevadm info -a /dev/input/js0 | grep -E 'DRIVER'
```

You will see `hid-generic` or `hid-playstation`.
Then set `axis_layout` in `config/hardware/joy/dualsense.yaml`
(and in `config/robot/ARTI-H1/joy.yaml` if you run the robot):

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
./examples/example_dualsense ../config/hardware/joy/dualsense.yaml
```

### Robot
```bash
cd ~/humanoid_controller/build
./ARTI-H1 ../config/robot/ARTI-H1/robot.yaml                 # AMP policy (robot.yaml model_file)
./ARTI-H1 ../config/robot/ARTI-H1/robot.yaml --ppo           # RL-only PPO policy (robot.yaml ppo_model_file)
./ARTI-H1 ../config/robot/ARTI-H1/robot.yaml --model ../config/robot/ARTI-H1/agent_s2r_340000.onnx   # any other ONNX
./ARTI-H1 ../config/robot/ARTI-H1/robot_velocity.yaml        # velocity-command policy (r21, new pelvis / -20-40-20 zero, 81-D obs)
```
**Right-leg zero check** (11-11: in the default pose the right sole does not reach the floor, while motor.yaml uses
the same homing_offset on both sides): `./ARTI-H1 ../config/robot/ARTI-H1/robot_velocity.yaml --observe --limp-right`.
After homing and the IMU calibration the right-leg motors go limp (kp 0, damping only) and the left leg keeps the default
pose. **Support the robot**, hold the pelvis level (printed roll / pitch near 0) and press both soles flat on the floor
with the right foot placed as the mirror image of the left. Every 0.5 s it prints the ~2 s average of left / right joint
angles, right - left, and the right motor's homing_offset now -> suggested (offset - (right - left)). Copy the
suggestion into motor.yaml once the numbers are steady, home again and check that the right sole now touches.

`--ppo` / `--model` combine with `--blind-imu` and `--observe`. With `--ppo` / `--model` the log file name includes
the model name, so AMP and PPO runs stay apart.

**Branch `arti-h1-new-pelvis`** (11-12): new pelvis (hip_yaw mount 317.07 mm, was 280.65) and leg joint zero
-20/40/-20 deg instead of -45/90/-45 (q_new = q_old + hip_pitch 25, knee -50, ankle_pitch -25 deg). `motor.yaml`
homing offsets / limits, `robot.urdf`, the default pose and the IMU offset (z 0.34667) follow the new zero. Re-home
after switching to this branch; the models trained on the old zero (r05-r16, `robot.yaml` AMP / PPO models) do not
match this `motor.yaml`. The old-pelvis setup stays on branch `arti-h1-s2r-600k` (r16).

`robot_velocity.yaml` runs `agent_vel_r21_240000.onnx` with `policy.observation: velocity_history`: the 81-D
`ArtiH1VelocityEnv` observation (pelvis gyro, gravity in the pelvis frame, joint pos - default pose, joint vel, the last
three actions, last joint target - default pose, raw command) instead of the 116-D AMP one, and
`policy.target_rate_limit: 5.24` (rad/s): each control step the joint target moves at most 5.24 * control_dt from the
previous one, as in training. `policy.target_lowpass_alpha: 0.6` low-passes the target before that limit
(target = previous + 0.6 * (q_des - previous)); without it the r12 policy chattered at 30 Hz on the robot; r14-r21 were trained with this low-pass in the loop; r15+ are also trained against per-joint homing errors (r21: up to 0.20 rad on ankle_pitch). Full stick = vx 0.15 m/s, vy 0.08 m/s, yaw 0.4 rad/s; a released stick (zero command)
stands still. The previous 45-D model still works with `model_file: .../agent_vel_r05_240000.onnx`,
`observation: velocity` and no `target_rate_limit`. `--observe` prints the velocity observation in both modes. Do not
use `--model` with a velocity ONNX on `robot.yaml` (the AMP layout would be fed to it; the ONNX input size check stops
it).

## Troubleshooting

### Joystick axes look swapped

Ubuntu and Jetson often use different joystick drivers.
Check the driver and set `axis_layout` as shown in the Run section.

### Jetson Orin Nano: `slcand` is missing

On a Jetson Orin Nano, `slcand` is not installed by default.
You need to build two extra kernel modules.

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

Check that the module matches your kernel (`uname -r`).
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
Use the same name as in your motor config. For example, `config/robot/ARTI-H1/motor.yaml` uses `can1`.
After the udev rules, the CANable adapter may appear as `/dev/CANable`.
```bash
sudo slcand -o -c -s8 /dev/CANable can1
sudo ip link set can1 up
```