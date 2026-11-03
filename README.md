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
- `build/examples/example_sysid` — ARTI-D1 joint check: every joint, home, demo replay
- `build/examples/example_action` — play a demo gait trajectory on the ARTI-D1 servos
- `build/ARTI-D1` — run the ARTI-D1 robot

Run the commands from `build/`. Press `Ctrl+C` to stop.

### Motor test
The last number is the goal position in radians.
```bash
cd ~/humanoid_controller/build
./examples/example_unitree ../config/hardware/motor/unitree.yaml 0.0
./examples/example_cubemars ../config/hardware/motor/cubemars.yaml 0.0
./examples/example_dynamixel ../config/hardware/motor/dynamixel.yaml 0.0
```

### Demo trajectory test (ARTI-D1)
See [ARTI-D1 run order](#arti-d1-run-order) below (`example_sysid`, `example_action`).

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
(and in `config/robot/ARTI-D1/joy.yaml` if you run the robot):

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

### ARTI-D1

ARTI-D1 uses Dynamixel servos only (legs XH540-W270, arms XM430-W350) on one bus (`/dev/Dynamixel`), trained with
[ARTI-D1_IsaacLab](https://github.com/SeonilChoi/ARTI-D1_IsaacLab).

The Dynamixel IDs follow the training (Isaac Lab) joint order:

| ID | joint | ID | joint |
|---:|---|---:|---|
| 1 | left_arm_m8 | 8 | right_hip_roll |
| 2 | left_hip_yaw | 9 | left_hip_pitch |
| 3 | right_arm_m8 | 10 | right_hip_pitch |
| 4 | right_hip_yaw | 11 | left_knee |
| 5 | left_arm_m8crank_plate | 12 | right_knee |
| 6 | left_hip_roll | 13 | left_ankle |
| 7 | right_arm_m8crank_plate | 14 | right_ankle |

Servo setup (Dynamixel Wizard): ID as above, Baud Rate 1 Mbps, Operating Mode 3 (Position) or 4 (Extended Position;
torque off before changing it), Return Delay Time 0. Every joint range stays inside one servo turn (pulse 921-3174).
The servos have absolute encoders, so there is no homing: `config/robot/ARTI-D1/motor.yaml` sets each joint zero with
`zero_offset` (motor angle in rad, pi = pulse 2048) and the direction with `axis`. `kp` / `kd` are the Position P / D
Gain register values the controller writes (factory 800 / 0, the values the training sim models).

Home = the real servo zero, where every servo reads pulse 2048 (pi): hip link level, legs straight and vertical, soles
flat. In URDF joint angles hip_roll +/-0.15, hip_pitch -0.6325, knee +1.3736, ankle -0.7411 rad (`robot.yaml`
`policy.home_pose`, `motor.yaml` `zero_offset = pi - axis * home angle`). Every program below starts with the servos
limp and moves from where the robot is; there is no homing run (the Dynamixel servos keep their zero).

### ARTI-D1 run order

Build once on the robot computer:
```bash
cd ~/humanoid_controller
git fetch origin && git checkout arti-d1-2830k && git pull
mkdir -p build && cd build && cmake .. && make -j2
```
Run everything from `build/`. **Hang the robot (feet off the ground) for steps 1 and 2 and for the first policy run.**

**1. Joint check -> home -> demo** (`config/sysid/ARTI-D1.yaml`; asks before every test: Enter = run, `s` = skip,
`q` = quit):
```bash
./examples/example_sysid      # = ../config/robot/ARTI-D1/motor.yaml ../config/sysid/ARTI-D1.yaml ./sysid_logs
```
1. prints the measured pose and stops if a joint is outside its URDF limit (wrong `axis` / `zero_offset`);
2. moves to the default pose (`hold_pose`);
3. `check` of every joint, legs then arms: +0.1 rad, then -0.1 rad, back. It prints what +0.1 rad must look like
   (e.g. `left_knee: left foot moves BACKWARD`); if the joint moved the other way, quit and flip its `axis` in
   `config/robot/ARTI-D1/motor.yaml`;
4. `home`: moves to the real servo zero and prints measured - home per joint (~0 expected);
5. `replay`: moves to the first frame of `config/681.yaml`, holds 2 s, plays the demo (15 s), back to the first frame;
6. back to home, hold 2 s, every servo limp.

Each test writes a CSV to `sysid_logs/` (target, angle, velocity, current, temperature of every joint). It stops with
every servo limp on a tracking error > 0.3 rad, current > 3 A, temperature > 70 C or a servo error. Response tests
(`step`, `chirp`, `sweep`) are in the yaml, commented out.

**2. Demo only** (no joint check): current pose -> home (1 s) -> first demo frame (2 s) -> 681.yaml -> back to the
first frame, held until `Ctrl+C`.
```bash
./examples/example_action                                   # = ../config/robot/ARTI-D1/motor.yaml ../config/681.yaml
./examples/example_action ../config/robot/ARTI-D1/motor.yaml ../config/681.yaml tracking_681.csv   # + tracking CSV
```
`config/681.yaml` is arti_d1 reference motion 681 (straight walk 0.04 m/s, 1.625 s gait cycle, 28 mm swing, 15 s), made
with `python scripts/replay_demo.py --motion_set arti_d1 --index 681 --output 681.yaml --no_realtime` in arti_d1.

**3. Trained policy (ONNX)**: current pose -> home (1 s) -> default pose (= the first demo frame) -> 10 s -> policy.
```bash
./ARTI-D1 ../config/robot/ARTI-D1/robot.yaml --observe   # optional: same start, no policy, prints the observation
./ARTI-D1 ../config/robot/ARTI-D1/robot.yaml
```
`config/robot/ARTI-D1/agent.onnx` is the arti_d1 policy (PhysX pure-RL run B, 2.83M steps; checked in the training
env: ONNX q_des = env target within 1e-6 rad, 16 robots x 20 s from standstill without a fall). The policy starts
standing in place. Left stick = forward / sideways (full stick 0.043 / 0.017 m/s), right stick = turn (0.099 rad/s);
release the sticks to stop. A pelvis tilt over 0.7 rad makes every servo limp. Each run writes
`arti_d1_run_<time>.csv` (send it when a run misbehaves).
Another policy: export it with `python scripts/pt_to_onnx.py --checkpoint <agent_N.pt> --output agent.onnx` in
ARTI-D1_IsaacLab and replace `config/robot/ARTI-D1/agent.onnx` (single file, no `agent.onnx.data`).

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
Use the same name as in your motor config. For example, `config/hardware/motor/cubemars.yaml` uses `can1`.
After the udev rules, the CANable adapter may appear as `/dev/CANable`.
```bash
sudo slcand -o -c -s8 /dev/CANable can1
sudo ip link set can1 up
```