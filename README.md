# 4 Wheel Holonomic Drive for Smiley Bot

Uses rpi and pico along with blitz for communication.

## Layout

- `ros2_ws/src/blitz_drive/`: The original Blitz package.
- `ros2_ws/src/robot_interfaces/`: Reused from robocon2026 repo
- `ros2_ws/src/omni_drive/`: one minimal joystick-to-velocity ROS node and launch file.
- `pico_firmware/`: PlatformIO + Arduino firmware for the RP2040 Pico and four BTS7960 modules.
- `tools/alias_pico_serial.sh`: optional device alias because original Blitz hardcodes
  `/dev/ttyUSB0`, while the Pico may enumerate as `/dev/ttyACM0`.

## Build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
sudo apt install python3-serial ros-jazzy-joy
colcon build --symlink-install
source install/setup.bash
```

In another terminal, after connecting to gamepad, run:
```bash
ros2 run joy joy_node
```

```bash
ros2 launch omni_drive omni_drive.launch.py
```

Inspect the input and output before connecting motor power:

```bash
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```

Use the same `/cmd_vel` topic to check that the joystick node is the intended publisher. Do not
run another manual or autonomous publisher on `/cmd_vel` while testing this joystick controller.

## Serial device

The unchanged `blitz_drive/blitz/packer.py` hardcodes `/dev/ttyUSB0` at 115200 baud. If the Pico
appears at `/dev/ttyACM0` and `/dev/ttyUSB0` is unused, create the optional alias:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
./tools/alias_pico_serial.sh /dev/ttyACM0
```

The script refuses to replace an existing real device or a different symlink. Do not create the
alias if `/dev/ttyUSB0` is used by another adapter. Ensure the user has permission to access the
serial port. The alias may need recreating after reboot/reconnection.

## Joystick configuration

Edit `ros2_ws/src/omni_drive/config/omni_drive.yaml`:
Currently
- Forward axis: Left stick y axis
- Lateral axis: Left stick x axis
- Deadman button: Left shoulder button (LB)
- Yaw uses buttons, disabled until `yaw_left_button` and `yaw_right_button` are set to real
  `buttons[]` indices. This avoids unintended rotation from D-pad axes with non-zero neutral values.
- Deadzone: 0.05; joystick timeout: 250 ms; publish rate: 50 Hz.

Hold down the deadman button and then move the sticks in order to make the bot move. Letting go of the deadman button
will stop the bot.

The published `vx`, `vy`, and `vw` are normalized commands in `[-1, 1]`, not SI velocity units.
The Pico applies open-loop PWM and mixes these three inputs into four wheel commands.

## Pico firmware

Open `pico_firmware/` in PlatformIO and build/upload:

```bash
pio run
pio run -t upload
```

The project targets the original RP2040 Pico with the Arduino framework. It uses USB CDC `Serial`
for the unmodified Blitz packet stream; do not print debug text to that port.

The example GPIO assignments in `pico_firmware/include/motor_config.h` are placeholders for four
BTS7960/IBT-2 modules. Confirm the actual board truth table and wiring before applying motor power.
The initial PWM cap is 35%, zero command requests active braking, direct reversal adds 10 ms
at zero PWM, and loss of valid packets for 250 ms requests braking. These safety behaviours need
physical testing with your exact driver boards and motors.

Motion mixing assumes X-drive wheel orientations:

```text
FL = +vx -vy -vw
FR = +vx +vy +vw
RL = +vx +vy -vw
RR = +vx -vy +vw
```

Lift the wheels and test at low power, one motion direction at a time. Adjust individual motor
polarity after confirming the physical direction.