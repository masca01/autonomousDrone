# First IMU test — ESP32-S3 + ICM42688P

This sensor-only program checks the chip identity (0x47), resets and configures
the sensor, checks configuration readback, and prints fresh accelerometer,
gyroscope and temperature readings. It does not run PID or drive motors.
It samples at 100 Hz and prints about 5 lines per second. Gyro bias is NOT removed
in this first diagnostic, so small stationary offsets are visible.

## 1. Wire with USB disconnected

Use your existing jumper wires; no breadboard or external resistors are required
for this SPI test if the breakout already has properly soldered headers.
Loose pins pushed through unsoldered holes are not reliable connections.

| IMU printed label | ESP32 printed GPIO / power label |
|---|---|
| VCC / 3V3 | 3V3 (not 5V) |
| GND | GND |
| SCL / SCK / SCLK | IO12 |
| SDA / SDI / MOSI | IO11 |
| SDO / MISO / AD0 | IO13 (verify it is the breakout's SPI SDO pin) |
| CS / nCS | IO10 |
| INT1, INT2 | Unconnected |

These are GPIO numbers, not physical pin positions. Follow the breakout's pin
labels, not an assumed connector order. If CS or SDO is not exposed, or labels
differ, verify the breakout schematic before wiring; do not guess. SDA/SCL here
are used for SPI, not I2C. Keep cables short. Rest the board on a nonconductive
surface. Keep motors, ESC and flight battery disconnected; use USB power only.

## 2. Upload the separate test

1. Connect your ESP32's USB-C port labelled UART to the computer.
2. In VS Code open this project and the PlatformIO sidebar (alien icon).
3. Expand Project Tasks → imu_test → General → Upload.
4. Then choose Monitor under imu_test. Baud rate: 115200.
5. If you missed the startup messages, press the ESP32 RST/RESET button once.

Alternatively, in a PlatformIO terminal:

```sh
pio run -e imu_test -t upload
pio device monitor -b 115200
```

Close other serial monitors before uploading. Select imu_test explicitly: the
project default is still the original stabilization environment.

## 3. Observe and move

Successful startup prints:

```text
DETECTED: ICM42688P (0x47). Configuration verified.
```

Each reading contains a sample counter, accel[g] XYZ, acceleration magnitude
|a|, gyro[deg/s] XYZ and temperature. Example only (not a measured result):

```text
n=20 | accel[g] X=+0.010 Y=-0.020 Z=+1.002 | |a|=1.002 | gyro[deg/s] X=+0.12 Y=-0.31 Z=+0.08 | temp=25.4 C
```

- Leave still for 10 seconds: |a| should be roughly 1 g, gyro near zero with some bias.
- Flat with the sensor's Z axis up: typically Z is near +1 g; orientation can change
  its sign. The magnitude should stay near 1 g in any stationary orientation.
- Gently tilt and stop: gravity moves between accelerometer axes; gyro changes
  while rotating and returns near its resting bias when stopped. Gyro is angular
  velocity, not tilt angle.
- Rotate around each axis in turn to identify axes/signs. Do not shake violently.

Identity/configuration are rechecked about once per second; a missing fresh sample
for one second stops the test. This is a communication diagnostic, not a complete
sensor self-test or flight reliability check. SPI has no acknowledgement/CRC here,
so plausible-looking data alone cannot prove every wire is reliable.

## Troubleshooting

- WHO_AM_I 0x00/0xFF: often wiring, power, chip-select or wrong interface; not a
  definitive diagnosis. Other unexpected IDs may mean a different sensor chip.
- Readback failure or no fresh samples: disconnect USB and check wiring/pin labels.
- No messages: check UART USB connector, data cable and selected serial port; reset.
- Small gyro offset at rest is expected before bias calibration. Large persistent
  readings, inconsistent gravity or intermittent errors need investigation first.

Send a few stationary lines and a few lines while tilting after this test.

## Developer verification

```sh
clang++ -std=c++17 -Wall -Wextra -pedantic -Iinclude native_tests/imu_decode_test.cpp -o /tmp/drone_imu_decode_test
/tmp/drone_imu_decode_test
pio run -e imu_test
```

Registers/scales follow the ICM-42688-P datasheet and the existing project's cached
ICM42688 driver. This isolated diagnostic uses Arduino SPI directly at 1 MHz;
it does not need another downloaded IMU library.
