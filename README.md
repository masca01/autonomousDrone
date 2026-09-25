# ESP32-S3 Quadcopter Stabilization Project

An educational quadcopter stabilization project using a Freenove ESP32-S3 WROOM
board and an ICM-42688-P IMU.
The first firmware reads an ICM-42688-P over SPI, estimates roll and pitch with a
complementary filter, evaluates two PID controllers, and streams CSV telemetry.
It intentionally does not generate motor signals yet.

**Start with the [IMU test guide](docs/IMU_TEST.md).** Select the separate
`imu_test` PlatformIO environment to check identity, wiring and sensor readings
before running the full estimator/PID program.

## Closed-loop controller experiment

Run `python3 simulation/simulate_closed_loop.py` to compare P-only control and
PD damping with the actual firmware PID implementation. A virtual roll axis
starts at 20 degrees and receives a gust torque at 4 seconds. The script saves
plots, CSV traces and settling metrics in `simulation/output/`.
See [the simulation guide](docs/CLOSED_LOOP_SIMULATION.md) for assumptions and instructions.
No hardware or firmware upload is required.

## What can be run without hardware

Open this folder in VS Code. PlatformIO should recognize `platformio.ini` as the
project configuration.

Run the native C++ control tests:

```bash
clang++ -std=c++17 -Wall -Wextra -pedantic -Iinclude \
  native_tests/control_test.cpp -o /tmp/drone_control_test
/tmp/drone_control_test
```

Install the Python dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Run the single-axis PID simulation:

```bash
python simulation/simulate_single_axis.py
```

The simulation applies a 10-degree command and a short wind-torque disturbance.
It writes a graph and CSV into `simulation/output/`. Its inertia, torque limits,
and gains are illustrative; they are not a validated model of the physical drone.

Run the sensor byte-decoding tests:

```bash
clang++ -std=c++17 -Wall -Wextra -pedantic -Iinclude \
  native_tests/imu_decode_test.cpp -o /tmp/drone_imu_decode_test
/tmp/drone_imu_decode_test
```

Build all firmware environments (no board required):

```bash
pio run -e board_test -e imu_test -e freenove_esp32_s3_wroom
```

Run these commands from the project root. `pio` is available in the PlatformIO
terminal in VS Code. The first build downloads the toolchain and IMU dependency.

## Project layout

| Location | Purpose |
|---|---|
| `src/board_test.cpp` | ESP32 USB serial test; optional external LED |
| `src/imu_test.cpp` | Standalone IMU identity, configuration and raw readings |
| `src/main.cpp` | Attitude estimation and PID telemetry; no motor output |
| `include/` | Pin configuration, PID, attitude filter and IMU decoding |
| `native_tests/` | C++ tests that run without the ESP32 |
| `simulation/` | Single-axis Python PID simulation |
| `tools/log_serial.py` | CSV recorder for `main.cpp`, not `imu_test.cpp` |
| `docs/` | [Board test](docs/FIRST_EXPERIMENT.md) and [IMU test](docs/IMU_TEST.md) guides |
| `platformio.ini` | Firmware build environments and dependencies |

Generated plots, telemetry recordings, CAD drawings, caches and local editor
settings are intentionally excluded from Git. The firmware does not yet implement
ESC output, motor mixing, arming, command reception or flight failsafes.

## Initial IMU wiring

Keep motors, ESCs and batteries disconnected during every bring-up step.

| ICM-42688-P label | ESP32-S3 connection | Purpose |
|---|---:|---|
| VCC / 3V3 | 3.3V | Sensor power |
| GND | GND | Common reference |
| SCL / SCLK | GPIO 12 | SPI clock |
| SDA / SDI / MOSI | GPIO 11 | Data from ESP32 to IMU |
| SDO / MISO | GPIO 13 | Data from IMU to ESP32 |
| CS | GPIO 10 | Chip select |
| INT1 | Leave disconnected initially | Data-ready interrupt later |

Use only 3.3 V logic for the IMU. Check the printed labels on the actual breakout
before applying power because inexpensive modules sometimes arrange the pins in a
different physical order.

## First hardware session

1. Inspect the solder joints and check for a short between 3.3 V and GND.
2. Connect the Freenove board through the USB-C port marked `UART`.
3. After the separate IMU test passes, upload the `freenove_esp32_s3_wroom` environment.
4. Open the serial monitor at 115200 baud.
5. Keep the IMU motionless during startup while the library estimates gyro bias.
6. Confirm that a flat, stationary sensor reports approximately 0 g, 0 g, 1 g and
   approximately 0 deg/s on every gyro axis.
7. Tilt one axis at a time and confirm the reported roll and pitch signs.
8. Save at least 30 seconds of stationary data before changing filters or gains.

After finding the serial-port name, record that stationary dataset with:

```bash
source .venv/bin/activate
python tools/log_serial.py /dev/cu.wchusbserialXXXX --seconds 30
```

On macOS, `ls /dev/cu.*` displays the available serial-port names.

If the sensor is mounted upside down or rotated relative to the quadcopter frame,
the axis mapping must be corrected in software before any motor testing.

## Software stages

1. Board and serial bring-up
2. IMU identity and raw-data checks
3. Stationary noise and bias measurement
4. Roll and pitch estimation
5. PID evaluation without motors
6. One-axis restrained test rig
7. ESC output and motor mixing
8. Restrained four-motor stabilization

Free flight belongs only after the restrained tests and failsafe behavior have been
verified.
