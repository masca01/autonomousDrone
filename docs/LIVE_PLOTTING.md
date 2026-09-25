# Live ESP32 plots

Uses the existing `pid_demo` or `attitude_test` text output; no firmware change is needed. Raw `imu_test` and the main CSV firmware are not supported by this plotter.

1. Upload **pid_demo** if it is not already on the board.
2. **Close PlatformIO Monitor** (Ctrl+C). Only one program should use the USB serial port.
3. From the project terminal, run:

```sh
source .venv/bin/activate
python tools/plot_live.py --list
python tools/plot_live.py
```

If there is exactly one detected USB serial device it is selected automatically. Otherwise specify the actual port from the list:

```sh
python3 tools/plot_live.py /dev/cu.YOUR_PORT
```

If dependencies are missing, use your project's Python virtual environment and install `requirements.txt` there. This script needs an interactive Matplotlib desktop backend; the simulation's headless Agg backend cannot display a live window.

Opening serial may restart the board. Keep the IMU still during calibration (about seven seconds). If no readings arrive, press RESET once. The status line shows startup/calibration messages. Gently tilt and hold to see the roll/pitch angles and their corrections. With attitude_test, the bottom plot stays empty because that firmware does not output corrections.

Plots show angle, angular velocity, and PID correction over the latest 20 seconds. Use `--window 60` for one minute. The existing firmware prints at about 5 Hz even though its controller updates at about 100 Hz. These plots are for observing slow hand movements, not measuring fast control dynamics. Timestamps are computer receive times, not sensor acquisition times. History is bounded and is not recorded to disk.

Close the plot window or press Ctrl+C to release USB before uploading or reopening PlatformIO Monitor. If USB is disconnected, reconnect and restart the plot command.

Preview without hardware:

```sh
python3 tools/plot_live.py --demo
```

The preview is clearly labeled synthetic. It does not read the ESP32.
