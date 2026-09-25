# Attitude bench test

Keep the existing SPI wiring. No motors, ESC or battery are needed.

1. In PlatformIO Project Tasks, choose **attitude_test > General > Upload**.
2. Choose **attitude_test > General > Monitor** (115200 baud).
3. Place the IMU flat and still, Z axis upward. If necessary press the ESP32 reset button after opening the monitor.
4. Wait for two seconds of warmup and about five seconds of calibration, then **CALIBRATED** and **READY**. Detected motion restarts calibration. Do not rotate slowly during calibration: the motion check cannot distinguish all slow rotation from sensor bias.
5. Gently tilt around X, hold, and observe roll. Tilt around Y and observe pitch. Start within +/-30 degrees. Corrected gyro readings return near zero when stopped, while tilt remains.
6. Rotate around Z while keeping the sensor approximately level: yaw_relative accumulates the rotation instead of returning to zero. It is relative to startup and drifts; this IMU has no absolute heading reference.

Calibration is repeated after every reset and is not saved. This corrects gyro bias only, not accelerometer offsets or mounting alignment. The existing complementary filter is a small-angle learning demonstration, not a general 3D attitude estimator. Shaking/translation disturbs the gravity-based tilt estimate. This program does not run a PID or drive motors.

The original **imu_test** environment still prints raw readings. Use it for wiring diagnostics. Unplug USB before changing wiring.
