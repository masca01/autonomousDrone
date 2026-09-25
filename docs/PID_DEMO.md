# PID bench demonstration (no motors)

1. Keep the working IMU wiring and power the ESP32 over USB.
2. Stop the existing serial monitor. In PlatformIO select **Project Tasks > pid_demo > General > Upload** and wait for SUCCESS. Opening Monitor alone does not upload firmware.
3. Open **Monitor** under **pid_demo**. Press RESET if you missed startup.
4. Place the sensor flat, Z up, and keep it still until CALIBRATED and READY (about seven seconds). Calibration removes gyro bias; it does not make an arbitrary starting tilt the zero-angle target.
5. Gently tilt around one axis, within +/-20 degrees, then hold still. Read that axis's angle, error, P, I, D and correction. Repeat for the other axis.

Both targets are 0 degrees. Error = target - measured angle. Positive correction means a request for positive rotation about that sensor axis; negative means negative rotation. This is a dimensionless demonstration, not a motor speed, force, or torque command.

With the default gains, holding +20 degrees gives error -20 and P=-0.400. After settling, D approaches zero, I remains zero, and correction is approximately -0.400. Holding -20 degrees gives approximately +0.400. At level while stationary the correction should be near zero; sensor offsets may leave a small residual.

P pulls toward the target. D opposes angular velocity: D=-Kd * filtered gyro rate. During movement D can oppose P to brake the motion, so the total correction need not always have the opposite sign to the angle. The gyro rate is low-pass filtered at 10 Hz. Output is clamped to [-1,+1].

The PID implementation supports I, but Ki=0 intentionally: this is initially a PD demonstration. There is no motor response to correct the hand-held sensor's error, so accumulating an integral here is not a useful tuning exercise. Conditional integration and integral limits are included for later experiments.

Demo gains in src/imu_test.cpp: Kp=0.020, Ki=0, Kd=0.003. These use degrees and degrees/second and are NOT flight gains. For a P-only comparison, change Kd to zero and re-upload pid_demo. Restore 0.003 to see damping.

The controller updates on sensor samples (nominally 100 Hz); printing occurs at 5 Hz. This uses the existing small-angle attitude estimator and assumes sensor axes match the displayed roll/pitch convention. No yaw controller, ESC signals, arming, or motor mixing are implemented by this demonstration. It does not physically stabilize anything until a plant and actuator loop are added and tested.

Original imu_test and attitude_test environments are preserved. The default environment is unchanged: select pid_demo explicitly.
