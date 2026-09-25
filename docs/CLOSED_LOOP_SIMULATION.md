# Closed-loop roll simulation

Run from the project terminal:

```sh
python3 simulation/simulate_closed_loop.py
```

Requires a C++17 compiler and matplotlib (already listed in requirements.txt).
If you use the project's virtual environment, activate it first. No ESP32 or upload is required.
The script compiles a temporary executable using the actual firmware `PidController`, runs numerical checks, and saves a comparison plot, three CSV traces, and a JSON metrics file in `simulation/output/`.

## Experiment

Start at +20 degrees, target level (0 degrees). At 4 seconds apply an external torque of +0.020 N m for 0.20 seconds. This represents a gust's rotational effect, not a specified wind speed.
Compare Kp=0.020 and Ki=0 with Kd=0 (P only), 0.003 (current bench demo), and 0.008 (more damping). Gyro derivative filtering is 10 Hz. The controller runs at 100 Hz and has the same normalized output/integral limits as pid_demo. Angle inputs are in degrees; rate inputs are in degrees/second.

## What closes the loop?

Angle and rate -> PID -> normalized correction -> requested torque -> actuator lag -> angular acceleration -> new rate and angle -> PID again.

The plant uses radians internally:

`inertia * angular_acceleration = actuator_torque + gust_torque - drag * angular_rate`

Illustrative assumptions: inertia 0.0025 kg m^2; drag 0.004 N m s/rad; full correction corresponds to 0.080 N m; first-order actuator time constant 0.030 s. Plant integration uses 1 ms steps while the controller holds its command for 10 ms.

The correction-to-torque scale is an explicit MODEL assumption, not an ESC conversion or a measured motor property. Real mixing, thrust curves, hover headroom and motor saturation remain future work. The model covers one rotational axis, with ideal angle/rate measurements: no IMU noise, yaw coupling, translational flight, battery effects or real hardware validation.

## Reading the result

Top: Does the angle return to zero, and does it overshoot or oscillate?
Middle: What correction does the controller request? It is limited to [-1,+1].
Bottom: How the actuator torque follows the correction, alongside the gust torque.

Settling time means entering and remaining within +/-1 degree through the rest of the relevant window. Initial settling is measured before the gust; gust recovery is measured from the end of the gust to the end of the run. At least 0.5 seconds of in-band data is required. A null value means settling was not confirmed in that window. Peak angle after the gust can include residual oscillation from the initial tilt.

To experiment, change the damping values in CASES in simulate_closed_loop.py. The runner checks finite outputs, correction/torque limits, a zero-controller baseline, initial correction direction, integration refinement, and recovery for the default demo gains. Altering the experiment may require changing its recovery expectations; investigate failed assertions rather than ignoring them.

These curves help explain feedback and damping. Do not transfer gains directly to a real drone without a measured model and restrained testing.
