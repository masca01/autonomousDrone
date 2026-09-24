#!/usr/bin/env python3
"""Simulate one rotational axis controlled by the same PID structure as the drone."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float
    derivative_cutoff_hz: float = 30.0
    integral: float = 0.0
    derivative: float = 0.0
    previous_measurement: float = 0.0
    initialized: bool = False

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        raw_derivative = 0.0
        if self.initialized:
            raw_derivative = -(measurement - self.previous_measurement) / dt

        rc = 1.0 / (2.0 * math.pi * self.derivative_cutoff_hz)
        alpha = dt / (rc + dt)
        self.derivative += alpha * (raw_derivative - self.derivative)

        p_term = self.kp * error
        d_term = self.kd * self.derivative
        candidate_i = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral + self.ki * error * dt),
        )
        candidate = p_term + candidate_i + d_term

        saturating_high = candidate > self.output_limit
        saturating_low = candidate < -self.output_limit
        if (
            (not saturating_high and not saturating_low)
            or (saturating_high and error < 0.0)
            or (saturating_low and error > 0.0)
        ):
            self.integral = candidate_i

        output = max(
            -self.output_limit,
            min(self.output_limit, p_term + self.integral + d_term),
        )
        self.previous_measurement = measurement
        self.initialized = True
        return output


def command_angle(time_s: float) -> float:
    """Ten-degree step beginning at 0.5 seconds."""
    return math.radians(10.0) if time_s >= 0.5 else 0.0


def wind_torque(time_s: float) -> float:
    """Short external torque pulse representing a modest wind gust."""
    return 0.020 if 2.0 <= time_s < 2.15 else 0.0


def main() -> None:
    output_dir = Path(__file__).resolve().parent / "output"
    output_dir.mkdir(parents=True, exist_ok=True)

    dt = 1.0 / 500.0
    duration = 4.0
    inertia = 0.0025       # kg m^2, illustrative single-axis inertia
    damping = 0.020        # N m s/rad
    actuator_limit = 0.080 # N m

    pid = PID(
        kp=0.5,
        ki=0,
        kd=0.02,
        output_limit=actuator_limit,
        integral_limit=0.025,
    )

    angle = 0.0
    angular_rate = 0.0
    rows: list[tuple[float, ...]] = []

    step_count = int(duration / dt) + 1
    for step in range(step_count):
        time_s = step * dt
        target = command_angle(time_s)
        controller_torque = pid.update(target, angle, dt)
        gust = wind_torque(time_s)

        angular_acceleration = (
            controller_torque + gust - damping * angular_rate
        ) / inertia
        angular_rate += angular_acceleration * dt
        angle += angular_rate * dt

        rows.append(
            (
                time_s,
                math.degrees(target),
                math.degrees(angle),
                math.degrees(angular_rate),
                controller_torque,
                gust,
            )
        )

    csv_path = output_dir / "single_axis_pid.csv"
    with csv_path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "time_s",
                "setpoint_deg",
                "angle_deg",
                "angular_rate_deg_s",
                "control_torque_nm",
                "wind_torque_nm",
            ]
        )
        writer.writerows(rows)

    time = [row[0] for row in rows]
    setpoint = [row[1] for row in rows]
    measured = [row[2] for row in rows]
    control = [row[4] for row in rows]
    gust = [row[5] for row in rows]

    figure, (axis_angle, axis_torque) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axis_angle.plot(time, setpoint, "--", label="command")
    axis_angle.plot(time, measured, label="simulated angle")
    axis_angle.set_ylabel("Angle (deg)")
    axis_angle.grid(True, alpha=0.3)
    axis_angle.legend()

    axis_torque.plot(time, control, label="PID torque")
    axis_torque.plot(time, gust, label="wind disturbance")
    axis_torque.set_xlabel("Time (s)")
    axis_torque.set_ylabel("Torque (N m)")
    axis_torque.grid(True, alpha=0.3)
    axis_torque.legend()

    figure.suptitle("Single-axis attitude PID preparation")
    figure.tight_layout()
    plot_path = output_dir / "single_axis_pid.png"
    figure.savefig(plot_path, dpi=160)
    plt.close(figure)

    peak_after_gust = max(
        abs(row[2] - row[1]) for row in rows if 2.0 <= row[0] <= 3.0
    )
    final_error = rows[-1][1] - rows[-1][2]
    print(f"Wrote {csv_path}")
    print(f"Wrote {plot_path}")
    print(f"Peak absolute error after gust: {peak_after_gust:.3f} deg")
    print(f"Final error: {final_error:.3f} deg")


if __name__ == "__main__":
    main()
