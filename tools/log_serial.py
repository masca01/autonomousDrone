#!/usr/bin/env python3
"""Record CSV telemetry produced by the ESP32 stabilization firmware."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, for example /dev/cu.wchusbserialXXXX")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("measurements/imu_stationary.csv"),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    deadline = time.monotonic() + args.seconds
    header: list[str] | None = None
    rows_written = 0

    with serial.Serial(args.port, args.baud, timeout=1.0) as device, args.output.open(
        "w", newline=""
    ) as output_file:
        writer = csv.writer(output_file)
        print(f"Recording {args.port} at {args.baud} baud for {args.seconds:g} s")

        while time.monotonic() < deadline:
            raw_line = device.readline().decode("utf-8", errors="replace").strip()
            if not raw_line:
                continue
            if raw_line.startswith("INFO,"):
                print(raw_line)
                continue
            if raw_line.startswith("ERROR,"):
                raise RuntimeError(raw_line)

            fields = [field.strip() for field in raw_line.split(",")]
            if fields[0] == "time_s":
                header = fields
                writer.writerow(header)
                continue
            if header is None or len(fields) != len(header):
                continue

            writer.writerow(fields)
            rows_written += 1

    print(f"Saved {rows_written} samples to {args.output}")


if __name__ == "__main__":
    main()
