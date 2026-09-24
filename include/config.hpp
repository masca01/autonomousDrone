#pragma once

#include <cstdint>

namespace config {

// Standard FSPI pins on the ESP32-S3. Verify the printed pin names on the
// Freenove board before wiring the sensor.
constexpr std::uint8_t kImuCsPin = 10;
constexpr std::uint8_t kImuMosiPin = 11;
constexpr std::uint8_t kImuSckPin = 12;
constexpr std::uint8_t kImuMisoPin = 13;

constexpr std::uint32_t kControlRateHz = 500;
constexpr std::uint32_t kControlPeriodUs = 1'000'000U / kControlRateHz;
constexpr std::uint32_t kTelemetryRateHz = 100;
constexpr std::uint32_t kTelemetryDivider = kControlRateHz / kTelemetryRateHz;

static_assert(kControlRateHz % kTelemetryRateHz == 0,
              "Telemetry rate must divide the control rate");

}  // namespace config
