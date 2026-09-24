#include <Arduino.h>
#include <ICM42688.h>
#include <SPI.h>

#include "config.hpp"
#include "control/complementary_attitude.hpp"
#include "control/pid_controller.hpp"

namespace {

ICM42688 imu(SPI, config::kImuCsPin);
control::ComplementaryAttitude estimator(0.50F);

// These are deliberately conservative starting values. The output is only
// telemetry at this stage; no motor pins are configured in this program.
control::PidController roll_pid({0.04F, 0.01F, 0.002F},
                                {-1.0F, 1.0F, -0.25F, 0.25F});
control::PidController pitch_pid({0.04F, 0.01F, 0.002F},
                                 {-1.0F, 1.0F, -0.25F, 0.25F});

std::uint32_t previous_loop_us = 0;
std::uint32_t next_loop_us = 0;
std::uint32_t loop_count = 0;

[[noreturn]] void stopWithError(const char* message, int status) {
  Serial.print("ERROR,");
  Serial.print(message);
  Serial.print(",status=");
  Serial.println(status);
  while (true) {
    delay(1000);
  }
}

void printTelemetryHeader() {
  Serial.println(
      "time_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,"
      "yaw_deg,roll_error,roll_p,roll_i,roll_d,roll_output,pitch_output");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("INFO,starting ESP32-S3 stabilization bring-up");
  Serial.println("INFO,keep the IMU completely still during initialization");

  SPI.begin(config::kImuSckPin, config::kImuMisoPin,
            config::kImuMosiPin, config::kImuCsPin);

  const int begin_status = imu.begin();
  if (begin_status < 0) {
    stopWithError("IMU initialization failed", begin_status);
  }

  if (imu.setAccelFS(ICM42688::gpm4) < 0 ||
      imu.setGyroFS(ICM42688::dps500) < 0 ||
      imu.setAccelODR(ICM42688::odr500) < 0 ||
      imu.setGyroODR(ICM42688::odr500) < 0) {
    stopWithError("IMU configuration failed", -1);
  }

  printTelemetryHeader();
  previous_loop_us = micros();
  next_loop_us = previous_loop_us + config::kControlPeriodUs;
}

void loop() {
  const std::uint32_t now_us = micros();
  if (static_cast<std::int32_t>(now_us - next_loop_us) < 0) {
    return;
  }

  const float dt_seconds =
      static_cast<float>(now_us - previous_loop_us) * 1.0e-6F;
  previous_loop_us = now_us;
  next_loop_us += config::kControlPeriodUs;

  // Recover from a long pause instead of executing many stale iterations.
  if (dt_seconds > 0.02F) {
    next_loop_us = now_us + config::kControlPeriodUs;
    return;
  }

  if (imu.getAGT() < 0) {
    stopWithError("IMU read failed", -1);
  }

  const auto attitude = estimator.update(
      imu.accX(), imu.accY(), imu.accZ(),
      imu.gyrX(), imu.gyrY(), imu.gyrZ(), dt_seconds);

  constexpr float kLevelSetpointDegrees = 0.0F;
  const auto roll_terms =
      roll_pid.update(kLevelSetpointDegrees, attitude.roll, dt_seconds);
  const auto pitch_terms =
      pitch_pid.update(kLevelSetpointDegrees, attitude.pitch, dt_seconds);

  if ((loop_count++ % config::kTelemetryDivider) == 0U) {
    Serial.print(static_cast<float>(now_us) * 1.0e-6F, 6);
    Serial.print(','); Serial.print(imu.accX(), 6);
    Serial.print(','); Serial.print(imu.accY(), 6);
    Serial.print(','); Serial.print(imu.accZ(), 6);
    Serial.print(','); Serial.print(imu.gyrX(), 6);
    Serial.print(','); Serial.print(imu.gyrY(), 6);
    Serial.print(','); Serial.print(imu.gyrZ(), 6);
    Serial.print(','); Serial.print(attitude.roll, 4);
    Serial.print(','); Serial.print(attitude.pitch, 4);
    Serial.print(','); Serial.print(attitude.yaw, 4);
    Serial.print(','); Serial.print(roll_terms.error, 4);
    Serial.print(','); Serial.print(roll_terms.proportional, 6);
    Serial.print(','); Serial.print(roll_terms.integral, 6);
    Serial.print(','); Serial.print(roll_terms.derivative, 6);
    Serial.print(','); Serial.print(roll_terms.output, 6);
    Serial.print(','); Serial.println(pitch_terms.output, 6);
  }
}
