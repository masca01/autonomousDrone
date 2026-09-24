#pragma once

#include <cmath>

namespace control {

struct AttitudeDegrees {
  float roll{0.0F};
  float pitch{0.0F};
  float yaw{0.0F};
};

class ComplementaryAttitude {
 public:
  explicit ComplementaryAttitude(float time_constant_seconds = 0.50F)
      : time_constant_seconds_(time_constant_seconds) {}

  AttitudeDegrees update(float ax_g, float ay_g, float az_g,
                         float gx_dps, float gy_dps, float gz_dps,
                         float dt_seconds) {
    if (!(dt_seconds > 0.0F) || !std::isfinite(dt_seconds)) {
      return attitude_;
    }

    constexpr float kRadiansToDegrees = 57.2957795131F;
    const float accel_norm =
        std::sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    const bool valid_acceleration = accel_norm > 0.25F && accel_norm < 2.5F;

    float roll_acc = attitude_.roll;
    float pitch_acc = attitude_.pitch;
    if (valid_acceleration) {
      roll_acc = std::atan2(ay_g, az_g) * kRadiansToDegrees;
      pitch_acc =
          std::atan2(-ax_g, std::sqrt(ay_g * ay_g + az_g * az_g)) *
          kRadiansToDegrees;
    }

    if (!initialized_) {
      attitude_.roll = roll_acc;
      attitude_.pitch = pitch_acc;
      initialized_ = valid_acceleration;
      return attitude_;
    }

    const float gyro_roll = attitude_.roll + gx_dps * dt_seconds;
    const float gyro_pitch = attitude_.pitch + gy_dps * dt_seconds;
    attitude_.yaw += gz_dps * dt_seconds;

    if (valid_acceleration) {
      const float alpha = time_constant_seconds_ /
                          (time_constant_seconds_ + dt_seconds);
      attitude_.roll = alpha * gyro_roll + (1.0F - alpha) * roll_acc;
      attitude_.pitch = alpha * gyro_pitch + (1.0F - alpha) * pitch_acc;
    } else {
      attitude_.roll = gyro_roll;
      attitude_.pitch = gyro_pitch;
    }

    return attitude_;
  }

  void reset() {
    attitude_ = {};
    initialized_ = false;
  }

  const AttitudeDegrees& attitude() const { return attitude_; }

 private:
  float time_constant_seconds_{0.50F};
  AttitudeDegrees attitude_{};
  bool initialized_{false};
};

}  // namespace control
