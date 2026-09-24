#pragma once

#include <algorithm>
#include <cmath>

namespace control {

struct PidGains {
  float kp{0.0F};
  float ki{0.0F};
  float kd{0.0F};
};

struct PidLimits {
  float output_min{-1.0F};
  float output_max{1.0F};
  float integral_min{-0.25F};
  float integral_max{0.25F};
};

struct PidTerms {
  float error{0.0F};
  float proportional{0.0F};
  float integral{0.0F};
  float derivative{0.0F};
  float output{0.0F};
};

class PidController {
 public:
  PidController(PidGains gains, PidLimits limits,
                float derivative_cutoff_hz = 30.0F)
      : gains_(gains),
        limits_(limits),
        derivative_cutoff_hz_(derivative_cutoff_hz) {}

  PidTerms update(float setpoint, float measurement, float dt_seconds) {
    if (!(dt_seconds > 0.0F) || !std::isfinite(dt_seconds)) {
      return last_terms_;
    }

    const float error = setpoint - measurement;

    float raw_derivative = 0.0F;
    if (initialized_) {
      // Differentiate the measurement to avoid a derivative kick when the
      // commanded angle changes suddenly.
      raw_derivative = -(measurement - previous_measurement_) / dt_seconds;
    }

    if (derivative_cutoff_hz_ > 0.0F) {
      constexpr float kTwoPi = 6.28318530718F;
      const float rc = 1.0F / (kTwoPi * derivative_cutoff_hz_);
      const float alpha = dt_seconds / (rc + dt_seconds);
      filtered_derivative_ += alpha * (raw_derivative - filtered_derivative_);
    } else {
      filtered_derivative_ = raw_derivative;
    }

    const float p_term = gains_.kp * error;
    const float d_term = gains_.kd * filtered_derivative_;
    const float candidate_integral =
        std::clamp(integral_term_ + gains_.ki * error * dt_seconds,
                   limits_.integral_min, limits_.integral_max);

    const float candidate_unsaturated = p_term + candidate_integral + d_term;
    const bool saturating_high = candidate_unsaturated > limits_.output_max;
    const bool saturating_low = candidate_unsaturated < limits_.output_min;

    // Conditional integration prevents windup while still allowing the
    // integral term to move the controller out of saturation.
    if ((!saturating_high && !saturating_low) ||
        (saturating_high && error < 0.0F) ||
        (saturating_low && error > 0.0F)) {
      integral_term_ = candidate_integral;
    }

    const float output = std::clamp(p_term + integral_term_ + d_term,
                                    limits_.output_min, limits_.output_max);

    previous_measurement_ = measurement;
    initialized_ = true;
    last_terms_ = {error, p_term, integral_term_, d_term, output};
    return last_terms_;
  }

  void reset(float measurement = 0.0F) {
    integral_term_ = 0.0F;
    filtered_derivative_ = 0.0F;
    previous_measurement_ = measurement;
    initialized_ = false;
    last_terms_ = {};
  }

  void setGains(PidGains gains) { gains_ = gains; }
  const PidGains& gains() const { return gains_; }

 private:
  PidGains gains_;
  PidLimits limits_;
  float derivative_cutoff_hz_{30.0F};
  float integral_term_{0.0F};
  float filtered_derivative_{0.0F};
  float previous_measurement_{0.0F};
  bool initialized_{false};
  PidTerms last_terms_{};
};

}  // namespace control
