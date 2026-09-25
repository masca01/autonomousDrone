#pragma once
#include <cmath>
#include "imu_test_decode.hpp"

namespace control {
// Five seconds at 100 Hz. Motion checks are a guard, not proof of stillness.
class GyroCalibration {
 public:
  bool add(const imu_test::Sample& s) {
    if (ready()) return true;
    const float v[6] = {s.gx, s.gy, s.gz, s.ax, s.ay, s.az};
    const float norm = std::sqrt(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
    for (float x : v) if (!std::isfinite(x)) { reset(); return false; }
    if (norm < 0.9F || norm > 1.1F || std::fabs(s.gx)>3 ||
        std::fabs(s.gy)>3 || std::fabs(s.gz)>3) {
      reset(); return false;
    }
    ++count_;
    for (int i=0; i<6; ++i) {
      const float delta = v[i]-mean_[i];
      mean_[i] += delta/count_;
      m2_[i] += delta*(v[i]-mean_[i]);
    }
    if (count_ == 500) {
      for (int i=0; i<6; ++i) {
        const float limit = i<3 ? 0.25F : 0.0004F;
        if (m2_[i]/499 > limit) { reset(); return false; }
      }
    }
    return ready();
  }
  bool ready() const { return count_ == 500; }
  unsigned count() const { return count_; }
  float bias(int axis) const { return mean_[axis]; }
  void reset() {
    count_=0;
    for (int i=0; i<6; ++i) mean_[i]=m2_[i]=0;
  }
 private:
  unsigned count_=0;
  float mean_[6]{}, m2_[6]{};
};
} // namespace control
