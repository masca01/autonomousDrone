#include <cassert>
#include <cmath>
#include <iostream>

#include "control/complementary_attitude.hpp"
#include "control/pid_controller.hpp"

namespace {

bool near(float actual, float expected, float tolerance) {
  return std::fabs(actual - expected) <= tolerance;
}

void testProportionalAndSaturation() {
  control::PidController pid({2.0F, 0.0F, 0.0F},
                             {-1.0F, 1.0F, -0.5F, 0.5F});
  const auto terms = pid.update(1.0F, 0.0F, 0.01F);
  assert(near(terms.proportional, 2.0F, 1.0e-6F));
  assert(near(terms.output, 1.0F, 1.0e-6F));
}

void testIntegralAntiWindup() {
  control::PidController pid({10.0F, 1.0F, 0.0F},
                             {-1.0F, 1.0F, -0.5F, 0.5F});
  for (int i = 0; i < 1000; ++i) {
    pid.update(1.0F, 0.0F, 0.01F);
  }
  const auto recovering = pid.update(0.0F, 0.2F, 0.01F);
  assert(recovering.integral <= 0.0F);
  assert(recovering.output < 0.0F);
}

void testFlatAttitudeInitialization() {
  control::ComplementaryAttitude estimator;
  const auto attitude = estimator.update(0.0F, 0.0F, 1.0F,
                                         0.0F, 0.0F, 0.0F, 0.002F);
  assert(near(attitude.roll, 0.0F, 1.0e-4F));
  assert(near(attitude.pitch, 0.0F, 1.0e-4F));
}

void testNinetyDegreeRollInitialization() {
  control::ComplementaryAttitude estimator;
  const auto attitude = estimator.update(0.0F, 1.0F, 0.0F,
                                         0.0F, 0.0F, 0.0F, 0.002F);
  assert(near(attitude.roll, 90.0F, 0.01F));
}

}  // namespace

int main() {
  testProportionalAndSaturation();
  testIntegralAntiWindup();
  testFlatAttitudeInitialization();
  testNinetyDegreeRollInitialization();
  std::cout << "All native control tests passed.\n";
  return 0;
}
