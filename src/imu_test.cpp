#include <Arduino.h>
#include <SPI.h>
#include <cmath>
#include "config.hpp"
#include "imu_test_decode.hpp"
#ifdef PID_DEMO
#include "control/pid_controller.hpp"
#endif
#ifdef ATTITUDE_TEST
#include "control/gyro_calibration.hpp"
#include "control/complementary_attitude.hpp"
#endif

namespace {
// Register definitions and scaling: ICM-42688-P datasheet, bank 0.
constexpr uint8_t kBank = 0x76, kIdentity = 0x75, kExpectedId = 0x47;
constexpr uint8_t kReset = 0x11, kPower = 0x4E;
constexpr uint8_t kGyroConfig = 0x4F, kAccelConfig = 0x50;
constexpr uint8_t kStatus = 0x2D, kData = 0x1D;
uint32_t last_sample_ms = 0, last_print_ms = 0, last_check_ms = 0;
uint32_t sample_count = 0;
#ifdef PID_DEMO
// Educational, dimensionless corrections: NOT motor commands or tuned flight gains.
constexpr control::PidGains kDemoGains{0.02F, 0.0F, 0.003F};
constexpr control::PidLimits kDemoLimits{-1.0F, 1.0F, -0.2F, 0.2F};
constexpr float kTargetDegrees = 0.0F;
control::PidController roll_pid(kDemoGains, kDemoLimits, 10.0F);
control::PidController pitch_pid(kDemoGains, kDemoLimits, 10.0F);
void printPid(const char* axis, float angle, float rate, const control::PidTerms& t) {
  Serial.printf("%s angle=%+.1f deg rate=%+.1f deg/s error=%+.1f P=%+.3f I=%+.3f D=%+.3f correction=%+.3f\n",
                axis, angle, rate, t.error, t.proportional, t.integral, t.derivative, t.output);
}
#endif
#ifdef ATTITUDE_TEST
control::GyroCalibration calibration;
control::ComplementaryAttitude estimator;
uint32_t warmup_start_ms = 0, previous_sample_us = 0;
#endif

void readRegisters(uint8_t address, uint8_t* bytes, size_t count) {
  // A modest clock is more forgiving of short jumper wires than 8 MHz.
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(config::kImuCsPin, LOW);
  SPI.transfer(address | 0x80);
  for (size_t i = 0; i < count; ++i) bytes[i] = SPI.transfer(0);
  digitalWrite(config::kImuCsPin, HIGH);
  SPI.endTransaction();
}
uint8_t readRegister(uint8_t address) {
  uint8_t result;
  readRegisters(address, &result, 1);
  return result;
}
void writeRegister(uint8_t address, uint8_t value) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(config::kImuCsPin, LOW);
  SPI.transfer(address & 0x7F);
  SPI.transfer(value);
  digitalWrite(config::kImuCsPin, HIGH);
  SPI.endTransaction();
  delay(1);  // Includes the settling interval after enabling sensor power.
}
[[noreturn]] void stop(const char* reason) {
  // Repeat so an error is still visible if the monitor opens after boot.
  for (;;) {
    Serial.print("IMU TEST STOPPED: ");
    Serial.println(reason);
    Serial.println("Unplug USB before changing wires. Check 3V3, GND, CS, SCK, MOSI, MISO; then reset.");
    delay(2000);
  }
}
void checkIdentity() {
  const auto id = readRegister(kIdentity);
  if (id != kExpectedId) {
    Serial.printf("WHO_AM_I=0x%02X; expected 0x47\n", id);
    stop("ICM42688P identity mismatch / SPI connection lost.");
  }
}
void checkConfiguration() {
  // FS=2 and ODR=8: +/-4 g, +/-500 dps, both 100 Hz.
  if (readRegister(kAccelConfig) != 0x48 ||
      readRegister(kGyroConfig) != 0x48 || readRegister(kPower) != 0x0F) {
    stop("Sensor configuration did not read back correctly.");
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nICM42688P SENSOR TEST - no motor outputs");
  Serial.println("SPI: CS=10 MOSI=11 SCK=12 MISO=13; power=3.3V");
  pinMode(config::kImuCsPin, OUTPUT);
  digitalWrite(config::kImuCsPin, HIGH);
  SPI.begin(config::kImuSckPin, config::kImuMisoPin,
            config::kImuMosiPin, config::kImuCsPin);
  delay(20);
  writeRegister(kBank, 0);
  checkIdentity();
  writeRegister(kReset, 0x01);
  delay(10);
  checkIdentity();
  writeRegister(kGyroConfig, 0x48);
  writeRegister(kAccelConfig, 0x48);
  writeRegister(kPower, 0x0F);  // Accelerometer and gyro in low-noise mode.
  delay(50);
  checkConfiguration();
  Serial.println("DETECTED: ICM42688P (0x47). Configuration verified.");
#ifndef ATTITUDE_TEST
  Serial.println("Acceleration in g; gyro in deg/s WITHOUT bias correction; temperature in C.");
  Serial.println("Leave still, then gently tilt/rotate. Gyro measures turning speed, not angle.");
#endif
#ifdef ATTITUDE_TEST
  Serial.println("ATTITUDE TEST: place the IMU flat, Z up, and leave it untouched.");
  Serial.println("2-second warmup, then 500 still samples (~5 seconds). Motion restarts calibration.");
  Serial.println("Small-tilt bench demonstration only; yaw is relative and will drift.");
#ifdef PID_DEMO
  Serial.println("PID DEMO - serial output ONLY; no motors. Targets: roll=0, pitch=0 deg.");
  Serial.println("Demo gains: Kp=0.020 Ki=0 Kd=0.003. Integral disabled (PD). Corrections limited to [-1,+1].");
#endif
  warmup_start_ms = millis();
#endif
  last_sample_ms = last_print_ms = last_check_ms = millis();
}

void loop() {
  const uint32_t now = millis();
  if (now - last_check_ms >= 1000) {
    checkIdentity();
    checkConfiguration();
    last_check_ms = now;
  }
  if ((readRegister(kStatus) & 0x08) == 0) {
    if (now - last_sample_ms > 1000) stop("No fresh sensor samples for one second.");
    delay(1);
    return;
  }
  uint8_t bytes[14];
  readRegisters(kData, bytes, sizeof(bytes));
  // 0x8000 denotes invalid accel/gyro data; do not display it as a measurement.
  for (size_t i = 2; i < sizeof(bytes); i += 2) {
    if (imu_test::signedWord(bytes[i], bytes[i + 1]) == -32768)
      stop("Sensor returned invalid accel/gyro data.");
  }
  const auto s = imu_test::decode(bytes);
  last_sample_ms = now;
  ++sample_count;
#ifdef ATTITUDE_TEST
  if (now - warmup_start_ms < 2000) return;
  if (!calibration.ready()) {
    if (calibration.add(s)) {
      Serial.printf("CALIBRATED gyro bias [deg/s]: X=%+.3f Y=%+.3f Z=%+.3f\n",
                    calibration.bias(0), calibration.bias(1), calibration.bias(2));
      Serial.println("READY: gently tilt about X or Y, then hold still. Angles should remain tilted.");
      previous_sample_us = micros();
    } else if (now - last_print_ms >= 1000) {
      last_print_ms = now;
      Serial.printf("Keep STILL: calibration %u/500 samples\n", calibration.count());
    }
    return;
  }
  const uint32_t sample_us = micros();
  const float dt = static_cast<uint32_t>(sample_us - previous_sample_us) * 1e-6F;
  previous_sample_us = sample_us;
  if (dt > 0.1F) stop("Sampling gap >100 ms. Reset to recalibrate.");
  const float gx = s.gx-calibration.bias(0);
  const float gy = s.gy-calibration.bias(1);
  const float gz = s.gz-calibration.bias(2);
  const auto angles = estimator.update(s.ax,s.ay,s.az,gx,gy,gz,dt);
#ifdef PID_DEMO
  const auto roll_terms = roll_pid.update(kTargetDegrees, angles.roll, dt, gx);
  const auto pitch_terms = pitch_pid.update(kTargetDegrees, angles.pitch, dt, gy);
#endif
  if (now - last_print_ms >= 200) {
    last_print_ms = now;
#ifdef PID_DEMO
    printPid("ROLL ", angles.roll, gx, roll_terms);
    printPid("PITCH", angles.pitch, gy, pitch_terms);
#else
    Serial.printf("roll=%+.1f deg | pitch=%+.1f deg | yaw_relative=%+.1f deg | gyro_corrected[deg/s] X=%+.2f Y=%+.2f Z=%+.2f\n",
                  angles.roll,angles.pitch,angles.yaw,gx,gy,gz);
#endif
  }
#else
  if (now - last_print_ms >= 200) {
    last_print_ms = now;
    const float norm = std::sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
    Serial.printf("n=%lu | accel[g] X=%+.3f Y=%+.3f Z=%+.3f | |a|=%.3f | gyro[deg/s] X=%+.2f Y=%+.2f Z=%+.2f | temp=%.1f C\n",
                  static_cast<unsigned long>(sample_count),
                  s.ax, s.ay, s.az, norm, s.gx, s.gy, s.gz, s.temperature_c);
  }
#endif
}
