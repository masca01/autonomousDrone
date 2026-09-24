#pragma once
#include <cstdint>

namespace imu_test {
// ICM42688 bank-0 data order: temperature, accel XYZ, gyro XYZ, big endian.
constexpr int signedWord(std::uint8_t hi, std::uint8_t lo) {
  const int word = (static_cast<int>(hi) << 8) | lo;
  return word >= 32768 ? word - 65536 : word;
}
struct Sample {
  float temperature_c;
  float ax, ay, az;  // g, configured for +/-4 g
  float gx, gy, gz;  // degrees/second, configured for +/-500 dps
};
inline Sample decode(const std::uint8_t* b) {
  return {signedWord(b[0], b[1]) / 132.48F + 25.0F,
          signedWord(b[2], b[3]) / 8192.0F,
          signedWord(b[4], b[5]) / 8192.0F,
          signedWord(b[6], b[7]) / 8192.0F,
          signedWord(b[8], b[9]) * (500.0F / 32768.0F),
          signedWord(b[10], b[11]) * (500.0F / 32768.0F),
          signedWord(b[12], b[13]) * (500.0F / 32768.0F)};
}
}  // namespace imu_test
