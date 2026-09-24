#include <cassert>
#include <cmath>
#include "imu_test_decode.hpp"
int main() {
  static_assert(imu_test::signedWord(0x7F, 0xFF) == 32767);
  static_assert(imu_test::signedWord(0x80, 0) == -32768);
  static_assert(imu_test::signedWord(0xFF, 0xFF) == -1);
  // 25 C; +1, -1, +0.5 g; +250, -250, 0 dps.
  const std::uint8_t bytes[] = {0,0, 0x20,0, 0xE0,0, 0x10,0,
                              0x40,0, 0xC0,0, 0,0};
  const auto s = imu_test::decode(bytes);
  assert(s.temperature_c == 25 && s.ax == 1 && s.ay == -1 && s.az == .5F);
  assert(s.gx == 250 && s.gy == -250 && s.gz == 0);
  const std::uint8_t cold[] = {0xFA,0xD3, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0};
  assert(std::fabs(imu_test::decode(cold).temperature_c - 15) < .01F);
}
