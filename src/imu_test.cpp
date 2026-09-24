#include <Arduino.h>
#include <SPI.h>
#include <cmath>
#include "config.hpp"
#include "imu_test_decode.hpp"

namespace {
// Register definitions and scaling: ICM-42688-P datasheet, bank 0.
constexpr uint8_t kBank = 0x76, kIdentity = 0x75, kExpectedId = 0x47;
constexpr uint8_t kReset = 0x11, kPower = 0x4E;
constexpr uint8_t kGyroConfig = 0x4F, kAccelConfig = 0x50;
constexpr uint8_t kStatus = 0x2D, kData = 0x1D;
uint32_t last_sample_ms = 0, last_print_ms = 0, last_check_ms = 0;
uint32_t sample_count = 0;

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
  Serial.println("Acceleration in g; gyro in deg/s WITHOUT bias correction; temperature in C.");
  Serial.println("Leave still, then gently tilt/rotate. Gyro measures turning speed, not angle.");
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
  if (now - last_print_ms >= 200) {
    last_print_ms = now;
    const float norm = std::sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
    Serial.printf("n=%lu | accel[g] X=%+.3f Y=%+.3f Z=%+.3f | |a|=%.3f | gyro[deg/s] X=%+.2f Y=%+.2f Z=%+.2f | temp=%.1f C\n",
                  static_cast<unsigned long>(sample_count),
                  s.ax, s.ay, s.az, norm, s.gx, s.gy, s.gz, s.temperature_c);
  }
}
