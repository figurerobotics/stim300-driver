// Copyright (c) 2019 Vortex NTNU  MIT License

#pragma once

#include <boost/crc.hpp>
#include <queue>
#include <string>
#include <vector>

#include "apps/imu_app/datagram_parser.h"
#include "apps/imu_app/driver_stim300.h"
#include "apps/imu_app/serial_unix.h"
#include "apps/imu_app/stim300_constants.h"

// These values can be of use in diagnostics

// The number of bytes max for the IMU 'Normal' datagram
const int kImuNormalDatagramBytes = 65;
const int kImuMaxDatagramBytes = kImuNormalDatagramBytes;

// The the typical time between the 'Normal' datagram which contains measurements
// This value is only used to attempt to watch for long gaps between datagrams
const int kImuNormalDatagramSpaceUs = 250;

// We can read several bytes at a time to minimize serial driver overhead.
// Here we define the amount of bytes we normally read and define a max allowed
const int kImuSerialReadBytes = 8;
const int kImuMaxSerialReadBytes = 64;

// Control over debug logs specific to the driver operation
const bool kImuDbgShowCrc = false;
const bool kImuDbgShowLongBreaks = true;

namespace stim_300 {

const int kImuNumBytesPerNormalRead = 1;

enum class Stim300Status {
  NORMAL,
  NEW_MEASURMENT,
  CONFIG_CHANGED,
  STARTING_SENSOR,
  SYSTEM_INTEGRITY_ERROR,
  OUTSIDE_OPERATING_CONDITIONS,
  OVERLOAD,
  ERROR_IN_MEASUREMENT_CHANNEL,
  ERROR,
  ERROR_IN_CRC
};

class DriverStim300 {
 public:
  DriverStim300(SerialDriver &serial_driver, DatagramIdentifier datagram_id,
                stim_300::GyroOutputUnit gyro_output_unit, stim_300::AccOutputUnit acc_output_unit,
                stim_300::InclOutputUnit incl_output_unit, stim_300::AccRange acc_range,
                stim_300::SampleFreq freq);
  explicit DriverStim300(SerialDriver &serial_driver);
  ~DriverStim300() = default;
  // The class is Non-Copyable
  DriverStim300(const DriverStim300 &a) = delete;
  DriverStim300 &operator=(const DriverStim300 &a) = delete;
  // The class is non-movable
  DriverStim300(DriverStim300 &&a) = delete;
  DriverStim300 &operator=(DriverStim300 &&a) = delete;

  // Here are measurement related queries
  double getAccelX() const noexcept;
  double getAccelY() const noexcept;
  double getAccelZ() const noexcept;
  double getGyroX() const noexcept;
  double getGyroY() const noexcept;
  double getGyroZ() const noexcept;
  double getTempAccelX() const noexcept;
  double getTempAccelY() const noexcept;
  double getTempAccelZ() const noexcept;
  double getTempGyroX() const noexcept;
  double getTempGyroY() const noexcept;
  double getTempGyroZ() const noexcept;
  double getInclX() const noexcept;
  double getInclY() const noexcept;
  double getInclZ() const noexcept;

  // Here are configuration related queries
  uint16_t getSampleRate() const noexcept;
  uint8_t isAccelerationIncluded() const noexcept;
  uint8_t isGyroscopeIncluded() const noexcept;
  uint8_t isInclinationIncluded() const noexcept;
  uint8_t isTemperatureIncluded() const noexcept;
  uint8_t isLineTerminationOn() const noexcept;

  uint16_t getLatency_us() const noexcept;
  double getAverageTemp() const noexcept;
  std::string printSensorConfig() const noexcept;
  bool isSensorStatusGood() const noexcept;
  uint8_t getInternalMeasurementCounter() const noexcept;
  Stim300Status update() noexcept;

  // Service mode for setup
  int32_t enterServiceMode();
  void exitServiceMode();
  int32_t setSampleRate(uint16_t samplesPerSec);
  std::string readReplyUntilChar(char c, int32_t maxCharToWait);
  std::string getInformation(char info_type, int32_t max_reply_length);
  int32_t setDatagramFormat(uint16_t mode);
  int32_t setAccelerometerFilter(uint16_t frequency);
  int32_t setGyroscopeFilter(uint16_t frequency);
  int32_t setLineTermination(uint16_t mode);
  int32_t saveConfigData();

 private:
  enum class Mode : uint8_t { Init, Normal, Service };
  Mode mode_{Mode::Init};
  enum class ReadingMode {
    IdentifyingDatagram,
    ReadingDatagram,
    VerifyingDatagramCR,
    VerifyingDatagramLF
  };
  ReadingMode reading_mode_{ReadingMode::IdentifyingDatagram};

  SerialDriver &serial_driver_;
  stim_300::DatagramParser datagram_parser_;
  std::vector<uint8_t> datagramBuffer_{};
  size_t n_new_bytes_{0};
  size_t n_checked_bytes{0};
  std::queue<uint8_t> bytesBuffered_{};
  bool readAnotherByte(uint8_t &byte, int readSize);

  uint8_t datagram_id_;
  uint8_t crc_dummy_bytes_;
  uint8_t datagram_size_;
  stim_300::SensorConfig sensor_config_;
  bool read_config_from_sensor_{true};
  bool datagramNotRecognizedReported_{false};
  stim_300::SensorData sensor_data_{};
  uint8_t sensor_status_{0};

  Stim300Status readDataStream();
  bool setDatagramFormat(stim_300::DatagramIdentifier id);
  static bool verifyChecksum(const std::vector<uint8_t>::const_iterator &begin,
                             const std::vector<uint8_t>::const_iterator &end,
                             const uint8_t &crc_dummy_bytes);

  void sendString(std::string data);
  void sendStringWithCR(std::string data);
  int32_t readUntilChar(char c, int32_t maxCharToWait);
  void askForConfigDatagram();
};
}  // namespace stim_300
