// Copyright (c) 2019 Vortex NTNU  MIT License

#include "apps/imu_app/driver_stim300.h"

#include <spdlog/spdlog.h>  // framework specific debug logger

#include <chrono>
#include <ctime>
#include <iostream>
#include <ratio>
#include <string>

#include "apps/imu_app/datagram_parser.h"
#include "apps/imu_app/stim300_constants.h"

namespace stim_300 {

DriverStim300::DriverStim300(SerialDriver &serial_driver, DatagramIdentifier datagram_id,
                             GyroOutputUnit gyro_output_unit, AccOutputUnit acc_output_unit,
                             InclOutputUnit incl_output_unit, AccRange acc_range, SampleFreq freq)
    : serial_driver_(serial_driver),
      datagram_parser_(datagram_id, gyro_output_unit, acc_output_unit, incl_output_unit, acc_range),
      datagram_id_(datagramIdentifierToRaw(datagram_id)),
      crc_dummy_bytes_(numberOfPaddingBytes(datagram_id)),
      datagram_size_(calculateDatagramSize(datagram_id)),
      sensor_config_{'0',
                     0,
                     0,
                     0,
                     0,
                     0,
                     freq,
                     datagram_id,
                     false,
                     gyro_output_unit,
                     acc_output_unit,
                     incl_output_unit,
                     acc_range} {}

DriverStim300::DriverStim300(SerialDriver &serial_driver)
    : DriverStim300(serial_driver, DatagramIdentifier::RATE_ACC_INCL,
                    GyroOutputUnit::AVERAGE_ANGULAR_RATE, AccOutputUnit::AVERAGE_ACCELERATION,
                    InclOutputUnit::AVERAGE_ACCELERATION, AccRange::G5, SampleFreq::S125) {}
double DriverStim300::getAccelX() const noexcept { return sensor_data_.acc[0]; }
double DriverStim300::getAccelY() const noexcept { return sensor_data_.acc[1]; }
double DriverStim300::getAccelZ() const noexcept { return sensor_data_.acc[2]; }
double DriverStim300::getGyroX() const noexcept { return sensor_data_.gyro[0]; }
double DriverStim300::getGyroY() const noexcept { return sensor_data_.gyro[1]; }
double DriverStim300::getGyroZ() const noexcept { return sensor_data_.gyro[2]; }
double DriverStim300::getTempGyroX() const noexcept { return sensor_data_.temp_gyro[0]; }
double DriverStim300::getTempGyroY() const noexcept { return sensor_data_.temp_gyro[1]; }
double DriverStim300::getTempGyroZ() const noexcept { return sensor_data_.temp_gyro[2]; }
double DriverStim300::getTempAccelX() const noexcept { return sensor_data_.temp_acc[0]; }
double DriverStim300::getTempAccelY() const noexcept { return sensor_data_.temp_acc[1]; }
double DriverStim300::getTempAccelZ() const noexcept { return sensor_data_.temp_acc[2]; }
double DriverStim300::getInclX() const noexcept { return sensor_data_.incl[0]; }
double DriverStim300::getInclY() const noexcept { return sensor_data_.incl[1]; }
double DriverStim300::getInclZ() const noexcept { return sensor_data_.incl[2]; }

uint16_t DriverStim300::getSampleRate() const noexcept {
  return stim_300::sampleFreq2int(sensor_config_.sample_freq);
}

// The included routines return 0 for no or non-zero for yes if type is in datagram
uint8_t DriverStim300::isAccelerationIncluded() const noexcept {
  return (sensor_config_.sys_config_1 & 0x02);
}
uint8_t DriverStim300::isGyroscopeIncluded() const noexcept {
  return 1;  // Gyroscope is always included
}
uint8_t DriverStim300::isInclinationIncluded() const noexcept {
  return (sensor_config_.sys_config_1 & 0x04);
}
uint8_t DriverStim300::isTemperatureIncluded() const noexcept {
  return (sensor_config_.sys_config_1 & 0x08);
}
uint8_t DriverStim300::isLineTerminationOn() const noexcept {
  return (sensor_config_.sys_config_2 & 0x01);
}

uint16_t DriverStim300::getLatency_us() const noexcept { return sensor_data_.latency_us; }
bool DriverStim300::isSensorStatusGood() const noexcept { return sensor_status_ == 0; }
uint8_t DriverStim300::getInternalMeasurementCounter() const noexcept {
  return sensor_data_.counter;
}

double DriverStim300::getAverageTemp() const noexcept {
  double sum{0};
  uint8_t count{0};

  return count != 0 ? sum / count : std::numeric_limits<double>::quiet_NaN();
}

// readAnotherByte  - Support multi-byte reads yet offer single byte api
//
// Read characters are saved to the bytesBuffered_ queue
//
// Returns true and one byte if more data is available
// Returns false if no data could be returned
//
bool DriverStim300::readAnotherByte(uint8_t &byte, int readSize) {
  bool retCode = true;
  uint8_t bytes[kImuMaxSerialReadBytes];
  int bytesRead = 0;

  if (!bytesBuffered_.empty()) {
    // use a byte already available
    byte = bytesBuffered_.front();
    bytesBuffered_.pop();
  } else {
    // Attempt a read from the device
    if (readSize > kImuMaxSerialReadBytes) {
      spdlog::error("ERROR: readAnotherByte cannot read {} bytes", readSize);
    } else {
      bytesRead = serial_driver_.readBytes(&bytes[0], readSize);
      if ((readSize >= 32) && (bytesRead > 1) && (bytesRead < 30)) {
        spdlog::debug("read {} got {}", readSize, bytesRead);
      }
      if (bytesRead <= 0) {
        retCode = false;  // no data on read
      } else {
        // Always we need first one
        byte = bytes[0];
        uint8_t *bytePtr = &bytes[1];
        while (bytesRead > 1) {
          bytesBuffered_.push(*bytePtr++);  // save the rest for next call
          bytesRead -= 1;
        }
        retCode = true;  // no really required but makes code clearer
      }
    }
  }
  return retCode;
}

Stim300Status DriverStim300::readDataStream() {
  // Read stream to identify the start of a datagram.
  // If no datagram is identified after 100 bytes, ask for a config datagram.
  // Read number of bytes belonging to the current datagram.
  // Verify datagram (CRLF and CRC).
  // Parse Datagram.
  uint8_t byte;

  // For detection of long delays.
  int bytesPerRead = 1;
  if (reading_mode_ == ReadingMode::ReadingDatagram) {
    bytesPerRead = kImuSerialReadBytes;
  }

  // Read at least 1 byte and continue reading till no data is read
  while (readAnotherByte(byte, bytesPerRead)) {
    // Driver state is in reading_mode_ and transitions as datagrams arrive.
    // The reader has some recovery to try to get back in sync with datagrams
    switch (reading_mode_) {
      case ReadingMode::IdentifyingDatagram:
        if (byte == datagram_id_) {
          // Use current datagram format
        } else if (byte == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION)) {
          setDatagramFormat(DatagramIdentifier::CONFIGURATION);
          datagramNotRecognizedReported_ = false;
        } else if (byte == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION_CRLF)) {
          setDatagramFormat(DatagramIdentifier::CONFIGURATION_CRLF);
          datagramNotRecognizedReported_ = false;
        } else if (byte == datagramIdentifierToRaw(DatagramIdentifier::RATE_ACC_INCL)) {
          datagramNotRecognizedReported_ = false;
          spdlog::debug("Reset dgId to NORMAL");
          setDatagramFormat(DatagramIdentifier::RATE_ACC_INCL);
        } else {
          if (++n_checked_bytes > kImuMaxDatagramBytes) {
            if (datagramNotRecognizedReported_ == false) {
              // Thin multiple errors per pass
              spdlog::warn("Not able to recognize datagram");
              datagramNotRecognizedReported_ = true;
            }
            if (read_config_from_sensor_) askForConfigDatagram();
            n_checked_bytes = 0;
          }
          continue;  // Go right to reading in next byte
        }
        n_checked_bytes = 0;
        reading_mode_ = ReadingMode::ReadingDatagram;
        datagramBuffer_.push_back(byte);
        n_new_bytes_ = 1;
        continue;  // Go right to reading in next byte

      case ReadingMode::ReadingDatagram:
        // Circular buffer
        datagramBuffer_.push_back(byte);
        n_new_bytes_++;
        while (datagramBuffer_.size() > datagram_size_) {
          datagramBuffer_.erase(datagramBuffer_.begin());
        }

        if (n_new_bytes_ < datagram_size_) {
          continue;  // Keep reading datagram with another char read
        }

        // else the buffer is filled with a potential new datagram
        if (sensor_config_.normal_datagram_CRLF ||
            datagram_id_ == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION_CRLF)) {
          reading_mode_ = ReadingMode::VerifyingDatagramCR;
          continue;  // Go right to reading in next byte
        }
        break;

      case ReadingMode::VerifyingDatagramCR:
        if (byte == 0x0D) {
          reading_mode_ = ReadingMode::VerifyingDatagramLF;
          continue;  // Go right to reading in next byte
        }
        // Did not see the CR so reset back to Identifying
        reading_mode_ = ReadingMode::IdentifyingDatagram;
        return Stim300Status::NORMAL;

      case ReadingMode::VerifyingDatagramLF:
        if (byte == 0x0A) {
          break;
        }
        // Did not see the LF so reset back to Identifying
        reading_mode_ = ReadingMode::IdentifyingDatagram;
        return Stim300Status::NORMAL;
    }  // end reading_mode_ switch

    // Calculate datagram crc if enough bytes for this datagram
    // Adding detecetion of CRC faults only at packet end  20230105
    if ((datagramBuffer_.size() >= datagram_size_) &&
        (!verifyChecksum(datagramBuffer_.cbegin(), datagramBuffer_.cend(), crc_dummy_bytes_))) {
      // The "ID" was likely a byte happening to be equal the datagram id,
      // and not actually the start of a datagram, thus the buffer does
      // not contain a complete datagram.
      // std::cerr << "CRC error" << std::endl;
      reading_mode_ = ReadingMode::IdentifyingDatagram;
      read_config_from_sensor_ = true;  // Added Jan2023 to allow reset to state machine
      spdlog::debug("ERROR: CRC on dgId 0x{:x}", datagram_id_);
      return Stim300Status::ERROR_IN_CRC;
    }

    if (datagram_id_ == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION_CRLF) ||
        datagram_id_ == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION)) {
      stim_300::SensorConfig sensor_config = datagram_parser_.parseConfig(datagramBuffer_.cbegin());
      Stim300Status status{Stim300Status::NORMAL};
      if (sensor_config != sensor_config_) status = Stim300Status::CONFIG_CHANGED;

      sensor_config_ = sensor_config;
      setDatagramFormat(sensor_config_.datagram_id);
      datagram_parser_.setDataParameters(sensor_config_);
      read_config_from_sensor_ = false;
      // Show a condensed sensor configuration report
      spdlog::debug("IMU: Rev {} FwRev {} AGIT{}{}{}{} Cfg: 0x{:x} 0x{:x} aFlt 0x{:x} gFlt 0x{:x}",
                    sensor_config.revision, sensor_config.firmware_version,
                    (isAccelerationIncluded() ? 1 : 0), (isGyroscopeIncluded() ? 1 : 0),
                    (isInclinationIncluded() ? 1 : 0), (isTemperatureIncluded() ? 1 : 0),
                    sensor_config.sys_config_1, sensor_config.sys_config_2,
                    sensor_config.accel_filters, sensor_config.gyro_filters);
      std::string detailedConfigMsg = printSensorConfig();
      spdlog::debug("IMU Detailed Config: {} ", detailedConfigMsg);

      // Go into waiting for normal reading reports
      reading_mode_ = ReadingMode::IdentifyingDatagram;
      return status;
    } else {
      // Normal (measurement) datagram
      // Given raw data from STIM300 parse out what we need into sensor_data_
      // The full Normal Mode datagram is periodic and details in STIM300 spec section 5.5.6
      //
      sensor_status_ = datagram_parser_.parseData(datagramBuffer_.cbegin(), sensor_data_);

      reading_mode_ = ReadingMode::IdentifyingDatagram;

      if (sensor_status_ == 0)
        return Stim300Status::NEW_MEASURMENT;
      else if (sensor_status_ & (1u << 6u))
        return Stim300Status::STARTING_SENSOR;
      else if (sensor_status_ & (1u << 7u))
        return Stim300Status::SYSTEM_INTEGRITY_ERROR;
      else if (sensor_status_ & (1u << 5u))
        return Stim300Status::OUTSIDE_OPERATING_CONDITIONS;
      else if (sensor_status_ & (1u << 4u))
        return Stim300Status::OVERLOAD;
      else if (sensor_status_ & (1u << 3u))
        return Stim300Status::ERROR_IN_MEASUREMENT_CHANNEL;
      else
        return Stim300Status::ERROR;
    }
  }  // End do for full read
  std::this_thread::sleep_for(std::chrono::microseconds(100));
  return Stim300Status::NORMAL;
}

// askForConfigDatagram - Requests the senor in NORMAL mode for config info
//
// The state machine that reads bytes decods the datagram returned
void DriverStim300::askForConfigDatagram() {
  spdlog::debug("Stim300: Request config");
  serial_driver_.writeByte('C');
  serial_driver_.writeByte('\r');
}

// Send a string without any added termination
void DriverStim300::sendString(std::string data) {
  for (std::string::size_type i = 0; i < data.size(); ++i) {
    serial_driver_.writeByte(data[i]);
  }
}

// Send a string followed by a Carrage return
void DriverStim300::sendStringWithCR(std::string data) {
  sendString(data);
  serial_driver_.writeByte('\r');
}

// { ----------------------- Routines for SERVICEMODE -------------------------------

// @brief read until special character comes in
//
// @return number of chars read till we got the desired char
//
// This would be better if we had a way to timeout
// besides a max char count to wait
int32_t DriverStim300::readUntilChar(char c, int32_t maxCharToWait) {
  uint8_t byte;
  int32_t charsRead = 0;
  while (serial_driver_.readByte(byte) && (charsRead < maxCharToWait)) {
    charsRead++;
    if (byte == c) {
      break;
    }
  }
  return charsRead;
}

// @brief read string until a special character comes in
//
// @return string that was read in full
//
std::string DriverStim300::readReplyUntilChar(char c, int32_t maxCharToWait) {
  uint8_t byte;
  std::string reply = "";
  int32_t charsRead = 0;
  while (serial_driver_.readByte(byte) && (charsRead < maxCharToWait)) {
    charsRead++;
    if (byte == c) {
      break;
    }
    reply.push_back(byte);
  }
  return reply;
}

// enterServiceMode:  Enters SERVICEMODE
//
// Enters the rather crude service mode that is not computer friendly
// The Stim318 does not have the clean 'Utility Mode' so this is 'crude'
//
// NOTE: The sensor must be up and responding before calling this routine
// Returns 0 if no error and we are in service mode
//
int32_t DriverStim300::enterServiceMode() {
  int32_t ret_code = 0;

  spdlog::debug("Stim300: Enter SERVICE Mode");
  sendStringWithCR("SERVICEMODE");

  // After sending  SERVICEMODE<CR> output stops after next datagram
  // wait for any in-progress datagram then flush serial input
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  serial_driver_.flush();

  // Ask for product name to test we are properly connected
  std::string reply = getInformation('n', 50);
  std::size_t found = reply.find("PRODUCT");
  if (found != std::string::npos) {
    spdlog::debug("Stim300: enterServiceMode IMU name {} chars", reply);
    spdlog::debug("Stim300: SERVICE Mode entered.");
  } else {
    spdlog::error("Stim300: SERVICE Mode NOT entered properly!");
    ret_code = -1;
  }

  return ret_code;
}

// exitServiceMode:  Exits SERVICEMODE
//
void DriverStim300::exitServiceMode() {
  spdlog::debug("Stim300: Return to NORMAL mode");
  sendStringWithCR("x N");  // Exits to normal mode right away, no confirm
}

// saveConfigData:  Saves configuration data to flash.
//
// Returns 0 if no error, non-zero for some type of error
//
// WARNING! Must not power off the IMU till the save is done
//
int32_t DriverStim300::saveConfigData() {
  int32_t ret_code = 0;
  int32_t returned_chars = 0;

  spdlog::debug("Stim300: Save configuration to flash memory");
  // Exits to normal mode right away, no confirm
  sendStringWithCR("s");

  // We sadly must wait and enter 'Y' to proceed as this is human interface
  returned_chars = readUntilChar(':', 100);  // text per datasheet
  if ((returned_chars < 30) || (returned_chars >= 100)) {
    spdlog::error("Stim300: saveConfigData invalid wait for confirm query!");
    return -2;
  }
  serial_driver_.writeByte('Y');  // Enter a 'Y' to proceed

  // This may take many milliseconds to write to flash
  // There is a chance it failed and we are not detecting failures robustly
  // We look for a reply length that is longer then the fault return lengths
  returned_chars = readUntilChar('>', 200);  // text per datasheet
  if ((returned_chars < 60) || (returned_chars >= 120)) {
    spdlog::error("Stim300: saveConfigData invalid reply for successful save!");
    return -3;
  }
  spdlog::debug("Stim300: Configuration saved to flash memory");

  return ret_code;
}

// Gets a reply string to an information query
//
// The single char input parameter matches Stim318 table 9.4 for type of data
//
// Sensor must be in SERVICEMODE prior to this call
//
std::string DriverStim300::getInformation(char info_type, int32_t max_reply_length) {
  std::string reply = "";

  serial_driver_.flush();

  // Ask for datagram format as a test we are properly connected
  // The sensor expects  "i T<CR>"  where T is the type
  sendString("i ");
  serial_driver_.writeByte(info_type);
  serial_driver_.writeByte('\r');
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  reply = readReplyUntilChar('>', max_reply_length);

  return reply;
}

// setDatagramFormat:  Set datagram format to included selected data values
//
// Returns 0 if no error, non-zero for some type of error
int32_t DriverStim300::setDatagramFormat(uint16_t datagramFormat) {
  int32_t ret_code = 0;
  int32_t returned_chars = 0;

  spdlog::debug("Stim300: setDatagramFormat setting format {}", datagramFormat);
  serial_driver_.flush();
  if (datagramFormat > 7) {
    spdlog::error("Stim300: setDatagramFormat parameter error!");
    return -1;
  }

  spdlog::debug("Stim300: Set datagram format to type {}", datagramFormat);

  // Use datagram given and termination <CR><LF>
  serial_driver_.writeByte('d');
  serial_driver_.writeByte(' ');
  serial_driver_.writeByte(datagramFormat);
  serial_driver_.writeByte(',');
  serial_driver_.writeByte('1');
  serial_driver_.writeByte('\r');

  // text per datasheet
  returned_chars = readUntilChar('>', 100);
  spdlog::debug("Stim300: Set datagram format reply was {} chars", returned_chars);
  if ((returned_chars < 15) || (returned_chars >= 100)) {
    spdlog::error("Stim300: setDatagramFormat unexpected reply error!");
    ret_code = -2;
  }
  return ret_code;
}

// setSampleRate: Sets sample rate to next lowest rate but minimum is 125
//
int32_t DriverStim300::setSampleRate(uint16_t samplesPerSec) {
  int32_t ret_code = 0;
  int32_t returned_chars = 0;

  spdlog::debug("Stim300: Set samples per sec to {}", samplesPerSec);
  if (samplesPerSec > 1000) {
    spdlog::error("Stim300: setSampleRate out of range sample rate {}!", samplesPerSec);
    return -1;
  }
  serial_driver_.flush();

  // Excuse the turse format as it is most effective for this sort of code
  char sampleRateSelect = '0';
  if (samplesPerSec >= 2000) sampleRateSelect = '4';
  if (samplesPerSec >= 1000) sampleRateSelect = '3';
  if (samplesPerSec >= 500) sampleRateSelect = '2';
  if (samplesPerSec >= 250) sampleRateSelect = '1';
  if (samplesPerSec <= 125) sampleRateSelect = '0';

  serial_driver_.writeByte('m');
  serial_driver_.writeByte(' ');
  serial_driver_.writeByte(sampleRateSelect);
  serial_driver_.writeByte('\r');

  returned_chars = readUntilChar('>', 30);  // text per datasheet
  if ((returned_chars < 15) || (returned_chars >= 30)) {
    spdlog::error("Stim300: setDatagramFormat unexpected reply error!");
    ret_code = -2;
  }
  return ret_code;
}

// Sets accelerometer sample rate with minimum of 16hz
int32_t DriverStim300::setAccelerometerFilter(uint16_t frequency) {
  int32_t ret_code = 0;
  int32_t returned_chars = 0;

  spdlog::debug("Stim300: Set accelerometer filter to {} hz", frequency);
  // Excuse the turse format as it is most effective for this sort of code
  char filterRateSelect = '0';
  if (frequency >= 262) filterRateSelect = '4';
  if (frequency >= 131) filterRateSelect = '3';
  if (frequency >= 66) filterRateSelect = '2';
  if (frequency >= 33) filterRateSelect = '1';
  if (frequency <= 16) filterRateSelect = '0';

  serial_driver_.flush();
  serial_driver_.writeByte('f');
  serial_driver_.writeByte(' ');
  serial_driver_.writeByte(filterRateSelect);
  serial_driver_.writeByte('\r');

  returned_chars = readUntilChar('>', 100);  // text per datasheet
  if ((returned_chars < 20) || (returned_chars >= 100)) {
    spdlog::error("Stim300: setDatagramFormat unexpected reply error!");
    ret_code = -2;
  }

  returned_chars = readUntilChar('>', 50);  // text per datasheet
  if ((returned_chars < 15) || (returned_chars >= 50)) {
    spdlog::error("Stim300: setAccelerometerFilter unexpected reply error!");
    ret_code = -2;
  }
  return ret_code;
}

// Sets gyroscope sample rate with minimum of 16hz
int32_t DriverStim300::setGyroscopeFilter(uint16_t frequency) {
  int32_t ret_code = 0;
  int32_t returned_chars = 0;

  spdlog::debug("Stim300: Set gyroscope filter to {} hz", frequency);
  if (frequency > 1000) {
    spdlog::error("Stim300: setGyroscopeFilter out of range as {}!", frequency);
    return -1;
  }

  // Excuse the turse format as it is most effective for this sort of code
  char filterRateSelect = '0';
  if (frequency >= 262) filterRateSelect = '4';
  if (frequency >= 131) filterRateSelect = '3';
  if (frequency >= 66) filterRateSelect = '2';
  if (frequency >= 33) filterRateSelect = '1';
  if (frequency <= 16) filterRateSelect = '0';

  serial_driver_.writeByte('g');
  serial_driver_.writeByte(' ');
  serial_driver_.writeByte(filterRateSelect);
  serial_driver_.writeByte('\r');

  returned_chars = readUntilChar('>', 50);  // text per datasheet
  if ((returned_chars < 15) || (returned_chars >= 50)) {
    spdlog::error("Stim300: setGyroscopeFilter unexpected reply error!");
    ret_code = -2;
  }
  return ret_code;
}

// Sets RS-422 line termination
int32_t DriverStim300::setLineTermination(uint16_t mode) {
  int32_t ret_code = 0;
  int32_t returned_chars = 0;

  if (mode == 0) {
    spdlog::debug("Stim300: Disable RS422 line termination");
    sendStringWithCR("r 0");
    returned_chars = readUntilChar('>', 50);  // text per datasheet
  } else {
    spdlog::debug("Stim300: Enable RS422 line termination");
    sendStringWithCR("r 1");
    returned_chars = readUntilChar('>', 50);  // text per datasheet
  }
  if ((returned_chars < 10) || (returned_chars >= 50)) {
    spdlog::error("Stim300: setLineTermination unexpected reply error!");
    ret_code = -2;
  }
  return ret_code;
}

// } -------------------- End  Routines for SERVICEMODE ----------------------

Stim300Status DriverStim300::update() noexcept {
  auto startTime = std::chrono::high_resolution_clock::now();

  switch (mode_) {
    case Mode::Init:
      serial_driver_.flush();
      if (read_config_from_sensor_) {
        askForConfigDatagram();
      }
      this->mode_ = Mode::Normal;
    case Mode::Normal:
      return readDataStream();
    case Mode::Service:
      spdlog::debug("Stim300: DriverStim300 Service");
      mode_ = Mode::Normal;
      return Stim300Status::NORMAL;
    default:
      spdlog::debug("Stim300: DriverStim300 Unknown mode");
      return readDataStream();
      break;
  }

  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = duration_cast<std::chrono::microseconds>(endTime - startTime);
  if (duration.count() > (kImuNormalDatagramSpaceUs)) {
    if (kImuDbgShowLongBreaks) {
      spdlog::debug("update took {} uS", duration.count());
    }
  }
  return Stim300Status::NORMAL;
}

bool DriverStim300::setDatagramFormat(DatagramIdentifier id) {
  datagram_id_ = datagramIdentifierToRaw(id);
  datagram_size_ = calculateDatagramSize(id);
  crc_dummy_bytes_ = numberOfPaddingBytes(id);
  return false;
}

bool DriverStim300::verifyChecksum(const std::vector<uint8_t>::const_iterator &begin,
                                   const std::vector<uint8_t>::const_iterator &end,
                                   const uint8_t &crc_dummy_bytes) {
  uint32_t crc = stim_300::DatagramParser::parseCRC(end - sizeof(uint32_t));

  boost::crc_basic<32> crc_32_calculator(0x04C11DB7, 0xFFFFFFFF, 0x00, false, false);
  uint8_t buffer_CRC[end - begin - sizeof(uint32_t) + crc_dummy_bytes];
  std::copy(begin, end - sizeof(uint32_t) + crc_dummy_bytes, buffer_CRC);

  /** Fill the Dummy bytes with 0x00. There are at the end of the buffer **/
  for (size_t i = 0; i < crc_dummy_bytes; ++i) buffer_CRC[sizeof(buffer_CRC) - (1 + i)] = 0x00;

  crc_32_calculator.process_bytes(buffer_CRC, sizeof(buffer_CRC));
  auto crc_calck = crc_32_calculator.checksum();
  if (kImuDbgShowCrc) {
    printf("MSG  CRC %d [0x%x]\r\n", crc, crc);
    printf("CALC CRC %d [0x%x]\r\n", crc_calck, crc_calck);
  }
  return crc_calck == crc;
}

std::string DriverStim300::printSensorConfig() const noexcept {
  std::stringstream ss;

  ss << "\nFirmware: " << sensor_config_.revision << std::to_string(sensor_config_.firmware_version)
     << std::endl;
  ss << "Sample_freq: ";
  switch (sensor_config_.sample_freq) {
    case stim_300::SampleFreq::S125:
      ss << "125 Hz";
      break;
    case stim_300::SampleFreq::S250:
      ss << "250 Hz";
      break;
    case stim_300::SampleFreq::S500:
      ss << "500 Hz";
      break;
    case stim_300::SampleFreq::S1000:
      ss << "1000 Hz";
      break;
    case stim_300::SampleFreq::S2000:
      ss << "2000 Hz";
      break;
    case stim_300::SampleFreq::TRG:
      ss << "External Trigger";
      break;
  }
  ss << std::endl;
  auto included_sensors = isIncluded(sensor_config_.datagram_id);
  ss << "Gyro:\t\t\t" << included_sensors[SensorIndx::GYRO] << std::endl;
  ss << "Accelerometer:\t" << included_sensors[SensorIndx::ACC] << std::endl;
  ss << "Inlcinometer:\t" << included_sensors[SensorIndx::INCL] << std::endl;
  ss << "Temprature:\t\t" << included_sensors[SensorIndx::TEMP] << std::endl;
  ss << "Aux:\t\t\t" << included_sensors[SensorIndx::AUX] << std::endl;
  ss << "Normal Datagram termination: " << sensor_config_.normal_datagram_CRLF << std::endl;
  ss << "Gyro output:\t\t\t";
  switch (sensor_config_.gyro_output_unit) {
    case GyroOutputUnit::ANGULAR_RATE:
      ss << "Angular rate";
      break;
    case GyroOutputUnit::AVERAGE_ANGULAR_RATE:
      ss << "Average angular rate";
      break;
    case GyroOutputUnit::INCREMENTAL_ANGLE:
      ss << "Incremental angle";
      break;
    case GyroOutputUnit::INTEGRATED_ANGLE:
      ss << "Integrated angle";
      break;
  }
  ss << std::endl;
  ss << "Accelerometer output:\t";
  switch (sensor_config_.acc_output_unit) {
    case AccOutputUnit::ACCELERATION:
      ss << "Acceleration";
      break;
    case AccOutputUnit::AVERAGE_ACCELERATION:
      ss << "Average acceleration";
      break;
    case AccOutputUnit::INCREMENTAL_VELOCITY:
      ss << "Incremental velocity";
      break;
    case AccOutputUnit::INTEGRATED_VELOCITY:
      ss << "Integrated velocity";
      break;
  }
  ss << std::endl;
  ss << "Inclinometer output:\t";
  switch (sensor_config_.incl_output_unit) {
    case InclOutputUnit::ACCELERATION:
      ss << "Acceleration";
      break;
    case InclOutputUnit::AVERAGE_ACCELERATION:
      ss << "Average acceleration";
      break;
    case InclOutputUnit::INCREMENTAL_VELOCITY:
      ss << "Incremental velocity";
      break;
    case InclOutputUnit::INTEGRATED_VELOCITY:
      ss << "Integrated velocity";
      break;
  }
  ss << std::endl;
  ss << "Acceleration range: ";
  switch (sensor_config_.acc_range) {
    case AccRange::G2:
      ss << "2";
      break;
    case AccRange::G5:
      ss << "5";
      break;
    case AccRange::G10:
      ss << "10";
      break;
    case AccRange::G30:
      ss << "30";
      break;
    case AccRange::G80:
      ss << "80";
      break;
  }
  ss << " g." << std::endl;
  return ss.str();
}
// namespace stim_300
}  // namespace stim_300
