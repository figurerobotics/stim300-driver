// This file is from a stim300 driver package available under MIT license
// File comes from https://github.com/vortexntnu/stim300-driver
// Copyright (c) 2019 Vortex NTNU  MIT License

#pragma once

#include <array>
#include <cassert>
#include <cstdint>
#include <sstream>
#include <vector>

#include "apps/imu_app/stim300_constants.h"

namespace stim_300 {
struct SensorConfig {
  char revision;
  uint8_t firmware_version;
  uint8_t sys_config_1;
  uint8_t sys_config_2;
  uint16_t accel_filters;
  uint16_t gyro_filters;
  SampleFreq sample_freq;
  DatagramIdentifier datagram_id;
  bool normal_datagram_CRLF;
  GyroOutputUnit gyro_output_unit;
  AccOutputUnit acc_output_unit;
  InclOutputUnit incl_output_unit;
  AccRange acc_range;

  inline bool operator!=(const SensorConfig &rhs) {
    bool val = false;
    val = (this->sample_freq != rhs.sample_freq || this->datagram_id != rhs.datagram_id ||
           this->normal_datagram_CRLF != rhs.normal_datagram_CRLF ||
           this->gyro_output_unit != rhs.gyro_output_unit ||
           this->acc_output_unit != rhs.acc_output_unit ||
           this->incl_output_unit != rhs.incl_output_unit || this->acc_range != rhs.acc_range);
    return val;
  }
};

struct SensorData {
  std::array<double, 3> gyro;
  std::array<double, 3> acc;
  std::array<double, 3> incl;
  std::array<double, 3> temp_gyro;
  std::array<double, 3> temp_acc;
  std::array<double, 3> temp_incl;
  double aux;
  uint8_t counter;
  uint16_t latency_us;
};

struct DatagramParser {
  DatagramParser(DatagramIdentifier dg_id, GyroOutputUnit gyro_o, AccOutputUnit acc_o,
                 InclOutputUnit incl_o, AccRange acc_range);
  void setDataParameters(SensorConfig sensor_config);
  static uint32_t parseCRC(std::vector<uint8_t>::const_iterator &&itr);

  /// @brief Parse a complete datagram into data values held in the parser object
  uint8_t parseData(std::vector<uint8_t>::const_iterator &&buffer_itr,
                    SensorData &sensor_data) const;

  /// @brief Parse a complete onfiguration datagram into config state held in the parser object
  SensorConfig parseConfig(std::vector<uint8_t>::const_iterator &&buffer_itr) const;

 private:
  std::array<bool, 5> is_included_;
  double temp_scale_;
  double aux_scale_;
  double gyro_scale_;
  double acc_scale_;
  double incl_scale_;
  void setDataScales(GyroOutputUnit gyro_o, AccOutputUnit acc_o, InclOutputUnit incl_o,
                     AccRange acc_range);
  // Meta data is stored as "unsigned word", we simply combine the bytes into
  // the right sized uint by left shifting them. Note the biggest is the CRC
  // which is 32 bits.
  static uint32_t parseUnsigned(std::vector<uint8_t>::const_iterator &it, uint8_t size) {
    uint32_t tmp{0};
    for (int i = 0; i < size; ++i) {
      tmp = (tmp << 8u) | *(it++);
    }
    return tmp;
  }

  // Sensor data is stored as two`s complement, we shift the bytes according to
  // the datasheet, bu we shift them to fill up the int32_t type then we shift
  // them back to their right position. When we shift them back the shift
  // operator will automatically sign-extend the value.
  static int32_t parseTwosComplement(std::vector<uint8_t>::const_iterator &it, const uint8_t size) {
    assert(size == 3 || size == 2);
    if (size == 3) {
      return ((*it++ << 24) | (*it++ << 16) | (*it++ << 8)) >> 8;
    } else if (size == 2) {
      return ((*it++ << 24) | (*it++ << 16)) >> 16;
    }
    return 0;
  }
};
// end namespace stim_300
}  // namespace stim_300
