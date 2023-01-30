// This file is from a stim300 driver package available under MIT license
// File comes from https://github.com/vortexntnu/stim300-driver
// Copyright (c) 2019 Vortex NTNU  MIT License

#include "apps/imu_app/datagram_parser.h"
#include <spdlog/spdlog.h>

namespace stim_300 {

DatagramParser::DatagramParser(DatagramIdentifier dg_id, GyroOutputUnit gyro_o, AccOutputUnit acc_o,
                               InclOutputUnit incl_o, AccRange acc_range)
    : is_included_(isIncluded(dg_id)), temp_scale_(tempScale()), aux_scale_(auxScale()) {
  setDataScales(gyro_o, acc_o, incl_o, acc_range);
}
void DatagramParser::setDataParameters(SensorConfig sensor_config) {
  is_included_ = isIncluded(sensor_config.datagram_id);
  setDataScales(sensor_config.gyro_output_unit, sensor_config.acc_output_unit,
                sensor_config.incl_output_unit, sensor_config.acc_range);
}
void DatagramParser::setDataScales(GyroOutputUnit gyro_o, AccOutputUnit acc_o,
                                   InclOutputUnit incl_o, AccRange acc_range) {
  switch (gyro_o) {
      // for units in rad/s
    case GyroOutputUnit::ANGULAR_RATE:
      // for units in rad/s
    case GyroOutputUnit::AVERAGE_ANGULAR_RATE:
      gyro_scale_ = gyroScale();
      break;
      // for units in rad/sample
    case GyroOutputUnit::INCREMENTAL_ANGLE:
      // for units in rad
    case GyroOutputUnit::INTEGRATED_ANGLE:
      gyro_scale_ = gyroIncrScale();
      break;
  }
  switch (acc_o) {
      // for units in g
    case AccOutputUnit::ACCELERATION:
      // for units in g
    case AccOutputUnit::AVERAGE_ACCELERATION:
      acc_scale_ = accScale(acc_range);
      break;
      // for units in m/s/sample
    case AccOutputUnit::INCREMENTAL_VELOCITY:
    case AccOutputUnit::INTEGRATED_VELOCITY:
      acc_scale_ = accIncrScale(acc_range);
      break;
  }
  switch (incl_o) {
    case InclOutputUnit::ACCELERATION:          // units are in g
    case InclOutputUnit::AVERAGE_ACCELERATION:  // units are in g
      incl_scale_ = inclScale();
      break;
    case InclOutputUnit::INCREMENTAL_VELOCITY:  // units are in m/s/sample
    case InclOutputUnit::INTEGRATED_VELOCITY:
      incl_scale_ = inclIncrScale();
      break;
  }
}

uint32_t DatagramParser::parseCRC(std::vector<uint8_t>::const_iterator &&itr) {
  return parseUnsigned(itr, N_BYTES_CRC);
}

// Given the raw Normal Mode datagram described on STIM300 datasheet
// section 5.5.6 Parse out X,Y,Z and status for Gyro, Accel, Incline, gyroTemp,
// accelTemp, inclinometer temp After that pull in the rest as may be required.
uint8_t DatagramParser::parseData(std::vector<uint8_t>::const_iterator &&buffer_itr,
                                  SensorData &sensor_data) const {
  uint8_t status{0};
  buffer_itr++;  // Skip datagram identifier
  if (is_included_[SensorIndx::GYRO]) {
    for (auto &gyro : sensor_data.gyro)
      gyro = gyro_scale_ * parseTwosComplement(buffer_itr, N_BYTES_INERTIAL_SENSOR);
    status |= parseUnsigned(buffer_itr, N_BYTES_STATUS);
  }
  if (is_included_[SensorIndx::ACC]) {
    for (auto &acc : sensor_data.acc)
      acc = acc_scale_ * parseTwosComplement(buffer_itr, N_BYTES_INERTIAL_SENSOR);
    status |= parseUnsigned(buffer_itr, N_BYTES_STATUS);
  }
  if (is_included_[SensorIndx::INCL]) {
    for (auto &incl : sensor_data.incl)
      incl = incl_scale_ * parseTwosComplement(buffer_itr, N_BYTES_INERTIAL_SENSOR);
    status |= parseUnsigned(buffer_itr, N_BYTES_STATUS);
  }
  if (is_included_[SensorIndx::TEMP]) {
    if (is_included_[SensorIndx::GYRO]) {
      for (auto &temp : sensor_data.temp_gyro) {
        int32_t tempVal = parseTwosComplement(buffer_itr, N_BYTES_TEMP_SENSOR);
        temp = tempScale() * tempVal;
      }
      status |= parseUnsigned(buffer_itr, N_BYTES_STATUS);
    }
    if (is_included_[SensorIndx::ACC]) {
      for (auto &temp : sensor_data.temp_acc) {
        int32_t tempVal = parseTwosComplement(buffer_itr, N_BYTES_TEMP_SENSOR);
        temp = tempScale() * tempVal;
      }
      status |= parseUnsigned(buffer_itr, N_BYTES_STATUS);
    }
    if (is_included_[SensorIndx::INCL]) {
      for (auto &temp : sensor_data.temp_incl)
        temp = tempScale() * parseTwosComplement(buffer_itr, N_BYTES_TEMP_SENSOR);
      status |= parseUnsigned(buffer_itr, N_BYTES_STATUS);
    }
  }
  if (is_included_[SensorIndx::AUX]) {
    sensor_data.aux = aux_scale_ * parseTwosComplement(buffer_itr, N_BYTES_AUX_SENSOR);
    status |= parseUnsigned(buffer_itr, N_BYTES_STATUS);
  }
  sensor_data.counter = parseUnsigned(buffer_itr, N_BYTES_COUNTER);

  sensor_data.latency_us = parseUnsigned(buffer_itr, N_BYTES_LATENCY);

  return status;
}

SensorConfig DatagramParser::parseConfig(std::vector<uint8_t>::const_iterator &&buffer_itr) const {
  SensorConfig sensor_config{};
  sensor_config.revision = buffer_itr[1];
  sensor_config.firmware_version = buffer_itr[2];
  switch (buffer_itr[3] >> 5u) {
    case 0u:
      sensor_config.sample_freq = SampleFreq::S125;
      break;
    case 1u:
      sensor_config.sample_freq = SampleFreq::S250;
      break;
    case 2u:
      sensor_config.sample_freq = SampleFreq::S500;
      break;
    case 3u:
      sensor_config.sample_freq = SampleFreq::S1000;
      break;
    case 4u:
      sensor_config.sample_freq = SampleFreq::S2000;
      break;
    case 5u:
      sensor_config.sample_freq = SampleFreq::TRG;
      break;
  }
  sensor_config.sys_config_1 = buffer_itr[3];
  sensor_config.sys_config_2 = buffer_itr[4];

  // Accel and gyro filters are packed nibbles with enable states and filter
  // settings Upper nibble is XYZ enables.  Then next nibbles are X, Y, Z filter
  // settings
  sensor_config.gyro_filters = ((buffer_itr[5] & 0x70) << 8) | ((buffer_itr[6] & 0x77) << 4) |
                               ((buffer_itr[7] & 0x70) >> 4);
  sensor_config.accel_filters = ((buffer_itr[8] & 0x70) << 8) | ((buffer_itr[9] & 0x77) << 4) |
                                ((buffer_itr[10] & 0x70) >> 4);

  sensor_config.normal_datagram_CRLF = (buffer_itr[3] & (1 << 0));
  std::array<bool, 5> included_sensors{};
  included_sensors[SensorIndx::GYRO] = true;
  included_sensors[SensorIndx::ACC] = (buffer_itr[3] & (1 << 1));
  included_sensors[SensorIndx::INCL] = (buffer_itr[3] & (1 << 2));
  included_sensors[SensorIndx::TEMP] = (buffer_itr[3] & (1 << 3));
  included_sensors[SensorIndx::AUX] = (buffer_itr[3] & (1 << 4));
  sensor_config.datagram_id = toDatagramID(included_sensors);

  switch (buffer_itr[5] & 0b00001111) {
    case 0:
      sensor_config.gyro_output_unit = GyroOutputUnit::ANGULAR_RATE;
      break;
    case 1:
      sensor_config.gyro_output_unit = GyroOutputUnit::INCREMENTAL_ANGLE;
      break;
    case 2:
      sensor_config.gyro_output_unit = GyroOutputUnit::AVERAGE_ANGULAR_RATE;
      break;
    case 3:
      sensor_config.gyro_output_unit = GyroOutputUnit::INTEGRATED_ANGLE;
      break;
    case 4:
      sensor_config.gyro_output_unit = GyroOutputUnit::ANGULAR_RATE;
      break;
    case 5:
      sensor_config.gyro_output_unit = GyroOutputUnit::INCREMENTAL_ANGLE;
      break;
    case 6:
      sensor_config.gyro_output_unit = GyroOutputUnit::AVERAGE_ANGULAR_RATE;
      break;
    case 7:
      sensor_config.gyro_output_unit = GyroOutputUnit::INTEGRATED_ANGLE;
      break;
  }

  switch (buffer_itr[8] & 0b00001111) {
    case 0:
      sensor_config.acc_output_unit = AccOutputUnit::ACCELERATION;
      break;
    case 1:
      sensor_config.acc_output_unit = AccOutputUnit::INCREMENTAL_VELOCITY;
      break;
    case 2:
      sensor_config.acc_output_unit = AccOutputUnit::AVERAGE_ACCELERATION;
      break;
    case 3:
      sensor_config.acc_output_unit = AccOutputUnit::INTEGRATED_VELOCITY;
      break;
  }

  switch (buffer_itr[11] & 0b00001111) {
    case 0:
      sensor_config.incl_output_unit = InclOutputUnit::ACCELERATION;
      break;
    case 1:
      sensor_config.incl_output_unit = InclOutputUnit::INCREMENTAL_VELOCITY;
      break;
    case 2:
      sensor_config.incl_output_unit = InclOutputUnit::AVERAGE_ACCELERATION;
      break;
    case 3:
      sensor_config.incl_output_unit = InclOutputUnit::INTEGRATED_VELOCITY;
      break;
  }
  // Assuming all the axes has the same range
  switch (buffer_itr[17] >> 4) {
    case 0:
      sensor_config.acc_range = AccRange::G10;
      break;
    case 2:
      sensor_config.acc_range = AccRange::G2;
      break;
    case 3:
      sensor_config.acc_range = AccRange::G5;
      break;
    case 4:
      sensor_config.acc_range = AccRange::G30;
      break;
    case 6:
      sensor_config.acc_range = AccRange::G80;
      break;
  }
  return sensor_config;
}
}  // namespace stim_300
