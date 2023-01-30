// This file is from a stim300 driver package available under MIT license
// File comes from https://github.com/vortexntnu/stim300-driver
// Copyright (c) 2019 Vortex NTNU  MIT License

#pragma once

#include <fcntl.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <string>

#include "apps/imu_app/serial_driver.h"

enum class BaudRate {  // defined as bit-rate in datasheet
  BAUD_377400,
  BAUD_460800,
  BAUD_921600,
  BAUD_1843200,
};

class SerialUnix : public SerialDriver {
 public:
  SerialUnix(const std::string &serial_port_name, BaudRate baudrate);
  ~SerialUnix();
  // The class is Non-Copyable
  SerialUnix(const SerialUnix &a) = delete;
  SerialUnix &operator=(const SerialUnix &a) = delete;
  // The class is non-movable
  SerialUnix(SerialUnix &&a) = delete;
  SerialUnix &operator=(SerialUnix &&a) = delete;

  bool writeByte(uint8_t byte) final;
  bool readByte(uint8_t &byte) final;
  int readBytes(uint8_t *bytes, int numBytes) final;
  bool flush() final;

 private:
  void open(const std::string &serial_port_name, BaudRate baudrate);
  void close();

  int file_handle_;
  struct termios config_;
};
