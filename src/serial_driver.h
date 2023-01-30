// This file is from a stim300 driver package available under MIT license
// File comes from https://github.com/vortexntnu/stim300-driver
// Copyright (c) 2019 Vortex NTNU  MIT License

#pragma once

#include <stdint.h>

class SerialDriver {
 public:
  virtual ~SerialDriver() = default;
  virtual bool readByte(uint8_t &byte) = 0;
  virtual int readBytes(uint8_t *bytes, int numBytes) = 0;
  virtual bool writeByte(uint8_t byte) = 0;
  virtual bool flush() = 0;
};
