// Copyright 2021 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "mbed.h"

#include "mjlib/base/string_span.h"

namespace moteus {

class Stm32I2c {
 public:
  struct Options {
    PinName sda = NC;
    PinName scl = NC;
    int frequency = 400000;
  };

  Stm32I2c(const Options& options) {
    i2c_init(&mbed_i2c_, options.sda, options.scl);
    i2c_ = mbed_i2c_.i2c.handle.Instance;
    // The mbed libraries only generate timings for a small number of
    // fixed scenarios.  We aren't in those, so we'll rely on a number
    // from CubeMX.
    i2c_->CR1 &= ~(I2C_CR1_PE);

    // TODO: Actually implement different bit rates.
    i2c_->TIMINGR = 0x10a0a6fb;

    // PE must be low for a bit, so wait.
    for (int i = 0; i < 1000; i++);
    i2c_->CR1 |= (I2C_CR1_PE);
  }

  void StartReadMemory(uint8_t slave_address,
                       uint8_t address,
                       mjlib::base::string_span data) {
    if (mode_ != Mode::kIdle) { return; }
    if ((i2c_->CR2 & I2C_CR2_START) != 0) {
      mode_ = Mode::kError;
      return;
    }

    slave_address_ = slave_address;
    rx_data_ = data;

    i2c_->CR2 = (
        I2C_CR2_START |
        // I2C_CR2_RD_WRN | // we are reading
        // I2C_CR2_AUTOEND | // we are going to send a repeated start
        (1 << I2C_CR2_NBYTES_Pos) |
        ((slave_address_ << 1) << I2C_CR2_SADD_Pos) |
        0);
    i2c_->TXDR = address;

    mode_ = Mode::kSentRegister;
  }

  enum class ReadStatus {
    kNoStatus,
    kComplete,
    kError,
  };

  ReadStatus CheckRead() {
    if (mode_ == Mode::kComplete) {
      mode_ = Mode::kIdle;
      return ReadStatus::kComplete;
    }
    if (mode_ == Mode::kError) {
      mode_ = Mode::kIdle;
      return ReadStatus::kError;
    }
    return ReadStatus::kNoStatus;
  }

  void Poll() {
    switch (mode_) {
      case Mode::kIdle:
      case Mode::kComplete:
      case Mode::kError: {
        break;
      }
      case Mode::kSentRegister: {
        if ((i2c_->ISR & I2C_ISR_TC) == 0) {
          break;
        }

        // Clear any NACKs
        i2c_->ICR |= I2C_ICR_NACKCF;

        // Now we send our repeated start to retrieve the result.
        i2c_->CR2 =
            (I2C_CR2_START |
             I2C_CR2_RD_WRN |
             I2C_CR2_AUTOEND |
             ((slave_address_ << 1) << I2C_CR2_SADD_Pos) |
             (rx_data_.size() << I2C_CR2_NBYTES_Pos) |
             0);

        offset_ = 0;
        mode_ = Mode::kReadingData;

        break;
      }
      case Mode::kReadingData: {
        if ((i2c_->ISR & I2C_ISR_RXNE) == 0) {
          break;
        }

        // We have data.
        rx_data_[offset_++] = i2c_->RXDR;

        if (offset_ >= rx_data_.size()) {
          // Clear any NACKs
          i2c_->ICR |= I2C_ICR_NACKCF;
          mode_ = Mode::kComplete;
        }

        break;
      }
    }

    if (i2c_->ISR & I2C_ISR_NACKF) {
      mode_ = Mode::kError;
      i2c_->ICR |= I2C_ICR_NACKCF;
      return;
    }
  }

 private:
  i2c_t mbed_i2c_;

  enum class Mode {
    kIdle,
    kSentRegister,
    kReadingData,
    kComplete,
    kError,
  };

  Mode mode_ = Mode::kIdle;
  I2C_TypeDef* i2c_ = nullptr;
  uint8_t slave_address_ = 0;
  mjlib::base::string_span rx_data_;
  int32_t offset_ = 0;
};

}
