// Copyright 2018-2020 Josh Pieper, jjp@pobox.com.
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

#include "hal/spi_api.h"

#include "fw/millisecond_timer.h"
#include "fw/moteus_hw.h"
#include "fw/stm32_spi.h"

namespace moteus {

class AS5047 {
 public:
  using Options = Stm32Spi::Options;

  AS5047(MillisecondTimer* timer, const Options& options)
      : timer_(timer),
        spi_([options]() {
          // The next frequency down is only 6MHz, so we run a bit out
          // of tolerance to save a fair amount of time.
          auto copy = options;
          copy.frequency = 12000000;
          copy.mode = 0;
          return copy;
        }()) {}

  uint16_t Sample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.write(0x0000);
  }

  void StartSample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.start_write(0x0000);
  }

  uint16_t FinishSample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.finish_write();
  }

  void SetFilterUs(uint16_t filter_us) {
    // The MA732 filter is in register 14.  First, read it to see if
    // it needs to be changed.
    spi_.write(0x4000 | 0x0e00);

    timer_->wait_us(2);

    current_value_ = spi_.write(0x0000);

    desired_value_ = [&]() {
      if (filter_us <= 64) { return 51; }
      if (filter_us <= 128) { return 68; }
      if (filter_us <= 256) { return 102; }
      if (filter_us <= 1024) { return 119; }  // the default
      if (filter_us <= 2048) { return 136; }
      if (filter_us <= 4096) { return 153; }
      if (filter_us <= 8192) { return 170; }
      if (filter_us <= 16384) { return 187; }
      return 187;
    }();

    if (desired_value_ == (current_value_ >> 8)) { return; }

    // We're going to update things:

    spi_.write(0x8000 | 0x0e00 | desired_value_);

    // Now we have to wait 20ms.
    timer_->wait_ms(20);

    // Finally, read to ensure we got the correct result.
    final_value_ = spi_.write(0x0000);

    MJ_ASSERT((final_value_ >> 8) == desired_value_);
  }

  private:
   MillisecondTimer* const timer_;
   Stm32Spi spi_;
   uint16_t current_value_ = 0;
   uint16_t desired_value_ = 0;
   uint16_t final_value_ = 0;
};

}
