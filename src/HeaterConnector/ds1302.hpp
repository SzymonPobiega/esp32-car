#pragma once
#include <Arduino.h>

// DS1302 real-time clock on three GPIOs (CLK, DAT, RST/CE). Only the time of day is used.
namespace ds1302 {

enum class Status {
  Missing,   // no chip answers (the RAM check failed)
  NotSet,    // chip found, but the clock is halted or its time was never set
  Ok
};

struct Time {
  uint8_t hour, minute, second;
};

void begin(int clkPin, int datPin, int rstPin);
Status read(Time &t);
void set(const Time &t);
void dump(Print &out);   // raw registers, for checking the wiring

}
