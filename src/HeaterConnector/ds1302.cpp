#include "ds1302.hpp"

namespace ds1302 {

// Command bytes (read = write | 1)
static const uint8_t REG_SECONDS = 0x80;   // bit 7: clock halt
static const uint8_t REG_MINUTES = 0x82;
static const uint8_t REG_HOURS = 0x84;     // bit 7: 12-hour mode
static const uint8_t REG_WP = 0x8E;        // bit 7: write protect
static const uint8_t REG_TRICKLE = 0x90;
static const uint8_t RAM_SET_FLAG = 0xC0;  // RAM byte 0: written when the time is set
static const uint8_t RAM_PROBE = 0xC2;     // RAM byte 1: presence check

static const uint8_t SET_MAGIC = 0xA5;
static const uint8_t PROBE_MAGIC = 0x5A;

static int clk, dat, rst;

static uint8_t fromBcd(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t toBcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

// Bits go out LSB first, latched on the rising edge. Before a read, the line is released
// after the last rising edge and CLK stays high: the chip drives the first data bit on the
// falling edge that follows, which must not happen while DAT is still an output.
static void writeByte(uint8_t v, bool thenRead = false) {
  pinMode(dat, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(dat, v & 1);
    delayMicroseconds(1);
    digitalWrite(clk, HIGH);
    delayMicroseconds(1);
    if (thenRead && i == 7) {
      pinMode(dat, INPUT_PULLDOWN);
    } else {
      digitalWrite(clk, LOW);
      delayMicroseconds(1);
    }
    v >>= 1;
  }
}

// Each bit is valid after a falling edge
static uint8_t readByte() {
  uint8_t v = 0;
  for (int i = 0; i < 8; i++) {
    digitalWrite(clk, HIGH);
    delayMicroseconds(1);
    digitalWrite(clk, LOW);
    delayMicroseconds(1);
    if (digitalRead(dat)) v |= 1 << i;
  }
  return v;
}

static void start() {
  digitalWrite(clk, LOW);
  digitalWrite(rst, HIGH);
  delayMicroseconds(4);
}

static void stop() {
  digitalWrite(rst, LOW);
  delayMicroseconds(4);
}

static uint8_t readReg(uint8_t cmd) {
  start();
  writeByte(cmd | 1, true);
  uint8_t v = readByte();
  stop();
  return v;
}

static void writeReg(uint8_t cmd, uint8_t v) {
  start();
  writeByte(cmd);
  writeByte(v);
  stop();
}

void begin(int clkPin, int datPin, int rstPin) {
  clk = clkPin;
  dat = datPin;
  rst = rstPin;
  pinMode(clk, OUTPUT);
  pinMode(rst, OUTPUT);
  digitalWrite(clk, LOW);
  digitalWrite(rst, LOW);

  writeReg(REG_WP, 0x00);
  writeReg(REG_TRICKLE, 0x00);   // no charging: the module holds a non-rechargeable coin cell
  writeReg(RAM_PROBE, PROBE_MAGIC);
}

Status read(Time &t) {
  if (readReg(RAM_PROBE) != PROBE_MAGIC) {
    writeReg(REG_WP, 0x00);   // in case the chip was connected after begin()
    writeReg(RAM_PROBE, PROBE_MAGIC);
    return Status::Missing;
  }

  uint8_t s = readReg(REG_SECONDS);
  uint8_t m = readReg(REG_MINUTES);
  uint8_t h = readReg(REG_HOURS);
  if ((s & 0x80) || (h & 0x80) || readReg(RAM_SET_FLAG) != SET_MAGIC) return Status::NotSet;

  t.second = fromBcd(s & 0x7F);
  t.minute = fromBcd(m & 0x7F);
  t.hour = fromBcd(h & 0x3F);
  if (t.second > 59 || t.minute > 59 || t.hour > 23) return Status::NotSet;
  return Status::Ok;
}

void dump(Print &out) {
  pinMode(dat, INPUT_PULLUP);
  delayMicroseconds(10);
  int datUp = digitalRead(dat);
  pinMode(dat, INPUT_PULLDOWN);
  delayMicroseconds(10);
  int datDown = digitalRead(dat);
  out.printf("DS1302 pins CLK=%d DAT=%d RST=%d, idle DAT with pull-up %d, pull-down %d\n",
             clk, dat, rst, datUp, datDown);
  writeReg(REG_WP, 0x00);
  writeReg(RAM_PROBE, PROBE_MAGIC);
  out.printf("probe %02X (want %02X), seconds %02X, minutes %02X, hours %02X, wp %02X, set flag %02X\n",
             readReg(RAM_PROBE), PROBE_MAGIC, readReg(REG_SECONDS), readReg(REG_MINUTES),
             readReg(REG_HOURS), readReg(REG_WP), readReg(RAM_SET_FLAG));
}

void set(const Time &t) {
  writeReg(REG_WP, 0x00);
  writeReg(REG_SECONDS, 0x80);   // halt while setting, so the seconds can't roll over midway
  writeReg(REG_HOURS, toBcd(t.hour));   // bit 7 clear: 24-hour mode
  writeReg(REG_MINUTES, toBcd(t.minute));
  writeReg(REG_SECONDS, toBcd(t.second));   // clears the halt bit
  writeReg(RAM_SET_FLAG, SET_MAGIC);
}

}
