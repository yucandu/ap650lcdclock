#pragma once
#include <Arduino.h>
#include <stdint.h>

// ──────────────────────────────────────────────────────────────────────────────
// AP650 (ValenceTech) LCD + keyboard controller, 2-wire bit-banged driver.
//
// Key facts from the datasheet that this driver is built around:
//   * LCD RAM is 4 bits wide: one nibble per SEG pin, bit0 = COM1 ... bit3 = COM4.
//   * RAM address = 0x80 + (SEG number - 1).
//   * A write transaction is  ST, 0x40, <address>, <data1>, <data2>, ... SP
//     For LCD RAM, each data byte fills TWO nibbles:
//         upper nibble -> <address>, lower nibble -> <address + 1>
//     so single-byte writes to one address would clobber the next one.
//     The driver therefore keeps a 40-nibble copy of the RAM and sends the whole
//     thing in one transaction (flush()).
//
// Usage:
//     lcd.begin();
//     lcd.setTime(12, 34);
//     lcd.colon(true);
//     lcd.flush();          // nothing reaches the display until flush()
// ──────────────────────────────────────────────────────────────────────────────

// Days of the week for setDay()
enum Weekday {
    DAY_SUN = 0,
    DAY_MON = 1,
    DAY_TUE = 2,
    DAY_WED = 3,
    DAY_THU = 4,
    DAY_FRI = 5,
    DAY_SAT = 6,
    DAY_NONE = 0xFF,    // pass to setDay() to clear all day labels
};

class AP650 {
public:
    AP650(uint8_t pinCLK, uint8_t pinDIO);

    // Returns true if the chip acknowledged every config write.
    // clearDisplay = false skips the final clear, so whatever is on the glass stays
    // visible until the next flush(). Use that on a wake from deep sleep.
    bool begin(bool clearDisplay = true);

    void displayOn();               // LCD + regulator/doubler + bias on
    void displayOff();              // everything off
    void blank(bool on);            // blank the glass without losing RAM contents

    // ── Drawing: these only change the local buffer. Call flush() to show it. ──
    void clearBuffer();
    void setDigit(uint8_t pos, uint8_t segMask);          // pos 0 = leftmost, mask uses SEG_A..SEG_DP bits
    void setDigitValue(uint8_t pos, uint8_t value);       // value 0-15 (hex), anything else = blank
    void setTime(uint8_t hours, uint8_t minutes, bool leadingZero = false);
    void setPixel(uint8_t seg, uint8_t com, bool on);     // seg 1-40, com 0-3 (0 = COM1). For exploring the glass.

    // Icons
    void colon(bool on);
    void am(bool on);
    void pm(bool on);
    void sun(bool on);                  // sunshine symbol
    void sensorTemp(bool on);           // "ST" (sensor temperature) label
    void lowBatt(bool on);              // LOW BATT label

    // Day of week: lights exactly one label (DAY_SUN..DAY_SAT), DAY_NONE clears them all
    void setDay(uint8_t day);

    // Decimal points between digits.
    //   afterDigit = 1 -> between the 2nd and 3rd digit
    //   afterDigit = 2 -> between the 3rd and 4th digit
    // setDigitValue() leaves these alone, so draw digits first or last, either order works.
    void decimalPoint(uint8_t afterDigit, bool on);

    // ── Bus ────────────────────────────────────────────────────────────────────
    bool flush();                   // send the whole buffer, returns true if every byte was ACKed
    void clear();                   // clearBuffer() + flush()

private:
    uint8_t _clk, _dio;
    uint8_t _ram[40];               // one nibble per SEG, index 0 = SEG1 = address 0x80

    void _setBit(uint8_t ramIndex, uint8_t com, bool on);
    void _drawDigit(uint8_t pos, uint8_t segMask, uint8_t numSegs);   // numSegs 7 = A-G only, 8 = A-G + DP

    void _start();
    void _stop();
    bool _sendByte(uint8_t v);      // returns true if the AP650 pulled DIO low on the 9th clock
    bool _writeReg(uint8_t reg, uint8_t data);
};

// Segment bit positions within a digit mask
//   mask bit:  7   6   5   4   3   2   1   0
//              DP  G   F   E   D   C   B   A
enum {
    SEG_A  = 0,
    SEG_B  = 1,
    SEG_C  = 2,
    SEG_D  = 3,
    SEG_E  = 4,
    SEG_F  = 5,
    SEG_G  = 6,
    SEG_DP = 7,
};

// Register addresses
enum {
    REG_SEGCFG1 = 0x00,     // SEG25-SEG28 pin function
    REG_SEGCFG2 = 0x01,     // SEG29-SEG32
    REG_SEGCFG3 = 0x02,     // SEG33-SEG36
    REG_SEGCFG4 = 0x03,     // SEG37-SEG40
    REG_LCDCTR  = 0x04,     // LCD control
    ADDR_SEG1   = 0x80,     // first LCD RAM address
    NUM_SEGMENTS = 40,
};

// 7-segment patterns, bit0 = A ... bit6 = G, bit7 = DP
static const uint8_t SEG7[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71, // F
    0x00, // blank
    0x40, // -
};

// Digit 0 (leftmost) has no F segment, so it only shows 1, 2, 3, 7, '-' and
// letters that don't need F. Handy extras for it, pass to setDigit():
static const uint8_t SEG7_LOWER_C = 0x58;   // D, E, G
static const uint8_t SEG7_MINUS   = 0x40;   // G