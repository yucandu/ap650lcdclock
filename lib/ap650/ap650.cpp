#include "ap650.h"

// ── Constants ─────────────────────────────────────────────────────────────────
static const uint8_t T_US = 5;              // bus delay unit; datasheet minimum SCK period is 1 us

static const uint8_t CMD_WRITE = 0x40;

// LCDCTR bits: [7]=0 [6]=MASK_EN [5]=HI_DRV [4]=BIAS0 [3]=BIAS1 [2]=REGEN [1]=LCDCLR [0]=LCDEN
//
// LCD supply mode:
//   0 = internal regulator/doubler OFF (REGEN=0). VLCD comes from the VLCD pin, which on
//       this module appears to be fed from VDD. Contrast follows VDD (3.3 V here).
//   1 = internal regulator/doubler ON  (REGEN=1). VLCD = 2 x VREG = about 3.0 V, fixed,
//       THIS IS THE LOW CURRENT MODE
#define AP650_INTERNAL_VLCD 0

#if AP650_INTERNAL_VLCD
static const uint8_t LCDCTR_ON    = 0x3D;   // HI_DRV, BIAS0, BIAS1, REGEN, LCDEN
static const uint8_t LCDCTR_BLANK = 0x3F;   // same as ON but LCDCLR=1 (glass blank, RAM kept)
#else
static const uint8_t LCDCTR_ON    = 0x11;   // Low-driving bias, REGEN ON, LCDEN on, LOWEST CURRENT MODE FOUND
static const uint8_t LCDCTR_BLANK = 0x13;   // Same as ON but LCDCLR=1 (glass blank, RAM kept)
#endif
static const uint8_t LCDCTR_OFF   = 0x00;

// SEGCFGx: two bits per pin, 01 = LCD segment output
static const uint8_t SEGCFG_ALL_LCD = 0x55;

// ── Segment map ───────────────────────────────────────────────────────────────
// Each entry is {RAM index, COM}. RAM index = SEG number - 1 (address = 0x80 + index).
// COM 0 = COM1 ... 3 = COM4.  RAM_NONE means "this segment does not exist on the glass".
//
// Layout found on this panel: for each digit, A/B/C/DP share one SEG pin and
// D/E/F/G share the next one:
//     ABC pin:   A=COM1  B=COM2  C=COM3  DP=COM4
//     DEFG pin:  F=COM1  G=COM2  E=COM3  D=COM4
static const uint8_t RAM_NONE = 0xFF;

struct SegPixel {
    uint8_t ramIndex;
    uint8_t com;
};

static const SegPixel DIGIT_MAP[4][8] = {
    //   A       B       C       D        E        F           G        DP
    { {9, 0}, {9, 1}, {9, 2}, {10, 3}, {10, 2}, {RAM_NONE, 0}, {10, 1}, {RAM_NONE, 0} },  // digit 0 (leftmost): no F by design (only 1 or 2), DP slot is the colon
    { {7, 0}, {7, 1}, {7, 2}, { 8, 3}, { 8, 2}, { 8, 0},       { 8, 1}, { 7, 3}       },  // digit 1
    { {5, 0}, {5, 1}, {5, 2}, { 6, 3}, { 6, 2}, { 6, 0},       { 6, 1}, { 5, 3}       },  // digit 2
    { {3, 0}, {3, 1}, {3, 2}, { 4, 3}, { 4, 2}, { 4, 0},       { 4, 1}, {RAM_NONE, 0} },  // digit 3 (rightmost): no DP, that slot is the sun symbol
};

// Icons (RAM index = SEG - 1, com = COM - 1)
static const SegPixel ICON_COLON   = { 9, 3};   // SEG10 COM4
static const SegPixel ICON_AM      = {13, 0};   // SEG14 COM1
static const SegPixel ICON_PM      = {10, 0};   // SEG11 COM1
static const SegPixel ICON_SUN     = { 3, 3};   // SEG4  COM4
static const SegPixel ICON_ST      = { 1, 1};   // SEG2  COM2  ("sensor temperature")
static const SegPixel ICON_LOWBATT = {14, 1};   // SEG15 COM2

// Decimal points. These are the same pixels as DP on digit 1 and digit 2.
static const SegPixel DP_AFTER_DIGIT1 = {7, 3}; // SEG8 COM4, between 2nd and 3rd digit
static const SegPixel DP_AFTER_DIGIT2 = {5, 3}; // SEG6 COM4, between 3rd and 4th digit

// Day labels, indexed by Weekday (0 = Sunday)
static const SegPixel DAY_MAP[7] = {
    {11, 2},    // SUN  SEG12 COM3
    {12, 0},    // MON  SEG13 COM1
    {13, 1},    // TUE  SEG14 COM2
    {12, 1},    // WED  SEG13 COM2
    {11, 0},    // THU  SEG12 COM1
    {12, 2},    // FRI  SEG13 COM3
    {11, 1},    // SAT  SEG12 COM2
};

// ── Construction / init ───────────────────────────────────────────────────────
AP650::AP650(uint8_t pinCLK, uint8_t pinDIO) {
    _clk = pinCLK;
    _dio = pinDIO;
    clearBuffer();
}

bool AP650::begin(bool clearDisplay) {
    bool ok = true;
    if (clearDisplay) {
    pinMode(_clk, OUTPUT);
    pinMode(_dio, OUTPUT);
    digitalWrite(_clk, HIGH);
    digitalWrite(_dio, HIGH);
    delay(50);                  // let the internal RC oscillator settle

    _stop();                    // datasheet: a STOP resets the serial link after power-up

    

    // SEG25-SEG40 pins as LCD segment outputs (SEG1-SEG24 are always LCD)
    ok &= _writeReg(REG_SEGCFG1, SEGCFG_ALL_LCD);
    ok &= _writeReg(REG_SEGCFG2, SEGCFG_ALL_LCD);
    ok &= _writeReg(REG_SEGCFG3, SEGCFG_ALL_LCD);
    ok &= _writeReg(REG_SEGCFG4, SEGCFG_ALL_LCD);

    ok &= _writeReg(REG_LCDCTR, LCDCTR_ON);

    
        clear();                // zero the RAM and blank the glass
    }
    else {
        pinMode(_clk, OUTPUT);
    pinMode(_dio, OUTPUT);
    digitalWrite(_clk, HIGH);
    digitalWrite(_dio, HIGH);
    
    // Re-apply configurations lost or left disabled, without clearing RAM
    _writeReg(REG_SEGCFG1, SEGCFG_ALL_LCD);
    _writeReg(REG_SEGCFG2, SEGCFG_ALL_LCD);
    _writeReg(REG_SEGCFG3, SEGCFG_ALL_LCD);
    _writeReg(REG_SEGCFG4, SEGCFG_ALL_LCD);
    _writeReg(REG_LCDCTR, LCDCTR_ON);
    }
    return ok;
}

void AP650::displayOn()  { _writeReg(REG_LCDCTR, LCDCTR_ON);  }
void AP650::displayOff() { _writeReg(REG_LCDCTR, LCDCTR_OFF); }

void AP650::blank(bool on) {
    _writeReg(REG_LCDCTR, on ? LCDCTR_BLANK : LCDCTR_ON);
}

// ── Bit-bang protocol ─────────────────────────────────────────────────────────
// START: DIO falls while CLK is high
void AP650::_start() {
    digitalWrite(_dio, HIGH);
    delayMicroseconds(T_US);
    digitalWrite(_clk, HIGH);
    delayMicroseconds(T_US);
    digitalWrite(_dio, LOW);
    delayMicroseconds(T_US);
    digitalWrite(_clk, LOW);
    delayMicroseconds(T_US);
}

// STOP: DIO rises while CLK is high
void AP650::_stop() {
    digitalWrite(_dio, LOW);
    delayMicroseconds(T_US);
    digitalWrite(_clk, HIGH);
    delayMicroseconds(T_US);
    digitalWrite(_dio, HIGH);
    delayMicroseconds(T_US);
}

bool AP650::_sendByte(uint8_t v) {
    // 8 data bits, MSB first
    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(_clk, LOW);
        delayMicroseconds(T_US);
        digitalWrite(_dio, (v & 0x80) ? HIGH : LOW);
        delayMicroseconds(T_US);
        digitalWrite(_clk, HIGH);
        delayMicroseconds(T_US);
        v <<= 1;
    }

    // 9th clock: release DIO so the AP650 can pull it low to acknowledge
    digitalWrite(_clk, LOW);
    delayMicroseconds(T_US);
    pinMode(_dio, INPUT_PULLUP);
    delayMicroseconds(T_US);

    digitalWrite(_clk, HIGH);
    delayMicroseconds(T_US);
    bool acked = (digitalRead(_dio) == LOW);   // sample while CLK is high
    delayMicroseconds(T_US);

    // CLK must go low before we take the data line back
    digitalWrite(_clk, LOW);
    delayMicroseconds(T_US);
    pinMode(_dio, OUTPUT);
    digitalWrite(_dio, LOW);

    return acked;
}

// Write one value to a control register:  ST 0x40 <reg> <data> SP
bool AP650::_writeReg(uint8_t reg, uint8_t data) {
    bool ok = true;
    _start();
    ok &= _sendByte(CMD_WRITE);
    ok &= _sendByte(reg);
    ok &= _sendByte(data);
    _stop();
    return ok;
}

// ── Display RAM ───────────────────────────────────────────────────────────────
// Sends all 40 nibbles in one transaction starting at 0x80.
// Each data byte carries two SEGs: upper nibble -> address, lower nibble -> address+1.
bool AP650::flush() {
    bool ok = true;
    _start();
    ok &= _sendByte(CMD_WRITE);
    ok &= _sendByte(ADDR_SEG1);
    for (uint8_t i = 0; i < NUM_SEGMENTS; i += 2) {
        uint8_t upper = _ram[i]     & 0x0F;
        uint8_t lower = _ram[i + 1] & 0x0F;
        ok &= _sendByte((uint8_t)((upper << 4) | lower));
    }
    _stop();
    return ok;
}

void AP650::clearBuffer() {
    for (uint8_t i = 0; i < NUM_SEGMENTS; i++) _ram[i] = 0;
}

void AP650::clear() {
    clearBuffer();
    flush();
}

void AP650::_setBit(uint8_t ramIndex, uint8_t com, bool on) {
    if (ramIndex >= NUM_SEGMENTS) return;
    if (com > 3) return;
    if (on) _ram[ramIndex] |=  (uint8_t)(1 << com);
    else    _ram[ramIndex] &= (uint8_t)~(1 << com);
}

// ── Drawing ───────────────────────────────────────────────────────────────────
void AP650::_drawDigit(uint8_t pos, uint8_t segMask, uint8_t numSegs) {
    if (pos > 3) return;
    for (uint8_t s = 0; s < numSegs; s++) {
        const SegPixel &p = DIGIT_MAP[pos][s];
        if (p.ramIndex == RAM_NONE) continue;
        _setBit(p.ramIndex, p.com, (segMask & (1 << s)) != 0);
    }
}

// Sets all 8 bits of the digit, including its decimal point (bit 7)
void AP650::setDigit(uint8_t pos, uint8_t segMask) {
    _drawDigit(pos, segMask, 8);
}

// Sets segments A-G only, so decimal points are left as they were
void AP650::setDigitValue(uint8_t pos, uint8_t value) {
    uint8_t mask = 0x00;
    if (value < 16) mask = SEG7[value];
    _drawDigit(pos, mask, 7);
}

void AP650::setTime(uint8_t hours, uint8_t minutes, bool leadingZero) {
    uint8_t h10 = (hours   / 10) % 10;
    uint8_t h1  =  hours   % 10;
    uint8_t m10 = (minutes / 10) % 10;
    uint8_t m1  =  minutes % 10;

    // Digit 0 has no F segment, so a "0" there would look wrong. Blank it by default.
    if (h10 == 0 && !leadingZero) setDigit(0, 0x00);
    else                          setDigitValue(0, h10);

    setDigitValue(1, h1);
    setDigitValue(2, m10);
    setDigitValue(3, m1);
}

// seg is the SEG pin number 1-40, com is 0-3 (COM1-COM4)
void AP650::setPixel(uint8_t seg, uint8_t com, bool on) {
    if (seg < 1 || seg > NUM_SEGMENTS) return;
    _setBit((uint8_t)(seg - 1), com, on);
}

void AP650::colon(bool on) { _setBit(ICON_COLON.ramIndex, ICON_COLON.com, on); }
void AP650::am(bool on)    { _setBit(ICON_AM.ramIndex,    ICON_AM.com,    on); }
void AP650::pm(bool on)    { _setBit(ICON_PM.ramIndex,    ICON_PM.com,    on); }

void AP650::sun(bool on)        { _setBit(ICON_SUN.ramIndex,     ICON_SUN.com,     on); }
void AP650::sensorTemp(bool on) { _setBit(ICON_ST.ramIndex,      ICON_ST.com,      on); }
void AP650::lowBatt(bool on)    { _setBit(ICON_LOWBATT.ramIndex, ICON_LOWBATT.com, on); }

// Lights one day label and turns the other six off.
// DAY_NONE (or any value above DAY_SAT) turns all of them off.
void AP650::setDay(uint8_t day) {
    for (uint8_t d = 0; d < 7; d++) {
        _setBit(DAY_MAP[d].ramIndex, DAY_MAP[d].com, d == day);
    }
}

void AP650::decimalPoint(uint8_t afterDigit, bool on) {
    if (afterDigit == 1) {
        _setBit(DP_AFTER_DIGIT1.ramIndex, DP_AFTER_DIGIT1.com, on);
    } else if (afterDigit == 2) {
        _setBit(DP_AFTER_DIGIT2.ramIndex, DP_AFTER_DIGIT2.com, on);
    }
}
