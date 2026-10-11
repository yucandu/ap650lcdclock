#include <Arduino.h>
#include <Wire.h>
#include "ap650.h"
#include <DS3232RTC.h>        // JChristensen DS3232RTC library
#include <TimeLib.h>
#include <Adafruit_AHTX0.h>  // Adafruit AHT20 library
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
// ── Pin Definitions ───────────────────────────────────────────────────────────
static const uint8_t PIN_CLK        = 4;      // AP650 CLK
static const uint8_t PIN_DIO        = 3;      // AP650 DIO
static const uint8_t PIN_RSTB       = 10;      // AP650 Reset
static const uint8_t PIN_BTN        = 0;      // Button (Active Low)
static const uint8_t PIN_LED        = 8;      // LED (Active Low)
static const uint8_t PIN_SENSOR_PWR = 5;      // Sensor VCC Power Gate
static const uint8_t PIN_VBAT       = 1;      // Battery divider midpoint (ADC1_CH1), 2 x 220k, always connected
#define PWR_SENSORS_GPIO ((gpio_num_t)PIN_SENSOR_PWR)
static const uint8_t I2C_SDA        = 7;
static const uint8_t I2C_SCL        = 6;

// ── Battery thresholds ────────────────────────────────────────────────────────
// vBat (volts) = analogReadMilliVolts(PIN_VBAT) / 500.0
// Equivalent in millivolts: pin mV * 2 (a 1:2 divider)
static const uint32_t VBAT_LOW_MV      = 3400;   // below this, lowBatt is on in every mode
static const uint32_t VBAT_3DIGIT_MV   = 4000;   // at or above this, only #.## fits

// ── RTC Retained State (Persists across Deep Sleep) ───────────────────────────
enum DisplayMode { MODE_CLOCK, MODE_TEMP, MODE_HUMIDITY, MODE_VBAT, MODE_COUNT };
enum State { STATE_NORMAL, STATE_SET_HOUR, STATE_SET_MINUTE };

RTC_DATA_ATTR DisplayMode currentMode = MODE_CLOCK;
RTC_DATA_ATTR State currentState = STATE_NORMAL;
RTC_DATA_ATTR uint8_t setHour = 12;
RTC_DATA_ATTR uint8_t setMin = 0;

// ── Globals ───────────────────────────────────────────────────────────────────
AP650 lcd(PIN_CLK, PIN_DIO);
DS3232RTC rtc;
Adafruit_AHTX0 aht;

static const uint8_t SEG7_LOWER_R = 0x50; // Segments E & G for lowercase 'r'

// Battery reading taken once per boot (before anything else loads the battery)
static uint32_t vBatMv = 0;
static bool lowBattActive = false;

static time_t compileTime() {
    static const char* const months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
    uint8_t month = 0;
    for (uint8_t i = 0; i < 12; ++i) {
        if (__DATE__[0] == months[i][0] &&
            __DATE__[1] == months[i][1] &&
            __DATE__[2] == months[i][2]) {
            month = i + 1;
            break;
        }
    }

    tmElements_t tm = {};
    tm.Year = CalendarYrToTm(atoi(__DATE__ + 7));
    tm.Month = month;
    tm.Day = atoi(__DATE__ + 4);
    tm.Hour = (__TIME__[0] - '0') * 10 + (__TIME__[1] - '0');
    tm.Minute = (__TIME__[3] - '0') * 10 + (__TIME__[4] - '0');
    tm.Second = (__TIME__[6] - '0') * 10 + (__TIME__[7] - '0');
    return makeTime(tm);
}

// ── Battery measurement ──────────────────────────────────────────────────────
// Returns battery voltage in millivolts (pin millivolts x 2).
// The divider is 2 x 220k, permanently connected (about 5 uA drain from the cell).
// That is a ~110k source impedance, so keep a 100 nF cap from the midpoint to ground:
// it holds the voltage steady and supplies the ADC's sample-and-hold charge.
static uint32_t readVBatMv() {
    //analogSetPinAttenuation(PIN_VBAT, ADC_11db);   // pin sees about 2.1 V at a full 4.2 V cell

    analogReadMilliVolts(PIN_VBAT);                // throw-away read: lets the ADC sample cap settle

    const uint8_t SAMPLES = 8;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < SAMPLES; i++) {
        sum += analogReadMilliVolts(PIN_VBAT);
    }
    uint32_t pinMv = sum / SAMPLES;

    return pinMv * 2;                              // same as (pinMv / 500.0) volts
}

// ── State Machine Handlers ────────────────────────────────────────────────────
void onButtonShortPress() {
    if (currentState == STATE_NORMAL) {
        currentMode = (DisplayMode)((currentMode + 1) % MODE_COUNT);
    } else if (currentState == STATE_SET_HOUR) {
        setHour = (setHour + 1) % 24;
    } else if (currentState == STATE_SET_MINUTE) {
        setMin = (setMin + 1) % 60;
    }
}

void onButtonHold() {
    if (currentState == STATE_NORMAL) {
        currentState = STATE_SET_HOUR;
        time_t now = rtc.get();
        setHour = hour(now);
        setMin = minute(now);
    } else if (currentState == STATE_SET_HOUR) {
        currentState = STATE_SET_MINUTE;
    } else if (currentState == STATE_SET_MINUTE) {
        tmElements_t now;
        breakTime(rtc.get(), now);
        now.Hour = setHour;
        now.Minute = setMin;
        now.Second = 0;
        rtc.set(makeTime(now));
        currentState = STATE_NORMAL;
        digitalWrite(PIN_LED, HIGH); // Turn off active-low LED
    }
}

// ── Display Update Routine ───────────────────────────────────────────────────
// Returns true if the AP650 acknowledged every byte of the frame.
bool updateDisplay() {
    lcd.clearBuffer();

    // lowBatt is on if the boot-time reading was low, regardless of screen
    bool showLowBatt = lowBattActive;

    if (currentState == STATE_NORMAL) {
        if (currentMode == MODE_CLOCK) {
            time_t now = rtc.get();
            uint8_t h = hour(now);
            uint8_t m = minute(now);
            bool isPM = (h >= 12);
            uint8_t h12 = h % 12;
            if (h12 == 0) h12 = 12;

            lcd.setTime(h12, m, false);
            lcd.am(!isPM);
            lcd.pm(isPM);
            lcd.colon(true);

        } else if (currentMode == MODE_TEMP) {
            sensors_event_t humidity, temp;
            aht.getEvent(&humidity, &temp);
            float t = constrain(temp.temperature, 0.0, 99.9);

            lcd.setDigit(0, SEG7_LOWER_C); 
            lcd.sensorTemp(true);
            
            lcd.setDigitValue(1, (int(t) / 10) % 10);
            lcd.setDigitValue(2, int(t) % 10);
            lcd.setDigitValue(3, int(t * 10.0) % 10);
            lcd.decimalPoint(2, true);

        } else if (currentMode == MODE_HUMIDITY) {
            sensors_event_t humidity, temp;
            aht.getEvent(&humidity, &temp);
            float h = constrain(humidity.relative_humidity, 0.0, 99.9);

            lcd.setDigit(0, SEG7_LOWER_R); 
            lcd.sun(true);
            
            lcd.setDigitValue(1, (int(h) / 10) % 10);
            lcd.setDigitValue(2, int(h) % 10);
            lcd.setDigitValue(3, int(h * 10.0) % 10);
            lcd.decimalPoint(2, true);

        } else if (currentMode == MODE_VBAT) {
            showLowBatt = true;     // always on while showing vBat

            if (vBatMv < VBAT_3DIGIT_MV) {
                // Below 4 V: show all four digits as volts, e.g. 3712 for 3.712 V.
                // NOTE: the glass has no decimal point after digit 0, so none is drawn.
                uint32_t mv = vBatMv;
                lcd.setDigitValue(0, (mv / 1000) % 10);
                lcd.setDigitValue(1, (mv / 100)  % 10);
                lcd.setDigitValue(2, (mv / 10)   % 10);
                lcd.setDigitValue(3,  mv         % 10);
            } else {
                // 4 V and up: digit 0 blank, #.## on the three right digits, e.g. _4.12
                uint32_t centiVolts = (vBatMv + 5) / 10;      // round to 0.01 V
                if (centiVolts > 999) centiVolts = 999;

                lcd.setDigit(0, 0x00);
                lcd.setDigitValue(1, (centiVolts / 100) % 10);
                lcd.setDigitValue(2, (centiVolts / 10)  % 10);
                lcd.setDigitValue(3,  centiVolts        % 10);
                lcd.decimalPoint(1, true);                    // between 2nd and 3rd digit
            }
        }
    } else {
        // Clock Setting Mode (24-hour format)
        lcd.setTime(setHour, setMin, true);
        lcd.colon(true);
        if ((millis() / 250) % 2 == 0) {
            if (currentState == STATE_SET_HOUR) {
                lcd.setDigit(0, 0x00);
                lcd.setDigit(1, 0x00);
            } else if (currentState == STATE_SET_MINUTE) {
                lcd.setDigit(2, 0x00);
                lcd.setDigit(3, 0x00);
            }
        }
    }

    lcd.lowBatt(showLowBatt);
    return lcd.flush();
}

// ── Prepare Power and Enter Deep Sleep ───────────────────────────────────────
void enterDeepSleep(uint32_t elapsedMs) {
    // Calculate sleep duration (60 seconds - elapsed time)
    const uint32_t targetSleepMs = 60000;
    uint32_t sleepDurationMs = (targetSleepMs > elapsedMs) ? (targetSleepMs - elapsedMs) : 1000;

    // 1. Unpower sensors to eliminate standby draw
    digitalWrite(PIN_SENSOR_PWR, LOW);
    gpio_pulldown_en(PWR_SENSORS_GPIO);
    gpio_pullup_dis(PWR_SENSORS_GPIO);
    gpio_hold_en(PWR_SENSORS_GPIO);
    delay(2);
    // 2. Release the I2C peripheral and float the pins so current doesn't leak into unpowered ICs
    Wire.end();
    pinMode(I2C_SDA, INPUT);
    pinMode(I2C_SCL, INPUT);
    gpio_pullup_dis((gpio_num_t)I2C_SDA);
    gpio_pulldown_en((gpio_num_t)I2C_SDA); // or dis depending on your hardware pull-up
    gpio_pullup_dis((gpio_num_t)I2C_SCL);
    gpio_pulldown_dis((gpio_num_t)I2C_SCL);

    // 3. Ensure LED is OFF (Active Low -> HIGH) and held
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    gpio_hold_en((gpio_num_t)PIN_LED);

    // 4. Drive & hold LCD control lines (RSTB, CLK, DIO HIGH) so AP650 stays active
    pinMode(PIN_RSTB, OUTPUT); digitalWrite(PIN_RSTB, HIGH);
    pinMode(PIN_CLK,  OUTPUT); digitalWrite(PIN_CLK, HIGH);
    pinMode(PIN_DIO,  OUTPUT); digitalWrite(PIN_DIO, HIGH);

    gpio_hold_en((gpio_num_t)PIN_RSTB);
    gpio_hold_en((gpio_num_t)PIN_CLK);
    gpio_hold_en((gpio_num_t)PIN_DIO);

    // 5. Button: make sure its pull-up survives deep sleep (the digital pad config is
    //    lost otherwise). Harmless if there is also an external pull-up.
    gpio_pullup_en((gpio_num_t)PIN_BTN);
    gpio_pulldown_dis((gpio_num_t)PIN_BTN);
    gpio_hold_en((gpio_num_t)PIN_BTN);

    // 6. The vBat divider pin is an ADC input; keep it a plain high-Z input
    pinMode(PIN_VBAT, INPUT);

    // 7. Enable global GPIO retention in deep sleep
    gpio_deep_sleep_hold_en();

    // 8. Configure wake up sources
    esp_sleep_enable_timer_wakeup((uint64_t)sleepDurationMs * 1000ULL);
    esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_BTN, ESP_GPIO_WAKEUP_GPIO_LOW);

    esp_deep_sleep_start();
}

// ── Setup & Execution ────────────────────────────────────────────────────────
void setup() {
    uint32_t startTime = millis();

    // Warm wake = we were in deep sleep with the AP650 still powered and showing a frame.
    // On a warm wake the glass is not cleared, so the old frame stays up until the new one
    // is flushed (no visible flash).
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool warmWake = (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER ||
                     wakeup_reason == ESP_SLEEP_WAKEUP_GPIO);

    // Release any GPIO holds from previous sleep cycle
    gpio_hold_dis((gpio_num_t)PIN_RSTB);
    gpio_hold_dis((gpio_num_t)PIN_CLK);
    gpio_hold_dis((gpio_num_t)PIN_DIO);
    gpio_hold_dis((gpio_num_t)PIN_LED);
    gpio_hold_dis((gpio_num_t)PIN_BTN);
    gpio_hold_dis(PWR_SENSORS_GPIO);
    gpio_deep_sleep_hold_dis();

    //Serial.begin(115200);

    // Read the battery first on every boot, before the sensors / LCD / LED add any load.
    // The result drives the lowBatt icon on all screens.
    vBatMv = readVBatMv();
    lowBattActive = (vBatMv < VBAT_LOW_MV);

    // Power up sensors and give them time to stabilize
    pinMode(PIN_SENSOR_PWR, OUTPUT);
    digitalWrite(PIN_SENSOR_PWR, HIGH);
    delay(40);

    // Init I2C & Sensors
    Wire.begin(I2C_SDA, I2C_SCL);

    rtc.begin(); 
    if (rtc.oscStopped()) { // OSF check
        rtc.set(compileTime() + 30);
    }
    // Clear status register (removes OSF and alarm flags)
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 1);
    uint8_t stat = Wire.read();
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.write(stat & 0x78); // Clear OSF (bit 7) and alarm flags (bits 0, 1)
    Wire.endTransmission();
    // DS3231 powers up with the 32 kHz output enabled and the INT/SQW pin active.
    // Turn both off so nothing toggles into a pull-up resistor all day.
    rtc.enable32kHz(false);
    rtc.squareWave(DS3232RTC::SQWAVE_NONE);
    

    aht.begin();

    // Init LCD Controller
    pinMode(PIN_RSTB, OUTPUT);
    digitalWrite(PIN_RSTB, HIGH);
    lcd.begin(!warmWake);       // warm wake: configure the chip but leave the glass alone

    // Init Controls
    pinMode(PIN_BTN, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Check if woken by Button
    if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
        uint32_t pressStart = millis();
        bool held = false;

        while (digitalRead(PIN_BTN) == LOW) {
            if (!held && (millis() - pressStart >= 2000)) {
                held = true;
                onButtonHold();
                break;
            }
            delay(10);
        }

        if (!held && (millis() - pressStart > 30)) {
            onButtonShortPress();
        }

        // Wait for button release before proceeding
        while (digitalRead(PIN_BTN) == LOW) { delay(10); }
    }

    // Interactive Loop (Only runs while actively in Clock Setting Mode)
    if (currentState != STATE_NORMAL) {
        uint32_t lastActivity = millis();
        while (currentState != STATE_NORMAL) {
            // Blink status LED
            digitalWrite(PIN_LED, (millis() / 250) % 2 == 0 ? LOW : HIGH);

            // Handle button presses while in menu
            static bool lastBtn = false;
            static uint32_t pressTime = 0;
            static bool handledHold = false;
            bool btn = (digitalRead(PIN_BTN) == LOW);

            if (btn && !lastBtn) {
                pressTime = millis();
                handledHold = false;
                lastActivity = millis();
            } else if (btn && lastBtn) {
                if (!handledHold && (millis() - pressTime >= 2000)) {
                    handledHold = true;
                    onButtonHold();
                    lastActivity = millis();
                }
            } else if (!btn && lastBtn) {
                if (!handledHold && (millis() - pressTime > 30)) {
                    onButtonShortPress();
                    lastActivity = millis();
                }
            }
            lastBtn = btn;

            updateDisplay();
            delay(20);

            // Timeout after 30 seconds of inactivity to prevent battery drain
            if (millis() - lastActivity > 30000) {
                currentState = STATE_NORMAL;
                digitalWrite(PIN_LED, HIGH);
            }
        }
    }

    // Normal path: update display once and go back to deep sleep
    updateDisplay();
    uint32_t elapsedMs = millis() - startTime;
    enterDeepSleep(elapsedMs);
}

void loop() {
    // Unused (Deep sleep restarts execution at setup())
}
