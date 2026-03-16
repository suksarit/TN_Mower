//MotorDriver.cpp

#include <Arduino.h>

#include "MotorDriver.h"
#include "HardwareConfig.h"
#include "GlobalState.h"

void setupPWM15K() {

  // --------------------------------------------------
  // FORCE TIMER RESET (CRITICAL)
  // reset register ก่อน config ใหม่
  // --------------------------------------------------
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR4A = 0;
  TCCR4B = 0;

  // ปิด PWM output ก่อน
  OCR3A = 0;
  OCR4A = 0;

  // --------------------------------------------------
  // TIMER3 → LEFT MOTOR PWM
  // Fast PWM mode 14 (ICR3 = TOP)
  // --------------------------------------------------
  TCCR3A = _BV(COM3A1) | _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30);

  ICR3 = 1067;  // ≈15kHz
  OCR3A = 0;    // duty = 0 (safe)

  // --------------------------------------------------
  // TIMER4 → RIGHT MOTOR PWM
  // Fast PWM mode 14 (ICR4 = TOP)
  // --------------------------------------------------
  TCCR4A = _BV(COM4A1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);

  ICR4 = 1067;  // ≈15kHz
  OCR4A = 0;    // duty = 0 (safe)
}


void setupFanPWM15K() {
  // Fast PWM, TOP = ICR5
  TCCR5A = _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS50);  // prescaler = 1

  ICR5 = 1067;  // ~15 kHz @ 16 MHz

  OCR5B = 0;  // FAN_R (pin 45)
  OCR5C = 0;  // FAN_L (pin 44)
}


inline void setPWM_L(uint16_t v) {

  if (v > ICR3)
    v = ICR3;

  OCR3A = v;
}


inline void setPWM_R(uint16_t v) {
  OCR4A = constrain(v, 0, ICR4);
}


inline void setFanPWM_L(uint8_t pwm) {
  OCR5C = map(pwm, 0, 255, 0, ICR5);
}


inline void setFanPWM_R(uint8_t pwm) {
  OCR5B = map(pwm, 0, 255, 0, ICR5);
}


void driveSafe() {

  // --------------------------------------------------
  // 1) DISABLE DRIVER
  // --------------------------------------------------
  digitalWrite(PIN_DRV_ENABLE, LOW);

  // --------------------------------------------------
  // 2) HARD PWM REGISTER CLEAR
  // --------------------------------------------------
  OCR3A = 0;
  OCR4A = 0;

  // --------------------------------------------------
  // 3) SOFTWARE PWM CLEAR
  // --------------------------------------------------
  setPWM_L(0);
  setPWM_R(0);

  // --------------------------------------------------
  // 4) RESET RAMP STATE
  // --------------------------------------------------
  curL = 0;
  curR = 0;

  targetL = 0;
  targetR = 0;

  // --------------------------------------------------
  // 5) FORCE H-BRIDGE NEUTRAL (SIMULTANEOUS)
  // --------------------------------------------------
  HBRIDGE_ALL_OFF();

  // --------------------------------------------------
  // 6) SHORT DEADTIME
  // --------------------------------------------------
  delayMicroseconds(5);
}


void motorShortBrake() {

  setPWM_L(0);
  setPWM_R(0);

  HBRIDGE_ALL_OFF();
}


