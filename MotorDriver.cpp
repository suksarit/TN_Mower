// ============================================================================
// MotorDriver.cpp (FINAL FIXED - SAFE + DETERMINISTIC)
// - PWM sync via Timer3 overflow (TOP)
// - atomic write ป้องกันค่าเพี้ยน
// - fan mapping ถูกต้อง
// - ไม่มี delay block
// ============================================================================

#include <Arduino.h>

#include "MotorDriver.h"
#include "HardwareConfig.h"
#include "GlobalState.h"

// ============================================================================
// PWM CONSTANT
// ============================================================================
#ifndef PWM_TOP
#define PWM_TOP 1067
#endif

// ============================================================================
// PWM BUFFER (WRITE FROM CONTROL LOOP)
// ============================================================================
volatile uint16_t pwmL_req = 0;
volatile uint16_t pwmR_req = 0;

volatile uint16_t fanPwmL_req = 0;
volatile uint16_t fanPwmR_req = 0;

// ============================================================================
// PWM SETUP (MOTOR)
// ============================================================================
void setupPWM15K()
{
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR4A = 0;
  TCCR4B = 0;

  OCR3A = 0;
  OCR4A = 0;

  // TIMER3 → LEFT
  TCCR3A = _BV(COM3A1) | _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30);
  ICR3 = PWM_TOP;

  // TIMER4 → RIGHT
  TCCR4A = _BV(COM4A1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);
  ICR4 = PWM_TOP;

  // ENABLE SYNC ISR
  TIMSK3 |= (1 << TOIE3);
}

// ============================================================================
// PWM SETUP (FAN)
// ============================================================================
void setupFanPWM15K()
{
  TCCR5A = _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS50);

  ICR5 = PWM_TOP;

  OCR5B = 0;
  OCR5C = 0;
}

// ============================================================================
// ATOMIC WRITE HELPERS (CRITICAL)
// ============================================================================
inline void atomicWrite16(volatile uint16_t &var, uint16_t value)
{
  uint8_t sreg = SREG;
  cli();
  var = value;
  SREG = sreg;
}

// ============================================================================
// WRITE BUFFER (SAFE)
// ============================================================================
void setPWM_L(uint16_t v)
{
  if (v > PWM_TOP) v = PWM_TOP;
  atomicWrite16(pwmL_req, v);
}

void setPWM_R(uint16_t v)
{
  if (v > PWM_TOP) v = PWM_TOP;
  atomicWrite16(pwmR_req, v);
}

void setFanPWM_L(uint16_t pwm)
{
  if (pwm > PWM_TOP) pwm = PWM_TOP;
  atomicWrite16(fanPwmL_req, pwm);
}

void setFanPWM_R(uint16_t pwm)
{
  if (pwm > PWM_TOP) pwm = PWM_TOP;
  atomicWrite16(fanPwmR_req, pwm);
}

// ============================================================================
// PWM SYNC ISR (CORE)
// ============================================================================
ISR(TIMER3_OVF_vect)
{
  uint16_t l  = pwmL_req;
  uint16_t r  = pwmR_req;
  uint16_t fl = fanPwmL_req;
  uint16_t fr = fanPwmR_req;

  // clamp กันค่าหลุด
  if (l  > PWM_TOP) l  = PWM_TOP;
  if (r  > PWM_TOP) r  = PWM_TOP;
  if (fl > PWM_TOP) fl = PWM_TOP;
  if (fr > PWM_TOP) fr = PWM_TOP;

  // apply sync
  OCR3A = l;
  OCR4A = r;

  // ✅ FIX: mapping ถูกต้อง (L → B, R → C)
  OCR5B = fl;
  OCR5C = fr;
}

// ============================================================================
// DRIVE SAFE
// ============================================================================
void driveSafe()
{
  digitalWrite(PIN_DRV_ENABLE, LOW);

  atomicWrite16(pwmL_req, 0);
  atomicWrite16(pwmR_req, 0);
  atomicWrite16(fanPwmL_req, 0);
  atomicWrite16(fanPwmR_req, 0);

  // apply ทันที
  OCR3A = 0;
  OCR4A = 0;
  OCR5B = 0;
  OCR5C = 0;

  curL = 0;
  curR = 0;
  targetL = 0;
  targetR = 0;

  HBRIDGE_ALL_OFF();
}

// ============================================================================
// SHORT BRAKE
// ============================================================================
void motorShortBrake()
{
  atomicWrite16(pwmL_req, 0);
  atomicWrite16(pwmR_req, 0);

  OCR3A = 0;
  OCR4A = 0;

  HBRIDGE_ALL_OFF();
}