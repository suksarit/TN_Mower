// ============================================================================
// MotorDriver.cpp 
// ============================================================================

#include <Arduino.h>

#include "MotorDriver.h"
#include "HardwareConfig.h"
#include "GlobalState.h"
#include "SensorManager.h"

// ============================================================================
// PWM CONSTANT
// ============================================================================
#ifndef PWM_TOP
#define PWM_TOP 1067
#endif

// ============================================================================
// PWM BUFFER
// ============================================================================
volatile uint16_t pwmL_req = 0;
volatile uint16_t pwmR_req = 0;

volatile uint16_t fanPwmL_req = 0;
volatile uint16_t fanPwmR_req = 0;

// ============================================================================
// INTERNAL LIMIT (ANTI SPIKE)
// ============================================================================
static uint16_t pwmL_last = 0;
static uint16_t pwmR_last = 0;

constexpr uint16_t PWM_SLEW_LIMIT = 40; // จำกัดการเปลี่ยนต่อ ISR

// ============================================================================
// PWM SETUP
// ============================================================================
void setupPWM15K()
{
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR4A = 0;
  TCCR4B = 0;

  TCCR3A = _BV(COM3A1) | _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30);
  ICR3 = PWM_TOP;

  OCR3A = 0;

  TCCR4A = _BV(COM4A1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);
  ICR4 = PWM_TOP;

  OCR4A = 0;

  TIMSK3 |= (1 << TOIE3);
  TIMSK3 |= (1 << OCIE3B);
}

// ============================================================================
// FAN PWM (ไม่เอาเข้า ISR แล้ว)
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
// ATOMIC WRITE
// ============================================================================
inline void atomicWrite16(volatile uint16_t &var, uint16_t value)
{
  uint8_t s = SREG;
  cli();
  var = value;
  SREG = s;
}

// ============================================================================
// SET PWM (REQUEST)
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
  OCR5C = pwm; // เขียนตรง ไม่ต้องผ่าน ISR
}

void setFanPWM_R(uint16_t pwm)
{
  if (pwm > PWM_TOP) pwm = PWM_TOP;
  OCR5B = pwm;
}

// ============================================================================
// SLEW LIMIT
// ============================================================================
inline uint16_t applySlew(uint16_t target, uint16_t last)
{
  if (target > last + PWM_SLEW_LIMIT)
    return last + PWM_SLEW_LIMIT;

  if (target + PWM_SLEW_LIMIT < last)
    return last - PWM_SLEW_LIMIT;

  return target;
}

// ============================================================================
// PWM APPLY (ISR)
// ============================================================================
ISR(TIMER3_OVF_vect)
{
  uint16_t l = pwmL_req;
  uint16_t r = pwmR_req;

  if (l > PWM_TOP) l = PWM_TOP;
  if (r > PWM_TOP) r = PWM_TOP;

  // 🔴 slew rate limit กันกระชาก
  l = applySlew(l, pwmL_last);
  r = applySlew(r, pwmR_last);

  pwmL_last = l;
  pwmR_last = r;

  OCR3A = l;
  OCR4A = r;
}

// ============================================================================
// ADC TRIGGER
// ============================================================================
ISR(TIMER3_COMPB_vect)
{
  sensorAdcTrigger();
}

// ============================================================================
// DRIVE SAFE (HARD CUT)
// ============================================================================
void driveSafe()
{
  digitalWrite(PIN_DRV_ENABLE, LOW);

  atomicWrite16(pwmL_req, 0);
  atomicWrite16(pwmR_req, 0);

  pwmL_last = 0;
  pwmR_last = 0;

  curL = 0;
  curR = 0;
  targetL = 0;
  targetR = 0;

  HBRIDGE_ALL_OFF();
}

// ============================================================================
// TRUE SHORT BRAKE (แก้จริง)
// ============================================================================
void motorShortBrake()
{
  atomicWrite16(pwmL_req, 0);
  atomicWrite16(pwmR_req, 0);

  // 🔴 short motor จริง
  PORTA |= 0b00001111; // L1 L2 R1 R2 = HIGH
}

