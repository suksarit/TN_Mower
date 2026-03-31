// ========================================================================================
// HardwareConfig.h  
// ========================================================================================

#pragma once
#include <Arduino.h>

// ======================================================
// PWM
// ======================================================
constexpr int16_t PWM_TOP = 1067;   // 15kHz PWM

// ======================================================
// CURRENT LIMITS
// ======================================================
constexpr float CUR_WARN_A = 20.0f;
constexpr float CUR_LIMP_A = 30.0f;
constexpr float CUR_SPIKE_A = 120.0f;

constexpr int16_t CUR_MIN_PLAUSIBLE = -10;
constexpr int16_t CUR_MAX_PLAUSIBLE = 150;

constexpr float CUR_LPF_ALPHA = 0.12f;

constexpr int16_t CUR_TRIP_A_CH[4] =
{
  100,
  100,
  90,
  90
};

// ======================================================
// CURRENT SENSOR
// ======================================================
constexpr float ACS_SENS_V_PER_A = 0.04f;

// ======================================================
// TEMPERATURE LIMIT
// ======================================================
constexpr int TEMP_WARN_C = 70;
constexpr int TEMP_LIMP_C = 80;
constexpr int TEMP_TRIP_C = 90;

// ======================================================
// VOLTAGE WARNING
// ======================================================
constexpr float V_WARN_LOW = 24.0f;
constexpr float V_WARN_CRITICAL = 23.0f;

// ======================================================
// ADS1115
// ======================================================
constexpr float ADS1115_LSB_V = 4.096f / 32768.0f;

constexpr uint8_t ADS_CUR_CH_MAP[4] =
{
  0,
  1,
  2,
  3
};

// ======================================================
// REVERSE CONTROL
// ======================================================
constexpr uint32_t REVERSE_DEADTIME_MS = 150;
constexpr uint32_t REVERSE_RECOVERY_MS = 500;
constexpr float REVERSE_RECOVERY_LIMIT = 0.5f;

// ======================================================
// AUTO REVERSE
// ======================================================
constexpr uint8_t MAX_AUTO_REVERSE = 5;

// ======================================================
// STALL CONTROL
// ======================================================
constexpr float STALL_CURRENT_A = 40.0f;
constexpr float STALL_DECAY = 0.95f;
constexpr float STALL_POWER_LIMIT = 1200.0f;

// ======================================================
// TIMEOUTS
// ======================================================
constexpr uint32_t LIMP_RECOVER_MS = 1000;
constexpr uint32_t DRIVE_SOFT_STOP_TIMEOUT_MS = 1500;
constexpr uint32_t BLADE_SOFT_STOP_TIMEOUT_MS = 3000;

// ======================================================
// EEPROM FAULT CONTROL
// ======================================================
constexpr uint8_t MAX_FAULT_WRITES_PER_BOOT = 8;
constexpr uint32_t FAULT_EEPROM_COOLDOWN_MS = 5000;

// ======================================================
// PINS
// ======================================================
constexpr uint8_t PIN_DRV_ENABLE = 7;
constexpr uint8_t PIN_CUR_TRIP = 8;

constexpr uint8_t PIN_BUZZER = 30;
constexpr uint8_t RELAY_WARN = 31;

constexpr uint8_t RELAY_IGNITION = 28;
constexpr uint8_t RELAY_STARTER = 29;

constexpr uint8_t PIN_HW_WD_HB = 34;

// ======================================================
// MOTOR DIR PINS
// ======================================================
constexpr uint8_t DIR_L1 = 22;
constexpr uint8_t DIR_L2 = 23;
constexpr uint8_t DIR_R1 = 24;
constexpr uint8_t DIR_R2 = 25;

// ======================================================
// FAN
// ======================================================
constexpr uint8_t FAN_L = 44;
constexpr uint8_t FAN_R = 45;

// ======================================================
// SERVO
// ======================================================
constexpr uint8_t SERVO_ENGINE_PIN = 9;

// ======================================================
// FAST HBRIDGE CONTROL (PORTA DIRECT)
// ======================================================
#define HBRIDGE_L_OFF() (PORTA &= ~0b00000011)

#define HBRIDGE_L_FWD() \
{ PORTA = (PORTA & ~0b00000011) | 0b00000001; }

#define HBRIDGE_L_REV() \
{ PORTA = (PORTA & ~0b00000011) | 0b00000010; }

#define HBRIDGE_R_OFF() (PORTA &= ~0b00001100)

#define HBRIDGE_R_FWD() \
{ PORTA = (PORTA & ~0b00001100) | 0b00000100; }

#define HBRIDGE_R_REV() \
{ PORTA = (PORTA & ~0b00001100) | 0b00001000; }

#define HBRIDGE_ALL_OFF() (PORTA &= ~0b00001111)


// ======================================================
// ไฟส่องสว่าง
// ======================================================
#define CH_LIGHT 5     // ช่องรีโมท (เช่น CH6 จริง)
#define PIN_LIGHT 30   // ขา output ไฟ

// ======================================================
// IBUS
// ======================================================
constexpr uint32_t IBUS_TIMEOUT_MS = 300;

constexpr uint8_t CH_STEER = 1;
constexpr uint8_t CH_THROTTLE = 2;
constexpr uint8_t CH_ENGINE = 8;
constexpr uint8_t CH_IGNITION = 6;
constexpr uint8_t CH_STARTER = 10;

// ======================================================
// ENGINE
// ======================================================
constexpr float ENGINE_RUNNING_VOLT = 27.2f;
constexpr float ENGINE_STOP_VOLT = 25.5f;
constexpr uint32_t ENGINE_CONFIRM_MS = 900;
constexpr uint32_t ENGINE_RESTART_GUARD_MS = 3000;

// ======================================================
// STARTER
// ======================================================
constexpr uint32_t STARTER_MAX_MS = 5000;

