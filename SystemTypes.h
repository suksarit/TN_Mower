// ========================================================================================
// SystemTypes.h  (FINAL - PRODUCTION SAFE / EXTENDABLE / EEPROM SAFE)
// ========================================================================================

#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>

// ============================================================================
// SAFETY STATE
// ============================================================================
enum class SafetyState : uint8_t {
  SAFE = 0,
  WARN = 1,
  LIMP = 2,
  EMERGENCY = 3,
  _COUNT
};

// ============================================================================
// DRIVE EVENT
// ⚠️ ห้ามเปลี่ยนค่าของตัวเดิม (EEPROM / LOG)
// เพิ่มใหม่ต้อง "ต่อท้าย" เท่านั้น
// ============================================================================
enum class DriveEvent : uint8_t {
  NONE = 0,
  IMBALANCE = 1,
  STUCK_LEFT = 2,
  STUCK_RIGHT = 3,
  WHEEL_LOCK = 4,
  AUTO_REVERSE = 5,

  // ==================================================
  // 🔴 เพิ่มของเดิมที่คุณมี
  // ==================================================
  WHEEL_STUCK = 6,        // ติดหล่ม (recoverable)
  TRACTION_LOSS = 7,      // ล้อฟรี (slip)

  // ==================================================
  // 🔥 เพิ่ม production (ต่อท้ายเท่านั้น)
  // ==================================================
  STALL = 8,              // มอเตอร์เริ่ม stall (soft protection)
  STUCK = 9,              // stuck แบบรวม (ไม่แยกฝั่ง)
  LOAD_HIGH = 10,         // โหลดสูง (หญ้าหนา)
  LOAD_SPIKE = 11,        // โหลดพุ่งเร็วผิดปกติ
  CURRENT_LIMIT = 12,     // ถูกจำกัดด้วย current
  SAFETY_LIMIT = 13,      // ถูกจำกัดด้วย safety
  THERMAL_LIMIT = 14,     // จำกัดเพราะความร้อน

  _COUNT
};

// ============================================================================
// SYSTEM STATE
// ============================================================================
enum class SystemState : uint8_t {
  INIT = 0,
  ACTIVE = 1,
  FAULT = 2,
  _COUNT
};

// ============================================================================
// DRIVE STATE
// ============================================================================
enum class DriveState : uint8_t {
  IDLE = 0,
  RUN = 1,
  LIMP = 2,
  SOFT_STOP = 3,
  LOCKED = 4,
  _COUNT
};

// ============================================================================
// BLADE STATE
// ============================================================================
enum class BladeState : uint8_t {
  IDLE = 0,
  RUN = 1,
  SOFT_STOP = 2,
  LOCKED = 3,
  _COUNT
};

// ============================================================================
// FAULT CODE (EXTENDABLE / EEPROM SAFE)
// ⚠️ ห้ามสลับลำดับ
// ============================================================================
enum class FaultCode : uint8_t {
  NONE = 0,

  IBUS_LOST,
  COMMS_TIMEOUT,

  SENSOR_TIMEOUT,

  LOGIC_WATCHDOG,

  CUR_SENSOR_FAULT,
  VOLT_SENSOR_FAULT,
  TEMP_SENSOR_FAULT,

  OVER_CURRENT,
  OVER_TEMP,

  DRIVE_TIMEOUT,
  BLADE_TIMEOUT,

  LOOP_OVERRUN,
  LOW_VOLTAGE_CRITICAL,

  _COUNT
};

// ============================================================================
// I2C RECOVERY STATE
// ============================================================================
enum class I2CRecoverState : uint8_t {
  IDLE,
  END_BUS,
  BEGIN_BUS,
  REINIT_ADS,
  DONE
};

// ============================================================================
// ACS CALIBRATION STATE
// ============================================================================
enum class ACSCalState : uint8_t {
  IDLE,
  START_CH,
  WAIT_CONV,
  NEXT_SAMPLE,
  NEXT_CH,
  DONE,
  FAIL
};

// ============================================================================
// ENUM VALIDATION (GENERIC / SAFE)
// ============================================================================
template<typename E>
inline bool isValidEnum(uint8_t raw) {
  return raw < static_cast<uint8_t>(E::_COUNT);
}

#endif