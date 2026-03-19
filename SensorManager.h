// ============================================================================
// SensorManager.h (FIXED - ADD CURRENT GETTERS)
// ============================================================================

#pragma once

#include <Arduino.h>

void i2cBusClear(void);

// ======================================================
// SENSOR CORE
// ======================================================

// อัปเดตค่าจาก sensor ทั้งระบบ
bool updateSensors(void);

// อ่านอุณหภูมิ driver (PT100)
bool readDriverTempsPT100(int &tL, int &tR);

// ======================================================
// CURRENT CALIBRATION
// ======================================================

// คาลิเบรต offset แบบไม่ block
bool calibrateCurrentOffsetNonBlocking(uint32_t now);

// auto rezero ตอน idle
void idleCurrentAutoRezero(uint32_t now);

// trigger ADC (เรียกจาก ISR)
void sensorAdcTrigger(void);

// ======================================================
// 🔴 CURRENT GETTERS (FIX COMPILATION ERROR)
// ======================================================

// อ่านค่ากระแสมอเตอร์ซ้าย (หน่วยตามที่คุณใช้ เช่น A หรือ mA)
float getMotorCurrentL(void);

// อ่านค่ากระแสมอเตอร์ขวา
float getMotorCurrentR(void);

// ======================================================
// SAFE CURRENT (REDUNDANCY)
// ======================================================
float getMotorCurrentSafeL(void);
float getMotorCurrentSafeR(void);

// ======================================================
// SENSOR TASK (NEW)
// ======================================================
void sensorTask(uint32_t now);

