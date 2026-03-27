// ============================================================================
// SensorManager.h  (CLEAN + SAFE + COMPLETE)
// ============================================================================

#pragma once

#include <Arduino.h>

// ======================================================
// LOW LEVEL
// ======================================================

// ล้างบัส I2C (ใช้ตอน recover)
void i2cBusClear(void);

// ======================================================
// SENSOR CORE
// ======================================================

// อัปเดตค่าจาก sensor ทั้งระบบ (เรียกใน loop หลัก)
bool updateSensors(void);

// ======================================================
// TEMPERATURE (PT100)
// ======================================================

// อ่านค่าอุณหภูมิ driver (ถ้าจะเรียกตรง)
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
// CURRENT GETTERS (IMPORTANT)
// ======================================================

// กระแสมอเตอร์ซ้าย (Amp)
float getMotorCurrentL(void);

// กระแสมอเตอร์ขวา (Amp)
float getMotorCurrentR(void);

// ======================================================
// SAFE CURRENT (REDUNDANCY)
// ======================================================

// เวอร์ชัน safe (ผ่าน clamp / fault)
float getMotorCurrentSafeL(void);
float getMotorCurrentSafeR(void);

// ======================================================
// SPEED (ใช้ current estimate)
// ======================================================

float getSpeedL(void);
float getSpeedR(void);

// ======================================================
// SENSOR TASK (MAIN ENTRY)
// ======================================================

// task หลักของ sensor (เรียกจาก loop)
void sensorTask(uint32_t now);

