// ========================================================================================
// GlobalState.cpp  (FINAL - INDUSTRIAL KILL + WATCHDOG + ANTI-GLITCH)
// ========================================================================================

#include "GlobalState.h"

// ==================================================
// GLOBAL BUFFER INSTANCE
// ==================================================

// 🔴 ใช้กับ ISR → ต้อง volatile
volatile DriveBuffer driveBufISR = {0.0f, 0.0f, 0.0f, 0.0f};

// 🔴 ใช้ใน main loop
DriveBuffer driveBufMain = {0.0f, 0.0f, 0.0f, 0.0f};

// ==================================================
// CONTROL / PHYSICS STATE
// ==================================================

// 🔴 dt ของ control loop (กัน 0 ตอน boot)
float controlDt_s = 0.02f;

// 🔴 ค่าเฉลี่ยแรงต้าน
float terrainDragAvg = 0.0f;

// ==================================================
// 🔴 KILL SYSTEM (INDUSTRIAL CORE)
// ==================================================

// 🔴 request จาก BT / Fault
KillType killRequest = KillType::NONE;

// 🔴 latch จริงของระบบ (ใช้ตัด loop)
bool killLatched = false;

// 🔴 ใช้ใน ISR (กัน PWM ค้างระดับ hardware timing)
volatile bool killISRFlag = false;

// ==================================================
// 🔴 CONTROL WATCHDOG (FREEZE DETECT)
// ==================================================

// 🔴 timestamp control ล่าสุด (อัปเดตใน runControlLoop)
volatile uint32_t lastControlExec_us = 0;

// ==================================================
// 🔴 ANTI-GLITCH FILTER STATE
// (ใช้ใน applyDrive หรือ runControlLoop เท่านั้น)
// ==================================================

float targetL_filtered = 0.0f;
float targetR_filtered = 0.0f;

// ==================================================
// 🔴 RC / COMM STATE
// ==================================================

// 🔴 timestamp ของ frame ล่าสุด (ใช้ detect RC freeze)
uint32_t rcLastFrame_ms = 0;

// ==================================================
// END OF FILE
// ==================================================

