// ========================================================================================
// GlobalState.cpp  (FINAL - INDUSTRIAL KILL SYSTEM)
// ========================================================================================

#include "GlobalState.h"

// ==================================================
// GLOBAL BUFFER INSTANCE
// ==================================================

// 🔴 ใช้กับ ISR → ต้อง volatile
volatile DriveBuffer driveBufISR = {0, 0, 0, 0};

// 🔴 ใช้ใน main loop
DriveBuffer driveBufMain = {0, 0, 0, 0};

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

// 🔴 latch จริงของระบบ
bool killLatched = false;

// 🔴 ใช้ใน ISR / PWM (เร็วสุด)
volatile bool killISRFlag = false;

// ==================================================
// 🔴 RC / COMM STATE
// ==================================================

// 🔴 timestamp ของ frame ล่าสุด
uint32_t rcLastFrame_ms = 0;

// ==================================================
// END OF FILE
// ==================================================

