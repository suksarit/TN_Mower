// ========================================================================================
// GlobalState.cpp  (TN MOWER GLOBAL STATE INSTANCE)
// ========================================================================================

#include "GlobalState.h"

// ==================================================
// GLOBAL BUFFER INSTANCE
// ==================================================

// ใช้กับ ISR → ต้อง volatile
volatile DriveBuffer driveBufISR = {};

// ใช้ใน main loop → ไม่ต้อง volatile
DriveBuffer driveBufMain = {};

