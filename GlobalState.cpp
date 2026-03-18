// ========================================================================================
// GlobalState.cpp  (TN MOWER GLOBAL STATE INSTANCE)
// ========================================================================================

#include "GlobalState.h"

// ==================================================
// GLOBAL BUFFER INSTANCE
// ==================================================

// ใช้กับ ISR → ต้อง volatile
volatile DriveBuffer driveBufISR = {0,0,0,0};

// ใช้ใน main loop → ไม่ต้อง volatile
DriveBuffer driveBufMain = {0,0,0,0};

// ==================================================
// GLOBAL VARIABLES (DEFINE HERE ONLY)
// ==================================================

float terrainDragAvg = 0.0f;
float controlDt_s = 0.0f;

