// CurrentCalibration.h

#pragma once
#include <Arduino.h>

// ======================================================
// CURRENT OFFSET CALIBRATION
// ======================================================

// เรียกใน SystemState::INIT
bool calibrateCurrentOffsetNonBlocking(uint32_t now);

// ปรับ offset แบบ dynamic ตอน idle
void idleCurrentAutoRezero(uint32_t now);

