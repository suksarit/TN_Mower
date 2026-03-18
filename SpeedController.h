// SpeedController.h

#pragma once

#include <stdint.h>

// ======================================================
// MAIN SPEED CONTROL
// ======================================================
void applySpeedControl(float &targetL, float &targetR);

// ======================================================
// FEEDBACK INPUT (ต้องมี sensor)
// ======================================================
float getWheelSpeedL();
float getWheelSpeedR();

