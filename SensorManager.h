//SensorManager.h

#pragma once

#include <Arduino.h>

// ======================================================
// SENSOR CORE
// ======================================================

bool updateSensors(void);
bool readDriverTempsPT100(int &tL, int &tR);

// ======================================================
// CURRENT CALIBRATION (โยกไป module แยก)
// ======================================================

bool calibrateCurrentOffsetNonBlocking(uint32_t now);
void idleCurrentAutoRezero(uint32_t now);
void sensorAdcTrigger(void);


