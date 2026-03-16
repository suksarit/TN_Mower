//SensorManager.h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31865.h>

#include "SystemTypes.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "DriveController.h"
#include "FaultManager.h"

#pragma once
#include <Arduino.h>

bool updateSensors();
bool readDriverTempsPT100(int &tL, int &tR);

void idleCurrentAutoRezero(uint32_t now);
bool calibrateCurrentOffsetNonBlocking(uint32_t now);