// ThermalManager.h

#pragma once
#include <Arduino.h>

float getThermalScale(void);
bool  isThermalEmergency(void);
void  updateThermalManager(uint32_t now);