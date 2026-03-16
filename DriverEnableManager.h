// DriverEnableManager.h

#include "SystemTypes.h"

#ifndef DRIVER_ENABLE_MANAGER_H
#define DRIVER_ENABLE_MANAGER_H

#include <Arduino.h>
#include "SystemTypes.h"
#include "GlobalState.h" 

struct DriverEnableContext
{
  // system state
  SystemState systemState;
  DriveState driveState;
  DriverState driverState;

  bool faultLatched;
  bool driverRearmRequired;
  bool requireIbusConfirm;
  bool autoReverseActive;

  // RC input
  uint16_t rcThrottle;
  uint16_t rcSteer;

  // motor state
  int16_t curL;
  int16_t curR;
  int16_t targetL;
  int16_t targetR;

  // timing
  uint32_t now;
  uint32_t systemActiveStart_ms;
  uint32_t driverStateStart_ms;
  uint32_t driverEnabled_ms;
  uint32_t driverActiveStart_ms;
};

void updateDriverEnable(DriverEnableContext &ctx);

#endif