// ============================================================================
// BluetoothManager.cpp (COMMAND INTERFACE)
// ============================================================================

#include <Arduino.h>
#include "SystemTypes.h"
#include "GlobalState.h"
#include "FaultManager.h"

extern KillType killRequest;

// ======================================================
// UPDATE COMMAND
// ======================================================

void updateBluetoothCommand()
{
  if (!Serial2.available())
    return;

  String cmd = Serial2.readStringUntil('\n');
  cmd.trim();

  // ==================================================
  // COMMAND PARSER
  // ==================================================

  if (cmd == "START")
  {
    systemState = SystemState::ACTIVE;
  }
  else if (cmd == "STOP")
  {
    killRequest = KillType::HARD;
  }
  else if (cmd == "RESET")
  {
    clearFault();
  }

#if DEBUG_SERIAL
  Serial.print(F("[BT CMD] "));
  Serial.println(cmd);
#endif
}

