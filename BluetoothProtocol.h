// ============================================================================
// BluetoothProtocol.h (CRC + SAFETY INTERFACE)
// ============================================================================

#ifndef BLUETOOTH_PROTOCOL_H
#define BLUETOOTH_PROTOCOL_H

#include <Arduino.h>

// ==================================================
// PUBLIC INTERFACE
// ==================================================

/**
 * @brief เริ่มต้นระบบ Bluetooth Protocol
 * ต้องเรียกใน setup() หลัง Serial2.begin()
 */
void btProtocolInit();

/**
 * @brief อัปเดตระบบ Bluetooth (อ่าน/parse/timeout)
 * ต้องเรียกใน loop() ทุก cycle
 */
void btProtocolUpdate();

#endif // BLUETOOTH_PROTOCOL_H

