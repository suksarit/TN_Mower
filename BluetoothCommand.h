#ifndef BLUETOOTH_COMMAND_H
#define BLUETOOTH_COMMAND_H

#include <Arduino.h>   // 🔴 ให้ type มาตรฐานครบ (uint8_t / millis)

/*
============================================================================
Bluetooth Command Interface
============================================================================
หน้าที่:
- รับคำสั่งจาก Android ผ่าน Bluetooth (Serial2)
- ตรวจ CRC + sequence
- ป้องกัน duplicate packet
- ส่ง ACK กลับ
- ตรวจ timeout เพื่อสั่ง kill ระบบ

สำคัญ:
- ต้องมี implementation ใน BluetoothCommand.cpp
- ห้ามใช้ static กับฟังก์ชันด้านล่าง (ไม่งั้น linker หาไม่เจอ)
============================================================================
*/

// ==================================================
// 🔴 รับคำสั่งจาก Bluetooth (เรียกใน loop())
// ==================================================
void btReceiveCommand(void);

// ==================================================
// 🔴 ตรวจ timeout ของคำสั่ง (failsafe)
// ==================================================
void btSafetyCheck(void);

#endif // BLUETOOTH_COMMAND_H