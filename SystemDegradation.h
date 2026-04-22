// SystemDegradation.h

#pragma once
#include <stdint.h>

// ======================================================
// SYSTEM MODE (ศูนย์กลางการตัดสินใจของทั้งระบบ)
// ======================================================
enum class SystemMode : uint8_t
{
    NORMAL = 0,        // ปกติ
    DEGRADED_L1,       // ลดกำลังเล็กน้อย
    DEGRADED_L2,       // ลดกำลังมาก
    FAULT              // หยุดระบบ
};

// ======================================================
// CORE API (ใช้ทั่วทั้งระบบ)
// ======================================================

// ตั้งค่า mode (มี priority ภายใน .cpp)
void setSystemMode(SystemMode mode);

// อ่าน mode ปัจจุบัน
SystemMode getSystemMode(void);

// ======================================================
// 🔴 POWER SCALE (ใช้โดย PowerManager)
// ======================================================
float getSystemPowerScale(void);

// ======================================================
// HELPER (ใช้ตรวจสถานะง่าย ๆ)
// ======================================================

// อยู่ในโหมด degraded หรือไม่
bool isSystemDegraded(void);

// อยู่ใน fault หรือไม่
bool isSystemFault(void);

