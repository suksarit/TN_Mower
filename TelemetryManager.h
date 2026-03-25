// ============================================================================
// TelemetryManager.h (CLEAN + SAFE)
// ============================================================================

#pragma once

#include <Arduino.h>

// ======================================================
// TELEMETRY MODE CONTROL
// ======================================================

#if (TELEMETRY_CSV == 1) && (TELEMETRY_BINARY == 1)
#error "ห้ามเปิด TELEMETRY_CSV และ TELEMETRY_BINARY พร้อมกัน"
#endif

#ifndef TELEMETRY_CSV
#define TELEMETRY_CSV     0
#endif

#ifndef TELEMETRY_BINARY
#define TELEMETRY_BINARY  1
#endif

// ======================================================
// DEBUG CONTROL
// ======================================================

#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL 1
#endif

#ifndef TEST_MODE
#define TEST_MODE 0
#endif

// ======================================================
// TELEMETRY TIMING
// ======================================================

#ifndef TELEMETRY_PERIOD_MS
#define TELEMETRY_PERIOD_MS 20
#endif

// ======================================================
// CORE
// ======================================================

void telemetryCSV(uint32_t now, uint32_t loopStart_us);
void telemetryBinary(uint32_t now, uint32_t loopStart_us);

// ======================================================
// DEBUG (ANTI-SPAM)
// ======================================================

void debugTelemetry(uint32_t now);
void debugTestMode(uint32_t now);
void debugIBus(uint32_t now);

