// ============================================================================
// DriveController.h
// ============================================================================

#pragma once
#include <Arduino.h>
#include "SystemTypes.h"

// ============================================================================
// MAIN DRIVE CONTROL
// ============================================================================

void runDrive(uint32_t now);
void applyDrive();
void driveSafe();

// ============================================================================
// TARGET / LIMIT CONTROL
// ============================================================================

void computeDriveTarget(
  float &finalTargetL,
  float &finalTargetR,
  uint32_t now);

void applyDriveLimits(
  float &finalTargetL,
  float &finalTargetR,
  float curA_L,
  float curA_R);

void updateDriveRamp(
  float finalTargetL,
  float finalTargetR);

void outputMotorPWM();

// ============================================================================
// STALL / LOAD MANAGEMENT
// ============================================================================

float computeStallScale(
  uint32_t now,
  float curLeft,
  float curRight);

// ============================================================================
// AUTO REVERSE
// ============================================================================

void startAutoReverse(uint32_t now);
void forceDriveSoftStop(uint32_t now);

// ============================================================================
// DRIVE FAULT DETECTION
// ============================================================================

void detectWheelStuck(uint32_t now);
void detectWheelLock();
bool detectMotorStall();
void detectSideImbalanceAndSteer();

// ============================================================================
// DRIVE COMMAND STATUS
// ============================================================================

bool driveCommandZero();