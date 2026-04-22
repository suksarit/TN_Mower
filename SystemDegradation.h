//SystemDegradation.h

#pragma once
#include <stdint.h>

enum class SystemMode : uint8_t
{
    NORMAL = 0,
    DEGRADED_L1,
    DEGRADED_L2,
    FAULT
};

void setSystemMode(SystemMode mode);
SystemMode getSystemMode(void);

// helper
bool isDegraded(void);
float getPowerScale(void);
float getRampScale(void);

