#pragma once

#include "Types.h"

struct RenderSettings {
    uint32 width;
    uint32 height;
    uint32 rayMarchingSteps;
    float rayMarchingEpsilon;
    float rayMarchingMaxDistance;
};
