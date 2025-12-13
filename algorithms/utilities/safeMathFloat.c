#include "safeMathFloat.h"

#include <math.h>

float safeAcosf(const float x) {
    if (x < -1.0F) {
        return acosf(-1.0F);
    }
    if (x > 1.0F) {
        return acosf(1.0F);
    }
    return acosf(x);
}

float safeAsinf(const float x) {
    if (x < -1.0F) {
        return asinf(-1.0F);
    }
    if (x > 1.0F) {
        return asinf(1.0F);
    }
    return asinf(x);
}

float safeSqrtf(const float x) {
    if (x < 0.0F) {
        return 0.0F;
    }
    return sqrtf(x);
}
